#!/usr/bin/env python3
"""地面站 ROS2 节点 —— 集中所有 ROS 通信，与界面代码解耦。

界面通过 get_annotations() 轮询取模型标注、调用 publish_* 发指令，
不直接接触任何 rclpy 对象。将来加订阅/发布只改这里。
"""
import threading
import time

import rclpy
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from rclpy.qos import (HistoryPolicy, QoSProfile, ReliabilityPolicy)
from sensor_msgs.msg import Image
from std_msgs.msg import String

from . import config
from .overlay import parse_message


class TeeLogger:
    """rclpy Logger 分流器: 终端日志照常输出, 同时把 (级别, 文本) 回调给
    界面日志框。回调可能来自任意线程 (抓取线程等), 线程安全由回调方保证
    (界面侧经 Qt 信号队列投递到 GUI 线程)。"""

    def __init__(self, ros_logger):
        self._ros = ros_logger
        self._sink = None

    def set_sink(self, sink):
        self._sink = sink

    def _tee(self, level, msg, *args, **kwargs):
        getattr(self._ros, level)(msg, *args, **kwargs)
        if self._sink is not None:
            try:
                self._sink(level.upper(), str(msg))
            except Exception:
                pass    # 界面日志框异常不拖累 ROS 日志与主流程

    def debug(self, msg, *a, **kw): self._tee('debug', msg, *a, **kw)
    def info(self, msg, *a, **kw): self._tee('info', msg, *a, **kw)
    def warning(self, msg, *a, **kw): self._tee('warning', msg, *a, **kw)
    def error(self, msg, *a, **kw): self._tee('error', msg, *a, **kw)
    def fatal(self, msg, *a, **kw): self._tee('fatal', msg, *a, **kw)


class DisplayForwarder(threading.Thread):
    """显示帧转发线程 —— 大图序列化/publish 从 GUI 线程整体挪出。

    为什么必须独立线程: RELIABLE 大图 (~1MB) 的 publish() 在写端历史被
    未确认样本占满时会阻塞等待 ACK —— 实测把 GUI 线程的 33ms 定时器拖到
    只剩 ~5Hz (ros2 topic hz 实测 display 4.5Hz / annotations 17Hz),
    画面帧率与解算帧率随之崩塌。转发与界面彻底解耦后:
    - GUI 线程只往单槽里放最新帧引用 (set_frame, 加锁赋值, 不拷贝不序列化)
    - 本线程按固定频率取"最新帧"发布; 上一帧还没发完 (publish 阻塞在
      ACK 上) 就跳过本轮, 新帧继续覆盖单槽 —— 永不排队积压, 永不阻塞
      任何别的线程, 行为等价于"最新帧优先, 慢了就丢帧"
    线程持有独立 node (rclpy 允许每线程一个 node), 无订阅者时不序列化。
    """

    def __init__(self):
        super().__init__(daemon=True)
        self._lock = threading.Lock()
        self._frame = None        # 最新待发帧 (ndarray 引用)
        self._publishing = False  # 上一帧是否仍在发送中 (阻塞时跳帧)
        self.fwd_count = 0        # 2s 统计窗口内的实际发布帧数 (供界面告警)
        self._running = True

    def set_frame(self, arr):
        """GUI 线程调用: 记录最新帧引用 (O(1), 无拷贝)。"""
        with self._lock:
            self._frame = arr

    def take_fwd_count(self):
        """取出并清零统计窗口计数 (界面每 2s 取一次)。"""
        with self._lock:
            count, self.fwd_count = self.fwd_count, 0
        return count

    def stop(self):
        self._running = False

    def _to_msg(self, arr):
        msg = Image()
        msg.height, msg.width = arr.shape[:2]
        msg.encoding = 'bgr8'
        msg.step = int(arr.strides[0])
        msg.header.frame_id = 'display'
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.data = arr.tobytes()
        return msg

    def run(self):
        self._node = rclpy.create_node('ground_station_display')
        # reliable: 大图走 UDP 分片, best-effort 丢一个分片整帧就没了
        # (实测丢帧率可达 40%+); 阻塞代价已由独立线程 + 跳帧机制消化。
        # 若测量发现 ACK 等待仍限制吞吐 (发布端稳定低于源帧率), 改
        # config.DISPLAY_IMAGE_RELIABLE = False 切 best-effort
        qos = QoSProfile(
            reliability=(
                ReliabilityPolicy.RELIABLE if config.DISPLAY_IMAGE_RELIABLE
                else ReliabilityPolicy.BEST_EFFORT),
            history=HistoryPolicy.KEEP_LAST, depth=2)
        pub = self._node.create_publisher(
            Image, config.DISPLAY_IMAGE_TOPIC, qos)
        period = 1.0 / max(config.DISPLAY_IMAGE_HZ, 1)
        try:
            while self._running and rclpy.ok():
                t0 = time.monotonic()
                with self._lock:
                    frame = self._frame
                    self._frame = None
                    busy = self._publishing
                # 无订阅者/上一帧未发完: 跳过, 新帧留在单槽里等下一轮
                if (frame is not None and not busy
                        and pub.get_subscription_count() > 0):
                    self._publishing = True
                    try:
                        pub.publish(self._to_msg(frame))
                        self.fwd_count += 1
                    finally:
                        self._publishing = False
                time.sleep(max(0.0, period - (time.monotonic() - t0)))
        finally:
            self._node.destroy_node()


class GroundStationNode(Node):
    def __init__(self):
        super().__init__('ground_station')

        # 日志分流: get_logger() 返回包装器, 终端照常 + 可挂界面日志框
        self._tee_logger = TeeLogger(super().get_logger())

        # 算法启停指令 (工具按钮按下/抬起时发)
        self.cmd_pub = self.create_publisher(String, config.CMD_TOPIC, 10)

        # 模型输出叠加: 各算法节点往 ANNOTATION_TOPIC 发 JSON 标注
        # (schema 见 overlay.py)。按算法分组存放, 各组独立替换/清屏,
        # 多个算法同时运行互不干扰
        self.anno_groups = {}
        self.create_subscription(
            String, config.ANNOTATION_TOPIC, self._anno_cb, 10)

        # 显示画面转发: 独立线程负责 (大图序列化/publish/等 ACK 全部
        # 移出 GUI 线程, 见 DisplayForwarder 类注释)。界面只往单槽里
        # 放最新帧引用, 无订阅者时转发线程自动跳过不序列化
        self.display_forwarder = DisplayForwarder()
        self.display_forwarder.start()

        # 相机机体系姿态 (ArUco 降落云台泛化): 当前显示相机相对机体的姿态
        # 四元数, 界面线程按 10Hz 调 publish_cam_orientation() 发布
        self.gimbal_pub = self.create_publisher(
            Quaternion, config.GIMBAL_STATE_TOPIC, 30)

        # 算法输出帧率统计 (标注消息到达频率, 供画面 HUD 显示)
        self._anno_count = 0
        self._anno_t = time.monotonic()
        self._anno_last_rx = 0.0
        self.anno_fps = 0.0

        # 降落节点状态回传 (tracking/landing/landed/idle):
        # 记最新状态 + 递增序号, 界面按序号变化取事件 (不漏 "成功" 这类瞬时状态)
        self._landing_status = ''
        self._landing_status_seq = 0
        self.create_subscription(
            String, config.LANDING_STATUS_TOPIC, self._landing_status_cb, 10)

    def get_logger(self):
        """覆盖 Node.get_logger: 返回分流日志器 (终端照常 + 界面日志框)。"""
        return self._tee_logger

    def set_log_sink(self, sink):
        """界面注册日志回调: 每条日志额外回调 sink(level, text)。
        可能从任意线程触发 (如抓取线程), 界面侧须投递到 GUI 线程。"""
        self._tee_logger.set_sink(sink)

    def _landing_status_cb(self, msg):
        self._landing_status = msg.data.strip()
        self._landing_status_seq += 1

    def _anno_cb(self, msg):
        parsed = parse_message(msg.data)
        if parsed is None:
            return
        algo, items = parsed
        self.anno_groups[algo] = items
        # 输出帧率: 每 0.5s 结算一次滑动窗口
        self._anno_count += 1
        now = time.monotonic()
        if now - self._anno_t >= 0.5:
            self.anno_fps = self._anno_count / (now - self._anno_t)
            self._anno_count = 0
            self._anno_t = now
        self._anno_last_rx = now

    # ---------- 供界面调用 ----------

    def get_annotations(self):
        """合并所有算法的标注 (每组独立替换, 无消息的组自然消失)。"""
        out = []
        for items in self.anno_groups.values():
            out.extend(items)
        return out

    def clear_annotations(self):
        """立即清空所有算法标注 (取消算法时界面先清, 不等节点响应)。"""
        self.anno_groups.clear()

    def get_algo_fps(self):
        """算法输出帧率 (标注消息到达频率); 超过 1s 无消息视为 0。"""
        if time.monotonic() - self._anno_last_rx > 1.0:
            return 0.0
        return self.anno_fps

    def get_landing_status(self):
        """返回 (最新降落状态, 事件序号); 序号变化 = 有新状态事件。"""
        return self._landing_status, self._landing_status_seq

    def publish_cmd(self, cmd, quiet=False):
        """发布算法启停等字符串指令; quiet=True 不打日志 (重发指令降噪)。"""
        self.cmd_pub.publish(String(data=cmd))
        if not quiet:
            self.get_logger().info(f'发布指令: {cmd}')

    def publish_cam_orientation(self, q):
        """发布当前显示相机相对机体的姿态四元数 (x, y, z, w, 相机 NED 体约定)。
        q 为 None (非 AirSim 源 / 尚无位姿数据) 时不发布 —— 订阅方超时后
        自动回退刚性下视假设, 无需特殊处理。"""
        if q is None:
            return
        msg = Quaternion()
        msg.x, msg.y, msg.z, msg.w = q
        self.gimbal_pub.publish(msg)

    def set_display_frame(self, arr):
        """把当前显示的帧 (BGR ndarray) 交给转发线程 (O(1) 引用传递)。
        真正的 tobytes/序列化/publish 在 DisplayForwarder 线程完成。"""
        self.display_forwarder.set_frame(arr)

    def take_fwd_count(self):
        """取 2s 统计窗口内的实际转发帧数 (取后清零)。"""
        return self.display_forwarder.take_fwd_count()

    def shutdown(self):
        """停止转发线程 (程序退出时调用, 先于 rclpy.shutdown())。"""
        self.display_forwarder.stop()
