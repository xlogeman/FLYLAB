#!/usr/bin/env python3
"""地面站 ROS2 节点 —— 集中所有 ROS 通信，与界面代码解耦。

界面通过 get_annotations() 轮询取模型标注、调用 publish_* 发指令，
不直接接触任何 rclpy 对象。将来加订阅/发布只改这里。
"""
import json
import threading
import time

import rclpy
from geometry_msgs.msg import Quaternion, Twist
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String

from . import config
from .overlay import parse_message


class TeeLogger:
    """rclpy Logger 分流器: 终端照常输出, 同时回调 (级别, 文本) 给界面日志框。
    回调可能来自任意线程, 线程安全由回调方保证。

    注意: rclpy 按"调用点代码行"记忆 severity, 同一行发不同级别会抛异常,
    因此各级别走独立调用点 (不用 getattr 转发)。
    """

    def __init__(self, ros_logger):
        self._ros = ros_logger
        self._sink = None

    def set_sink(self, sink):
        self._sink = sink

    def _tee(self, level, msg, *args, **kwargs):
        if self._sink is not None:
            try:
                self._sink(level.upper(), str(msg))
            except Exception:
                pass    # 界面日志框异常不拖累 ROS 日志与主流程

    def debug(self, msg, *a, **kw):
        try:
            return self._ros.debug(msg, *a, **kw)
        except Exception:
            pass    # 日志异常永不炸调用方线程 (飞控/抓取线程曾因此崩溃)
        self._tee('DEBUG', msg)

    def info(self, msg, *a, **kw):
        try:
            return self._ros.info(msg, *a, **kw)
        except Exception:
            pass
        self._tee('INFO', msg)

    def warning(self, msg, *a, **kw):
        try:
            return self._ros.warning(msg, *a, **kw)
        except Exception:
            pass
        self._tee('WARNING', msg)

    def error(self, msg, *a, **kw):
        try:
            return self._ros.error(msg, *a, **kw)
        except Exception:
            pass
        self._tee('ERROR', msg)

    def fatal(self, msg, *a, **kw):
        try:
            return self._ros.fatal(msg, *a, **kw)
        except Exception:
            pass
        self._tee('FATAL', msg)


class DisplayForwarder(threading.Thread):
    """显示帧转发线程 —— 大图序列化/publish 从 GUI 线程整体挪出。

    RELIABLE 大图 publish() 可能阻塞等 ACK (实测曾把 GUI 定时器拖到 ~5Hz),
    故独立成线程: GUI 线程只往单槽放最新帧引用, 本线程按固定频率发布;
    上一帧没发完就跳过本轮 —— 最新帧优先, 慢了就丢帧, 永不积压。
    线程持有独立 node, 无订阅者时不序列化。
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
        # reliable: best-effort 下大图 UDP 分片丢一个整帧就没 (实测丢帧 40%+);
        # 若 ACK 等待限制吞吐, 改 config.DISPLAY_IMAGE_RELIABLE = False
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
                    except Exception:
                        # 退出竞态: shutdown 作废上下文时正好在 publish, 静默吞掉
                        pass
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

        # 模型输出叠加: 各算法节点发 JSON 标注, 按算法分组存放互不干扰
        self.anno_groups = {}
        self.create_subscription(
            String, config.ANNOTATION_TOPIC, self._anno_cb, 10)

        # 显示画面转发: 独立线程负责, 界面只放最新帧引用 (见 DisplayForwarder)
        self.display_forwarder = DisplayForwarder()
        self.display_forwarder.start()

        # 相机机体系姿态 (ArUco 降落云台泛化): 当前显示相机相对机体的姿态
        # 四元数, 界面线程按 10Hz 调 publish_cam_orientation() 发布
        self.gimbal_pub = self.create_publisher(
            Quaternion, config.GIMBAL_STATE_TOPIC, 30)

        # 视场角同步发布器 (latched 约定见 config.CAMERA_FOV_TOPIC)
        self.fov_pub = self.create_publisher(
            Float64, config.CAMERA_FOV_TOPIC,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST, depth=1))

        # ArUco 码边长同步发布器 (latched 约定见 config.ARUCO_MARKER_LENGTH_TOPIC)
        self.marker_len_pub = self.create_publisher(
            Float64, config.ARUCO_MARKER_LENGTH_TOPIC,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST, depth=1))

        # 算法输出帧率统计 (标注消息到达频率, 供画面 HUD 显示)
        self._anno_count = 0
        self._anno_t = time.monotonic()
        self._anno_last_rx = 0.0
        self.anno_fps = 0.0

        # 降落节点状态回传: 记最新状态 + 递增序号, 界面按序号变化取事件
        # (不漏 "成功" 这类瞬时状态)
        self._landing_status = ''
        self._landing_status_seq = 0
        self.create_subscription(
            String, config.LANDING_STATUS_TOPIC, self._landing_status_cb, 10)

        # 算法输出面板数据 (右栏): JSON 按 src 分组 (detect/landing),
        # 各存最新一条, 界面轮询 get_aruco_info() 取用
        self.aruco_info = {}
        self.create_subscription(
            String, config.ARUCO_INFO_TOPIC, self._aruco_info_cb, 10)

        # 手动遥控: 归一化遥控量发布 (双摇杆, -1..1) + 遥控节点状态回传
        self.teleop_cmd_pub = self.create_publisher(
            Twist, config.TELEOP_CMD_TOPIC, 10)
        self._teleop_status = ''
        self._teleop_status_seq = 0
        self.create_subscription(
            String, config.TELEOP_STATUS_TOPIC, self._teleop_status_cb, 10)

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

    def _teleop_status_cb(self, msg):
        self._teleop_status = msg.data.strip()
        self._teleop_status_seq += 1

    def _aruco_info_cb(self, msg):
        """算法输出面板数据: JSON 解析失败静默 (节点端保证 schema)。"""
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        src = data.get('src')
        if src:
            self.aruco_info[src] = {'data': data, 't': time.monotonic()}

    def get_aruco_info(self):
        """各来源 (src) 最新一条面板数据 {src: {data, t}}。"""
        return self.aruco_info

    def publish_teleop_twist(self, fwd, right, up, yaw_left):
        """发布归一化遥控量 (-1..1): fwd 前+ / right 右+ / up 升+ / yaw_left 左转+。
        teleop_node 侧按各自速度上限换算成机体系速度 + 偏航角速率。"""
        t = Twist()
        t.linear.x = float(fwd)
        t.linear.y = float(right)
        t.linear.z = float(up)
        t.angular.z = float(yaw_left)
        self.teleop_cmd_pub.publish(t)

    def get_teleop_status(self):
        """返回 (最新遥控状态, 事件序号); 序号变化 = 有新状态事件。"""
        return self._teleop_status, self._teleop_status_seq

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

    def publish_camera_fov(self, deg):
        """发布当前相机视场角 (度) 供算法节点同步估算内参。
        deg 为 None (尚未连上 AirSim / 非 AirSim 源) 时不发布 —— 连上后
        抓取线程已采纳相机真实 FOV, 节点启动即能同步。"""
        if deg is None:
            return
        msg = Float64()
        msg.data = float(deg)
        self.fov_pub.publish(msg)

    def publish_marker_length(self, length_m):
        """发布 ArUco 码边长 (米) 供 aruco_node / aruco_landing_node
        运行中同步 marker_length (位姿解算尺度)。"""
        msg = Float64()
        msg.data = float(length_m)
        self.marker_len_pub.publish(msg)

    def set_display_frame(self, arr):
        """把当前显示的帧 (BGR ndarray) 交给转发线程 (O(1) 引用传递)。
        真正的 tobytes/序列化/publish 在 DisplayForwarder 线程完成。"""
        self.display_forwarder.set_frame(arr)

    def take_fwd_count(self):
        """取 2s 统计窗口内的实际转发帧数 (取后清零)。"""
        return self.display_forwarder.take_fwd_count()

    def shutdown(self):
        """停止转发线程 (先于 rclpy.shutdown(), 等线程退出防止退出栈 RCLError)。"""
        forwarder = self.display_forwarder
        forwarder.stop()
        forwarder.join(timeout=1.0)
