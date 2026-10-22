#!/usr/bin/env python3
"""地面站 ROS2 节点 —— 集中所有 ROS 通信，与界面代码解耦。

界面通过 get_annotations() 轮询取模型标注、调用 publish_* 发指令，
不直接接触任何 rclpy 对象。将来加订阅/发布只改这里。
"""
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from rclpy.qos import (HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import Image
from std_msgs.msg import String

from . import config
from .overlay import parse_message


class GroundStationNode(Node):
    def __init__(self):
        super().__init__('ground_station')

        # 算法启停指令 (工具按钮按下/抬起时发)
        self.cmd_pub = self.create_publisher(String, config.CMD_TOPIC, 10)

        # 模型输出叠加: 各算法节点往 ANNOTATION_TOPIC 发 JSON 标注
        # (schema 见 overlay.py)。按算法分组存放, 各组独立替换/清屏,
        # 多个算法同时运行互不干扰
        self.anno_groups = {}
        self.create_subscription(
            String, config.ANNOTATION_TOPIC, self._anno_cb, 10)

        # 显示画面转发: 当前显示的帧发给算法节点 (无订阅者时自动跳过,
        # 不白白占用 CPU 序列化大图)。reliable: 900KB 大图走 UDP 分片,
        # best-effort 下丢一个分片整帧就没了 (实测丢帧率可达 40%+);
        # keep-last 2 保证接收端永远只处理最新帧, 不积压
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=2)
        self.image_pub = self.create_publisher(
            Image, config.DISPLAY_IMAGE_TOPIC, qos)

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

    def publish_frame(self, arr):
        """把当前显示的帧 (BGR ndarray, 原图分辨率) 发布为 sensor_msgs/Image。
        无订阅者时跳过序列化。返回是否真正发布。"""
        if self.image_pub.get_subscription_count() == 0:
            return False
        msg = Image()
        msg.height, msg.width = arr.shape[:2]
        msg.encoding = 'bgr8'
        msg.step = int(arr.strides[0])
        msg.header.frame_id = 'display'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = arr.tobytes()
        self.image_pub.publish(msg)
        return True
