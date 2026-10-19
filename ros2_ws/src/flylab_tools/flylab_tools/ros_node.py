#!/usr/bin/env python3
"""地面站 ROS2 节点 —— 集中所有 ROS 通信，与界面代码解耦。

界面通过 get_annotations() 轮询取模型标注、调用 publish_* 发指令，
不直接接触任何 rclpy 对象。将来加订阅/发布只改这里。
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, String

from . import config
from .overlay import parse_message


class GroundStationNode(Node):
    def __init__(self):
        super().__init__('ground_station')

        # 算法启停指令 (工具按钮按下/抬起时发)
        self.cmd_pub = self.create_publisher(String, config.CMD_TOPIC, 10)

        # 框选目标 (画面上拖拽框选确认后发)
        self.bbox_pub = self.create_publisher(
            Int32MultiArray, config.BBOX_TOPIC, 10)

        # 模型输出叠加: 各算法节点往 ANNOTATION_TOPIC 发 JSON 标注
        # (schema 见 overlay.py)。按算法分组存放, 各组独立替换/清屏,
        # 多个算法同时运行互不干扰
        self.anno_groups = {}
        self.create_subscription(
            String, config.ANNOTATION_TOPIC, self._anno_cb, 10)

        # 显示画面转发: 当前显示的帧发给算法节点 (无订阅者时自动跳过,
        # 不白白占用 CPU 序列化大图)
        self.image_pub = self.create_publisher(
            Image, config.DISPLAY_IMAGE_TOPIC, qos_profile_sensor_data)

    def _anno_cb(self, msg):
        parsed = parse_message(msg.data)
        if parsed is None:
            return
        algo, items = parsed
        self.anno_groups[algo] = items

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

    def publish_cmd(self, cmd):
        """发布算法启停等字符串指令。"""
        self.cmd_pub.publish(String(data=cmd))
        self.get_logger().info(f'发布指令: {cmd}')

    def publish_bbox(self, x, y, w, h):
        """发布框选目标 (原图坐标)。"""
        self.bbox_pub.publish(Int32MultiArray(data=[int(x), int(y), int(w), int(h)]))
        self.get_logger().info(f'发布目标框: x={x} y={y} w={w} h={h}')

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
