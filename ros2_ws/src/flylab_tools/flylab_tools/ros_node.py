#!/usr/bin/env python3
"""地面站 ROS2 节点 —— 集中所有 ROS 通信，与界面代码解耦。

界面通过 get_annotations() 轮询取模型标注、调用 publish_* 发指令，
不直接接触任何 rclpy 对象。将来加订阅/发布只改这里。
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String

from . import config
from .overlay import parse_annotations


class GroundStationNode(Node):
    def __init__(self):
        super().__init__('ground_station')

        # 算法启停指令 (工具按钮按下/抬起时发)
        self.cmd_pub = self.create_publisher(String, config.CMD_TOPIC, 10)

        # 框选目标 (画面上拖拽框选确认后发)
        self.bbox_pub = self.create_publisher(
            Int32MultiArray, config.BBOX_TOPIC, 10)

        # 模型输出叠加: 各模型节点往 ANNOTATION_TOPIC 发 JSON 标注列表
        # (schema 见 overlay.py)，界面叠加到画面上
        self.annotations = []
        self.create_subscription(
            String, config.ANNOTATION_TOPIC, self._anno_cb, 10)

    def _anno_cb(self, msg):
        self.annotations = parse_annotations(msg.data)

    # ---------- 供界面调用 ----------

    def get_annotations(self):
        """取最新一组模型标注。"""
        return self.annotations

    def publish_cmd(self, cmd):
        """发布算法启停等字符串指令。"""
        self.cmd_pub.publish(String(data=cmd))
        self.get_logger().info(f'发布指令: {cmd}')

    def publish_bbox(self, x, y, w, h):
        """发布框选目标 (原图坐标)。"""
        self.bbox_pub.publish(Int32MultiArray(data=[int(x), int(y), int(w), int(h)]))
        self.get_logger().info(f'发布目标框: x={x} y={y} w={w} h={h}')
