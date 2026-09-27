#!/usr/bin/env python3
"""flylab 地面站 GUI（PyQt5）—— 程序入口

功能：
  1. 订阅多个相机话题，下拉框切换显示（也可手动输入新话题；深度图自动伪彩）
  2. 鼠标拖拽框选目标 -> 画面自动冻结 -> 回车/双击确认发布 /target/bbox [x,y,w,h]，Esc 取消
  3. 可配置的算法启停按钮，发布 std_msgs/String 指令，算法节点自行订阅执行

用法：
  ros2 run flylab_tools ground_station
  ros2 run flylab_tools ground_station --ros-args -p camera_topics:="['/airsim/left_cam/image_raw']"

依赖：sudo apt install python3-pyqt5

模块结构：
  config.py        全局配置（话题、按钮、显示参数，改这里）
  ros_node.py      ROS2 节点（订阅/发布/图像转换）
  video_label.py   视频显示控件（拖拽框选交互）
  main_window.py   主窗口（布局、刷新、深度伪彩、按钮面板）
  ground_station.py 入口（本文件）
"""
import threading

import rclpy
from PyQt5.QtWidgets import QApplication

from .main_window import MainWindow
from .ros_node import GroundStationNode


def main(args=None):
    rclpy.init(args=args)
    node = GroundStationNode()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    app = QApplication([])
    win = MainWindow(node)
    win.show()
    ret = app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(ret)


if __name__ == '__main__':
    main()
