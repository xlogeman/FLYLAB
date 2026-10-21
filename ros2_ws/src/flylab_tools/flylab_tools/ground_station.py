#!/usr/bin/env python3
"""flylab 地面站 GUI（PyQt5）—— 程序入口

单画面 + 工具切换按钮（结构见 main_window.py）：
  画面: 直连 AirSim RPC / 本机摄像头，支持拖拽框选目标
  工具按钮: 目标检测/目标追踪等，按下=启动、再按=停止 (发指令给算法节点)
  模型输出叠加: 模型节点发 JSON 标注到 ANNOTATION_TOPIC，自动画到画面上
              (schema 见 overlay.py，任何模型都能接入)

用法：
  ros2 run flylab_tools ground_station

模块结构：
  config.py        全局配置（按钮、话题、端口，改这里）
  overlay.py       模型输出叠加接口（标注 schema + 解析）
  ros_node.py      ROS2 节点（订阅/发布，界面经它与 ROS 通信）
  video_label.py   视频显示控件（出图/缩放/拖拽框选/标注叠加）
  camera_grabber.py AirSim RPC / 本机摄像头抓取线程（与界面解耦）
  camera_widget.py 相机视图控件（下拉框 + 画面 + 状态栏）
  main_window.py   主窗口（按钮行 + 相机画面 + 叠加轮询）
  ground_station.py 入口（本文件）
"""
import os
import signal

import rclpy
from PyQt5.QtCore import QLibraryInfo
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QApplication

from . import config
from .main_window import MainWindow
from .ros_node import GroundStationNode


def main(args=None):
    rclpy.init(args=args)
    node = GroundStationNode()

    # opencv-python 自带一套 Qt 库, 与 PyQt5 冲突: 强制 Qt 使用 PyQt5 对应的系统插件
    os.environ.setdefault('QT_QPA_PLATFORM_PLUGIN_PATH',
                          QLibraryInfo.location(QLibraryInfo.PluginsPath))

    app = QApplication([])
    # 显式指定含中文字形的字体, 避免无中文字体环境下界面显示为方框
    app.setFont(QFont('Noto Sans CJK SC', 10))

    # QWidget 必须在 QApplication 之后创建
    win = MainWindow(node)
    # 窗口大小按屏幕可用区域的比例设定 (比例见 config.WINDOW_SIZE_RATIO),
    # 画面随窗口等比缩放
    screen = app.primaryScreen().availableGeometry()
    ratio_w, ratio_h = config.WINDOW_SIZE_RATIO
    win.resize(int(screen.width() * ratio_w), int(screen.height() * ratio_h))
    win.show()

    # Ctrl+C 修复: 主线程阻塞在 Qt 事件循环里, 改为收到 SIGINT 直接退出事件循环
    signal.signal(signal.SIGINT, lambda *_: QApplication.instance().quit())

    try:
        ret = app.exec_()
    except KeyboardInterrupt:
        ret = 0
    finally:
        win.shutdown()          # 停抓取线程/释放摄像头, 避免 C++ 栈被硬杀
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    raise SystemExit(ret)


if __name__ == '__main__':
    main()
