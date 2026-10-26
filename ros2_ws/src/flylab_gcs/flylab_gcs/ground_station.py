#!/usr/bin/env python3
"""flylab 地面站 GUI（PyQt5）—— 程序入口

用法：
  ros2 run flylab_gcs ground_station

界面结构 / 模块清单见 main_window.py，全局配置见 config.py，
标注叠加 schema 见 overlay.py。
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

    # Ctrl+C / kill 修复: 主线程阻塞在 Qt 事件循环里, 收到 SIGINT/SIGTERM
    # 统一优雅退出事件循环 → finally 回收全部托管进程 (不留孤儿链路)
    signal.signal(signal.SIGINT, lambda *_: QApplication.instance().quit())
    signal.signal(signal.SIGTERM, lambda *_: QApplication.instance().quit())

    try:
        ret = app.exec_()
    except KeyboardInterrupt:
        ret = 0
    finally:
        win.shutdown()          # 停抓取线程/释放摄像头, 避免 C++ 栈被硬杀
        node.shutdown()         # 停显示帧转发线程 (先于 rclpy.shutdown)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    raise SystemExit(ret)


if __name__ == '__main__':
    main()
