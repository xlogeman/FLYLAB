#!/usr/bin/env python3
"""flylab 相机查看器 —— 轻量独立调试工具（单窗口，只有相机视图）

直连 AirSim RPC / 本机 V4L2 摄像头看画面，不依赖任何 ROS 链路。
地面站里同功能的选项卡见 camera_widget.py（共用 camera_grabber.py）。

用法：
  ros2 run flylab_tools camera_view
  ros2 run flylab_tools camera_view --ros-args -p host:=<UE主机IP> -p port:=41451
  host 默认取环境变量 PX4_SIM_HOST_ADDR（跨主机看 UE），否则 127.0.0.1
"""
import os
import signal

import rclpy
from PyQt5.QtCore import QLibraryInfo
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QApplication, QMainWindow

from .camera_widget import CameraViewWidget


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('camera_view')
    host = node.declare_parameter(
        'host', os.environ.get('PX4_SIM_HOST_ADDR', '127.0.0.1')).value
    port = node.declare_parameter('port', 41451).value

    # opencv-python 自带一套 Qt 库, 与 PyQt5 冲突: 强制 Qt 使用 PyQt5 对应的系统插件
    os.environ.setdefault('QT_QPA_PLATFORM_PLUGIN_PATH',
                          QLibraryInfo.location(QLibraryInfo.PluginsPath))

    widget = CameraViewWidget(node.get_logger().info, host, port)
    win = QMainWindow()
    win.setWindowTitle('FLYLAB 相机查看器')
    win.setCentralWidget(widget)

    app = QApplication([])
    # 显式指定含中文字形的字体, 避免无中文字体环境下界面显示为方框
    app.setFont(QFont('Noto Sans CJK SC', 10))
    win.show()

    # Ctrl+C 修复: 主线程阻塞在 Qt 事件循环里, 改为收到 SIGINT 直接退出事件循环
    signal.signal(signal.SIGINT, lambda *_: QApplication.instance().quit())

    try:
        ret = app.exec_()
    except KeyboardInterrupt:
        ret = 0
    finally:
        widget.shutdown()       # 停抓取线程/释放摄像头, 避免 C++ 栈被硬杀
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    raise SystemExit(ret)


if __name__ == '__main__':
    main()
