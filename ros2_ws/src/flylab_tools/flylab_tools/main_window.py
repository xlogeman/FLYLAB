#!/usr/bin/env python3
"""地面站主窗口 —— 左侧画面 + 右侧工具栏 + 模型输出叠加。

布局：画面占主区域, 右栏是输入源选择 + 工具切换按钮 (检测/追踪等) + 状态。
模型输出不画在界面里：模型节点往 config.ANNOTATION_TOPIC 发 JSON 标注
(见 overlay.py)，本窗口轮询并叠加到画面上。
加新工具按钮 = 在 config.TOOLS 里加一项。
"""
import os

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QMainWindow, QPushButton

from . import config
from .camera_widget import CameraViewWidget


class MainWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.setWindowTitle('FLYLAB 地面站')
        self.node = node

        # 输入源视图 (UE 相机/本机摄像头/视频文件), 右栏布局
        host = os.environ.get('PX4_SIM_HOST_ADDR', '127.0.0.1')
        self.camera_tab = CameraViewWidget(
            self.node.get_logger().info, host, config.AIRSIM_RPC_PORT,
            side_panel=True)
        self.camera_tab.video.set_select_enabled(config.SELECT_TARGET)
        self.camera_tab.video.bbox_selected.connect(self._on_bbox_selected)

        # 工具按钮 (按下=启动, 再按=停止), 插到右栏输入源下方
        for tool in config.TOOLS:
            btn = QPushButton(tool['name'])
            btn.setCheckable(True)
            btn.setStyleSheet(self._BTN_STYLE)
            btn.toggled.connect(
                lambda on, t=tool: self.node.publish_cmd(t['start'] if on else t['stop']))
            self.camera_tab.add_side_widget(btn)

        self.setCentralWidget(self.camera_tab)

        # 模型标注叠加: 轮询 ROS 节点缓存的最新标注 (在 ROS 线程更新, 界面线程绘制)
        self.anno_timer = QTimer(self)
        self.anno_timer.timeout.connect(self._poll_annotations)
        self.anno_timer.start(33)

    _BTN_STYLE = """
        QPushButton { min-height: 32px; }
        QPushButton:checked { background-color: #2d8a3e; color: white; }
    """

    def _on_bbox_selected(self, x, y, w, h):
        self.node.publish_bbox(x, y, w, h)

    def _poll_annotations(self):
        self.camera_tab.video.set_annotations(self.node.get_annotations())

    def shutdown(self):
        """优雅退出: 停掉后台线程。"""
        self.camera_tab.shutdown()
