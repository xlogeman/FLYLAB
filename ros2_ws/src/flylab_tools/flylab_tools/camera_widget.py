#!/usr/bin/env python3
"""输入源视图控件 —— 三类输入源统一到一个下拉框：

  - UE 相机: AirSim RPC (left_cam / front_cam / depth_cam ...)
  - 本机摄像头: local:/dev/videoX
  - 视频文件: 下拉选"打开视频文件..."弹出文件对话框 (mp4 等, 循环播放)

布局两种模式：
默认上下排列 (camera_view 独立窗口用)
side_panel=True 左右排列: 画面在左, 输入源/状态在右, 外部可用
add_side_widget() 往右栏插工具按钮 (ground_station 用)

云台面板: AirSim 相机源可用, 方向键步进旋转视角 (按住连发) + 回中,
经 simSetCameraPose 实现, 仅影响画面不影响飞机姿态。
"""
import os

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (QComboBox, QFileDialog, QFrame, QGridLayout,
                           QHBoxLayout, QLabel, QPushButton, QSlider,
                           QVBoxLayout, QWidget)

from . import config
from .camera_grabber import (DEFAULT_CAMERAS, FILE_PREFIX, LOCAL_PREFIX,
                           Grabber, colorize_depth, list_local_cameras)
from .video_label import VideoLabel

FILE_OPEN_ITEM = '打开视频文件...'
VIDEO_FILTER = '视频 (*.mp4 *.avi *.mkv *.mov *.webm);;所有文件 (*)'


class CameraViewWidget(QWidget):
    def __init__(self, log, host, port, side_panel=False, parent=None):
        super().__init__(parent)
        self.log = log
        self.grabber = Grabber(log, host, port)
        self.grabber.start()
        self._file_map = {}    # 下拉框显示名 -> 视频文件完整路径

        # 输入源下拉框
        self.combo = QComboBox()
        self.combo.setEditable(True)
        self.combo.addItems(
            list(DEFAULT_CAMERAS)
            + [f'{LOCAL_PREFIX}{d}' for d in list_local_cameras()]
            + [FILE_OPEN_ITEM])
        self.combo.activated.connect(self._on_source_changed)
        self.combo.lineEdit().returnPressed.connect(self._on_source_changed)

        self.video = VideoLabel()
        self.status = QLabel('未连接')
        self.status.setStyleSheet('color: #666; padding: 2px;')
        self.status.setWordWrap(True)

        # 视频文件播放控制条: 横排在画面下方 (播放/暂停 | 停止 | 进度条 | 时长)。
        # 非文件源自动置灰。
        self._scrub = False      # 用户正在拖动进度条时, 定时器不回写进度
        self.btn_play = QPushButton('暂停')
        self.btn_play.setFixedWidth(56)
        self.btn_play.setToolTip('播放 / 暂停 (空格)')
        self.btn_play.clicked.connect(self._on_play_clicked)
        self.btn_stop = QPushButton('停止')
        self.btn_stop.setFixedWidth(56)
        self.btn_stop.setToolTip('停止并回到开头')
        self.btn_stop.clicked.connect(self.grabber.stop_file)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, 1000)
        self.slider.sliderPressed.connect(lambda: setattr(self, '_scrub', True))
        self.slider.sliderMoved.connect(
            lambda v: self.grabber.set_file_position(v / 1000))
        self.slider.sliderReleased.connect(self._on_slider_released)
        self.time_label = QLabel('00:00 / 00:00')
        self.time_label.setStyleSheet(
            'color: #444; font-family: monospace;')
        self.time_label.setMinimumWidth(96)
        self.time_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.file_ctrl = QWidget()
        ctrl_layout = QHBoxLayout(self.file_ctrl)
        ctrl_layout.setContentsMargins(0, 0, 0, 0)
        ctrl_layout.addWidget(self.btn_play)
        ctrl_layout.addWidget(self.btn_stop)
        ctrl_layout.addWidget(self.slider, stretch=1)
        ctrl_layout.addWidget(self.time_label)
        self.file_ctrl.setEnabled(False)
        self._build_gimbal()

        src_label = QLabel('输入源:')
        if side_panel:                       # 左画面+控制条 / 右栏 (地面站)
            outer = QHBoxLayout(self)
            left = QVBoxLayout()
            left.addWidget(self.video, stretch=1)
            left.addWidget(self.file_ctrl)
            outer.addLayout(left, stretch=1)
            line = QFrame()                  # 画面区与功能区之间的竖分隔线
            line.setFrameShape(QFrame.VLine)
            line.setFrameShadow(QFrame.Sunken)
            outer.addWidget(line)
            panel = QVBoxLayout()
            outer.addLayout(panel)
            panel.addWidget(src_label)
            panel.addWidget(self.combo)
            self._panel = panel
            self._panel_stretch_index = panel.count()
            panel.addWidget(self.gimbal)
            self._panel_stretch_index += 1
            panel.addStretch(1)
            panel.addWidget(self.status)
        else:                                # 上下排列 (独立调试窗口)
            layout = QVBoxLayout(self)
            row = QHBoxLayout()
            row.addWidget(src_label)
            row.addWidget(self.combo, stretch=1)
            layout.addLayout(row)
            layout.addWidget(self.video)
            layout.addWidget(self.file_ctrl)
            layout.addWidget(self.gimbal)
            layout.addWidget(self.status)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(33)

    def add_side_widget(self, w):
        """往右侧栏插一个控件 (位于弹性空白之前)。仅 side_panel 模式有效。"""
        if hasattr(self, '_panel'):
            self._panel.insertWidget(self._panel_stretch_index, w)
            self._panel_stretch_index += 1

    def _build_gimbal(self):
        """云台控制面板: 方向键步进 + 回中 + 角度读数。
        仅 AirSim 相机源可用 (本机/文件源在 _refresh 里置灰)。"""
        step = config.GIMBAL_STEP_DEG

        def dir_btn(text, tip, dyaw, dpitch):
            b = QPushButton(text)
            b.setToolTip(f'{tip} (可按住连发)')
            b.setAutoRepeat(True)          # 按住连续步进, 模拟连续转动
            b.setAutoRepeatDelay(300)
            b.setAutoRepeatInterval(100)
            b.clicked.connect(
                lambda: self.grabber.gimbal_step(dyaw, dpitch))
            return b

        self.gimbal = QWidget()
        g = QGridLayout(self.gimbal)
        g.setContentsMargins(0, 0, 0, 0)
        title = QLabel('云台:')
        self.gimbal_angle = QLabel('航向 +0° 俯仰 +0°')
        self.gimbal_angle.setStyleSheet(
            'color: #444; font-family: monospace;')
        g.addWidget(title, 0, 0, 1, 3)
        g.addWidget(dir_btn('↑', f'俯仰上 {step}°', 0, +step), 1, 1)
        g.addWidget(dir_btn('←', f'偏航左 {step}°', -step, 0), 2, 0)
        btn_center = QPushButton('回中')
        btn_center.setToolTip('云台回中 (恢复挂载原始姿态)')
        btn_center.clicked.connect(self.grabber.gimbal_reset)
        g.addWidget(btn_center, 2, 1)
        g.addWidget(dir_btn('→', f'偏航右 {step}°', +step, 0), 2, 2)
        g.addWidget(dir_btn('↓', f'俯仰下 {step}°', 0, -step), 3, 1)
        g.addWidget(self.gimbal_angle, 4, 0, 1, 3)

    def _on_source_changed(self):
        text = self.combo.currentText().strip()
        if not text:
            return
        if text == FILE_OPEN_ITEM:           # 弹出文件选择对话框
            path, _ = QFileDialog.getOpenFileName(
                self, '选择视频文件', os.path.expanduser('~'), VIDEO_FILTER)
            if not path:
                self.combo.setCurrentIndex(0)    # 取消则回到第一个源
                return
            label = f'{FILE_PREFIX}{os.path.basename(path)}'
            self._file_map[label] = path
            if self.combo.findText(label) < 0:
                self.combo.addItem(label)
            self.combo.setCurrentText(label)
            self.grabber.set_camera(f'{FILE_PREFIX}{path}')
            return
        if text.startswith(FILE_PREFIX):     # 之前加过的视频文件条目
            path = self._file_map.get(text, text[len(FILE_PREFIX):])
            self.grabber.set_camera(f'{FILE_PREFIX}{path}')
            return
        self.grabber.set_camera(text)        # AirSim 相机 / 本机摄像头

    def _on_play_clicked(self):
        paused = not self.grabber.file_paused
        self.grabber.set_file_paused(paused)
        self.btn_play.setText('播放' if paused else '暂停')

    def keyPressEvent(self, ev):          # 空格 = 播放/暂停 (仅文件源)
        if ev.key() == Qt.Key_Space and self.file_ctrl.isEnabled():
            self._on_play_clicked()
        else:
            super().keyPressEvent(ev)

    def _on_slider_released(self):
        self._scrub = False
        self.grabber.set_file_position(self.slider.value() / 1000)

    @staticmethod
    def _fmt_time(sec):
        sec = max(0, int(round(sec)))
        return f'{sec // 60:02d}:{sec % 60:02d}'

    def _refresh(self):
        self._refresh_playback()
        gim = self.grabber.get_gimbal()
        self.gimbal.setEnabled(gim is not None)   # 仅 AirSim 源可用
        if gim is not None:
            yaw, pitch = gim
            self.gimbal_angle.setText(f'航向 {yaw:+.0f}° 俯仰 {pitch:+.0f}°')
        frame, src, fps = self.grabber.get()
        if frame is None:
            self.status.setText(f'[{src}] 等待图像...')
            self.video.set_hud(None)
            return
        arr, is_depth = frame
        if is_depth:
            arr = colorize_depth(arr)
        self.video.show_frame(arr)
        self.video.set_hud(f'{fps:.1f} fps')   # 帧率显示在画面左上角
        self.status.setText('')                # 源/分辨率不再展示

    def _refresh_playback(self):
        """播放控件状态: 仅文件源启用, 进度条/时长随播放前进 (拖动中不回写)。"""
        prog = self.grabber.get_progress()
        self.file_ctrl.setEnabled(prog is not None)
        if prog is None:
            return
        pos, total, paused, file_fps = prog
        self.btn_play.setText('播放' if paused else '暂停')
        self.time_label.setText(
            f'{self._fmt_time(pos / file_fps)} / {self._fmt_time(total / file_fps)}')
        if not self._scrub and total > 0:
            self.slider.blockSignals(True)
            self.slider.setValue(int(pos / total * 1000))
            self.slider.blockSignals(False)

    def shutdown(self):
        """停止抓取线程并释放设备 (退出前调用)。"""
        self.grabber.stop()
        self.grabber.join(timeout=2.0)
        self.grabber.release_local()
