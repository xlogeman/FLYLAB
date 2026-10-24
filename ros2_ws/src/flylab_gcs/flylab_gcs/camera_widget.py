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
import threading

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtWidgets import (QComboBox, QFileDialog, QFrame, QGridLayout,
                           QGroupBox, QHBoxLayout, QLabel, QPushButton,
                           QSlider, QVBoxLayout, QWidget)

from . import config
from .camera_grabber import (DEFAULT_CAMERAS, FILE_PREFIX, LOCAL_PREFIX,
                           Grabber, colorize_depth, list_local_cameras)
from .video_label import VideoLabel

FILE_OPEN_ITEM = '打开视频文件...'
VIDEO_FILTER = '视频 (*.mp4 *.avi *.mkv *.mov *.webm);;所有文件 (*)'


class _SourceCombo(QComboBox):
    """输入源下拉框: 展开时触发重扫本机摄像头 (运行中热插拔即时可见)。"""
    popup_shown = pyqtSignal()
    cams_found = pyqtSignal(list)

    def showPopup(self):
        self.popup_shown.emit()
        super().showPopup()


class CameraViewWidget(QWidget):
    def __init__(self, log, host, port, side_panel=False, parent=None):
        super().__init__(parent)
        self.log = log
        self.grabber = Grabber(log, host, port)
        self.grabber.start()
        self._file_map = {}    # 下拉框显示名 -> 视频文件完整路径

        # 输入源下拉框 (展开时重扫本机摄像头, 支持启动后插入设备)
        self._scanning = False
        self.combo = _SourceCombo()          # 只读下拉框, 不允许键盘输入
        self.combo.addItems(
            list(DEFAULT_CAMERAS)
            + [f'{LOCAL_PREFIX}{d}' for d in list_local_cameras()]
            + [FILE_OPEN_ITEM])
        self.combo.popup_shown.connect(self._refresh_local_cameras)
        self.combo.cams_found.connect(self._add_local_cameras)
        self.combo.activated.connect(self._on_source_changed)

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
        self._build_gimbal(in_group=side_panel)

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
            # 输入源 / 云台 各自成框 (分组标题已说明用途, 框内不再重复标题)
            src_box = QGroupBox('输入源')
            src_lay = QVBoxLayout(src_box)
            src_lay.setContentsMargins(6, 4, 6, 6)
            src_lay.addWidget(self.combo)
            panel.addWidget(src_box)
            self._panel = panel
            self._panel_stretch_index = panel.count()
            gimbal_box = QGroupBox('云台')
            gim_lay = QVBoxLayout(gimbal_box)
            gim_lay.setContentsMargins(6, 4, 6, 6)
            gim_lay.addWidget(self.gimbal)
            panel.addWidget(gimbal_box)
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

    def _build_gimbal(self, in_group=False):
        """云台控制面板: 方向键步进 + 回中 + 角度读数。
        仅 AirSim 相机源可用 (本机/文件源在 _refresh 里置灰)。
        in_group=True 时外面已有分组框, 面板内不再重复"云台:"标题。"""
        step = config.GIMBAL_STEP_DEG
        self._gimbal_btns = {}   # 方向键 -> 按钮 (到限位标红), 见 _update_gimbal_limit
        self._btn_styles = {}    # 按钮当前样式缓存, 避免每帧重复 setStyleSheet

        def dir_btn(key, text, tip, dyaw, dpitch):
            b = QPushButton(text)
            b.setToolTip(f'{tip} (可按住连发)')
            b.setAutoRepeat(True)          # 按住连续步进, 模拟连续转动
            b.setAutoRepeatDelay(300)
            b.setAutoRepeatInterval(100)
            b.clicked.connect(
                lambda: self.grabber.gimbal_step(dyaw, dpitch))
            self._gimbal_btns[key] = b
            return b

        self.gimbal = QWidget()
        g = QGridLayout(self.gimbal)
        g.setContentsMargins(0, 0, 0, 0)
        limit_tip = (f'限位: 航向 ±{config.GIMBAL_YAW_LIMIT_DEG:.0f}°, '
                     f'俯仰 {config.GIMBAL_PITCH_MIN_DEG:.0f}°'
                     f'~+{config.GIMBAL_PITCH_MAX_DEG:.0f}° (到限位该方向键标红)')
        title = QLabel('云台:')
        self.gimbal_angle = QLabel('航向 +0° 俯仰 +0°')
        self.gimbal_angle.setStyleSheet(
            'color: #444; font-family: monospace;')
        self.gimbal_angle.setToolTip(limit_tip)
        if not in_group:
            g.addWidget(title, 0, 0, 1, 3)
        g.addWidget(dir_btn('pitch+', '↑', f'俯仰上 {step}°', 0, +step), 1, 1)
        g.addWidget(dir_btn('yaw-', '←', f'偏航左 {step}°', -step, 0), 2, 0)
        btn_center = QPushButton('回中')
        btn_center.setToolTip('云台回中 (平滑转动到挂载原始姿态, 按方向键可打断)')
        btn_center.clicked.connect(self.grabber.gimbal_reset)
        g.addWidget(btn_center, 2, 1)
        g.addWidget(dir_btn('yaw+', '→', f'偏航右 {step}°', +step, 0), 2, 2)
        g.addWidget(dir_btn('pitch-', '↓', f'俯仰下 {step}°', 0, -step), 3, 1)
        g.addWidget(self.gimbal_angle, 4, 0, 1, 3)

    def _update_gimbal_limit(self):
        """到限位提示: 按键被限位拒绝 (再按不再动作) 的方向键标红, 该方向
        恢复可动即清红 (不禁用按钮, 保持按住回转的手感)。拒绝状态由
        grabber 在会话内判定, 打开地面站时一律不红, 不沿用上次会话停在哪;
        样式只在变化时更新。"""
        blocked = self.grabber.get_gimbal_blocked()
        for key, btn in self._gimbal_btns.items():
            style = 'color: #d9534f; font-weight: bold;' if blocked[key] else ''
            if self._btn_styles.get(key) != style:
                btn.setStyleSheet(style)
                self._btn_styles[key] = style

    def _refresh_local_cameras(self):
        """下拉展开时后台重扫 /dev/video*，新插入的摄像头即时出现在列表里。
        本地探测不经 AirSim，没开 UE 也能选；WSL2 下摄像头需先 usbipd attach。"""
        if self._scanning:
            return
        self._scanning = True

        def scan():
            cams = list_local_cameras()
            self._scanning = False
            if cams:
                self.combo.cams_found.emit(cams)

        threading.Thread(target=scan, daemon=True).start()

    def _add_local_cameras(self, devs):
        """把新发现的设备加进下拉框 (信号在 GUI 线程触发, 线程安全)。"""
        for dev in devs:
            item = f'{LOCAL_PREFIX}{dev}'
            if self.combo.findText(item) < 0:
                self.combo.addItem(item)

    def _switch_source(self, name):
        """切换抓取源并立即清屏: 新源暂时无帧 (设备不存在/UE 相机未配置/
        文件打不开等) 时, 界面回到"等待图像..."占位, 而不是残留上一源的
        定格画面 (看起来像图传卡死)。"""
        self.video.clear_frame()
        self.grabber.set_camera(name)

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
            self._switch_source(f'{FILE_PREFIX}{path}')
            return
        if text.startswith(FILE_PREFIX):     # 之前加过的视频文件条目
            path = self._file_map.get(text, text[len(FILE_PREFIX):])
            self._switch_source(f'{FILE_PREFIX}{path}')
            return
        self._switch_source(text)            # AirSim 相机 / 本机摄像头

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
        # 仅 AirSim 源且 RPC 已连通可用: 打开地面站/UE 未 play 时默认置灰
        self.gimbal.setEnabled(
            gim is not None and self.grabber.airsim_connected())
        if gim is not None:
            yaw, pitch = gim
            self.gimbal_angle.setText(f'航向 {yaw:+.0f}° 俯仰 {pitch:+.0f}°')
            self._update_gimbal_limit()
        frame, src, fps = self.grabber.get()
        if frame is None:
            # 无信号提示直接写在画面中央 (带源名), 右栏状态条不再重复
            self.video.set_waiting(f'[{src}] 等待图像...')
            self.status.setText('')
            return
        arr, is_depth = frame
        if is_depth:
            arr = colorize_depth(arr)
        self.video.show_frame(arr)
        self.video.set_hud(f'图传 {fps:.1f}')   # 帧率显示在画面左上角
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
