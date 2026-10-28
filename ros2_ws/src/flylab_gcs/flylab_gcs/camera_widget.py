#!/usr/bin/env python3
"""输入源视图控件 —— 三类输入源统一到一个工具按钮：

  - UE 相机: AirSim RPC (left_cam / front_cam / depth_cam ...)
  - 本机摄像头: local:/dev/videoX
  - 视频文件: 菜单选"打开视频文件..."弹出文件对话框 (mp4 等, 循环播放)

源选择是 QToolButton (弹出菜单), 由主窗口放进顶部 QToolBar:
地面站把它当工具栏第一项, 独立调试窗口仍在画面上方横排。
后续新工具按钮直接往 QToolBar 右侧加即可。

布局两种模式：
默认上下排列 (camera_view 独立窗口用)
side_panel=True 左右排列: 画面在左, 云台/状态在右, 外部可用
add_side_widget() 往右栏插工具按钮 (ground_station 用)

云台面板: AirSim 相机源可用, 摇杆连续转动视角 (偏转量=角速度) + 回中,
经 simSetCameraPose 实现, 仅影响画面不影响飞机姿态。
"""
import os
import threading

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtWidgets import (QFileDialog, QFrame, QGridLayout,
                           QGroupBox, QHBoxLayout, QLabel, QMenu,
                           QPushButton, QScrollArea, QSlider, QToolButton,
                           QVBoxLayout, QWidget)

from . import config
from .camera_grabber import (DEFAULT_CAMERA, DEFAULT_CAMERAS, FILE_PREFIX,
                           LOCAL_PREFIX, Grabber, colorize_depth,
                           list_local_cameras)
from .teleop_pad import VirtualJoystick
from .video_label import VideoLabel

FILE_OPEN_ITEM = '打开视频文件...'
VIDEO_FILTER = '视频 (*.mp4 *.avi *.mkv *.mov *.webm);;所有文件 (*)'


class CameraViewWidget(QWidget):
    # 本机摄像头扫描完成 (菜单在展开状态下也能即时补上新设备)
    cams_found = pyqtSignal()

    def __init__(self, log, host, port, side_panel=False, parent=None):
        super().__init__(parent)
        self.log = log
        self.grabber = Grabber(log, host, port)
        self.grabber.start()
        self._file_map = {}    # 显示名 -> 视频文件完整路径
        self._scanning = False

        # 输入源工具按钮 (VS Code 风格工具栏第一项): 点击弹出源菜单,
        # 按钮文字跟随当前源。菜单展开时后台重扫本机摄像头 (热插拔可见)
        self._local_cams = list_local_cameras()
        self.src_btn = QToolButton()
        self.src_btn.setText(f'视频源: {DEFAULT_CAMERA}')
        self.src_btn.setPopupMode(QToolButton.InstantPopup)
        self.src_btn.setToolTip(
            '选择视频输入源: UE 相机 / 本机摄像头 / 视频文件')
        src_menu = QMenu(self.src_btn)
        self.src_btn.setMenu(src_menu)
        src_menu.aboutToShow.connect(self._rebuild_src_menu)
        src_menu.triggered.connect(self._on_src_action)
        self.cams_found.connect(self._rebuild_src_menu)

        self.video = VideoLabel()
        # 数字变焦: 滚轮放大缩小 / 双击复位 (变焦在抓取层裁切, 见 grabber)
        self.video.zoom_step.connect(self._on_zoom_step)
        self.video.zoom_reset.connect(
            lambda: self.grabber.set_zoom(1.0))
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
            # 源选择按钮由主窗口放进顶部工具栏, 这里只排画面/右栏
            outer = QHBoxLayout(self)
            left = QVBoxLayout()
            self._left_layout = left   # 供 add_bottom_widget 往画面下方插控件
            left.addWidget(self.video, stretch=1)
            left.addWidget(self.file_ctrl)
            # 宽度比例 7:3: 左画面 (含日志框) 占大头, 右功能区窄栏
            outer.addLayout(left, stretch=7)
            line = QFrame()                  # 画面区与功能区之间的竖分隔线
            line.setFrameShape(QFrame.VLine)
            line.setFrameShadow(QFrame.Sunken)
            outer.addWidget(line)
            # 右栏套 QScrollArea: 窗口拉小时出滚动条, 避免组件互相挤压
            body = QWidget()
            panel = QVBoxLayout(body)
            panel.setContentsMargins(0, 0, 0, 0)
            scroll = QScrollArea()
            scroll.setWidgetResizable(True)   # body 随视口伸缩, 宽时组件撑满
            scroll.setFrameShape(QFrame.NoFrame)
            scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
            scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
            scroll.setWidget(body)
            outer.addWidget(scroll, stretch=3)
            # 云台分组框不直接入栏: 主窗口单独往右栏插入
            gimbal_box = QGroupBox('云台')
            gim_lay = QVBoxLayout(gimbal_box)
            gim_lay.setContentsMargins(6, 4, 6, 6)
            gim_lay.addWidget(self.gimbal)
            self.gimbal_box = gimbal_box
            self._panel = panel
            self._panel_stretch_index = panel.count()
            panel.addStretch(1)
            panel.addWidget(self.status)
        else:                                # 上下排列 (独立调试窗口)
            layout = QVBoxLayout(self)
            row = QHBoxLayout()
            row.addWidget(src_label)
            row.addWidget(self.src_btn)
            row.addStretch(1)
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

    def add_bottom_widget(self, w):
        """往画面下方插一个控件 (与画面同宽)。仅 side_panel 模式有效。"""
        if hasattr(self, '_left_layout'):
            self._left_layout.addWidget(w)

    def _build_gimbal(self, in_group=False):
        """云台控制面板: 摇杆连续转动 (偏转量=角速度) + 回中 + 角度读数
        (限位卡住标红)。仅 AirSim 相机源可用。in_group=True 时不再重复标题。"""
        self._gimbal_locked = False   # 视觉反控进行中锁定云台 (见 set_gimbal_locked)
        self._angle_red = {'yaw': False, 'pitch': False}   # 各轴读数标红缓存

        self._joy = VirtualJoystick()
        self._joy.setToolTip(
            '拖动转动云台: 方向=偏转方向, 偏离中心越远转得越快, 松手停')
        self._joy.changed.connect(self._on_gimbal_joy)

        btn_center = QPushButton('回中')
        btn_center.setFixedWidth(40)
        btn_center.setToolTip('云台回中 (平滑转动到挂载原始姿态, 摇杆可打断)')
        btn_center.clicked.connect(self.grabber.gimbal_reset)

        btn_stab = QPushButton('稳定')
        btn_stab.setFixedWidth(40)
        btn_stab.setCheckable(True)
        btn_stab.setChecked(self.grabber.get_gimbal_stab())
        btn_stab.setStyleSheet(
            'QPushButton:checked { background-color: #2d8a3e; color: white; }')
        btn_stab.setToolTip(
            '云台稳定 (follow 模式): 开启后持续按世界系重下发视角, '
            '俯仰/横滚不随机体晃动, 偏航跟随机头; 关闭则相机刚性随机体 '
            '(FPV 手感)')
        btn_stab.toggled.connect(self.grabber.set_gimbal_stab)

        btn_follow = QPushButton('跟随')
        btn_follow.setFixedWidth(40)
        btn_follow.setCheckable(True)
        btn_follow.setEnabled(False)   # 可用性由接管控制门控刷新 (遥控接管中)
        btn_follow.setStyleSheet(
            'QPushButton:checked { background-color: #2d8a3e; color: white; }')
        btn_follow.setToolTip(
            '航向跟随: 转动云台偏航时机头跟着转向视线方向, 追上后云台偏航'
            '自动回中 (限位变为最大超前角)。需先开启通信链路并接管手动遥控'
            ' (指令走遥控 offboard 通道); 反控降落中不可用')
        btn_follow.toggled.connect(self._on_follow_toggle)
        self.btn_follow = btn_follow
        self._follow_avail = False

        self.gimbal = QWidget()
        g = QGridLayout(self.gimbal)
        g.setContentsMargins(0, 0, 0, 0)
        title = QLabel('云台:')
        # 航向/俯仰读数独立成标签: 各轴被限位卡住时只红自己的读数
        self._gimbal_labels = {}
        for axis, name in (('yaw', '航向'), ('pitch', '俯仰')):
            lab = QLabel(f'{name} +0°')
            lab.setStyleSheet('color: #444; font-family: monospace;')
            lab.setToolTip(
                f'限位: 航向 ±{config.GIMBAL_YAW_LIMIT_DEG:.0f}°, '
                f'俯仰 {config.GIMBAL_PITCH_MIN_DEG:.0f}°'
                f'~+{config.GIMBAL_PITCH_MAX_DEG:.0f}° (被限位卡住时标红)')
            self._gimbal_labels[axis] = lab
        if not in_group:
            g.addWidget(title, 0, 0, 1, 2)
        g.addWidget(self._joy, 1, 0, alignment=Qt.AlignHCenter)
        side = QVBoxLayout()
        side.addWidget(btn_center)
        side.addWidget(btn_stab)
        side.addWidget(btn_follow)
        side.addStretch(1)
        g.addLayout(side, 1, 1)
        angles = QHBoxLayout()
        angles.setSpacing(10)
        angles.addWidget(self._gimbal_labels['yaw'])
        angles.addWidget(self._gimbal_labels['pitch'])
        angles.addStretch(1)
        g.addLayout(angles, 2, 0, 1, 2)

        # 视场角滑杆 (竖排): 经 simSetCameraFov 实时调整 UE 相机 FOV,
        # 本机/文件源随云台整体置灰。上=广角, 下=拉近
        fov_col = QVBoxLayout()
        fov_col.setSpacing(2)
        fov_title = QLabel('视场')
        fov_title.setAlignment(Qt.AlignHCenter)
        self.fov_slider = QSlider(Qt.Vertical)
        self.fov_slider.setRange(int(config.GIMBAL_FOV_MIN_DEG),
                                 int(config.GIMBAL_FOV_MAX_DEG))
        self.fov_slider.setValue(int(config.GIMBAL_FOV_DEFAULT_DEG))
        self.fov_slider.setMinimumHeight(96)
        self.fov_slider.setToolTip(
            f'相机视场角 ({config.GIMBAL_FOV_MIN_DEG:.0f}~'
            f'{config.GIMBAL_FOV_MAX_DEG:.0f}°): 实时调整 UE 相机, '
            '所有相机共用一个值, 重启/重连自动恢复; '
            '未动过时保持 settings.json 原值')
        self.fov_slider.valueChanged.connect(self._on_fov_slider)
        self.fov_value = QLabel()
        self.fov_value.setStyleSheet('color: #444; font-family: monospace;')
        self.fov_value.setAlignment(Qt.AlignHCenter)
        fov_col.addWidget(fov_title)
        # 滑杆宽度固定, 布局默认靠左放 —— 显式居中, 与上下文字同轴线
        fov_col.addWidget(self.fov_slider, stretch=1, alignment=Qt.AlignHCenter)
        fov_col.addWidget(self.fov_value)
        g.addLayout(fov_col, 1, 2, 2, 1)   # 右侧整列 (与摇杆/角度读数同高)

        # 数字变焦滑杆 (视场滑杆右侧): 显示层中心裁切, 与 FOV 相互独立;
        # 滑杆整数值 = 倍数×100 (分辨率 0.01)
        zoom_col = QVBoxLayout()
        zoom_col.setSpacing(2)
        zoom_title = QLabel('变焦')
        zoom_title.setAlignment(Qt.AlignHCenter)
        self.zoom_slider = QSlider(Qt.Vertical)
        self.zoom_slider.setRange(int(config.DISPLAY_ZOOM_MIN * 100),
                                  int(config.DISPLAY_ZOOM_MAX * 100))
        self.zoom_slider.setValue(int(config.DISPLAY_ZOOM_MIN * 100))
        self.zoom_slider.setMinimumHeight(96)
        self.zoom_slider.setToolTip(
            f'数字变焦 (×{config.DISPLAY_ZOOM_MIN:g}~×{config.DISPLAY_ZOOM_MAX:g}): '
            '画面中心裁切放大, 不改 UE 相机真实视场; 滚轮/双击画面亦可')
        self.zoom_slider.valueChanged.connect(self._on_zoom_slider)
        self.zoom_value = QLabel()
        self.zoom_value.setStyleSheet('color: #444; font-family: monospace;')
        self.zoom_value.setAlignment(Qt.AlignHCenter)
        zoom_col.addWidget(zoom_title)
        zoom_col.addWidget(self.zoom_slider, stretch=1, alignment=Qt.AlignHCenter)
        zoom_col.addWidget(self.zoom_value)
        g.addLayout(zoom_col, 1, 3, 2, 1)   # 视场滑杆右侧同高
        self._zoom_sync_label(self.zoom_slider.value() / 100.0)

    def _on_gimbal_joy(self, x, y):
        """云台摇杆偏转 → 角速度 (x=偏航右+, y=俯仰上+), 回中即停。
        航向跟随中偏航分量由抓取线程改写世界方位锚点带动机头。"""
        rate = config.GIMBAL_JOY_RATE_DEG_S
        self.grabber.gimbal_set_rate(x * rate, y * rate)

    def _on_follow_toggle(self, on):
        """跟随开关 → 抓取线程 (方位锚点快照/冻结由其处理)。"""
        self.grabber.set_follow(on)

    def set_follow_available(self, ok):
        """跟随按钮可用性 (接管控制 33ms 轮询门控): 需手动遥控接管中。
        父面板置灰 (反控锁云台) 时按钮随父级, 解锁后按本标志恢复。"""
        if ok == self._follow_avail:
            return
        self._follow_avail = ok
        self.btn_follow.setEnabled(ok)

    def _on_fov_slider(self, value):
        """视场滑杆变化 → 下发 FOV (拖动中实时跟手, 节流靠抓取帧率)。
        界面同步回写滑杆时已 blockSignals, 不会误触发本槽。"""
        self._fov_sync_label(value)
        self.grabber.set_fov(float(value))

    def _fov_sync_label(self, value):
        self.fov_value.setText(f'{value}°')

    def _on_zoom_step(self, step):
        """滚轮变焦: 每档 ±DISPLAY_ZOOM_STEP, 夹在 1.0~DISPLAY_ZOOM_MAX。
        滑杆由 _refresh 同步回写 (blockSignals 不回环)。"""
        z = self.grabber.get_zoom() + step * config.DISPLAY_ZOOM_STEP
        self.grabber.set_zoom(z)

    def _on_zoom_slider(self, value):
        """变焦滑杆变化 → 下发 (滑杆整数 = 倍数×100)。界面同步回写时已
        blockSignals, 不会误触发本槽。"""
        z = value / 100.0
        self._zoom_sync_label(z)
        self.grabber.set_zoom(z)

    def _zoom_sync_label(self, z):
        self.zoom_value.setText(f'×{z:g}')

    def set_gimbal_locked(self, locked):
        """锁定/解锁云台面板 (视觉反控进行中锁定, 防止反控时转动视角)。
        锁定立即生效; 解锁交给 _refresh 按可用性恢复。"""
        if self._gimbal_locked == locked:
            return
        self._gimbal_locked = locked
        if locked:
            self.gimbal.setEnabled(False)

    def _update_gimbal_limit(self):
        """到限位提示: 被限位卡住时只红对应轴的读数, 恢复可动即清红。
        拒绝状态由 grabber 会话内判定, 样式只在变化时更新。"""
        blocked = self.grabber.get_gimbal_blocked()
        red = {'yaw': blocked['yaw+'] or blocked['yaw-'],
               'pitch': blocked['pitch+'] or blocked['pitch-']}
        for axis, hit in red.items():
            if hit == self._angle_red[axis]:
                continue
            self._angle_red[axis] = hit
            self._gimbal_labels[axis].setStyleSheet(
                'color: #d9534f; font-weight: bold; font-family: monospace;'
                if hit else 'color: #444; font-family: monospace;')

    @staticmethod
    def _src_display(raw):
        """源原始名 -> 短显示名 (去 local:/file: 前缀与文件路径)。"""
        if raw.startswith(LOCAL_PREFIX):
            return raw[len(LOCAL_PREFIX):]
        if raw.startswith(FILE_PREFIX):
            return os.path.basename(raw[len(FILE_PREFIX):])
        return raw

    def _rebuild_src_menu(self):
        """重建源菜单: UE 相机 + 本机摄像头 + 已打开文件 + 打开文件对话框。
        展开时后台重扫本机摄像头, 热插拔即时可见。"""
        menu = self.src_btn.menu()
        menu.clear()
        cur = self.grabber.camera
        groups = (
            list(DEFAULT_CAMERAS),
            [f'{LOCAL_PREFIX}{d}' for d in self._local_cams],
            [f'{FILE_PREFIX}{p}' for p in self._file_map.values()],
        )
        for group in groups:
            for raw in group:
                act = menu.addAction(self._src_display(raw))
                act.setData(raw)
                act.setCheckable(True)
                act.setChecked(raw == cur)
            if group:
                menu.addSeparator()
        menu.addAction(FILE_OPEN_ITEM)
        self._scan_local_cams()

    def _scan_local_cams(self):
        """后台重扫 /dev/video*，新插入的摄像头即时出现在菜单里。
        本地探测不经 AirSim，没开 UE 也能选；WSL2 下摄像头需先 usbipd attach。"""
        if self._scanning:
            return
        self._scanning = True

        def scan():
            cams = list_local_cameras()
            self._scanning = False
            if cams != self._local_cams:
                self._local_cams = cams
                self.cams_found.emit()   # GUI 线程触发菜单刷新

        threading.Thread(target=scan, daemon=True).start()

    def _on_src_action(self, act):
        """菜单点击: 切到所选源; "打开视频文件..." 先弹对话框登记再切。"""
        text = act.text()
        if text == FILE_OPEN_ITEM:
            path, _ = QFileDialog.getOpenFileName(
                self, '选择视频文件', os.path.expanduser('~'), VIDEO_FILTER)
            if not path:
                return
            self._file_map[f'{FILE_PREFIX}{os.path.basename(path)}'] = path
            raw = f'{FILE_PREFIX}{path}'
        else:
            raw = act.data()
            if raw is None:
                return
        self._switch_source(raw)
        self.src_btn.setText(f'视频源: {self._src_display(raw)}')

    def _switch_source(self, name):
        """切换抓取源并立即清屏: 新源暂时无帧时不残留上一源定格画面。"""
        self.video.clear_frame()
        self.grabber.set_camera(name)

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
        # 仅 AirSim 源且 RPC 已连通可用 (反控锁定期间强制置灰): 打开地面站/UE 未 play 时默认置灰
        self.gimbal.setEnabled(
            not self._gimbal_locked
            and gim is not None and self.grabber.airsim_connected())
        if gim is not None:
            yaw, pitch = gim
            self._gimbal_labels['yaw'].setText(f'航向 {yaw:+.0f}°')
            self._gimbal_labels['pitch'].setText(f'俯仰 {pitch:+.0f}°')
            self._update_gimbal_limit()
            # 跟随按钮随抓取线程状态同步 (重连等外部因素强制退出时弹起)
            if self.btn_follow.isChecked() != self.grabber.get_follow():
                self.btn_follow.blockSignals(True)
                self.btn_follow.setChecked(self.grabber.get_follow())
                self.btn_follow.blockSignals(False)
            # 视场滑杆跟随当前相机 FOV; 用户拖动中不回写
            fov = self.grabber.get_fov()
            if fov is not None and not self.fov_slider.isSliderDown():
                self.fov_slider.blockSignals(True)
                self.fov_slider.setValue(int(round(fov)))
                self.fov_slider.blockSignals(False)
                self._fov_sync_label(int(round(fov)))
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
        # 变焦滑杆跟随当前倍率 (滚轮/双击改的也同步); 拖动中不回写
        zoom = self.grabber.get_zoom()
        if not self.zoom_slider.isSliderDown():
            self.zoom_slider.blockSignals(True)
            self.zoom_slider.setValue(int(round(zoom * 100)))
            self.zoom_slider.blockSignals(False)
            self._zoom_sync_label(zoom)
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
