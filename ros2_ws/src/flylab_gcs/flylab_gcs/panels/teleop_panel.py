#!/usr/bin/env python3
"""手动遥控面板 (MainWindow 的功能域 mixin) —— teleop_node 的界面侧。

offboard 接管之二 (与视觉反控互斥, 门控表在 landing_panel.py):
工具栏 接管控制 → 手动遥控 勾选后 teleop_node 接管 offboard, 右栏双摇杆
(或游戏手柄) 出归一化 Twist, 节点换算机体系 FRD 速度 + 偏航角速率。

航向跟随外环也在本模块: 云台面板"跟随"开关开启后 25Hz P 律把云台偏航
误差折算成偏航角速率并入遥控量 (内环是 teleop_node/PX4 的偏航角速率
闭环); 游戏手柄经 GamepadReader 线程注入双摇杆 (set_external), 与鼠标
拖动走同一条 changed 信号下发链路, 统一出口 _teleop_publish 防覆盖。
菜单项 (逻辑 QAction + 显示项) 构造也在本模块 (_build_teleop_menu_item),
由接管控制按钮 (_build_takeover_button) 组装。
"""
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QGroupBox, QHBoxLayout, QLabel, QPushButton, \
    QVBoxLayout

from .. import config
from ..gamepad import GamepadReader
from ..teleop_pad import VirtualJoystick
from .landing_panel import make_action_pair


class TeleopPanelMixin:
    """MainWindow 的手动遥控功能域 (self.* 为主窗口属性)。"""

    # ---------- 接管控制菜单里的"手动遥控"项 ----------

    def _build_teleop_menu_item(self, menu):
        """接管控制菜单的手动遥控项 (逻辑 QAction + 菜单显示项),
        勾选/弹起联动启停流程 _on_teleop_toggle。"""
        logic, disp = make_action_pair(
            menu, '手动遥控',
            '需先开启通信链路。开启后接管 offboard, 右栏摇杆控速度 '
            '(回中=悬停); 再点交还控制权')
        logic.toggled.connect(self._on_teleop_toggle)
        return logic, disp

    # ---------- 右栏摇杆遥控板 ----------

    def _build_teleop_panel(self):
        """右栏摇杆遥控板: 只放双摇杆 (开关在工具栏 接管控制 菜单)。
        同时启动航向跟随外环定时器 (云台偏航误差 → 偏航角速率并入遥控量)。"""
        box = QGroupBox('手动遥控')
        box.setToolTip('开启方式: 工具栏 接管控制 → 手动遥控 '
                       '(需先开启通信链路)')
        lay = QVBoxLayout(box)
        lay.setContentsMargins(6, 4, 6, 6)
        # 双摇杆 (同真实遥控器): 左 = 升降/偏航, 右 = 平移 (力度=偏离圆心距离)
        pads = QHBoxLayout()
        self._teleop_widgets = []

        def joy_group(title, cb):
            g = QGroupBox(title)
            g_lay = QVBoxLayout(g)
            g_lay.setContentsMargins(4, 2, 4, 4)
            joy = VirtualJoystick()
            joy.changed.connect(cb)
            g_lay.addWidget(joy, alignment=Qt.AlignHCenter)
            self._teleop_widgets.append(joy)
            return joy, g

        self._teleop_joy_l, g_l = joy_group(
            '高度 / 偏航',
            # 左摇杆: x 右拖=右转(右+, w 轴左转+ 取反), y 上拖=升
            lambda x, y: (self._teleop_axis('w', -x), self._teleop_axis('z', y)))
        self._teleop_joy_r, g_r = joy_group(
            '平移 (机头方向)',
            # 右摇杆: x 右拖=右移, y 上拖=前进
            lambda x, y: (self._teleop_axis('y', x), self._teleop_axis('x', y)))
        pads.addWidget(g_l)
        pads.addWidget(g_r)
        lay.addLayout(pads)
        # 游戏手柄接管: 打开后手柄双摇杆驱动界面摇杆 (画面旋钮联动, 与鼠标
        # 拖动同一条下发链路), 可直观看操控力度; 与鼠标拖动互抢, 后动者生效
        gp_row = QHBoxLayout()
        self._gamepad_btn = QPushButton('手柄控制')
        self._gamepad_btn.setCheckable(True)
        self._gamepad_btn.setToolTip(
            '游戏手柄双摇杆驱动界面摇杆 (左手柄杆=高度/偏航, 右=平移, 同布局)。\n'
            'WSL 需先 usbipd 直通: usbipd attach --busid <busid> --wsl <发行版>\n'
            '轴号不对手柄时改 config.GAMEPAD_AXES (xpad 原始轴序默认)')
        self._gamepad_status = QLabel('未检测手柄')
        self._gamepad_status.setStyleSheet('color: #888; font-size: 11px;')
        gp_row.addWidget(self._gamepad_btn)
        gp_row.addWidget(self._gamepad_status, 1)
        lay.addLayout(gp_row)
        self._gamepad = GamepadReader()
        self._gamepad.stick.connect(self._on_gamepad_stick)
        self._gamepad.plugged.connect(self._on_gamepad_plugged)
        self._gamepad.failed.connect(self._on_gamepad_failed)
        self._gamepad_btn.toggled.connect(self._on_gamepad_toggle)
        for w in self._teleop_widgets:   # 默认置灰, 接管后解锁
            w.setEnabled(False)
        # 航向跟随外环 (云台面板"跟随"开关, 见 camera_widget): 25Hz P 律,
        # 指令并入 _teleop_publish 的 w 轴; _teleop_live 标记遥控接管中
        self._follow_cmd = 0.0
        self._teleop_live = False
        self._follow_timer = QTimer(self)
        self._follow_timer.timeout.connect(self._follow_tick)
        self._follow_timer.start(int(round(1000.0 / config.GIMBAL_FOLLOW_HZ)))
        return box

    def _follow_tick(self):
        """航向跟随外环 (GIMBAL_FOLLOW_HZ, 界面定时器): 误差 = 反卷后的
        云台偏航 (视线世界方位 − 航向, 右+), 死区外按 P 律折算偏航角速率,
        归一化后并入遥控量 angular.z (符号取反: teleop 左转+)。内环是
        teleop_node/PX4 的偏航角速率闭环。未接管遥控 / 数据过期发零
        (跟随开关本身只在接管中可开, 这里兜底复位残留指令)。"""
        cmd = 0.0
        if self._teleop_live:
            err = self.camera_tab.grabber.get_follow_error()
            if err is not None and abs(err) > config.GIMBAL_FOLLOW_DEADBAND_DEG:
                rate = min(abs(err) * config.GIMBAL_FOLLOW_KP,
                           config.GIMBAL_FOLLOW_YAW_RATE_DEG_S)
                cmd = -(rate / config.GIMBAL_FOLLOW_YAW_RATE_DEG_S) \
                    * (1.0 if err > 0 else -1.0)
        if cmd != self._follow_cmd:
            self._follow_cmd = cmd
            self._teleop_publish()

    def _teleop_axis(self, key, val):
        """更新单轴归一化量并立即下发 (摇杆拖动中实时跟手)。"""
        if not self.teleop_act.isChecked():
            return
        self._teleop_axes[key] = val
        self._teleop_publish()

    def _teleop_publish(self):
        """合并下发遥控量: 双摇杆四轴 + 航向跟随偏航指令 (取和夹紧),
        统一出口防止两个输入源互相覆盖。"""
        a, f = self._teleop_axes, self._follow_cmd
        w = max(-1.0, min(1.0, a['w'] + f))
        self.node.publish_teleop_twist(a['x'], a['y'], a['z'], w)

    def _teleop_clear_axes(self):
        """界面侧全部回中 (交还/复位时调用), 防止残留遥控量。
        摇杆 reset 会经 changed 信号回写各轴, 最后统一发零。"""
        self._follow_cmd = 0.0
        self._teleop_joy_l.reset()
        self._teleop_joy_r.reset()
        for key in self._teleop_axes:
            self._teleop_axes[key] = 0.0
        self.node.publish_teleop_twist(0.0, 0.0, 0.0, 0.0)

    # ---------- 游戏手柄输入 (gamepad.py 线程 → 双摇杆联动) ----------

    @staticmethod
    def _gamepad_status_style(color):
        return f'color: {color}; font-size: 11px;'

    def _on_gamepad_toggle(self, on):
        """手柄开关: 控制读取线程生命周期 (连接检测随之启停)。"""
        if on:
            self._gamepad.start()
        else:
            self._gamepad.stop()
            self._gamepad_status.setText('未检测手柄')
            self._gamepad_status.setStyleSheet(
                self._gamepad_status_style('#888'))

    def _on_gamepad_stick(self, lx, ly, rx, ry):
        """手柄量 → 双摇杆 set_external: 画面旋钮联动 + 走 changed 信号
        同一条下发链路 (与鼠标拖动一致)。仅遥控接管中 (engaged) 生效。"""
        if not self._teleop_live:
            return
        self._teleop_joy_l.set_external(lx, ly)
        self._teleop_joy_r.set_external(rx, ry)

    def _on_gamepad_plugged(self, ok, name):
        """连接状态 → 面板标签 (绿=已连, 灰=未连)。"""
        if ok:
            self._gamepad_status.setText(f'已连接: {name}' if name else '已连接')
            self._gamepad_status.setStyleSheet(
                self._gamepad_status_style('#2d8a3e'))
        else:
            self._gamepad_status.setText('未检测手柄 (usbipd 直通?)')
            self._gamepad_status.setStyleSheet(
                self._gamepad_status_style('#888'))

    def _on_gamepad_failed(self, msg):
        """线程初始化失败: 只报一次 (线程已退出), 开关弹起防假开。"""
        self.log.error(msg)
        if self._gamepad_btn.isChecked():
            self._gamepad_btn.setChecked(False)   # 触发 toggle(False) → stop

    # ---------- 接管启停与状态联动 ----------

    def _on_teleop_toggle(self):
        if self.teleop_act.isChecked():
            # 遥控接管优先 (防炸机出口): 先取消反控, 降落节点交还 offboard
            # 后遥控再接管
            if self.landing_act.isChecked():
                self.landing_act.setChecked(False)
            # 避障链路跟踪器同样走 offboard, 与遥控互斥, 先整链路退出
            if self.running_flight is not None and self.running_flight.get('need_link'):
                self._stop_flight()
                self.log.info('手动遥控接管, 已自动退出避障链路')
            # 链路参数跟随工具栏所选链路 (mavros / uxrce)
            link = self._selected_link() or 'mavros'
            cmd = list(config.TELEOP_CMD) + ['--ros-args', '-p', f'link:={link}']
            self.teleop_proc.start(cmd)
            # 等节点完成 DDS 发现 + 收到飞控状态后再发接管指令 (重发多次, 节点幂等)
            QTimer.singleShot(1000, lambda: self._send_teleop_start(6))
        else:
            self.node.publish_cmd(config.TELEOP_STOP_CMD)
            self._teleop_clear_axes()
            self._set_teleop_controls_enabled(False)
            self.takeover_btn.setText('接管控制: 交还中 ...')
            QTimer.singleShot(1500, self.teleop_proc.stop)   # 留时间走完交还流程

    def _send_teleop_start(self, retries_left):
        if not self.teleop_act.isChecked():
            return
        rc = self.teleop_proc.exited()
        if rc is not None:
            self.log.error(f'遥控节点进程启动失败 (退出码 {rc}), '
                           f'检查编译: colcon build --packages-select flylab_vision')
            self.teleop_proc.clear()
            self._teleop_reset()
            return
        self.node.publish_cmd(config.TELEOP_START_CMD)
        if retries_left > 0:
            QTimer.singleShot(500, lambda: self._send_teleop_start(retries_left - 1))

    def _check_teleop_status(self):
        """遥控节点状态事件: engaged=接管成功 (解锁摇杆), idle/failed/error=
        已结束 (菜单项弹起 + 回收进程)。状态带序号, 只处理新事件。"""
        status, seq = self.node.get_teleop_status()
        if seq == self._last_teleop_seq:
            return
        self._last_teleop_seq = seq
        if not status:
            return
        if status == 'engaged':
            self.takeover_btn.setText('接管控制: 遥控中')
            self._set_teleop_controls_enabled(True)
        elif status == 'engaging':
            self.takeover_btn.setText('接管控制: 接管中 ...')
        elif status == 'releasing':
            self.takeover_btn.setText('接管控制: 交还中 ...')
        else:   # idle / failed / error:*
            if status.startswith('error:') or status == 'failed':
                self.log.error(f'手动遥控: {status}')
            self.teleop_proc.stop()
            self.teleop_proc.clear()
            self._teleop_reset()

    def _teleop_reset(self):
        """菜单项/摇杆复位为空闲态 (异常或流程结束调用)。"""
        self._teleop_clear_axes()
        self._set_teleop_controls_enabled(False)
        self.teleop_act.setChecked(False)
        self.takeover_btn.setText('接管控制')

    def _set_teleop_controls_enabled(self, enabled):
        for w in self._teleop_widgets:
            w.setEnabled(enabled)
        self._teleop_live = enabled   # 跟随外环只在接管成功 (engaged) 后工作
        if not enabled:
            # 交还/复位只退 UI 来源的跟随; 算法来源 (反控请求) 不受
            # 遥控生命周期影响, 由算法自己退或 gimbal_cmd 心跳超时退
            self.camera_tab.grabber.clear_ui_follow()
