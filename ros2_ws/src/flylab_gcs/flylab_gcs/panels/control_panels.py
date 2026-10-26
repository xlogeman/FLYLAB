#!/usr/bin/env python3
"""视觉反控 (ArUco 降落) + 手动遥控面板 (MainWindow 的功能域 mixin)。

两者都走 offboard, 门控逻辑互相咬合 (互斥 + 遥控最高优先防炸机出口),
故放在同一模块。样式常量 _TOOL_BTN_STYLE/_LAND_BTN_STYLE 定义在
main_window (ui_style.py)。
"""
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (QGroupBox, QHBoxLayout, QPushButton,
                           QToolButton, QVBoxLayout)

from .. import config
from ..teleop_pad import VirtualJoystick


class ControlPanelsMixin:
    """MainWindow 的视觉反控 + 手动遥控功能域 (self.* 为主窗口属性)。"""

    # ---------- 视觉反控 (降落) / 手动遥控 门控 (都走 offboard, 互斥) ----------

    def _update_landing_btn(self):
        """offboard 功能互斥门控 (33ms 轮询):
        - 视觉反控: 需已解锁升空 + ArUco检测运行中 + AirSim 源 + 链路已开;
          进行中锁链路与云台, 允许开遥控抢回控制权 (防炸机出口)
        - 遥控: 仅需链路已开; 进行中锁链路与视觉反控"""
        link_ok = self._selected_link() is not None
        # 进行中的按钮自身必须保持可点 (否则无法取消), 只锁定其余项
        if self.teleop_btn.isChecked():
            self.link_btn.setEnabled(False)
            self.landing_btn.setEnabled(False)
            self.camera_tab.set_gimbal_locked(False)
            return
        if self.landing_btn.isChecked():
            self.link_btn.setEnabled(False)
            self.camera_tab.set_gimbal_locked(True)
            # 反控中允许开遥控抢回控制权 (防炸机): 接管时自动取消反控
            if self.teleop_btn.isEnabled() != link_ok:
                self.teleop_btn.setEnabled(link_ok)
            return
        self.camera_tab.set_gimbal_locked(False)
        self.link_btn.setEnabled(True)
        if self.teleop_btn.isEnabled() != link_ok:
            self.teleop_btn.setEnabled(link_ok)
        ok = False
        algo = self.running_algo
        if algo is not None and algo.get('enables_landing') and link_ok:
            # 需已解锁且在空中 (地面/未解锁点反控无意义); 俯仰由点击时
            # 自动打到垂直向下, 只要求 AirSim 源 (有云台)
            ok = (self.veh_ctrl.is_flying()
                  and self.camera_tab.grabber.get_gimbal() is not None)
        if self.landing_btn.isEnabled() != ok:
            self.landing_btn.setEnabled(ok)

    def _on_landing_toggle(self):
        if self.landing_btn.isChecked():
            # 反控前自动把云台俯仰打到垂直向下, 省去每次手动调整
            self.camera_tab.grabber.gimbal_pitch_bottom()
            # 链路参数跟随工具栏所选链路 (mavros / uxrce)
            link = self._selected_link() or 'mavros'
            cmd = list(config.LANDING_CMD) + ['--ros-args', '-p', f'link:={link}']
            self.land_proc.start(cmd)
            # 新进程需完成 DDS 发现 + 收首帧飞控状态 (1~3s), 重发多次防吞;
            # 节点对重复 start 幂等
            QTimer.singleShot(1000, lambda: self._send_landing_start(6))
        else:
            self.landing_btn.setText('视觉反控')
            self._stop_landing()

    def _check_landing_status(self):
        """降落节点状态事件处理: 按钮文字/弹起随状态联动
        (tracking=对准下降中, landing=已切 AUTO.LAND, landed=成功, idle=结束)。
        状态带事件序号, 只在有新事件时处理 (不漏瞬时的"成功")。"""
        status, seq = self.node.get_landing_status()
        if seq == self._last_landing_seq:
            return
        self._last_landing_seq = seq
        if not status:
            return
        if status == 'tracking':
            self.landing_btn.setText('反控中: 对准下降')
            return
        if status == 'landing':
            self.landing_btn.setText('反控中: 自动降落')
            return
        if status == 'landed':
            self.log.info('\033[32mArUco 视觉降落完成, 飞机已上锁\033[0m')
            self._flash_landing_btn('#2d8a3e')      # 绿色高亮闪一下
        elif status == 'idle':
            self.log.info('降落流程已结束 (取消/中止), 按钮已复位')
        else:
            return
        # 按钮弹起 (checked = 降落进行中) + 文字复位 + 回收降落节点进程
        self.landing_btn.setChecked(False)
        self.landing_btn.setText('视觉反控')
        self.land_proc.stop()

    def _flash_landing_btn(self, color):
        """按钮背景色闪烁提示, 1.5s 后恢复原样式。"""
        self.landing_btn.setStyleSheet(
            self._TOOL_BTN_STYLE +
            f'\nQToolButton {{ background-color: {color}; color: white; }}')
        QTimer.singleShot(1500, lambda: self.landing_btn.setStyleSheet(
            self._TOOL_BTN_STYLE))

    def _send_landing_start(self, retries_left):
        """触发降落并按 500ms 间隔重发; 按钮已抬起或节点进程已死则跳过。"""
        if not self.landing_btn.isChecked():
            return
        rc = self.land_proc.exited()
        if rc is not None:
            self.log.error(
                f'降落节点进程启动失败 (退出码 {rc}), '
                f'检查编译: colcon build --packages-select flylab_vision')
            self.land_proc.clear()
            self.landing_btn.setChecked(False)
            self.landing_btn.setText('视觉反控')
            return
        self.node.publish_cmd(config.LANDING_START_CMD)
        if retries_left > 0:
            QTimer.singleShot(500, lambda: self._send_landing_start(retries_left - 1))

    def _stop_landing(self):
        """取消降落: 发 landing_stop (节点自行切回 LOITER 交还控制权并清标注),
        稍后回收进程, 给节点留出发送停止指令与清屏的时间。"""
        self.node.publish_cmd(config.LANDING_STOP_CMD)
        QTimer.singleShot(500, self.land_proc.stop)

    # ---------- 手动遥控 (toggle + 双摇杆, 执行节点 flylab_vision/teleop_node) ----------

    def _build_teleop_panel(self):
        box = QGroupBox('手动遥控')
        lay = QVBoxLayout(box)
        lay.setContentsMargins(6, 4, 6, 6)
        self.teleop_btn = QPushButton('开启遥控')
        self.teleop_btn.setCheckable(True)
        self.teleop_btn.setEnabled(False)
        self.teleop_btn.setStyleSheet(self._LAND_BTN_STYLE)
        self.teleop_btn.setToolTip(
            '需先: 1) 工具栏开启通信链路  2) 飞行控制起飞升空。'
            '开启后接管 offboard, 摇杆控速度 (回中=悬停); 关闭交还控制权')
        self.teleop_btn.toggled.connect(self._on_teleop_toggle)
        lay.addWidget(self.teleop_btn)
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
        for w in self._teleop_widgets:   # 默认置灰, 接管后解锁
            w.setEnabled(False)
        return box

    def _teleop_axis(self, key, val):
        """更新单轴归一化量并立即下发 (摇杆拖动中实时跟手)。"""
        if not self.teleop_btn.isChecked():
            return
        self._teleop_axes[key] = val
        a = self._teleop_axes
        self.node.publish_teleop_twist(a['x'], a['y'], a['z'], a['w'])

    def _teleop_clear_axes(self):
        """界面侧全部回中 (交还/复位时调用), 防止残留遥控量。
        摇杆 reset 会经 changed 信号回写各轴, 最后统一发零。"""
        self._teleop_joy_l.reset()
        self._teleop_joy_r.reset()
        for key in self._teleop_axes:
            self._teleop_axes[key] = 0.0
        self.node.publish_teleop_twist(0.0, 0.0, 0.0, 0.0)

    def _on_teleop_toggle(self):
        if self.teleop_btn.isChecked():
            # 遥控接管优先 (防炸机出口): 先取消反控, 降落节点交还 offboard
            # 后遥控再接管
            if self.landing_btn.isChecked():
                self.landing_btn.setChecked(False)
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
            self.teleop_btn.setText('交还控制权 ...')
            QTimer.singleShot(1500, self.teleop_proc.stop)   # 留时间走完交还流程

    def _send_teleop_start(self, retries_left):
        if not self.teleop_btn.isChecked():
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
        已结束 (按钮弹起 + 回收进程)。状态带序号, 只处理新事件。"""
        status, seq = self.node.get_teleop_status()
        if seq == self._last_teleop_seq:
            return
        self._last_teleop_seq = seq
        if not status:
            return
        if status == 'engaged':
            self.teleop_btn.setText('遥控中 (OFFBOARD)')
            self._set_teleop_controls_enabled(True)
        elif status == 'engaging':
            self.teleop_btn.setText('接管中 ...')
        elif status == 'releasing':
            self.teleop_btn.setText('交还控制权 ...')
        else:   # idle / failed / error:*
            if status.startswith('error:') or status == 'failed':
                self.log.error(f'手动遥控: {status}')
            self.teleop_proc.stop()
            self.teleop_proc.clear()
            self._teleop_reset()

    def _teleop_reset(self):
        """按钮/摇杆复位为空闲态 (异常或流程结束调用)。"""
        self._teleop_clear_axes()
        self._set_teleop_controls_enabled(False)
        self.teleop_btn.setChecked(False)
        self.teleop_btn.setText('开启遥控')

    def _set_teleop_controls_enabled(self, enabled):
        for w in self._teleop_widgets:
            w.setEnabled(enabled)

    # ---------- 视觉反控 (工具栏 toggle = ArUco降落, 执行节点 aruco_landing_node) ----------

    def _build_landing_button(self):
        """视觉反控工具按钮 (ArUco 视觉降落): 点击拉起 aruco_landing_node
        并接管 offboard, 按视觉输出对准码并调速下降, 满足条件切 AUTO.LAND;
        再点取消降落交还控制权。仅 ArUco检测运行中可点。"""
        self.landing_btn = QToolButton()
        self.landing_btn.setText('视觉反控')
        self.landing_btn.setCheckable(True)
        self.landing_btn.setEnabled(False)
        self.landing_btn.setStyleSheet(self._TOOL_BTN_STYLE)
        self.landing_btn.setToolTip(
            '根据当前算法的视觉输出控制飞机; '
            '开启手动遥控可随时抢回控制权')
        self.landing_btn.toggled.connect(self._on_landing_toggle)
        return self.landing_btn
