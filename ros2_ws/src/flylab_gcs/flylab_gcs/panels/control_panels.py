#!/usr/bin/env python3
"""接管控制下拉菜单 (视觉反控 + 手动遥控) 面板 (MainWindow 的功能域 mixin)。

两者都走 offboard, 门控逻辑互相咬合 (互斥 + 遥控最高优先防炸机出口),
故放在同一模块。菜单项为常驻 QAction (勾选 = 运行中, 再点取消),
门控状态由 33ms 轮询落在 QAction 的 enabled 上, 菜单展开即见最新。

视觉反控按运行中的算法展开模式子菜单 (反控行为因算法而异): 目前只有
ArUco检测 (enables_landing), 子菜单含 图像空间 / 3D 姿态 两个模式项,
模式经 -p landing_mode 传给降落节点; 其他算法将来在 config.ALGORITHMS
注册各自的反控形态。LandingActions 句柄对外保持旧 landing_act 单项语义,
其余引用方 (main_window / vehicle_panel / algo_panel) 无需感知子菜单。
样式常量 _TOOL_BTN_STYLE 定义在 main_window (ui_style.py)。
"""
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (QGroupBox, QHBoxLayout, QLabel, QMenu,
                           QPushButton, QToolButton, QVBoxLayout)

from .. import config
from ..gamepad import GamepadReader
from ..teleop_pad import VirtualJoystick


class LandingActions:
    """视觉反控菜单全部 QAction (模式子菜单两项 + 普通单项) 的统一句柄,
    对外兼容旧 landing_act 单项语义:
    - isChecked() = 任一形态运行中
    - setChecked(False) = 全部弹起 (勾选由用户点选, 不支持 True)
    - setEnabled() = 同步门控到全部项
    菜单形态按运行中的算法切换 (_update_landing_menu_shape): 声明多个
    landing_modes 的算法 (ArUco检测) 用模式子菜单, 其余用普通单项。
    """

    def __init__(self, flat_act, image_act, pose_act):
        self.flat = flat_act
        self.image = image_act
        self.pose = pose_act

    def mode_action(self, mode):
        """模式常量 -> 对应菜单项 (普通单项不走模式, 由调用方显式传入)。"""
        return self.pose if mode == config.LANDING_MODE_POSE3D else self.image

    def isChecked(self):
        return (self.flat.isChecked() or self.image.isChecked()
                or self.pose.isChecked())

    def setChecked(self, checked):
        if checked:
            raise ValueError('视觉反控模式由用户点选, 不支持外部 setChecked(True)')
        self.flat.setChecked(False)
        self.image.setChecked(False)
        self.pose.setChecked(False)

    def setEnabled(self, enabled):
        self.flat.setEnabled(enabled)
        self.image.setEnabled(enabled)
        self.pose.setEnabled(enabled)


class ControlPanelsMixin:
    """MainWindow 的接管控制功能域 (self.* 为主窗口属性)。"""

    # ---------- 视觉反控 (降落) / 手动遥控 门控 (都走 offboard, 互斥) ----------

    def _update_takeover_gates(self):
        """offboard 功能互斥门控 (33ms 轮询), 门控结果落在菜单项上:
        - 视觉反控: 需已解锁升空 + ArUco检测运行中 + AirSim 源 + 链路已开;
          进行中锁链路与云台, 允许开遥控抢回控制权 (防炸机出口);
          模式子菜单 (图像空间/3D 姿态) 互斥: 运行中非现役模式置灰
        - 遥控: 仅需链路已开; 进行中仍可点视觉反控 (先取消遥控再反控)
        同时维护接管按钮的状态灯 (任一功能运行中点亮)。"""
        link_ok = self._selected_link() is not None
        # 视觉反控的完整门控条件 (进行中恒可点以便取消)
        landing_ok = False
        algo = self.running_algo
        if algo is not None and algo.get('enables_landing') and link_ok:
            # 需已解锁且在空中 (地面/未解锁点反控无意义); 俯仰由点击时
            # 自动打到垂直向下, 只要求 AirSim 源 (有云台)
            landing_ok = (self.veh_ctrl.is_flying()
                          and self.camera_tab.grabber.get_gimbal() is not None)
        if self.teleop_act.isChecked():
            self.link_btn.setEnabled(False)
            self.camera_tab.set_gimbal_locked(False)
            # 遥控中仍可点视觉反控: 接管时自动取消遥控
            self.landing_act.setEnabled(landing_ok)
        elif self.landing_act.isChecked():
            self.link_btn.setEnabled(False)
            self.camera_tab.set_gimbal_locked(True)
            # 反控中允许开遥控抢回控制权 (防炸机): 接管时自动取消反控
            self.teleop_act.setEnabled(link_ok)
            # 模式互斥: 现役模式保持可点 (再点即取消), 另一模式置灰 ——
            # 反控运行中不存在模式切换, 只能先取消再换模式
            self.landing_act.image.setEnabled(
                self.landing_act.image.isChecked())
            self.landing_act.pose.setEnabled(
                self.landing_act.pose.isChecked())
        else:
            self.camera_tab.set_gimbal_locked(False)
            self.link_btn.setEnabled(True)
            self.teleop_act.setEnabled(link_ok)
            self.landing_act.setEnabled(landing_ok)
        self._set_btn_on(self.takeover_btn,
                         self.landing_act.isChecked()
                         or self.teleop_act.isChecked())
        # 航向跟随按钮: 仅手动遥控接管成功 (offboard 通道可用) 且 AirSim
        # 源在线时可开; 反控中云台父面板整体置灰, 此处只管按钮自身
        follow_ok = (self._teleop_live
                     and self.camera_tab.grabber.airsim_connected()
                     and self.camera_tab.grabber.get_gimbal() is not None)
        self.camera_tab.set_follow_available(follow_ok)

    def _on_landing_mode_toggled(self, mode, act=None):
        """视觉反控菜单项勾选/弹起 (模式子菜单两项 / 普通单项共用入口)。"""
        if act is None:
            act = self.landing_act.mode_action(mode)
        if not act.isChecked():
            # 弹起: 模式切换时另一模式已接管 (仍勾选), 不停流程;
            # 真正取消 (无任一模式在运行) 才收尾交还控制权
            if self.landing_act.isChecked():
                return
            self.takeover_btn.setText('接管控制')
            self._stop_landing()
            return
        self._start_landing(mode)

    def _start_landing(self, mode):
        """按所选模式启动视觉反控降落 (image=图像空间 / pose3d=3D 姿态)。"""
        # 互斥: 先弹起另一模式 (其弹起分支见 _on_landing_mode_toggled,
        # 此时本项已勾选不会误停), 再取消遥控, 节点交还 offboard 后反控再接管
        other = self.landing_act.pose if mode == config.LANDING_MODE_IMAGE \
            else self.landing_act.image
        if other.isChecked():
            other.setChecked(False)
        if self.teleop_act.isChecked():
            self.teleop_act.setChecked(False)
        # 反控前自动摆位: 仅俯仰转垂直向下 (俯拍是降落几何前提),
        # 偏航保持用户当前设置不动
        self.camera_tab.grabber.gimbal_pitch_bottom()
        # 链路参数跟随工具栏所选链路 (mavros / uxrce); 模式经 landing_mode
        # 传给降落节点 (节点未声明前被忽略, 接口预留)
        link = self._selected_link() or 'mavros'
        cmd = (list(config.LANDING_CMD)
               + ['--ros-args', '-p', f'link:={link}',
                  '-p', f'landing_mode:={mode}'])
        self.land_proc.start(cmd)
        # 新进程需完成 DDS 发现 + 收首帧飞控状态 (1~3s), 重发多次防吞;
        # 节点对重复 start 幂等
        QTimer.singleShot(1000, lambda: self._send_landing_start(6))

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
            self.takeover_btn.setText('接管控制: 对准下降')
            return
        if status == 'landing':
            self.takeover_btn.setText('接管控制: 自动降落')
            return
        if status == 'landed':
            self.log.info('\033[32mArUco 视觉降落完成, 飞机已上锁\033[0m')
            self._flash_landing_btn('#2d8a3e')      # 绿色高亮闪一下
        elif status == 'idle':
            self.log.info('降落流程已结束 (取消/中止), 按钮已复位')
        else:
            return
        # 菜单项弹起 (checked = 降落进行中) + 文字复位 + 回收降落节点进程
        self.landing_act.setChecked(False)
        self.takeover_btn.setText('接管控制')
        self.land_proc.stop()

    def _flash_landing_btn(self, color):
        """按钮背景色闪烁提示, 1.5s 后恢复原样式。"""
        self.takeover_btn.setStyleSheet(
            self._TOOL_BTN_STYLE +
            f'\nQToolButton {{ background-color: {color}; color: white; }}')
        QTimer.singleShot(1500, lambda: self.takeover_btn.setStyleSheet(
            self._TOOL_BTN_STYLE))

    def _send_landing_start(self, retries_left):
        """触发降落并按 500ms 间隔重发; 菜单项已抬起或节点进程已死则跳过。"""
        if not self.landing_act.isChecked():
            return
        rc = self.land_proc.exited()
        if rc is not None:
            self.log.error(
                f'降落节点进程启动失败 (退出码 {rc}), '
                f'检查编译: colcon build --packages-select flylab_vision')
            self.land_proc.clear()
            self.landing_act.setChecked(False)
            self.takeover_btn.setText('接管控制')
            return
        self.node.publish_cmd(config.LANDING_START_CMD)
        if retries_left > 0:
            QTimer.singleShot(500, lambda: self._send_landing_start(retries_left - 1))

    def _stop_landing(self):
        """取消降落: 发 landing_stop (节点自行切回 LOITER 交还控制权并清标注),
        稍后回收进程, 给节点留出发送停止指令与清屏的时间。"""
        self.node.publish_cmd(config.LANDING_STOP_CMD)
        QTimer.singleShot(500, self.land_proc.stop)

    # ---------- 手动遥控 (执行节点 flylab_vision/teleop_node, 右栏双摇杆) ----------

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

    # ---------- 接管控制 (工具栏下拉菜单 = 视觉反控 + 手动遥控) ----------

    def _build_takeover_button(self):
        """接管控制下拉按钮: 两项均需先开通信链路 (走 offboard), 同一时刻
        只运行一个, 点运行中的项即取消。
        - 视觉反控: 形态随运行中的算法 (_update_landing_menu_shape):
            声明多个 landing_modes (ArUco检测) → 模式子菜单:
              图像空间 = 按图像空间量对准码下降并切 AUTO.LAND
              3D 姿态  = 以码的 3D 位姿 (机体系米级误差) 闭环降落
            其余反控算法 / 无算法 → 普通"视觉反控"单项 (无下级选项)
        - 手动遥控: teleop_node 接管 offboard, 右栏摇杆控速度; 反控中可点
          遥控抢回控制权 (防炸机出口)"""
        self.takeover_btn = QToolButton()
        self.takeover_btn.setText('接管控制')
        self.takeover_btn.setPopupMode(QToolButton.InstantPopup)
        self.takeover_btn.setStyleSheet(self._TOOL_BTN_STYLE)
        self.takeover_btn.setToolTip(
            '接管飞机控制权 (offboard): 视觉反控 = 按视觉输出降落 '
            '(ArUco检测运行时展开 图像空间 / 3D 姿态 模式子菜单); '
            '手动遥控 = 摇杆控速度。均需先开启通信链路')
        menu = QMenu(self.takeover_btn)
        self.takeover_btn.setMenu(menu)
        # 形态一: 视觉反控模式子菜单 (仅声明多个 landing_modes 的算法运行时显示)
        self.landing_menu = menu.addMenu('视觉反控')
        self.landing_img_act = self.landing_menu.addAction('图像空间')
        self.landing_img_act.setCheckable(True)
        self.landing_img_act.setEnabled(False)
        self.landing_img_act.setToolTip(
            '需先: ArUco检测运行中 + 已解锁升空 + 通信链路已开。'
            '按图像空间量 (对中/面积) 对准码下降并自动降落; 再点取消')
        self.landing_img_act.toggled.connect(
            lambda checked: self._on_landing_mode_toggled(
                config.LANDING_MODE_IMAGE))
        self.landing_pose_act = self.landing_menu.addAction('3D 姿态')
        self.landing_pose_act.setCheckable(True)
        self.landing_pose_act.setEnabled(False)
        self.landing_pose_act.setToolTip(
            '需先: ArUco检测运行中 + 已解锁升空 + 通信链路已开 + 码边长准确 '
            '(位姿尺度可信)。按码的 3D 位姿换算机体系米级误差: 水平 P 对中, '
            '下降速度随离码高度调度, 近地自动切 AUTO.LAND; 再点取消')
        self.landing_pose_act.toggled.connect(
            lambda checked: self._on_landing_mode_toggled(
                config.LANDING_MODE_POSE3D))
        # 形态二: 普通"视觉反控"单项 (其余反控算法 / 无算法运行时显示),
        # 按图像模式启动 (模式参数对未实现的节点无感, 见 config 注释)
        self.landing_flat_act = menu.addAction('视觉反控')
        self.landing_flat_act.setCheckable(True)
        self.landing_flat_act.setEnabled(False)
        self.landing_flat_act.setToolTip(
            '需先: 反控算法运行中 + 已解锁升空 + 通信链路已开。'
            '按当前视觉算法的输出反控降落; 再点取消')
        self.landing_flat_act.toggled.connect(
            lambda checked: self._on_landing_mode_toggled(
                config.LANDING_MODE_IMAGE, self.landing_flat_act))
        # 统一句柄 (兼容旧 landing_act 单项语义, 其余引用方零改动)
        self.landing_act = LandingActions(
            self.landing_flat_act, self.landing_img_act, self.landing_pose_act)
        self.teleop_act = menu.addAction('手动遥控')
        self.teleop_act.setCheckable(True)
        self.teleop_act.setEnabled(False)
        self.teleop_act.setToolTip(
            '需先开启通信链路。开启后接管 offboard, 右栏摇杆控速度 '
            '(回中=悬停); 再点交还控制权')
        self.teleop_act.toggled.connect(self._on_teleop_toggle)
        self._update_landing_menu_shape()   # 初始无算法 → 普通单项形态
        return self.takeover_btn

    def _update_landing_menu_shape(self):
        """按运行中的算法切换视觉反控菜单形态 (反控行为因算法而异):
        算法在 config.ALGORITHMS 声明了多个 landing_modes (ArUco检测:
        图像空间/3D 姿态) 时展开模式子菜单; 其余反控算法/无算法运行时
        退化为普通单项。形态切换只影响菜单展示, 勾选状态由统一句柄
        LandingActions 承载, 引用方无感。"""
        algo = self.running_algo
        modes = algo.get('landing_modes') if algo is not None else None
        use_menu = bool(modes) and len(modes) > 1
        self.landing_menu.menuAction().setVisible(use_menu)
        self.landing_flat_act.setVisible(not use_menu)
