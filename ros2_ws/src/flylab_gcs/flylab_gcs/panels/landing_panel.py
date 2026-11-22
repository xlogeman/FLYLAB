#!/usr/bin/env python3
"""视觉反控 (ArUco 降落) 面板 (MainWindow 的功能域 mixin)。

接管控制下拉按钮 (工具栏) 的本体在此: 视觉反控 + 云台跟随 + 手动遥控
三块菜单共用一个 QToolButton, 手动遥控的菜单项由 TeleopPanelMixin 提供
(_build_teleop_menu_item), 互斥门控 (_update_takeover_gates) 也在此 ——
门控是一张整体决策表 (谁运行中锁谁、谁是防炸机出口), 拆开反而要重复
分支结构; 遥控专属的启停流程/摇杆/手柄见 teleop_panel.py。目标追踪
反控 (追踪节点接管 offboard 悬停跟拍) 机制同视觉反控降落, 亦挂在
本菜单统一门控。

菜单为常驻双层结构: 逻辑 QAction (checkable, 不进菜单) 承载勾选状态机,
菜单显示项 (非 checkable, 绿点 = 运行中, 再点取消) 经 triggered -> 逻辑项
toggle、逻辑项 changed -> 显示项图标/门控联动; 门控状态由 33ms 轮询落在
逻辑 QAction 的 enabled 上, 菜单展开即见最新。

视觉反控按运行中的算法展开模式子菜单 (反控行为因算法而异): 目前有
ArUco检测 / fractal检测 (enables_landing), 子菜单含 图像空间 / 3D 姿态
两个模式项, 模式经 -p landing_mode 传给降落节点, 检测器类型经 -p detector
随算法传给同一降落节点; 其他算法将来在 config.ALGORITHMS 注册各自的反控
形态。LandingActions 句柄对外保持旧 landing_act 单项语义,
其余引用方 (main_window / vehicle_panel / algo_panel) 无需感知子菜单。
样式常量 _TOOL_BTN_STYLE 定义在 ui_style.py。
"""
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QAction, QMenu, QToolButton

from .. import config
from . import ui_style


def make_action_pair(parent_menu, text, tooltip):
    """双层菜单项: 逻辑 QAction (checkable, 不进菜单, 承载勾选状态机)
    + 菜单显示项 (非 checkable —— 可勾选菜单项会被 Qt 画成空心方块,
    状态改用绿点图标标记)。显示项 triggered -> 逻辑项 toggle。
    视觉反控与手动遥控两个 mixin 共用。"""
    logic = QAction(parent_menu)
    logic.setText(text)
    logic.setCheckable(True)   # 勾选状态机在逻辑项上 (不进菜单)
    logic.setEnabled(False)
    disp = parent_menu.addAction(text)
    disp.setEnabled(False)
    disp.setToolTip(tooltip)
    disp.triggered.connect(logic.toggle)
    return logic, disp


class LandingActions:
    """视觉反控全部 QAction (逻辑三项 + 菜单显示三项) 的统一句柄,
    对外兼容旧 landing_act 单项语义:
    - isChecked() = 任一形态运行中 (读逻辑项)
    - setChecked(False) = 全部弹起 (勾选由用户点选, 不支持 True)
    - setEnabled() = 同步门控到全部逻辑项与显示项
    菜单形态按运行中的算法切换 (_update_landing_menu_shape): 声明多个
    landing_modes 的算法 (ArUco检测) 用模式子菜单, 其余用普通单项。
    显示项的绿点图标由 logic.changed 联动 (_build_takeover_button), 门控
    enabled 双侧同步由本句柄与 changed 各覆盖一半。
    """

    def __init__(self, flat_act, image_act, pose_act,
                 flat_disp, image_disp, pose_disp):
        self.flat = flat_act
        self.image = image_act
        self.pose = pose_act
        self.disp_flat = flat_disp
        self.disp_image = image_disp
        self.disp_pose = pose_disp

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
        self.disp_flat.setEnabled(enabled)
        self.disp_image.setEnabled(enabled)
        self.disp_pose.setEnabled(enabled)


class LandingPanelMixin:
    """MainWindow 的视觉反控功能域 (self.* 为主窗口属性)。"""

    # ---------- 视觉反控 (降落) / 手动遥控 门控 (都走 offboard, 互斥) ----------

    def _update_takeover_gates(self):
        """offboard 功能互斥门控 (33ms 轮询), 门控结果落在菜单项上:
        - 视觉反控: 需已解锁升空 + ArUco检测运行中 + AirSim 源 + 链路已开;
          进行中锁链路与云台, 允许开遥控抢回控制权 (防炸机出口);
          模式子菜单 (图像空间/3D 姿态) 互斥: 运行中非现役模式置灰
        - 目标追踪反控: 需已解锁升空 + 目标追踪算法运行中 + AirSim 源 +
          链路已开 (与视觉反控降落同一门控, 接管 offboard 悬停跟拍);
          进行中锁链路与云台, 允许开遥控抢回 (防炸机出口)
        - 遥控: 仅需链路已开; 进行中仍可点视觉反控/目标追踪反控 (自动让位)
        同时维护接管按钮的状态灯 (任一功能运行中点亮)。"""
        link_ok = self._selected_link() is not None
        algo = self.running_algo
        # 视觉反控/目标追踪反控的完整门控条件 (进行中恒可点以便取消):
        # 都接管 offboard, 需已解锁升空 + AirSim 源 (云台伺服需要)
        landing_ok = False
        takeover_ok = False
        if algo is not None and link_ok:
            air = (self.veh_ctrl.is_flying()
                   and self.camera_tab.grabber.get_gimbal() is not None)
            if algo.get('enables_landing'):
                landing_ok = air
            if algo.get('enables_takeover'):
                takeover_ok = air
        takeover_on = self.gimbal_act.isChecked()
        if self.teleop_act.isChecked():
            self.link_btn.setEnabled(False)
            self.camera_tab.set_gimbal_locked(False)
            # 遥控中仍可点反控: 接管时自动取消遥控 (防炸机出口同理)
            self.landing_act.setEnabled(landing_ok)
            self.gimbal_act.setEnabled(takeover_ok)
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
            self.gimbal_act.setEnabled(False)   # offboard 互斥 (双保险)
        elif takeover_on:
            self.link_btn.setEnabled(False)           # 反控锁链路 (同降落)
            self.camera_tab.set_gimbal_locked(True)   # 云台/变焦交算法
            # 反控中允许开遥控抢回控制权 (防炸机): 接管时自动取消反控
            self.teleop_act.setEnabled(link_ok)
            self.landing_act.setEnabled(False)        # offboard 互斥 (双保险)
        else:
            self.camera_tab.set_gimbal_locked(False)
            self.link_btn.setEnabled(True)
            self.teleop_act.setEnabled(link_ok)
            self.landing_act.setEnabled(landing_ok)
            self.gimbal_act.setEnabled(takeover_ok)
        self._set_btn_on(self.takeover_btn,
                         self.landing_act.isChecked()
                         or self.teleop_act.isChecked()
                         or takeover_on)
        # 航向跟随按钮: 手动遥控接管成功 (offboard 通道可用) 或目标追踪
        # 反控中 (复用为机头跟随开关, 经 /flylab/track_follow 下发节点,
        # offboard 单写者 —— 偏航指令仍由算法注入, 按钮只门控), 均需
        # AirSim 源在线; 视觉反控降落中不可开 (对准码朝向是降落几何前提)。
        # 手动遥控中跟随默认开且锁定不可点 (固有行为, 开关在 camera_widget
        # 按语义落地), 退出接管自动关闭
        airsim_ok = (self.camera_tab.grabber.airsim_connected()
                     and self.camera_tab.grabber.get_gimbal() is not None)
        self.camera_tab.set_track_follow_cb(self.node.publish_track_follow)
        self.camera_tab.set_follow_mode('track' if takeover_on else 'teleop')
        follow_ok = ((self._teleop_live and link_ok) or takeover_on) \
            and airsim_ok
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
        # 传给降落节点; 检测器类型随运行中算法 (fractal检测传 detector:=fractal,
        # aruco 默认不传), 同一降落节点三态状态机共用
        link = self._selected_link() or 'mavros'
        cmd = (list(config.LANDING_CMD)
               + ['--ros-args', '-p', f'link:={link}',
                  '-p', f'landing_mode:={mode}'])
        algo = self.running_algo
        detector = algo.get('landing_detector') if algo is not None else None
        if detector:
            cmd += ['-p', f'detector:={detector}']
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

    # ---------- 目标追踪反控 (追踪节点接管 offboard: 悬停跟拍 + 机头跟随) ----------

    def _on_track_takeover_toggled(self, checked):
        """目标追踪反控开关 (机制同 ArUco 降落反控): 发接管启停指令给
        追踪节点 (track_takeover_start/stop, 经 /flylab/vision_cmd), 节点
        接管 offboard 拿到飞机控制权 (悬停跟拍 + 机头跟随, 云台伺服保持
        目标居中; 飞机跟着目标飞在节点同一反控环扩展), 再点取消或停算法
        才交还。互斥: 先取消遥控/降落 (都走 offboard)。关闭时兜底清零
        云台速率 —— 速率指令是保持型, 节点已死时最后一条非零速率会让
        云台一直转。开启时无需已框选目标: 先开再框选亦可。"""
        if checked:
            if self.teleop_act.isChecked():
                self.teleop_act.setChecked(False)   # 触发遥控交还流程
            if self.landing_act.isChecked():
                self.landing_act.setChecked(False)   # offboard 互斥 (双保险)
                self._stop_landing()
            self.takeover_btn.setText('接管控制: 目标追踪')
            self.node.publish_cmd(config.TRACK_TAKEOVER_START_CMD)
            self.log.info('目标追踪反控: 接管 offboard 悬停跟拍 '
                          '(取消: 再点本项或停止算法即交还控制权)')
        else:
            self.takeover_btn.setText('接管控制')
            self.node.publish_cmd(config.TRACK_TAKEOVER_STOP_CMD)
            self.camera_tab.grabber.note_algo_cmd(0.0, 0.0)   # 兜底清速率

    def _cancel_track_takeover(self):
        """目标追踪反控联动取消 (追踪算法停止/切换/异常退出/开遥控抢回时
        调用): 未开启时无副作用。弹起分支发 stop 指令 (节点切回 LOITER
        交还控制权) 并清零云台速率。"""
        if self.gimbal_act.isChecked():
            self.gimbal_act.setChecked(False)

    def _check_takeover_status(self):
        """目标追踪反控节点状态事件 (engaged/idle/failed, 同降落状态回传
        模式): engaged=接管成功 (按钮文字确认), failed=接管失败 (弹起 +
        报错), idle=已交还 (节点自愈: 上锁/外部切模式/图像断流 → 菜单项
        自动弹起)。状态带事件序号, 只在有新事件时处理。"""
        status, seq = self.node.get_track_status()
        if seq == self._last_takeover_seq:
            return
        self._last_takeover_seq = seq
        if not status:
            return
        if status == 'engaged':
            self.takeover_btn.setText('接管控制: 目标追踪')
            return
        if status == 'failed':
            self.log.error('目标追踪反控: offboard 接管失败 (检查通信链路/'
                           '飞机状态), 已交还控制权')
        # idle / failed: 菜单项弹起复位 (节点已交还, stop 指令幂等)
        if self.gimbal_act.isChecked():
            self.gimbal_act.setChecked(False)

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
          遥控抢回控制权 (防炸机出口)
        - 目标追踪: 追踪节点接管 offboard 悬停跟拍 (云台伺服保持目标居中
          + 机头跟随), 机制同视觉反控降落"""
        self.takeover_btn = QToolButton()
        self.takeover_btn.setText('接管控制')
        self.takeover_btn.setPopupMode(QToolButton.InstantPopup)
        self.takeover_btn.setStyleSheet(self._TOOL_BTN_STYLE)
        self.takeover_btn.setToolTip(
            '接管飞机控制权 (offboard): 视觉反控 = 按视觉输出降落 '
            '(反控算法运行时展开 图像空间 / 3D 姿态 模式子菜单); '
            '手动遥控 = 摇杆控速度; 目标追踪 = 追踪算法悬停跟拍。'
            '均需先开启通信链路')
        menu = QMenu(self.takeover_btn)
        self.takeover_btn.setMenu(menu)

        # 形态一: 视觉反控模式子菜单 (仅声明多个 landing_modes 的算法运行时显示)
        self.landing_menu = menu.addMenu('视觉反控')
        self.landing_img_act, self.landing_img_disp = make_action_pair(
            self.landing_menu, '图像空间',
            '需先: 反控算法运行中 (ArUco检测/fractal检测) + 已解锁升空 '
            '+ 通信链路已开。按图像空间量 (对中/面积) 对准码下降并自动降落; '
            '再点取消')
        self.landing_img_act.toggled.connect(
            lambda checked: self._on_landing_mode_toggled(
                config.LANDING_MODE_IMAGE))
        self.landing_pose_act, self.landing_pose_disp = make_action_pair(
            self.landing_menu, '3D 姿态',
            '需先: 反控算法运行中 + 已解锁升空 + 通信链路已开 + 码边长准确 '
            '(位姿尺度可信)。按码的 3D 位姿换算机体系米级误差: 水平 P 对中, '
            '下降速度随离码高度调度, 近地自动切 AUTO.LAND; 再点取消')
        self.landing_pose_act.toggled.connect(
            lambda checked: self._on_landing_mode_toggled(
                config.LANDING_MODE_POSE3D))
        # 形态二: 普通"视觉反控"单项 (其余反控算法 / 无算法运行时显示),
        # 按图像模式启动 (模式参数对未实现的节点无感, 见 config 注释)
        self.landing_flat_act, self.landing_flat_disp = make_action_pair(
            menu, '视觉反控',
            '需先: 反控算法运行中 + 已解锁升空 + 通信链路已开。'
            '按当前视觉算法的输出反控降落; 再点取消')
        self.landing_flat_act.toggled.connect(
            lambda checked: self._on_landing_mode_toggled(
                config.LANDING_MODE_IMAGE, self.landing_flat_act))
        # 目标追踪反控 (追踪节点接管 offboard): 机制同 ArUco 降落反控 ——
        # 拿到飞机控制权悬停跟拍 (云台伺服保持目标居中 + 机头跟随), 与
        # 手动遥控/视觉反控降落互斥 (都走 offboard), 开遥控可抢回
        self.gimbal_act, self.gimbal_disp = make_action_pair(
            menu, '目标追踪',
            '需先: 目标追踪算法运行中 (yolo26s-track/ORTrack) + 已解锁升空 '
            '+ 通信链路已开。接管 offboard 拿到飞机控制权: 原地悬停跟拍 '
            '(云台伺服保持目标在画面中央 + 机头跟随视线方向, 机头跟随可由'
            '云台面板"跟随"按钮开关, 飞机跟目标飞将在此反控环扩展); '
            '可先开再框选目标; 再点取消或停止算法即交还')
        self.gimbal_act.toggled.connect(self._on_track_takeover_toggled)
        # 手动遥控菜单项 (TeleopPanelMixin, 勾选联动其启停流程)
        self.teleop_act, self.teleop_disp = self._build_teleop_menu_item(menu)
        # 显示项联动: 逻辑项 enabled/checked 任何变化同步到菜单显示项
        # (绿点 = 运行中); 33ms 轮询门控落在逻辑项上, 显示项跟随
        def _sync(logic, disp):
            disp.setEnabled(logic.isEnabled())
            ui_style.set_menu_icon(disp, logic.isChecked())
        for logic, disp in (
                (self.landing_img_act, self.landing_img_disp),
                (self.landing_pose_act, self.landing_pose_disp),
                (self.landing_flat_act, self.landing_flat_disp),
                (self.gimbal_act, self.gimbal_disp),
                (self.teleop_act, self.teleop_disp)):
            logic.changed.connect(lambda l=logic, d=disp: _sync(l, d))
            _sync(logic, disp)
        # 统一句柄 (兼容旧 landing_act 单项语义, 其余引用方零改动)
        self.landing_act = LandingActions(
            self.landing_flat_act, self.landing_img_act, self.landing_pose_act,
            self.landing_flat_disp, self.landing_img_disp,
            self.landing_pose_disp)
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
        self.landing_flat_disp.setVisible(not use_menu)
