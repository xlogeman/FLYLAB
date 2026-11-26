#!/usr/bin/env python3
"""视觉反控 (ArUco 降落) 面板 (MainWindow 的功能域 mixin)。

接管控制下拉按钮 (工具栏) 的本体在此: 视觉反控 + 手动遥控两项菜单
共用一个 QToolButton, 手动遥控的菜单项由 TeleopPanelMixin 提供
(_build_teleop_menu_item), 互斥门控 (_update_takeover_gates) 也在此 ——
门控是一张整体决策表 (谁运行中锁谁、谁是防炸机出口), 拆开反而要重复
分支结构; 遥控专属的启停流程/摇杆/手柄见 teleop_panel.py。目标追踪
反控挂在"视觉反控"子菜单的追踪项: 悬停跟拍 = 追踪节点接管 offboard
(云台伺服保持目标居中), 俯冲打击 = 子菜单 (曲线打击 = 两相制导定速撞击 / 直线打击 =
全程沿视线不分相, track_takeover_start 带模式后缀切节点模式), 机体跟随 =
子菜单 (水平跟随 = 保持设定水平距离的距离环跟随 [现役] / 直线跟随 = 保持
斜距跟随 [预留]); 前提 = 悬停跟拍运行中, 互斥, 启动时自动按下云台面板"跟随"
按钮 (机头跟随云台视线), 悬停跟拍弹起时联动取消, 亦在本菜单统一门控。

菜单为常驻双层结构: 逻辑 QAction (checkable, 不进菜单) 承载勾选状态机,
菜单显示项 (非 checkable, 绿点 = 运行中, 再点取消) 经 triggered -> 逻辑项
toggle、逻辑项 changed -> 显示项图标/门控联动; 门控状态由 33ms 轮询落在
逻辑 QAction 的 enabled 上, 菜单展开即见最新。

"视觉反控"子菜单形态随运行中的算法两选一显隐 (_update_landing_menu_
shape): 码检测 (enables_landing 且声明多个 landing_modes, ArUco检测/
fractal检测) → 模式子项 图像空间/3D 姿态, 模式经 -p landing_mode 传给
降落节点, 检测器类型经 -p detector 随算法传给同一降落节点; 目标追踪
(enables_takeover) → 追踪三态子项; 无适用子项 (其余反控算法/无算法)
时子项全隐、子菜单置灰不可展开。其他算法将来在 config.ALGORITHMS
注册各自的反控形态。LandingActions 句柄对外保持旧 landing_act 单项
语义, 其余引用方 (main_window / vehicle_panel / algo_panel) 无需感知
子菜单。
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
    """视觉反控全部 QAction (逻辑两项 + 菜单显示两项) 的统一句柄,
    对外兼容旧 landing_act 单项语义:
    - isChecked() = 任一模式运行中 (读逻辑项)
    - setChecked(False) = 全部弹起 (勾选由用户点选, 不支持 True)
    - setEnabled() = 同步门控到全部逻辑项与显示项
    子菜单形态按运行中的算法切换 (_update_landing_menu_shape): 声明多个
    landing_modes 的算法 (码检测) 用模式子项, enables_takeover (目标
    追踪) 用追踪三态子项; 无适用子项时子菜单置灰不可展开。
    显示项的绿点图标由 logic.changed 联动 (_build_takeover_button), 门控
    enabled 双侧同步由本句柄与 changed 各覆盖一半。
    """

    def __init__(self, image_act, pose_act, image_disp, pose_disp):
        self.image = image_act
        self.pose = pose_act
        self.disp_image = image_disp
        self.disp_pose = pose_disp

    def mode_action(self, mode):
        """模式常量 -> 对应菜单项。"""
        return self.pose if mode == config.LANDING_MODE_POSE3D else self.image

    def isChecked(self):
        return self.image.isChecked() or self.pose.isChecked()

    def setChecked(self, checked):
        if checked:
            raise ValueError('视觉反控模式由用户点选, 不支持外部 setChecked(True)')
        self.image.setChecked(False)
        self.pose.setChecked(False)

    def setEnabled(self, enabled):
        self.image.setEnabled(enabled)
        self.pose.setEnabled(enabled)
        self.disp_image.setEnabled(enabled)
        self.disp_pose.setEnabled(enabled)


class LandingPanelMixin:
    """MainWindow 的视觉反控功能域 (self.* 为主窗口属性)。"""

    # ---------- 视觉反控 (降落) / 手动遥控 门控 (都走 offboard, 互斥) ----------

    def _set_track_mode_gates(self, enabled):
        """目标追踪子菜单门控: 悬停跟拍按 offboard 门控 (追踪算法运行中 +
        已解锁升空 + AirSim 源 + 链路已开); 打击/跟随子项前提 = 悬停跟拍
        运行中 (悬停跟拍未勾选时恒置灰, 见 _set_track_stub_gates)。
        显示项 enabled 由 logic.changed -> _sync 联动, 此处只落逻辑项。"""
        self.gimbal_act.setEnabled(enabled)
        self._set_track_stub_gates()

    def _set_track_stub_gates(self):
        """曲线/直线打击/机体跟随两项可用性: 前提 = 悬停跟拍运行中
        (gimbal_act 勾选)。互斥切换/启停动作在点击流程
        (_on_track_submode_toggled), 此处只管可用性。"""
        stub_ok = self.gimbal_act.isChecked()
        self.dive_act.setEnabled(stub_ok)
        self.straight_act.setEnabled(stub_ok)
        self.bodyfollow_act.setEnabled(stub_ok)
        self.bodyfollow_slant_act.setEnabled(stub_ok)

    def _update_takeover_gates(self):
        """offboard 功能互斥门控 (33ms 轮询), 门控结果落在菜单项上:
        - 视觉反控: 需已解锁升空 + 码检测运行中 + AirSim 源 + 链路已开;
          进行中锁链路与云台, 允许开遥控抢回控制权 (防炸机出口);
          模式子菜单 (图像空间/3D 姿态) 互斥: 运行中非现役模式置灰
        - 目标追踪反控: 需已解锁升空 + 目标追踪算法运行中 + AirSim 源 +
          链路已开 (与视觉反控降落同一门控, 接管 offboard 悬停跟拍);
          进行中锁链路与云台, 允许开遥控抢回 (防炸机出口)。视觉反控
          子菜单的追踪三态统一门控与互斥; 预留接口 (俯冲打击/机体跟随)
          另有前提 = 悬停跟拍运行中, 悬停跟拍弹起时运行中的预留项自动取消
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
        # 联动取消: 悬停跟拍弹起 (用户取消/节点交还/算法停止) 时, 运行中
        # 的俯冲打击/机体跟随自动取消 (前提失效, 正常路径已在 toggled
        # 流程联动, 此处兜底)
        if not self.gimbal_act.isChecked():
            for stub in (self.dive_act, self.straight_act,
                         self.bodyfollow_act, self.bodyfollow_slant_act):
                if stub.isChecked():
                    stub.setChecked(False)
        # 目标追踪任一模式 (悬停跟拍/曲线/直线打击/机体跟随两项) 运行中:
        # 统一按 offboard 反控处理 (锁链路/云台, 允许遥控抢回, 状态灯点亮)
        takeover_on = (self.gimbal_act.isChecked()
                       or self.dive_act.isChecked()
                       or self.straight_act.isChecked()
                       or self.bodyfollow_act.isChecked()
                       or self.bodyfollow_slant_act.isChecked())
        if self.teleop_act.isChecked():
            self.link_btn.setEnabled(False)
            self.camera_tab.set_gimbal_locked(False)
            # 遥控中仍可点反控: 接管时自动取消遥控 (防炸机出口同理)
            self.landing_act.setEnabled(landing_ok)
            self._set_track_mode_gates(takeover_ok)
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
            self._set_track_mode_gates(False)   # offboard 互斥 (双保险)
        elif takeover_on:
            self.link_btn.setEnabled(False)           # 反控锁链路 (同降落)
            self.camera_tab.set_gimbal_locked(True)   # 云台/变焦交算法
            # 反控中允许开遥控抢回控制权 (防炸机): 接管时自动取消反控
            self.teleop_act.setEnabled(link_ok)
            self.landing_act.setEnabled(False)        # offboard 互斥 (双保险)
            self._set_track_stub_gates()   # 悬停跟拍运行中 → 预留项可点
        else:
            self.camera_tab.set_gimbal_locked(False)
            self.link_btn.setEnabled(True)
            self.teleop_act.setEnabled(link_ok)
            self.landing_act.setEnabled(landing_ok)
            self._set_track_mode_gates(takeover_ok)
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

    def _on_landing_mode_toggled(self, mode):
        """视觉反控菜单项勾选/弹起 (模式子菜单两项共用入口)。"""
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
        self.land_proc.start(cmd, capture_output=True)
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

    def _on_track_submode_toggled(self, name, act, mode=None):
        """目标追踪子模式 (俯冲打击两项/机体跟随两项) 勾选/弹起: 前提 =
        悬停跟拍运行中 (gimbal_act 勾选, 不满足则拒绝并弹起); 四项互斥,
        一项运行中点另一项即取消当前项再启动新项。现役项勾选发
        "track_takeover_start <mode>" (节点切模式, 接管不断), 弹起且
        悬停跟拍仍在 → 发 start 不带模式切回悬停跟拍 (互斥切换时后启动项
        随后带模式覆盖, 指令顺序最终正确); 悬停跟拍弹起的联动弹起不发指令
        (stop 由其弹起分支统一发)。启动时自动按下云台面板"跟随"按钮
        (打击方位收口靠机头跟随, 已开则不重复发布)。mode=None = 预留接口
        (直线跟随, 不发模式串, 仅占位)。"""
        if not act.isChecked():
            # 弹起 (用户取消/互斥切换/联动取消): 悬停跟拍仍在运行则切回
            # 其模式, 全部结束才复位
            if self.gimbal_act.isChecked():
                self.takeover_btn.setText('接管控制: 目标追踪')
                if mode is not None:
                    self.node.publish_cmd(config.TRACK_TAKEOVER_START_CMD)
            elif not (self.dive_act.isChecked()
                      or self.straight_act.isChecked()
                      or self.bodyfollow_act.isChecked()
                      or self.bodyfollow_slant_act.isChecked()):
                self.takeover_btn.setText('接管控制')
            return
        if not self.gimbal_act.isChecked():
            act.setChecked(False)   # toggled 再入 (弹起分支), 无副作用
            self.log.warning(f'{name}: 需先开启目标追踪 (悬停跟拍运行中才可点)')
            return
        for other in (self.dive_act, self.straight_act, self.bodyfollow_act,
                      self.bodyfollow_slant_act):
            if other is not act:
                other.setChecked(False)   # 互斥: 弹起运行中的其他项
        # 自动按下"跟随"按钮 (机头跟随云台视线); 已开则无动作不重复发布
        if not self.camera_tab.btn_follow.isChecked():
            self.camera_tab.btn_follow.setChecked(True)
        self.takeover_btn.setText(f'接管控制: {name}')
        if mode is None:
            self.log.info(f'{name}: 已启动, "跟随"已自动按下 (接口预留, '
                          '行为待实现)')
        else:
            self.node.publish_cmd(
                f'{config.TRACK_TAKEOVER_START_CMD} {mode}')
            hint = ('保持设定水平距离跟随目标' if name == '水平跟随' else
                    '沿云台目标方向撞击')
            self.log.info(f'{name}: {hint} (取消: 再点本项切回悬停跟拍)')

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
            # 同步按下"跟随"按钮 (机头跟随默认开): 不等 33ms 轮询, 与俯冲/
            # 机体跟随子项同款 —— 否则轮询瞬间若恰逢 AirSim 重连窗口, 按下
            # 后会被可用性门控弹回且无人补发。此刻语义可能还是 teleop,
            # toggled 先走抓取器 (无副作用), 轮询切 track 语义时会下发 True
            if not self.camera_tab.btn_follow.isChecked():
                self.camera_tab.btn_follow.setChecked(True)
            self.takeover_btn.setText('接管控制: 目标追踪')
            self.node.publish_cmd(config.TRACK_TAKEOVER_START_CMD)
            self.log.info('目标追踪反控: 接管 offboard 悬停跟拍 '
                          '(取消: 再点本项或停止算法即交还控制权)')
        else:
            # 前提失效联动: 悬停跟拍取消/交还时弹起四个子模式项 (其弹起
            # 分支只复位文字, 本分支随后统一复位)
            self.dive_act.setChecked(False)
            self.straight_act.setChecked(False)
            self.bodyfollow_act.setChecked(False)
            self.bodyfollow_slant_act.setChecked(False)
            self.takeover_btn.setText('接管控制')
            self.node.publish_cmd(config.TRACK_TAKEOVER_STOP_CMD)
            self.camera_tab.grabber.note_algo_cmd(0.0, 0.0)   # 兜底清速率

    def _cancel_track_takeover(self):
        """目标追踪反控联动取消 (追踪算法停止/切换/异常退出时调用):
        未开启时无副作用。悬停跟拍弹起分支发 stop 指令 (节点切回 LOITER
        交还控制权) 并清零云台速率; 预留接口两项幂等弹起 (无节点流程,
        弹起分支只在无其他模式运行中时复位按钮文字)。"""
        if self.gimbal_act.isChecked():
            self.gimbal_act.setChecked(False)
        self.dive_act.setChecked(False)
        self.straight_act.setChecked(False)
        self.bodyfollow_act.setChecked(False)
        self.bodyfollow_slant_act.setChecked(False)

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
            self.takeover_btn.setText(
                '接管控制: 曲线打击' if self.dive_act.isChecked() else
                '接管控制: 直线打击' if self.straight_act.isChecked() else
                '接管控制: 水平跟随' if self.bodyfollow_act.isChecked() else
                '接管控制: 直线跟随'
                if self.bodyfollow_slant_act.isChecked() else
                '接管控制: 目标追踪')
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
        - 视觉反控: 子菜单形态随运行中的算法 (_update_landing_menu_shape):
            码检测 (声明多个 landing_modes) → 模式子项:
              图像空间 = 按图像空间量对准码下降并切 AUTO.LAND
              3D 姿态  = 以码的 3D 位姿 (机体系米级误差) 闭环降落
            目标追踪算法 (enables_takeover) → 追踪子项:
              悬停跟拍 = 云台伺服保持目标居中 + 机头跟随
              俯冲打击 = 子菜单: 曲线打击 (两相制导) / 直线打击 (全程沿视线);
              机体跟随 = 子菜单: 水平跟随 (保持设定水平距离) /
              直线跟随 (预留接口)
            无适用子项 (其余反控算法/无算法) → 子项全隐, 子菜单置灰不可点
        - 手动遥控: teleop_node 接管 offboard, 右栏摇杆控速度; 反控中可点
          遥控抢回控制权 (防炸机出口)"""
        self.takeover_btn = QToolButton()
        self.takeover_btn.setText('接管控制')
        self.takeover_btn.setPopupMode(QToolButton.InstantPopup)
        self.takeover_btn.setStyleSheet(self._TOOL_BTN_STYLE)
        self.takeover_btn.setToolTip(
            '接管飞机控制权 (offboard): 视觉反控 = 子菜单随运行中算法切换 '
            '(码检测: 图像空间/3D 姿态; 目标追踪: 悬停跟拍/俯冲打击子菜单 '
            '[曲线/直线打击]/机体跟随子菜单 [水平/直线跟随]); '
            '手动遥控 = 摇杆控速度。均需先开启通信链路')
        menu = QMenu(self.takeover_btn)
        self.takeover_btn.setMenu(menu)

        # 视觉反控子菜单 (常驻顶级, 子项形态随运行中算法两态显隐,
        # 无适用子项时整项置灰不可展开)
        self.landing_menu = menu.addMenu('视觉反控')
        # 形态一: 码检测模式子项 (声明多个 landing_modes 的算法运行时显示)
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
        # 形态二: 目标追踪子项 (enables_takeover 算法运行时显示):
        # 悬停跟拍/俯冲打击/机体跟随(水平) = 现役 (追踪节点接管 offboard,
        # 机制同降落反控 —— 与手动遥控/视觉反控降落互斥, 开遥控可抢回;
        # 模式常量 TRACK_TAKEOVER_*, 打击/跟随子项前提 = 悬停跟拍运行中),
        # 机体跟随-直线 = 预留接口
        self.gimbal_act, self.gimbal_disp = make_action_pair(
            self.landing_menu, '悬停跟拍',
            '需先: 目标追踪算法运行中 (yolo26s-track/ORTrack) + 已解锁升空 '
            '+ 通信链路已开。接管 offboard 拿到飞机控制权: 原地悬停跟拍 '
            '(云台伺服保持目标在画面中央 + 机头跟随视线方向, 机头跟随可由'
            '云台面板"跟随"按钮开关, 机体可经"机体跟随"子菜单保持距离'
            '跟随目标); '
            '可先开再框选目标; 再点取消或停止算法即交还')
        self.gimbal_act.toggled.connect(self._on_track_takeover_toggled)
        # 俯冲打击 = 子菜单, 两个下级项互斥 (同一打击状态机的两种制导律):
        # 曲线打击 = 两相制导 (拉角段收敛到撞击带, 漏斗曲线, 末段全速);
        # 直线打击 = 全程沿视线不分相 (合速度 min(V, D/sinθ), 陡角自动降速)
        self.dive_menu = self.landing_menu.addMenu('俯冲打击')
        self.dive_act, self.dive_disp = make_action_pair(
            self.dive_menu, '曲线打击',
            '需先: 悬停跟拍运行中 (上级菜单"悬停跟拍"项)。两相制导沿云台 '
            '目标方向撞击: 拉角段垂直压满收敛视线角, 进撞击带后全速收口 '
            '(漏斗曲线, 末段全速); 与直线打击/机体跟随互斥; 再点本项切回 '
            '悬停跟拍')
        self.dive_act.toggled.connect(
            lambda checked: self._on_track_submode_toggled(
                '曲线打击', self.dive_act, config.TRACK_TAKEOVER_DIVE))
        self.straight_act, self.straight_disp = make_action_pair(
            self.dive_menu, '直线打击',
            '需先: 悬停跟拍运行中 (上级菜单"悬停跟拍"项)。全程沿云台目标 '
            '视线方向飞, 不分相: 合速度 = min(撞击速度, 垂直域/sinθ), 航迹 '
            '始终贴合视线近似直线收口, 陡角时自动降速; 与曲线打击/机体跟随 '
            '互斥; 再点本项切回悬停跟拍')
        self.straight_act.toggled.connect(
            lambda checked: self._on_track_submode_toggled(
                '直线打击', self.straight_act, config.TRACK_TAKEOVER_STRAIGHT))
        # 机体跟随 = 子菜单, 两个下级项互斥: 水平跟随 = 保持设定水平距离
        # 的距离环跟随 (现役, 保持高度环俯视角度); 直线跟随 = 保持设定
        # 斜距跟随 (预留接口, 飞控行为待实现)
        self.bodyfollow_menu = self.landing_menu.addMenu('机体跟随')
        self.bodyfollow_act, self.bodyfollow_disp = make_action_pair(
            self.bodyfollow_menu, '水平跟随',
            '需先: 悬停跟拍运行中 (上级菜单"悬停跟拍"项)。机体保持与目标的 '
            '设定水平距离跟随目标移动: 水平速度按水平距离误差 P 律收放, '
            '方向沿视线方位, 高度环维持俯视角度; 与打击项/直线跟随互斥 '
            '(点另一项自动取消本项); 启动时自动按下云台面板"跟随"按钮 '
            '(飞机跟着云台转); 再点本项切回悬停跟拍')
        self.bodyfollow_act.toggled.connect(
            lambda checked: self._on_track_submode_toggled(
                '水平跟随', self.bodyfollow_act,
                config.TRACK_TAKEOVER_BODYFOLLOW))
        self.bodyfollow_slant_act, self.bodyfollow_slant_disp = \
            make_action_pair(
                self.bodyfollow_menu, '直线跟随',
                '需先: 悬停跟拍运行中。机体保持与目标的设定直线距离 (斜距) '
                '跟随目标移动 (预留接口, 飞控行为待实现); 与其他子模式互斥')
        self.bodyfollow_slant_act.toggled.connect(
            lambda checked: self._on_track_submode_toggled(
                '直线跟随', self.bodyfollow_slant_act))
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
                (self.gimbal_act, self.gimbal_disp),
                (self.dive_act, self.dive_disp),
                (self.straight_act, self.straight_disp),
                (self.bodyfollow_act, self.bodyfollow_disp),
                (self.bodyfollow_slant_act, self.bodyfollow_slant_disp),
                (self.teleop_act, self.teleop_disp)):
            logic.changed.connect(lambda l=logic, d=disp: _sync(l, d))
            _sync(logic, disp)
        # 统一句柄 (兼容旧 landing_act 单项语义, 其余引用方零改动)
        self.landing_act = LandingActions(
            self.landing_img_act, self.landing_pose_act,
            self.landing_img_disp, self.landing_pose_disp)
        self._update_landing_menu_shape()   # 初始无算法 → 子菜单置灰不可点
        return self.takeover_btn

    def _update_landing_menu_shape(self):
        """按运行中的算法切换"视觉反控"子菜单形态 (反控行为因算法而异,
        两种形态互斥显隐, 子菜单本体常驻):
        - 码检测: 声明多个 landing_modes (ArUco/fractal检测) → 模式子项
          (图像空间/3D 姿态)
        - 目标追踪: 声明 enables_takeover (yolo26s-track/ORTrack) →
          追踪子项 (悬停跟拍/俯冲打击子菜单 [曲线打击/直线打击]/
          机体跟随子菜单 [水平跟随/直线跟随])
        无适用子项 (其余反控算法/无算法运行) 时子项全隐、子菜单整项置灰
        不可展开 (不隐藏)。形态切换只影响菜单展示, 勾选状态由各自逻辑项
        承载 (追踪子模式见 gimbal_act/dive_act/straight_act/bodyfollow_act/
        bodyfollow_slant_act), 引用方无感。"""
        algo = self.running_algo
        modes = algo.get('landing_modes') if algo is not None else None
        track = bool(algo.get('enables_takeover')) \
            if algo is not None else False
        use_modes = bool(modes) and len(modes) > 1
        self.landing_img_disp.setVisible(use_modes)
        self.landing_pose_disp.setVisible(use_modes)
        for disp in (self.gimbal_disp, self.dive_disp,
                     self.straight_disp, self.bodyfollow_disp,
                     self.bodyfollow_slant_disp):
            disp.setVisible(track)
        # 俯冲打击/机体跟随子菜单本体 (下级项的容器) 随形态显隐
        self.dive_menu.menuAction().setVisible(track)
        self.bodyfollow_menu.menuAction().setVisible(track)
        # 子菜单常驻: 有适用子项才可展开, 其余反控算法/无算法置灰不可点
        self.landing_menu.menuAction().setEnabled(use_modes or track)
