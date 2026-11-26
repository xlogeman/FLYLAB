#!/usr/bin/env python3
"""目标追踪反控面板 (MainWindow 的功能域 mixin)。

从 landing_panel 拆出的追踪反控域: "视觉反控"子菜单里的追踪三态
(悬停跟拍 = 追踪节点接管 offboard, 云台伺服保持目标居中; 俯冲打击 =
子菜单, 曲线打击 = 两相制导定速撞击 / 直线打击 = 全程沿视线不分相;
机体跟随 = 子菜单, 水平跟随 = 保持设定水平距离的距离环跟随 [现役] /
直线跟随 = 保持斜距跟随 [预留]), track_takeover_start 带模式后缀切
节点模式; 打击/跟随子项前提 = 悬停跟拍运行中, 四项互斥, 启动时自动
按下云台面板"跟随"按钮 (机头跟随云台视线), 悬停跟拍弹起时联动取消。
本 mixin 提供: 追踪子项菜单构建 (_build_track_menu_items, 由
landing_panel._build_takeover_button 调用)、子模式启停流程、节点状态
回传处理 (_check_takeover_status) 与互斥门控的应用。互斥决策表在
offboard_gates.decide_offboard (视觉反控降落/遥控/追踪三域共用)。
"""
from PyQt5.QtWidgets import QMenu

from .. import config


class TrackTakeoverPanelMixin:
    """MainWindow 的目标追踪反控功能域 (self.* 为主窗口属性)。"""

    # ---------- 菜单构建 (landing_panel._build_takeover_button 调用) ----------

    def _build_track_menu_items(self, landing_menu):
        """在"视觉反控"子菜单下创建追踪反控子项, 返回 (逻辑项, 显示项)
        对列表供显示联动接线。悬停跟拍/俯冲打击/机体跟随(水平) = 现役
        (追踪节点接管 offboard, 机制同降落反控 —— 与手动遥控/视觉反控
        降落互斥, 开遥控可抢回; 模式常量 TRACK_TAKEOVER_*, 打击/跟随
        子项前提 = 悬停跟拍运行中), 机体跟随-直线 = 预留接口。"""
        self.gimbal_act, self.gimbal_disp = self._make_pair(
            landing_menu, '悬停跟拍',
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
        self.dive_menu = QMenu('俯冲打击', landing_menu)
        landing_menu.addMenu(self.dive_menu)
        self.dive_act, self.dive_disp = self._make_pair(
            self.dive_menu, '曲线打击',
            '需先: 悬停跟拍运行中 (上级菜单"悬停跟拍"项)。两相制导沿云台 '
            '目标方向撞击: 拉角段垂直压满收敛视线角, 进撞击带后全速收口 '
            '(漏斗曲线, 末段全速); 与直线打击/机体跟随互斥; 再点本项切回 '
            '悬停跟拍')
        self.dive_act.toggled.connect(
            lambda checked: self._on_track_submode_toggled(
                '曲线打击', self.dive_act, config.TRACK_TAKEOVER_DIVE))
        self.straight_act, self.straight_disp = self._make_pair(
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
        self.bodyfollow_menu = QMenu('机体跟随', landing_menu)
        landing_menu.addMenu(self.bodyfollow_menu)
        self.bodyfollow_act, self.bodyfollow_disp = self._make_pair(
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
            self._make_pair(
                self.bodyfollow_menu, '直线跟随',
                '需先: 悬停跟拍运行中。机体保持与目标的设定直线距离 (斜距) '
                '跟随目标移动 (预留接口, 飞控行为待实现); 与其他子模式互斥')
        self.bodyfollow_slant_act.toggled.connect(
            lambda checked: self._on_track_submode_toggled(
                '直线跟随', self.bodyfollow_slant_act))
        return [(self.gimbal_act, self.gimbal_disp),
                (self.dive_act, self.dive_disp),
                (self.straight_act, self.straight_disp),
                (self.bodyfollow_act, self.bodyfollow_disp),
                (self.bodyfollow_slant_act, self.bodyfollow_slant_disp)]

    @staticmethod
    def _make_pair(parent_menu, text, tooltip):
        """双层菜单项构建 (委托 landing_panel.make_action_pair, 运行时
        由 MRO 解析 —— 两域共用同一工具函数与绿点联动约定)。"""
        from .landing_panel import make_action_pair
        return make_action_pair(parent_menu, text, tooltip)

    # ---------- 门控应用 (决策表 offboard_gates.decide_offboard) ----------

    def track_stub_actions(self):
        """打击/跟随四个子模式逻辑项 (互斥组 + 门控/取消遍历用)。"""
        return (self.dive_act, self.straight_act,
                self.bodyfollow_act, self.bodyfollow_slant_act)

    def track_actions(self):
        """目标追踪反控全部逻辑项 (悬停跟拍 + 四子项)。"""
        return (self.gimbal_act,) + self.track_stub_actions()

    def offboard_busy(self):
        """任一 offboard 功能 (遥控/视觉反控/目标追踪反控) 运行中:
        飞行控制命令会打断接管, 主窗口飞控按钮门控复用本判断
        (互斥条件的单一出处, 原来在 main_window 抄了三份)。"""
        return (self.teleop_act.isChecked()
                or self.landing_act.isChecked()
                or any(a.isChecked() for a in self.track_actions()))

    def _apply_track_gates(self, decision):
        """把决策表的追踪域门控落到逻辑项上 (显示项经 changed 联动,
        此处只落逻辑项)。"""
        self.gimbal_act.setEnabled(decision.gimbal_act_enabled)
        for stub in self.track_stub_actions():
            stub.setEnabled(decision.stub_enabled)

    # ---------- 子模式启停流程 ----------

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
            elif not any(a.isChecked() for a in self.track_stub_actions()):
                self.takeover_btn.setText('接管控制')
            return
        if not self.gimbal_act.isChecked():
            act.setChecked(False)   # toggled 再入 (弹起分支), 无副作用
            self.log.warning(f'{name}: 需先开启目标追踪 (悬停跟拍运行中才可点)')
            return
        for other in self.track_stub_actions():
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
            hint = ('保持设定水平距离跟随目标'
                    if mode == config.TRACK_TAKEOVER_BODYFOLLOW else
                    '沿云台目标方向撞击')
            self.log.info(f'{name}: {hint} (取消: 再点本项切回悬停跟拍)')

    def _on_track_takeover_toggled(self, checked):
        """目标追踪反控开关 (机制同 ArUco 降落反控): 发接管启停指令给
        追踪节点 (track_takeover_start/stop, 经 /flylab/vision_cmd), 节点
        接管 offboard 拿到飞机控制权 (悬停跟拍 + 机头跟随, 云台伺服保持
        目标居中; 机体跟目标飞经"机体跟随"子菜单), 再点取消或停算法
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
            for stub in self.track_stub_actions():
                stub.setChecked(False)
            self.takeover_btn.setText('接管控制')
            self.node.publish_cmd(config.TRACK_TAKEOVER_STOP_CMD)
            self.camera_tab.grabber.note_algo_cmd(0.0, 0.0)   # 兜底清速率

    def _cancel_track_takeover(self):
        """目标追踪反控联动取消 (追踪算法停止/切换/异常退出时调用):
        未开启时无副作用。悬停跟拍弹起分支发 stop 指令 (节点切回 LOITER
        交还控制权) 并清零云台速率; 子模式项幂等弹起 (直线跟随无节点
        流程, 弹起分支只在无其他模式运行中时复位按钮文字)。"""
        for act in self.track_actions():
            act.setChecked(False)

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
