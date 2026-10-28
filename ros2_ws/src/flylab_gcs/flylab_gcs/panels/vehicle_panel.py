#!/usr/bin/env python3
"""飞行控制工具按钮面板 (MainWindow 的功能域 mixin)。

AirSim 飞控命令菜单 (解锁/起飞/降落/悬停/返航/上锁), RPC 直发 PX4,
与 offboard 跟踪器相互独立; 可用性在 33ms 轮询里同步 (_poll_once)。
样式常量 _TOOL_BTN_STYLE 定义在 main_window (ui_style.TOOL_BTN_STYLE)。
"""
from PyQt5.QtWidgets import QMenu, QToolButton

from .. import config


class VehiclePanelMixin:
    """MainWindow 的飞行控制功能域 (self.* 为主窗口属性)。"""

    # ---------- 飞行控制 (AirSim API: 解锁/起飞/降落/悬停/返航/上锁) ----------

    def _build_vehicle_button(self):
        """飞行控制工具按钮: 弹出 AirSim 飞控命令菜单 (RPC 直发 PX4)。
        仅飞控 RPC 会话已连通时可点 (33ms 轮询同步可用性)。"""
        self.veh_btn = QToolButton()
        self.veh_btn.setText('飞行控制')
        self.veh_btn.setPopupMode(QToolButton.InstantPopup)
        self.veh_btn.setStyleSheet(self._TOOL_BTN_STYLE)
        self.veh_btn.setToolTip(
            'AirSim 飞控命令: 解锁/起飞/降落/悬停/返航/上锁 '
            '(需先启动 PX4 SITL 且 UE 端 AirSim 已连通; 异步下发, 结果看 QGC / mavros state)')
        menu = QMenu(self.veh_btn)
        self.veh_btn.setMenu(menu)
        menu.aboutToShow.connect(self._rebuild_vehicle_menu)
        menu.triggered.connect(self._on_vehicle_action)
        return self.veh_btn

    def _rebuild_vehicle_menu(self):
        """重建飞控命令菜单: RPC 未连通时整组置灰; 手动遥控/视觉反控
        进行中同样置灰 (offboard 接管期间不被飞控命令打断, 想干预先
        点"开启遥控"/取消反控抢回)。"""
        menu = self.veh_btn.menu()
        menu.clear()
        ok = (self.veh_ctrl.is_connected()
              and not self.teleop_act.isChecked()
              and not self.landing_act.isChecked())
        for item in config.VEHICLE_CMDS:
            if item['cmd'] == 'disarm':
                menu.addSeparator()      # 上锁前分隔 (飞行中误点会掉桨)
            act = menu.addAction(item['name'])
            act.setData(item['cmd'])
            act.setEnabled(ok)

    def _on_vehicle_action(self, act):
        """菜单点击: 命令置入 VehicleController 队列 (独立线程执行)。"""
        key = act.data()
        if key is None:
            return
        self.veh_ctrl.request(key)
