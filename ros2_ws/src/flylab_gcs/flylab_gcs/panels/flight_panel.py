#!/usr/bin/env python3
"""飞行链路工具按钮面板 (MainWindow 的功能域 mixin)。

flylab_flight 一键 ros2 launch 整链路 (感知建图/避障/VIO/SLAM), 再点/
切换即停; need_link 的避障链路在通信链路未开启或遥控接管中置灰。
样式常量 _TOOL_BTN_STYLE 定义在 main_window (ui_style.TOOL_BTN_STYLE)。
"""
from PyQt5.QtWidgets import QMenu, QToolButton

from .. import config


class FlightPanelMixin:
    """MainWindow 的飞行链路功能域 (self.* 为主窗口属性)。"""

    # ---------- 飞行链路 (flylab_flight 一键 launch, 工具栏下拉) ----------

    def _build_flight_button(self):
        """飞行链路工具按钮: 下拉选链路即 ros2 launch 整链路 (传感器/
        桥接/算法/跟踪器), 再点运行中的链路即停止, 选其他停旧换新。
        互不携带启停指令 (launch 进程生命周期 = 链路开关)。"""
        self.flight_btn = QToolButton()
        self.flight_btn.setText('飞行链路')
        self.flight_btn.setPopupMode(QToolButton.InstantPopup)
        self.flight_btn.setStyleSheet(self._TOOL_BTN_STYLE)
        self.flight_btn.setToolTip(
            '飞行链路 (flylab_flight): 选择即运行感知建图/避障/VIO/SLAM '
            '整链路; 再点运行中的链路即停止, 选其他停旧换新。'
            '避障/跟踪类链路需先在工具栏开启通信链路')
        menu = QMenu(self.flight_btn)
        self.flight_btn.setMenu(menu)
        menu.aboutToShow.connect(self._rebuild_flight_menu)
        menu.triggered.connect(self._on_flight_action)
        return self.flight_btn

    def _rebuild_flight_menu(self):
        """重建链路菜单: ✓ = 正在运行 (与机型/算法菜单同款交互);
        need_link 的链路 (EGO/SUPER 避障) 在通信链路未开启或手动遥控
        接管中时置灰 —— 跟踪器依赖通信桥, 且与遥控互斥 (都走 offboard)。
        菜单每次展开时重建, 门控状态总是新鲜的。"""
        menu = self.flight_btn.menu()
        menu.clear()
        cur = self.running_flight['name'] if self.running_flight else None
        link_ok = (self._selected_link() is not None
                   and not self.teleop_act.isChecked())
        for link in config.FLIGHT_LINKS:
            act = menu.addAction(link['name'])
            act.setData(link['name'])
            act.setCheckable(True)
            act.setChecked(link['name'] == cur)
            act.setToolTip(' '.join(link['cmd']))
            if link.get('need_link') and not link_ok:
                act.setEnabled(False)
                act.setToolTip('需先开启通信链路, 且手动遥控未接管')

    def _on_flight_action(self, act):
        """菜单点击: 未运行 → 启动; 点运行中的链路 → 停止;
        运行中选其他链路 → 停旧换新。"""
        name = act.data()
        link = next((l for l in config.FLIGHT_LINKS if l['name'] == name), None)
        if link is None:
            return
        if self.running_flight is not None:
            was = self.running_flight        # 先记住 (stop 会清掉)
            self._stop_flight()
            if was is link:                  # 刚才运行的就是它 → 纯停止
                return
        self.running_flight = link
        self.flight_proc.start(link['cmd'])
        self._update_flight_state()

    def _stop_flight(self):
        """停止飞行链路: SIGTERM 整个进程组 (ros2 launch 及其全部子进程,
        含 RViz/跟踪器), 超时 SIGKILL。"""
        link = self.running_flight
        if link is None:
            return
        self.flight_proc.stop()
        self.running_flight = None
        self._update_flight_state()
        self.log.info(f'飞行链路已停止: {link["name"]}')

    def _check_flight_alive(self):
        """轮询: 链路进程异常退出 (launch 失败/崩溃) 则复位按钮并提示。"""
        if self.running_flight is not None and self.flight_proc.exited() is not None:
            rc = self.flight_proc.proc.returncode
            self.log.error(f'飞行链路进程退出 (退出码 {rc}): '
                           f'{self.running_flight["name"]}')
            self.flight_proc.clear()
            self.running_flight = None
            self._update_flight_state()

    def _update_flight_state(self):
        """按钮文字跟随当前运行的链路, 无则回到"飞行链路"; 运行中点亮。"""
        running = self.running_flight['name'] if self.running_flight else None
        self.flight_btn.setText(f'飞行链路: {running}' if running else '飞行链路')
        self._set_btn_on(self.flight_btn, running is not None)
