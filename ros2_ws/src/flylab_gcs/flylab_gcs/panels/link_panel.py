#!/usr/bin/env python3
"""通信链路工具按钮面板 (MainWindow 的功能域 mixin)。

uXRCE-DDS / MAVROS 互斥起停 (一次只开一条), 进程异常退出自动复位;
所选链路作为降落/遥控节点的 link 参数。
样式常量 _TOOL_BTN_STYLE 定义在 main_window (ui_style.TOOL_BTN_STYLE)。
"""
from PyQt5.QtWidgets import QMenu, QToolButton

from .. import config
from ..processes import ManagedProcess


class LinkPanelMixin:
    """MainWindow 的通信链路功能域 (self.* 为主窗口属性)。"""

    # ---------- 通信链路 (工具栏下拉按钮: 菜单勾选起停) ----------

    _LINKS = (  # (key, 菜单显示名, 启动命令)
        ('uxrce', 'uXRCE-DDS', config.UXRCE_CMD),
        ('mavros', 'MAVROS', config.MAVROS_CMD),
    )

    def _build_link_button(self):
        """通信链路工具按钮: 点击弹出链路菜单 (✓=开启中), 点已勾选项即停,
        点其他项先停旧的再开 (互斥)。按钮文字跟随当前链路。"""
        self.link_btn = QToolButton()
        self.link_btn.setText('通信链路')
        self.link_btn.setPopupMode(QToolButton.InstantPopup)
        self.link_btn.setStyleSheet(self._TOOL_BTN_STYLE)
        self.link_btn.setToolTip(
            '起/停通信链路 (互斥): uXRCE-DDS Agent / MAVROS 桥, '
            '悬停菜单项可看启动命令')
        menu = QMenu(self.link_btn)
        self.link_btn.setMenu(menu)
        menu.aboutToShow.connect(self._rebuild_link_menu)
        menu.triggered.connect(self._on_link_action)
        return self.link_btn

    def _rebuild_link_menu(self):
        """重建链路菜单: 每条链路一个勾选项 (✓=开启中)。"""
        menu = self.link_btn.menu()
        menu.clear()
        running = self._selected_link()
        for key, text, cmd in self._LINKS:
            act = menu.addAction(text)
            act.setData(key)
            act.setCheckable(True)
            act.setChecked(key == running)
            act.setToolTip(' '.join(cmd))

    def _on_link_action(self, act):
        """菜单点击: 已开启的链路点击即停; 选其他链路先停旧的再开 (互斥)。"""
        key = act.data()
        entry = next((e for e in self._LINKS if e[0] == key), None)
        if entry is None:
            return
        _, title, cmd = entry
        if self._selected_link() == key:
            self._stop_link(key, title)
            return
        for other in [k for k in self.link_procs if k != key]:
            self._stop_link(other)
        self._start_link(key, cmd, title)

    def _start_link(self, key, cmd, title):
        proc = ManagedProcess(self.log, title)
        if not proc.start(cmd):
            self._update_link_btn_text()
            return
        self.link_procs[key] = proc
        self._update_link_btn_text()

    def _stop_link(self, key, title=None):
        proc = self.link_procs.pop(key, None)
        self._update_link_btn_text()
        if proc is None:
            return
        proc.stop()
        self.log.info(f'通信链路已停止: {title or key}')

    def _check_link_alive(self):
        """轮询: 桥进程异常退出则复位按钮 (用户可重新点起)。"""
        for key, proc in list(self.link_procs.items()):
            rc = proc.exited()
            if rc is not None:
                self.log.error(
                    f'通信链路进程退出 (退出码 {rc}), 请重新点击启动')
                del self.link_procs[key]
                self._update_link_btn_text()

    def _selected_link(self):
        """当前开启的通信链路 (uXRCE-DDS / MAVROS), 未选返回 None。
        以存活进程为准 (link_procs 只在有进程时持有 key)。"""
        for key in self.link_procs:
            return key
        return None

    def _update_link_btn_text(self):
        """按钮文字跟随当前开启的链路, 无则回到"通信链路"; 链路开启中
        按钮点亮 (存活进程 = 链路通, 见 _selected_link)。"""
        running = self._selected_link()
        title = next((t for k, t, _ in self._LINKS if k == running), None)
        self.link_btn.setText(f'通信链路: {title}' if title else '通信链路')
        self._set_btn_on(self.link_btn, running is not None)
