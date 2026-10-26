#!/usr/bin/env python3
"""视觉算法工具按钮面板 (MainWindow 的功能域 mixin)。

点选即拉起算法进程 + 重发 start 指令 (防 DDS 发现窗口丢消息), 再点/
切换即停; 停 ArUco检测时联动取消视觉反控。
样式常量 _TOOL_BTN_STYLE 定义在 main_window (ui_style.TOOL_BTN_STYLE)。
"""
import time

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QMenu, QToolButton

from .. import config


class AlgoPanelMixin:
    """MainWindow 的视觉算法功能域 (self.* 为主窗口属性)。"""

    # ---------- 视觉算法 (工具栏下拉按钮: 点选即推理+可视化, 再点/切换即取消) ----------

    def _build_algo_button(self):
        self.algo_btn = QToolButton()
        self.algo_btn.setText('视觉算法')
        self.algo_btn.setPopupMode(QToolButton.InstantPopup)
        self.algo_btn.setStyleSheet(self._TOOL_BTN_STYLE)
        self.algo_btn.setToolTip(
            '视觉算法 (flylab_vision): 选择即运行 (拉起进程 + 发启动指令, '
            '结果叠加到画面); 再点运行中的算法即停止, 选其他停旧换新')
        menu = QMenu(self.algo_btn)
        self.algo_btn.setMenu(menu)
        menu.aboutToShow.connect(self._rebuild_algo_menu)
        menu.triggered.connect(self._on_algo_action)
        return self.algo_btn

    def _rebuild_algo_menu(self):
        """重建算法菜单: ✓ = 正在运行; 再点运行中的算法即停止。"""
        menu = self.algo_btn.menu()
        menu.clear()
        cur = self.running_algo['name'] if self.running_algo else None
        for algo in config.ALGORITHMS:
            act = menu.addAction(algo['name'])
            act.setData(algo['name'])
            act.setCheckable(True)
            act.setChecked(algo['name'] == cur)

    def _on_algo_action(self, act):
        """菜单点击: 未运行 → 启动 (拉起进程 + 发 start 指令);
        点运行中的算法 → 停止; 运行中选其他算法 → 停旧换新。"""
        name = act.data()
        algo = next((a for a in config.ALGORITHMS if a['name'] == name), None)
        if algo is None:
            return
        if self.running_algo is not None:
            was = self.running_algo        # 先记住 (stop 会清掉)
            self._stop_algo()
            if was is algo:                # 刚才运行的就是它 → 纯停止
                return
        self.running_algo = algo
        self._algo_started_at = time.monotonic()
        if algo.get('cmd'):
            self.algo_proc.start(algo['cmd'])
        # 新进程的 DDS 发现需要约 1s, 延迟发 start; 节点对重复 start 无副作用
        delay_ms = 1000 if algo.get('cmd') else 0
        QTimer.singleShot(delay_ms, lambda: self._send_start(algo))
        self._update_algo_state()

    def _send_start(self, algo, retries_left=6):
        """发送启动指令并按 400ms 间隔重发多次: 单发一条 start 撞上
        DDS 发现窗口会被丢掉 (volatile 消息不补发); 节点对重复 start
        幂等。算法被取消/切换/进程已死则跳过。"""
        if self.running_algo is not algo:
            return
        rc = self.algo_proc.exited()
        if rc is not None:
            self.log.error(
                f'算法进程启动失败 (退出码 {rc}), 请检查 config.ALGORITHMS 里的 cmd')
            self._reset_algo()
            return
        self.node.publish_cmd(algo['start'], quiet=retries_left < 6)
        if retries_left > 0:
            QTimer.singleShot(
                400, lambda: self._send_start(algo, retries_left - 1))

    def _stop_algo(self):
        """停掉正在运行的算法: 发停止指令 + 界面立即清标注 + 回收进程。
        停 ArUco检测时视觉反控失去输入源, 自动一并退出。"""
        algo = self.running_algo
        if algo is None:
            return
        self.node.publish_cmd(algo['stop'])
        self.node.clear_annotations()
        self.algo_proc.stop()
        self.running_algo = None
        self._update_algo_state()   # 文字复位 + 熄灭状态灯
        # ArUco检测停止时视觉反控 (降落) 失去输入源, 自动一并取消
        if algo.get('enables_landing') and self.landing_btn.isChecked():
            self.node.publish_cmd(config.LANDING_STOP_CMD)
            self.landing_btn.setChecked(False)

    def _reset_algo(self):
        """算法异常退出时复位状态 (不重复发停止指令)。"""
        self.running_algo = None
        self.algo_proc.clear()
        self.node.clear_annotations()
        self._update_algo_state()

    def _check_algo_alive(self):
        """轮询里顺带检查进程存活: 异常退出则复位并提示。"""
        if self.running_algo is not None and self.algo_proc.exited() is not None:
            self.log.error(
                f'算法进程异常退出 (退出码 {self.algo_proc.proc.returncode})')
            self._reset_algo()

    def _update_algo_state(self):
        """按钮文字跟随当前运行的算法, 无则回到"算法"; 运行中点亮状态灯。"""
        running = self.running_algo['name'] if self.running_algo else None
        self.algo_btn.setText(f'视觉算法: {running}' if running else '视觉算法')
        self._set_btn_on(self.algo_btn, running is not None)
