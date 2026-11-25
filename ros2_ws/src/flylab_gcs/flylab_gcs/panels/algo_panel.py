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
from . import ui_style


class AlgoPanelMixin:
    """MainWindow 的视觉算法功能域 (self.* 为主窗口属性)。"""

    # ---------- 视觉算法 (工具栏下拉按钮: 点选即推理+可视化, 再点/切换即取消) ----------

    def _build_algo_button(self):
        self.algo_btn = QToolButton()
        self.algo_btn.setText('视觉算法')
        self.algo_btn.setPopupMode(QToolButton.InstantPopup)
        self.algo_btn.setStyleSheet(self._TOOL_BTN_STYLE)
        self.algo_btn.setToolTip(
            '视觉算法 (flylab_vision): 一级菜单=任务分组, 二级=具体算法/模型。'
            '选择即运行 (拉起进程 + 发启动指令, 结果叠加到画面); '
            '再点运行中的算法即停止, 选其他停旧换新; '
            '菜单末尾"停止算法"一键停止运行中的算法')
        menu = QMenu(self.algo_btn)
        self.algo_btn.setMenu(menu)
        menu.aboutToShow.connect(self._rebuild_algo_menu)
        return self.algo_btn

    def _rebuild_algo_menu(self):
        """重建算法菜单 (一级=任务分组, 二级=具体算法/模型):
        绿点 = 正在运行; 再点运行中的算法即停止。运行中任务所在的一级
        分组名前亮绿色圆点并加粗 (无需展开二级即知当前任务) —— QAction
        文字无法按条目单独上色, 用同色图标 + 字体实现, 配色与工具栏
        状态灯一致。每个算法项独立连接 triggered —— 不依赖根菜单
        triggered 对子菜单动作的传播行为, 各 Qt 版本行为一致且不会双触发。"""
        menu = self.algo_btn.menu()
        menu.clear()
        cur = self.running_algo['name'] if self.running_algo else None
        for group in config.ALGORITHMS:
            if 'items' in group:
                sub = menu.addMenu(group['name'])
                target = sub
                entries = group['items']
                on = cur is not None and any(
                    a['name'] == cur for a in entries)
                gact = sub.menuAction()
                font = gact.font()
                font.setBold(on)
                gact.setFont(font)
                ui_style.set_menu_icon(gact, on)
            else:   # 旧式平铺条目 (无 items 键): 直接一级显示
                target = menu
                entries = [group]
            for algo in entries:
                act = target.addAction(algo['name'])
                ui_style.set_menu_icon(act, algo['name'] == cur)
                act.triggered.connect(
                    lambda _checked=False, name=algo['name']:
                        self._on_algo_name(name))
        # 末尾固定"停止当前算法"项: 免去进二级菜单找运行中条目的操作。
        # 无算法运行时置灰 (红点也清空), 运行中显示算法名
        menu.addSeparator()
        stop_act = menu.addAction('停止算法')
        stop_act.setEnabled(cur is not None)
        ui_style.set_stop_menu_icon(stop_act, cur is not None)
        stop_act.triggered.connect(self._stop_algo)

    def _on_algo_name(self, name):
        """菜单点击: 未运行 → 启动 (拉起进程 + 发 start 指令);
        点运行中的算法 → 停止; 运行中选其他算法 → 停旧换新。"""
        algo = next((a for a in config.iter_algorithms()
                     if a['name'] == name), None)
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
            self.algo_proc.start(algo['cmd'], capture_output=True)
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
        takeover_on = self.gimbal_act.isChecked()
        self.node.publish_cmd(algo['stop'])
        self.node.clear_annotations()
        if takeover_on:
            # 目标追踪反控中停算法: 先发交还指令 (节点切回 LOITER), 稍后
            # 回收进程, 与降落节点收尾同一节奏
            self._cancel_track_takeover()
            QTimer.singleShot(500, self.algo_proc.stop)
        else:
            self.algo_proc.stop()
        self.running_algo = None
        self._update_algo_state()   # 文字复位 + 熄灭状态灯
        # ArUco检测停止时视觉反控 (降落) 失去输入源, 自动一并取消
        if algo.get('enables_landing') and self.landing_act.isChecked():
            self.node.publish_cmd(config.LANDING_STOP_CMD)
            self.landing_act.setChecked(False)

    def _reset_algo(self):
        """算法异常退出时复位状态 (不重复发停止指令)。"""
        self.running_algo = None
        self.algo_proc.clear()
        self.node.clear_annotations()
        self._update_algo_state()
        # 目标追踪异常退出时反控失去输入源, 自动一并取消 (节点已死,
        # offboard 断流由 PX4 failsafe 兜底)
        self._cancel_track_takeover()

    def _check_algo_alive(self):
        """轮询里顺带检查进程存活: 异常退出则复位并提示。"""
        if self.running_algo is not None and self.algo_proc.exited() is not None:
            self.log.error(
                f'算法进程异常退出 (退出码 {self.algo_proc.proc.returncode})')
            self._reset_algo()

    def _update_algo_state(self):
        """按钮文字跟随当前运行的算法, 无则回到"算法"; 运行中点亮状态灯。
        算法启停/切换同时联动接管控制菜单的视觉反控形态 (子菜单/单项)。"""
        running = self.running_algo['name'] if self.running_algo else None
        self.algo_btn.setText(f'视觉算法: {running}' if running else '视觉算法')
        self._set_btn_on(self.algo_btn, running is not None)
        self._update_landing_menu_shape()
