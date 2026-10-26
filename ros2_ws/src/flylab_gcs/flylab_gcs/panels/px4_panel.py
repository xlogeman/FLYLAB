#!/usr/bin/env python3
"""PX4 SITL 工具按钮面板 (MainWindow 的功能域 mixin)。

机型菜单选型即启动 (联动拉起 Windows 端 AirSim 导出项目), 再点即停,
选其他机型停旧换新; 进程异常退出自动复位按钮。
样式常量 _TOOL_BTN_STYLE 定义在 main_window (ui_style.TOOL_BTN_STYLE)。
"""
import os

from PyQt5.QtWidgets import QMenu, QToolButton

from .. import config


class Px4PanelMixin:
    """MainWindow 的 PX4 SITL 功能域 (self.* 为主窗口属性)。"""

    # ---------- PX4 SITL (工具栏按钮: 菜单选机型, 选中即启动) ----------

    def _build_px4_button(self):
        """PX4 SITL 工具按钮: 点击弹出机型菜单, 点机型即启动 (运行中选
        其他机型 = 停旧换新); 再点运行中的机型即停止。"""
        self.px4_btn = QToolButton()
        self.px4_btn.setText('PX4 SITL')
        self.px4_btn.setPopupMode(QToolButton.InstantPopup)
        self.px4_btn.setStyleSheet(self._TOOL_BTN_STYLE)
        self.px4_btn.setToolTip(
            '启动 PX4 SITL: 选机型即启动 (经 scripts/run-px4.sh, 需先开 '
            'Windows 端 UE/AirSim); 再点运行中的机型即停止, 选其他机型停旧换新')
        menu = QMenu(self.px4_btn)
        self.px4_btn.setMenu(menu)
        menu.aboutToShow.connect(self._rebuild_px4_menu)
        menu.triggered.connect(self._on_px4_action)
        return self.px4_btn

    def _rebuild_px4_menu(self):
        """重建机型菜单: ✓ = 正在运行; 再点运行中的机型即停止
        (与通信链路菜单同款交互, 不设独立"停止"项)。"""
        menu = self.px4_btn.menu()
        menu.clear()
        cur = self.px4_model['name'] if self.px4_model else None
        for model in config.PX4_MODELS:
            act = menu.addAction(model['name'])
            act.setData(model['target'])
            act.setCheckable(True)
            act.setChecked(model['name'] == cur)

    def _on_px4_action(self, act):
        """菜单点击: 未运行 → 直接启动; 点运行中的机型 → 停止;
        运行中点其他机型 → 停旧换新。"""
        target = act.data()
        if target is None:
            return
        model = next((m for m in config.PX4_MODELS
                      if m['target'] == target), None)
        if model is None:
            return
        if self.px4.is_running():
            was = self.px4_model            # 先记住 (stop 会清掉)
            self._stop_px4()
            if was is model:                # 刚才运行的就是它 → 纯停止
                return
        self.px4_model = model
        self._start_px4()

    def _start_px4(self):
        """拉起 PX4 SITL (进程组回收/输出接管见 processes.ManagedProcess)。
        run-px4.sh 内部 make 已编译时秒级启动, 未编译则需等编译完成。"""
        if self.px4.is_running():
            return
        model = self.px4_model
        if model is None:
            return
        if not self.px4.start(['bash', config.PX4_SCRIPT, model['target']],
                              capture_output=True):
            self._reset_px4_btn()           # 启动失败 (脚本缺失等)
            return
        self.px4_btn.setText(f'PX4: {model["name"]}')
        self._set_btn_on(self.px4_btn, True)   # 机型运行中 → 按钮点亮
        self._start_airsim_app(model)

    def _start_airsim_app(self, model):
        """联动拉起机型对应的 Windows 端 AirSim 导出项目 (WSL interop 直启
        .exe, 与 PX4 按钮同生命周期: 停止/崩溃/退出时一并回收)。
        路径缺失只告警不阻断 PX4 启动; 已在运行则跳过。"""
        exe = config.airsim_app_for(model['name'])
        if self.airsim_proc.is_running():
            return
        if not os.path.isfile(exe):
            self.log.error(f'AirSim 导出项目不存在, 跳过拉起: {exe}')
            return
        # Windows 真实进程不随 Linux 侧中继死亡, 停止时按映像名 taskkill
        # 补杀 (/T 连子进程; 按名会全杀, 若手动另开了同名项目会被一并关闭)
        self.airsim_proc.kill_cmd = ['taskkill.exe', '/F', '/T', '/IM',
                                     os.path.basename(exe)]
        # 工作目录 = exe 所在目录 (打包项目按相对路径找资源)
        self.airsim_proc.start([exe], cwd=os.path.dirname(exe))

    def _stop_px4(self):
        """停止 PX4 SITL: SIGTERM 整个进程组 (make → px4 整棵树), 超时 SIGKILL。"""
        title = self.px4_model['name'] if self.px4_model else ''
        self.px4.stop()
        self.airsim_proc.stop()     # AirSim 项目窗口随 PX4 一并回收
        self._reset_px4_btn()
        if title:
            self.log.info(f'PX4 SITL 已停止: {title}')

    def _reset_px4_btn(self):
        """按钮复位为空闲态: 文案回 "PX4 SITL", 清除运行中机型 ——
        菜单的 ✓ 语义是"正在运行", 停止/崩溃/启动失败后不再勾选。"""
        self.px4_model = None
        self.px4_btn.setText('PX4 SITL')
        self._set_btn_on(self.px4_btn, False)   # 停止/崩溃/失败 → 熄灭

    def _check_px4_alive(self):
        """轮询: PX4 进程异常退出 (崩溃/自爆) 则复位按钮, 用户可重新点起。"""
        rc = self.px4.exited()
        if rc is not None and self.px4_model is not None:
            self.log.error(f'PX4 SITL 进程退出 (退出码 {rc}), 请重新点击启动')
            self.px4.clear()
            self.airsim_proc.stop()     # AirSim 项目窗口一并回收
            self._reset_px4_btn()
