#!/usr/bin/env python3
"""地面站主窗口 —— 界面组装与调度中枢。

模块分层 (细节见各文件):
  config.py           全局配置 (话题/端口/机型/算法注册表, 改这里)
  ros_node.py         ROS 节点 (订阅/发布 + TeeLogger 日志分流)
  camera_grabber.py   视频源抓取线程 (AirSim RPC/本机摄像头/文件) + 软件云台
  vehicle_control.py  飞行控制 (独立 RPC 会话: 解锁/起飞/降落/悬停/返航/上锁)
  processes.py        托管子进程 (PX4 SITL/通信链路/算法/降落节点的生命周期)
  log_panel.py        画面下方日志框 (TeeLogger 分流 + PX4 输出接管, 抗刷屏)
  main_window.py      本文件: 工具栏/面板组装 + 33ms 轮询调度 + 按钮门控

界面结构:
  顶部工具栏 (QToolBar): 视频源 / PX4 SITL / 通信链路 / 飞行控制, 全部下拉交互
  左侧画面 + 底部日志框 (camera_widget 内部布局)
  右栏: 云台 (camera_widget) + 算法选择 + ArUco降落
"""
import os
import time

import rclpy
from PyQt5.QtCore import QTimer, pyqtSignal
from PyQt5.QtWidgets import (QComboBox, QGroupBox, QHBoxLayout, QMainWindow,
                           QMenu, QPushButton, QToolBar, QToolButton,
                           QVBoxLayout)

from . import config
from .camera_widget import CameraViewWidget
from .log_panel import LogPanel
from .processes import ManagedProcess
from .vehicle_control import VehicleController


class MainWindow(QMainWindow):
    # 日志行 (level, text) —— 任意线程可 emit, 经队列投递到 GUI 线程上屏
    log_line = pyqtSignal(str, str)

    def __init__(self, node):
        super().__init__()
        self.setWindowTitle('FLYLAB 地面站')
        self.node = node
        self.log = node.get_logger()   # tee logger (终端照常 + 界面日志框)
        host = os.environ.get('PX4_SIM_HOST_ADDR', '127.0.0.1')

        # 视频源视图 (画面/底部日志框挂载点/右栏挂载点)
        self.camera_tab = CameraViewWidget(
            self.log.info, host, config.AIRSIM_RPC_PORT, side_panel=True)

        # 日志框: TeeLogger 分流先挂上, 后续所有日志都能上屏
        self.log_panel = LogPanel()
        self.log_line.connect(self.log_panel.append_line)
        self.camera_tab.add_bottom_widget(self.log_panel)
        self.node.set_log_sink(
            lambda level, text: self.log_line.emit(level, text))

        # 托管进程 (机制见 processes.py): PX4 输出接管进日志框, 其余直通终端
        self.px4 = ManagedProcess(self.log, 'PX4 SITL',
                                  on_line=lambda text: self.log_line.emit('PX4', text))
        self.px4_model = None      # 运行中的机型 (config.PX4_MODELS 项), None=未运行
        self.link_procs = {}       # key -> ManagedProcess (有存活进程 = 链路开启中)
        self.algo_proc = ManagedProcess(self.log, '算法节点')
        self.land_proc = ManagedProcess(self.log, '降落节点')

        # 飞行控制: 独立 RPC 会话 (与相机抓取互不拖累)
        self.veh_ctrl = VehicleController(self.log, host, config.AIRSIM_RPC_PORT)
        self.veh_ctrl.start()

        # 算法/降落 UI 状态
        self.running_algo = None
        self._last_landing_seq = 0     # 已处理的降落状态事件序号
        # 显示帧转发统计 (运行中每 2 秒打一条日志; 计数在转发线程累计)
        self._fwd_last = time.monotonic()
        self._algo_started_at = 0.0    # 算法进程启动时刻 (发现窗口内不告警)
        self._last_src = ''
        # 相机机体系姿态发布节流 (10Hz, 供 aruco_landing 云台泛化映射)
        self._cam_ori_last = 0.0

        # 顶部工具栏 (VS Code 风格): 工具按钮横排, 后续新功能往右加
        self.toolbar = QToolBar()
        self.toolbar.setMovable(False)
        self.toolbar.setStyleSheet('QToolBar { spacing: 6px; padding: 2px 6px; }')
        self.camera_tab.src_btn.setStyleSheet(self._TOOL_BTN_STYLE)
        self.toolbar.addWidget(self.camera_tab.src_btn)
        self.toolbar.addWidget(self._build_px4_button())
        self.toolbar.addWidget(self._build_link_button())
        self.toolbar.addWidget(self._build_vehicle_button())
        self.addToolBar(self.toolbar)

        # 算法选择: 下拉框 + 运行/取消按钮, 插到右栏。
        # 同一时刻只跑一个算法; 运行中下拉框锁定, 取消后恢复。
        # 配了 cmd 的算法点运行时自动拉起进程 (用户无需手动 ros2 run)
        self.algo_panel = QGroupBox('算法')
        algo_lay = QVBoxLayout(self.algo_panel)
        algo_lay.setContentsMargins(6, 4, 6, 6)
        self.algo_combo = QComboBox()
        for algo in config.ALGORITHMS:
            self.algo_combo.addItem(algo['name'])
        self.algo_run_btn = QPushButton('运行算法')
        self.algo_cancel_btn = QPushButton('取消')
        self.algo_cancel_btn.setEnabled(False)
        self.algo_run_btn.setStyleSheet(self._BTN_STYLE)
        self.algo_cancel_btn.setStyleSheet(self._BTN_STYLE)
        self.algo_run_btn.clicked.connect(self._on_run_algo)
        self.algo_cancel_btn.clicked.connect(self._on_cancel_algo)
        algo_lay.addWidget(self.algo_combo)
        btn_row = QHBoxLayout()          # 运行/取消并排一行, 省纵向空间
        btn_row.addWidget(self.algo_run_btn, stretch=1)
        btn_row.addWidget(self.algo_cancel_btn)
        algo_lay.addLayout(btn_row)
        # ArUco 降落: 独立 toggle 按钮 (与算法下拉框互不影响, 检测可同时运行)。
        # 按下: 拉起 aruco_landing_node (按所选通信链路传 link 参数) 并发
        # landing_start 触发三态状态机; 抬起: 发 landing_stop 取消降落并交还控制权。
        # 默认禁用, 门控见 _update_landing_btn: 需 ArUco检测运行中 且
        # 云台俯仰转到垂直向下 且 已开启通信链路 (降落进行中保持可点, 随时取消)。
        self.landing_btn = QPushButton('ArUco降落')
        self.landing_btn.setCheckable(True)
        self.landing_btn.setEnabled(False)
        self.landing_btn.setToolTip(
            '需先: 1) 运行 ArUco检测  2) 云台俯仰调到垂直向下 (↓到限位)  '
            '3) 工具栏开启通信链路 (uXRCE-DDS / MAVROS)')
        self.landing_btn.setStyleSheet(self._LAND_BTN_STYLE)
        self.landing_btn.clicked.connect(self._on_landing_toggle)
        algo_lay.addWidget(self.landing_btn)
        # 右栏分组框: 云台 (camera_widget) + 算法 (这里);
        # 视频源 / PX4 SITL / 通信链路 / 飞行控制已上移到顶部工具栏
        self.camera_tab.add_side_widget(self.algo_panel)

        self.setCentralWidget(self.camera_tab)

        # 33ms 主轮询: 标注上屏/进程存活/按钮门控/画面转发 (见 _poll_annotations)
        self.anno_timer = QTimer(self)
        self.anno_timer.timeout.connect(self._poll_annotations)
        self.anno_timer.start(33)

    _BTN_STYLE = """
        QPushButton { min-height: 32px; }
        QPushButton:checked { background-color: #2d8a3e; color: white; }
    """

    # 工具栏按钮统一样式 (QToolButton: 视频源 / PX4 SITL / 通信链路 / 飞行控制)
    _TOOL_BTN_STYLE = """
        QToolButton { min-height: 24px; padding: 2px 10px; border: 1px solid #bbb;
                      border-radius: 4px; background: #f6f6f6; }
        QToolButton:hover { background: #e8e8e8; }
        QToolButton:checked { background-color: #2d8a3e; color: white;
                              border-color: #2d8a3e; }
    """

    _LAND_BTN_STYLE = """
        QPushButton { min-height: 32px; }
        QPushButton:checked { background-color: #b03030; color: white; }
    """

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

    def _stop_px4(self):
        """停止 PX4 SITL: SIGTERM 整个进程组 (make → px4 整棵树), 超时 SIGKILL。"""
        title = self.px4_model['name'] if self.px4_model else ''
        self.px4.stop()
        self._reset_px4_btn()
        if title:
            self.log.info(f'PX4 SITL 已停止: {title}')

    def _reset_px4_btn(self):
        """按钮复位为空闲态: 文案回 "PX4 SITL", 清除运行中机型 ——
        菜单的 ✓ 语义是"正在运行", 停止/崩溃/启动失败后不再勾选。"""
        self.px4_model = None
        self.px4_btn.setText('PX4 SITL')

    def _check_px4_alive(self):
        """轮询: PX4 进程异常退出 (崩溃/自爆) 则复位按钮, 用户可重新点起。"""
        rc = self.px4.exited()
        if rc is not None and self.px4_model is not None:
            self.log.error(f'PX4 SITL 进程退出 (退出码 {rc}), 请重新点击启动')
            self.px4.clear()
            self._reset_px4_btn()

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
        """按钮文字跟随当前开启的链路, 无则回到"通信链路"。"""
        running = self._selected_link()
        title = next((t for k, t, _ in self._LINKS if k == running), None)
        self.link_btn.setText(f'通信链路: {title}' if title else '通信链路')

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
            '(需 UE 端 AirSim 已连通; 异步下发, 结果看 QGC / mavros state)')
        menu = QMenu(self.veh_btn)
        self.veh_btn.setMenu(menu)
        menu.aboutToShow.connect(self._rebuild_vehicle_menu)
        menu.triggered.connect(self._on_vehicle_action)
        return self.veh_btn

    def _rebuild_vehicle_menu(self):
        """重建飞控命令菜单: RPC 未连通时整组置灰。"""
        menu = self.veh_btn.menu()
        menu.clear()
        ok = self.veh_ctrl.is_connected()
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

    # ---------- 算法运行/取消 (进程生命周期见 processes.py) ----------

    def _on_run_algo(self):
        """运行选中的算法: 拉起进程 (若配了 cmd) 并发启动指令;
        若已有别的算法在跑, 先停掉旧的再启动新的 (无缝切换);
        重复点击同一算法忽略。未配 cmd 则假定节点已在外部运行, 只发指令。"""
        algo = config.ALGORITHMS[self.algo_combo.currentIndex()]
        if self.running_algo is not None:
            if self.running_algo is algo:
                return                    # 已在运行, 忽略重复点击
            self._stop_algo()
        self.running_algo = algo
        self._algo_started_at = time.monotonic()
        if algo.get('cmd'):
            self.algo_proc.start(algo['cmd'])
        # 新进程的 DDS 发现需要约 1s, 延迟发 start; 节点对重复 start 无副作用
        delay_ms = 1000 if algo.get('cmd') else 0
        QTimer.singleShot(delay_ms, lambda: self._send_start(algo))
        self._update_algo_state()

    def _send_start(self, algo, retries_left=6):
        """发送启动指令并按 400ms 间隔重发多次: 新进程的 DDS 发现可能
        超过 1s, 单发一条 start 撞上发现窗口就永远丢掉 (volatile 消息
        不补发, 表现为"点了运行没反应"); 节点对重复 start 幂等, 多发无副作用。
        触发前算法被取消/切换/进程已死则跳过。"""
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

    def _on_cancel_algo(self):
        """取消当前正在运行的算法 (停止的是运行中的, 与下拉框当前选中无关)。"""
        if self.running_algo is None:
            return
        self._stop_algo()
        self._update_algo_state()

    def _stop_algo(self):
        """停掉正在运行的算法: 发停止指令 + 界面立即清标注 + 回收进程。"""
        algo = self.running_algo
        if algo is None:
            return
        self.node.publish_cmd(algo['stop'])
        self.node.clear_annotations()
        self.algo_proc.stop()
        self.running_algo = None

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
        """运行中: 下拉框锁定、运行按钮变状态提示、取消按钮可用; 空闲则相反。"""
        running = self.running_algo is not None
        self.algo_combo.setEnabled(not running)
        self.algo_run_btn.setEnabled(not running)
        self.algo_cancel_btn.setEnabled(running)
        if running:
            self.algo_run_btn.setText(f'运行中: {self.running_algo["name"]}')
        else:
            self.algo_run_btn.setText('运行算法')

    # ---------- ArUco 降落 (独立 toggle 按钮) ----------

    def _update_landing_btn(self):
        """降落按钮门控 (33ms 轮询): 需同时满足
        1) ArUco检测运行中
        2) 云台俯仰到垂直向下限位 (绝对 -90°, 即相对俯仰到 GIMBAL_PITCH_MIN_DEG)
        3) 已开启通信链路 (uXRCE-DDS / MAVROS 之一)
        降落进行中不参与门控 —— 保持可点, 随时可取消交还控制权;
        同时禁用链路下拉按钮 (换桥会断掉正在跑的降落控制)。"""
        self.link_btn.setEnabled(not self.landing_btn.isChecked())
        ok = False
        algo = self.running_algo
        if (algo is not None and algo.get('enables_landing')
                and self._selected_link() is not None):
            gim = self.camera_tab.grabber.get_gimbal()
            if gim is not None:
                _, pitch = gim
                ok = pitch <= config.GIMBAL_PITCH_MIN_DEG + 1e-3
        if self.landing_btn.isEnabled() != ok:
            self.landing_btn.setEnabled(ok)

    def _on_landing_toggle(self):
        if self.landing_btn.isChecked():
            # 链路参数跟随工具栏所选链路 (mavros / uxrce)
            link = self._selected_link() or 'mavros'
            cmd = list(config.LANDING_CMD) + ['--ros-args', '-p', f'link:={link}']
            self.land_proc.start(cmd)
            # 新进程要完成 DDS 发现 + 收到首帧飞控状态 (共 1~3s), 单发一条
            # landing_start 撞上任何一步都会被吞掉 (表现为"第一次点击无效"),
            # 故启动后重发多次: 节点对重复 start 幂等, 多发无副作用
            QTimer.singleShot(1000, lambda: self._send_landing_start(6))
        else:
            self._stop_landing()

    def _check_landing_status(self):
        """降落节点状态事件处理: 成功(landed)/结束(idle) → 按钮弹起并给反馈。
        状态带事件序号, 只在有新事件时处理 (不漏瞬时的"成功")。"""
        status, seq = self.node.get_landing_status()
        if seq == self._last_landing_seq:
            return
        self._last_landing_seq = seq
        if not status:
            return
        if status == 'landed':
            self.log.info('\033[32mArUco 视觉降落完成, 飞机已上锁\033[0m')
            self._flash_landing_btn('#2d8a3e')      # 绿色高亮闪一下
        elif status == 'idle':
            self.log.info('降落流程已结束 (取消/中止), 按钮已复位')
        else:
            return
        # 按钮弹起 (checked = 降落进行中) + 回收降落节点进程
        self.landing_btn.setChecked(False)
        self.land_proc.stop()

    def _flash_landing_btn(self, color):
        """按钮背景色闪烁提示, 1.5s 后恢复原样式。"""
        self.landing_btn.setStyleSheet(
            self._LAND_BTN_STYLE +
            f'\nQPushButton {{ background-color: {color}; color: white; }}')
        QTimer.singleShot(1500, lambda: self.landing_btn.setStyleSheet(
            self._LAND_BTN_STYLE))

    def _send_landing_start(self, retries_left):
        """触发降落并按 500ms 间隔重发; 按钮已抬起或节点进程已死则跳过。"""
        if not self.landing_btn.isChecked():
            return
        rc = self.land_proc.exited()
        if rc is not None:
            self.log.error(
                f'降落节点进程启动失败 (退出码 {rc}), '
                f'检查编译: colcon build --packages-select flylab_vision')
            self.land_proc.clear()
            self.landing_btn.setChecked(False)
            return
        self.node.publish_cmd(config.LANDING_START_CMD)
        if retries_left > 0:
            QTimer.singleShot(500, lambda: self._send_landing_start(retries_left - 1))

    def _stop_landing(self):
        """取消降落: 发 landing_stop (节点自行切回 LOITER 交还控制权并清标注),
        稍后回收进程, 给节点留出发送停止指令与清屏的时间。"""
        self.node.publish_cmd(config.LANDING_STOP_CMD)
        QTimer.singleShot(500, self.land_proc.stop)

    # ---------- 主轮询 (33ms, GUI 线程) ----------

    def _poll_annotations(self):
        # 驱动 ROS 收包 (节点没有独立 spin 线程, 由本定时器在 GUI 线程代转,
        # 回调天然在主线程执行, 无并发问题); 不 spin 则订阅永远收不到消息
        try:
            rclpy.spin_once(self.node, timeout_sec=0)
        except Exception:
            pass
        # 标注上屏门控: 仅在"算法运行中或降落进行中"时显示。防止外部遗留的
        # 算法节点进程 (仍处于 start 状态) 在用户未运行任何算法时擅自叠加;
        # 此种场景直接清掉已有分组, 不让来路不明的过期标注上屏
        if self.running_algo is None and not self.landing_btn.isChecked():
            if self.node.get_annotations():
                self.node.clear_annotations()
            annos = []
        else:
            annos = self.node.get_annotations()
        self.camera_tab.video.set_annotations(annos)
        # HUD 第二行: 算法输出帧率 (运行中才显示, 停止/无输出自动隐藏)
        fps = self.node.get_algo_fps() if self.running_algo is not None else 0.0
        self.camera_tab.video.set_algo_hud(
            f'解算 {fps:.1f}' if fps > 0 else None)
        self._check_algo_alive()
        self._check_px4_alive()      # PX4 进程异常退出则复位按钮
        self._check_link_alive()     # 桥进程异常退出则复位按钮
        # 飞行控制按钮: 仅飞控 RPC 会话已连通可点 (UE 未 play / 断线置灰)
        veh_ok = self.veh_ctrl.is_connected()
        if self.veh_btn.isEnabled() != veh_ok:
            self.veh_btn.setEnabled(veh_ok)
        self._check_landing_status()  # 降落结果反馈 (成功/中止 → 按钮弹起)
        self._update_landing_btn()   # 降落按钮门控 (检测运行中 + 云台垂直向下 + 已选链路)
        # 显示画面转发: 有算法节点订阅时, 把当前显示的帧发给它
        if config.DISPLAY_IMAGE_HZ > 0:
            self._publish_display_frame()
        self._log_forward_stats()   # 转发断流告警 (2s 一次, 计数在转发线程)
        # 相机机体系姿态 (10Hz): 供 aruco_landing 做云台泛化的方向映射;
        # 非 AirSim 源 get_cam_orientation() 返回 None, 不发布
        now_mono = time.monotonic()
        if now_mono - self._cam_ori_last >= 0.1:
            self._cam_ori_last = now_mono
            self.node.publish_cam_orientation(
                self.camera_tab.grabber.get_cam_orientation())

    def _publish_display_frame(self):
        """取当前显示的帧交给转发线程 (深度伪彩帧跳过, 无检测意义)。
        这里只做 O(1) 引用传递: 大图的 tobytes/序列化/publish/等 ACK
        全部在 DisplayForwarder 线程完成, 不再占用 GUI 线程。"""
        frame, src, fps = self.camera_tab.grabber.get()
        if frame is None:
            return
        self._last_src = src
        arr, is_depth = frame
        if is_depth or arr.ndim != 3:
            return
        self.node.set_display_frame(arr)

    def _log_forward_stats(self):
        """转发完全断流时告警 (正常运行不刷屏)。"""
        now = time.monotonic()
        if now - self._fwd_last < 2.0:
            return
        self._fwd_last = now
        cnt = self.node.take_fwd_count()
        if self.running_algo is None:
            return
        # 启动后 3s 内是算法节点的 DDS 发现窗口, 订阅者尚未匹配,
        # 转发计数为 0 是正常现象, 不告警
        if cnt == 0 and now - self._algo_started_at > 3.0:
            self.log.warning(
                f'画面转发: 近 2 秒 0 帧 (源 {self._last_src or "?"} 无图像?)')

    def shutdown(self):
        """退出: 回收全部托管进程, 停掉后台线程。"""
        if self.running_algo is not None:
            self._stop_algo()
        if self.landing_btn.isChecked():
            self.node.publish_cmd(config.LANDING_STOP_CMD)
        self.land_proc.stop()
        self._stop_px4()             # PX4 SITL 随地面站退出一并回收
        for key in list(self.link_procs):
            self._stop_link(key)
        self.veh_ctrl.stop()         # 飞控 RPC 会话线程
        self.camera_tab.shutdown()
