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
  顶部工具栏 (QToolBar): 视频源 / PX4 SITL / 飞行控制 / 通信链路 /
  视觉算法 / 飞行链路 / 视觉反控, 全部下拉或 toggle 交互
  左侧画面 + 底部日志框 (camera_widget 内部布局)
  右栏: 航线地图 + 云台/罗盘 + 手动遥控 (双摇杆)
"""
import os
import time
import traceback

import rclpy
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtWidgets import (QGroupBox, QHBoxLayout, QMainWindow,
                           QMenu, QPushButton, QToolBar, QToolButton,
                           QVBoxLayout, QWidget)

from . import config
from .camera_widget import CameraViewWidget
from .compass import Compass
from .log_panel import LogPanel
from .map_view import MapView
from .telemetry_bar import TelemetryBar
from .processes import ManagedProcess
from .teleop_pad import VirtualJoystick
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
        self.tele_bar = TelemetryBar()   # QGC 式遥测六值条 (画面下方, 日志框之上)
        self.camera_tab.add_bottom_widget(self.tele_bar)
        self.camera_tab.add_bottom_widget(self.log_panel)
        self.node.set_log_sink(
            lambda level, text: self.log_line.emit(level, text))

        # 托管进程 (机制见 processes.py): PX4 输出接管进日志框, 其余直通终端
        self.px4 = ManagedProcess(self.log, 'PX4 SITL',
                                  on_line=lambda text: self.log_line.emit('PX4', text))
        self.px4_model = None      # 运行中的机型 (config.PX4_MODELS 项), None=未运行
        self.link_procs = {}       # key -> ManagedProcess (有存活进程 = 链路开启中)
        self.algo_proc = ManagedProcess(self.log, '视觉算法节点')
        self.flight_proc = ManagedProcess(self.log, '飞行链路')   # ros2 launch 整链路
        self.land_proc = ManagedProcess(self.log, '降落节点')
        self.teleop_proc = ManagedProcess(self.log, '遥控节点')

        # 飞行控制: 独立 RPC 会话 (与相机抓取互不拖累)
        self.veh_ctrl = VehicleController(self.log, host, config.AIRSIM_RPC_PORT)
        self.veh_ctrl.start()

        # 视觉算法/飞行链路/降落/遥控 UI 状态
        self.running_algo = None
        self.running_flight = None   # 运行中的飞行链路 (config.FLIGHT_LINKS 项)
        self._last_landing_seq = 0     # 已处理的降落状态事件序号
        self._last_teleop_seq = 0      # 已处理的遥控状态事件序号
        # 归一化遥控量 (-1..1): x 前 y 右 z 升 w 左转, 组合成 Twist 下发
        self._teleop_axes = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}
        # 显示帧转发统计 (运行中每 2 秒打一条日志; 计数在转发线程累计)
        self._fwd_last = time.monotonic()
        self._algo_started_at = 0.0    # 算法进程启动时刻 (发现窗口内不告警)
        self._last_src = ''
        self._poll_err_last = 0.0   # 上次轮询异常上屏时刻 (限频用)
        # 相机机体系姿态发布节流 (10Hz, 供 aruco_landing 云台泛化映射)
        self._cam_ori_last = 0.0

        # 顶部工具栏 (VS Code 风格): 工具按钮横排, 后续新功能往右加
        self.toolbar = QToolBar()
        self.toolbar.setMovable(False)
        self.toolbar.setStyleSheet('QToolBar { spacing: 6px; padding: 2px 6px; }')
        self.camera_tab.src_btn.setStyleSheet(self._TOOL_BTN_STYLE)
        self.toolbar.addWidget(self.camera_tab.src_btn)
        self.toolbar.addWidget(self._build_px4_button())
        self.toolbar.addWidget(self._build_vehicle_button())
        self.toolbar.addWidget(self._build_link_button())
        self.toolbar.addWidget(self._build_algo_button())
        self.toolbar.addWidget(self._build_flight_button())
        self.toolbar.addWidget(self._build_landing_button())
        self.addToolBar(self.toolbar)

        # 手动遥控: toggle 开关 + 双摇杆 (左=升降/偏航, 右=机体系平移)。
        # 开启: 拉起 teleop_node (按所选通信链路传 link 参数) 并发 teleop_start
        # 接管 offboard; 摇杆控速度、松开回中悬停; 关闭: 交还控制权。
        # 与视觉反控 (ArUco降落) 互斥 (都走 offboard, 会争抢 setpoint)
        # 右栏布局 (自上而下): 航线地图 → 云台 (左) + 罗盘 (右, 撑满等高)
        # 并排 → 手动遥控
        self.compass = Compass()
        self.map_view = MapView()   # 俯视轨迹, 数据同遥测: 飞控心跳采样
        self.camera_tab.add_side_widget(self.map_view)
        top_row = QWidget()
        row_lay = QHBoxLayout(top_row)
        row_lay.setContentsMargins(0, 0, 0, 0)
        row_lay.addWidget(self.camera_tab.gimbal_box, stretch=2)
        row_lay.addWidget(self.compass, stretch=1)
        self.camera_tab.add_side_widget(top_row)
        self.camera_tab.add_side_widget(self._build_teleop_panel())

        self.setCentralWidget(self.camera_tab)

        # 33ms 主轮询: 标注上屏/进程存活/按钮门控/画面转发 (见 _poll_annotations)
        self.anno_timer = QTimer(self)
        self.anno_timer.timeout.connect(self._poll_annotations)
        self.anno_timer.start(33)

    @staticmethod
    def _set_btn_on(btn, on):
        """工具按钮状态灯: on 动态属性切换点亮/熄灭 (样式见 _TOOL_BTN_STYLE)。
        状态不变不重刷 (轮询里每拍调用不产生样式抖动)。"""
        if bool(btn.property('on') or False) == bool(on):
            return
        btn.setProperty('on', bool(on))
        btn.style().unpolish(btn)
        btn.style().polish(btn)

    # 工具栏按钮统一样式 (QToolButton: 视频源 / PX4 SITL / 通信链路 / 飞行控制)
    # 点亮态: 动态属性 on=true (状态灯: PX4 运行中 / 链路开启 / 飞控控制中)
    _TOOL_BTN_STYLE = """
        QToolButton { min-height: 24px; padding: 2px 10px; border: 1px solid #bbb;
                      border-radius: 4px; background: #f6f6f6; }
        QToolButton:hover { background: #e8e8e8; }
        QToolButton:checked { background-color: #2d8a3e; color: white;
                              border-color: #2d8a3e; }
        QToolButton[on="true"] { background-color: #2d8a3e; color: white;
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
        self._set_btn_on(self.px4_btn, True)   # 机型运行中 → 按钮点亮

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
        self._set_btn_on(self.px4_btn, False)   # 停止/崩溃/失败 → 熄灭

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
        """按钮文字跟随当前开启的链路, 无则回到"通信链路"; 链路开启中
        按钮点亮 (存活进程 = 链路通, 见 _selected_link)。"""
        running = self._selected_link()
        title = next((t for k, t, _ in self._LINKS if k == running), None)
        self.link_btn.setText(f'通信链路: {title}' if title else '通信链路')
        self._set_btn_on(self.link_btn, running is not None)

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
              and not self.teleop_btn.isChecked()
              and not self.landing_btn.isChecked())
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
                   and not self.teleop_btn.isChecked())
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

    # ---------- 视觉反控 (降落) / 手动遥控 门控 (都走 offboard, 互斥) ----------

    def _update_landing_btn(self):
        """门控 (33ms 轮询), offboard 功能互斥 + 反控锁云台 + 遥控最高优先:
        - 视觉反控 (降落): 需已解锁升空 + ArUco检测运行中 + AirSim 源 +
          已开启链路 (点击时自动把云台俯仰转到垂直向下);
          进行中保持可点 (随时取消),
          锁定链路与云台面板; 允许开遥控 (遥控接管即自动取消反控, 防炸机)
        - 遥控: 仅需已开启链路; 进行中锁定链路与视觉反控"""
        link_ok = self._selected_link() is not None
        # 进行中的按钮自身必须保持可点 (否则无法取消), 只锁定其余项
        if self.teleop_btn.isChecked():
            self.link_btn.setEnabled(False)
            self.landing_btn.setEnabled(False)
            self.camera_tab.set_gimbal_locked(False)
            return
        if self.landing_btn.isChecked():
            self.link_btn.setEnabled(False)
            self.camera_tab.set_gimbal_locked(True)
            # 反控中允许开遥控抢回控制权 (防炸机): 接管时自动取消反控
            if self.teleop_btn.isEnabled() != link_ok:
                self.teleop_btn.setEnabled(link_ok)
            return
        self.camera_tab.set_gimbal_locked(False)
        self.link_btn.setEnabled(True)
        if self.teleop_btn.isEnabled() != link_ok:
            self.teleop_btn.setEnabled(link_ok)
        ok = False
        algo = self.running_algo
        if algo is not None and algo.get('enables_landing') and link_ok:
            # 需已解锁且在空中 (地面/未解锁点反控无意义); 俯仰由点击时
            # 自动打到垂直向下, 只要求 AirSim 源 (有云台)
            ok = (self.veh_ctrl.is_flying()
                  and self.camera_tab.grabber.get_gimbal() is not None)
        if self.landing_btn.isEnabled() != ok:
            self.landing_btn.setEnabled(ok)

    def _on_landing_toggle(self):
        if self.landing_btn.isChecked():
            # 反控前自动把云台俯仰打到垂直向下, 省去每次手动调整
            self.camera_tab.grabber.gimbal_pitch_bottom()
            # 链路参数跟随工具栏所选链路 (mavros / uxrce)
            link = self._selected_link() or 'mavros'
            cmd = list(config.LANDING_CMD) + ['--ros-args', '-p', f'link:={link}']
            self.land_proc.start(cmd)
            # 新进程要完成 DDS 发现 + 收到首帧飞控状态 (共 1~3s), 单发一条
            # landing_start 撞上任何一步都会被吞掉 (表现为"第一次点击无效"),
            # 故启动后重发多次: 节点对重复 start 幂等, 多发无副作用
            QTimer.singleShot(1000, lambda: self._send_landing_start(6))
        else:
            self.landing_btn.setText('视觉反控')
            self._stop_landing()

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
            self.landing_btn.setText('反控中: 对准下降')
            return
        if status == 'landing':
            self.landing_btn.setText('反控中: 自动降落')
            return
        if status == 'landed':
            self.log.info('\033[32mArUco 视觉降落完成, 飞机已上锁\033[0m')
            self._flash_landing_btn('#2d8a3e')      # 绿色高亮闪一下
        elif status == 'idle':
            self.log.info('降落流程已结束 (取消/中止), 按钮已复位')
        else:
            return
        # 按钮弹起 (checked = 降落进行中) + 文字复位 + 回收降落节点进程
        self.landing_btn.setChecked(False)
        self.landing_btn.setText('视觉反控')
        self.land_proc.stop()

    def _flash_landing_btn(self, color):
        """按钮背景色闪烁提示, 1.5s 后恢复原样式。"""
        self.landing_btn.setStyleSheet(
            self._TOOL_BTN_STYLE +
            f'\nQToolButton {{ background-color: {color}; color: white; }}')
        QTimer.singleShot(1500, lambda: self.landing_btn.setStyleSheet(
            self._TOOL_BTN_STYLE))

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
            self.landing_btn.setText('视觉反控')
            return
        self.node.publish_cmd(config.LANDING_START_CMD)
        if retries_left > 0:
            QTimer.singleShot(500, lambda: self._send_landing_start(retries_left - 1))

    def _stop_landing(self):
        """取消降落: 发 landing_stop (节点自行切回 LOITER 交还控制权并清标注),
        稍后回收进程, 给节点留出发送停止指令与清屏的时间。"""
        self.node.publish_cmd(config.LANDING_STOP_CMD)
        QTimer.singleShot(500, self.land_proc.stop)

    # ---------- 手动遥控 (toggle + 双摇杆, 执行节点 flylab_vision/teleop_node) ----------

    def _build_teleop_panel(self):
        box = QGroupBox('手动遥控')
        lay = QVBoxLayout(box)
        lay.setContentsMargins(6, 4, 6, 6)
        self.teleop_btn = QPushButton('开启遥控')
        self.teleop_btn.setCheckable(True)
        self.teleop_btn.setEnabled(False)
        self.teleop_btn.setStyleSheet(self._LAND_BTN_STYLE)
        self.teleop_btn.setToolTip(
            '需先: 1) 工具栏开启通信链路  2) 飞行控制起飞升空。'
            '开启后接管 offboard, 摇杆控速度 (回中=悬停); 关闭交还控制权')
        self.teleop_btn.toggled.connect(self._on_teleop_toggle)
        lay.addWidget(self.teleop_btn)
        # 双摇杆 (同真实遥控器): 左 = 升降/偏航, 右 = 平移 (力度=偏离圆心距离)
        pads = QHBoxLayout()
        self._teleop_widgets = []

        def joy_group(title, cb):
            g = QGroupBox(title)
            g_lay = QVBoxLayout(g)
            g_lay.setContentsMargins(4, 2, 4, 4)
            joy = VirtualJoystick()
            joy.changed.connect(cb)
            g_lay.addWidget(joy, alignment=Qt.AlignHCenter)
            self._teleop_widgets.append(joy)
            return joy, g

        self._teleop_joy_l, g_l = joy_group(
            '高度 / 偏航',
            # 左摇杆: x 右拖=右转(右+, w 轴左转+ 取反), y 上拖=升
            lambda x, y: (self._teleop_axis('w', -x), self._teleop_axis('z', y)))
        self._teleop_joy_r, g_r = joy_group(
            '平移 (机头方向)',
            # 右摇杆: x 右拖=右移, y 上拖=前进
            lambda x, y: (self._teleop_axis('y', x), self._teleop_axis('x', y)))
        pads.addWidget(g_l)
        pads.addWidget(g_r)
        lay.addLayout(pads)
        for w in self._teleop_widgets:   # 默认置灰, 接管后解锁
            w.setEnabled(False)
        return box

    def _teleop_axis(self, key, val):
        """更新单轴归一化量并立即下发 (摇杆拖动中实时跟手)。"""
        if not self.teleop_btn.isChecked():
            return
        self._teleop_axes[key] = val
        a = self._teleop_axes
        self.node.publish_teleop_twist(a['x'], a['y'], a['z'], a['w'])

    def _teleop_clear_axes(self):
        """界面侧全部回中 (交还/复位时调用), 防止残留遥控量。
        摇杆 reset 会经 changed 信号回写各轴, 最后统一发零。"""
        self._teleop_joy_l.reset()
        self._teleop_joy_r.reset()
        for key in self._teleop_axes:
            self._teleop_axes[key] = 0.0
        self.node.publish_teleop_twist(0.0, 0.0, 0.0, 0.0)

    def _on_teleop_toggle(self):
        if self.teleop_btn.isChecked():
            # 遥控接管优先 (防炸机出口): 反控进行中先取消降落, 降落节点
            # 切回 LOITER 交还 offboard, 遥控 1s 后再接管不会打架
            if self.landing_btn.isChecked():
                self.landing_btn.setChecked(False)
            # 避障链路 (EGO/SUPER) 与遥控互斥: 跟踪器同样走 offboard,
            # 开遥控先整链路退出, offboard 交由遥控节点独占
            if self.running_flight is not None and self.running_flight.get('need_link'):
                self._stop_flight()
                self.log.info('手动遥控接管, 已自动退出避障链路')
            # 链路参数跟随工具栏所选链路 (mavros / uxrce)
            link = self._selected_link() or 'mavros'
            cmd = list(config.TELEOP_CMD) + ['--ros-args', '-p', f'link:={link}']
            self.teleop_proc.start(cmd)
            # 等节点完成 DDS 发现 + 收到飞控状态后再发接管指令 (重发多次, 节点幂等)
            QTimer.singleShot(1000, lambda: self._send_teleop_start(6))
        else:
            self.node.publish_cmd(config.TELEOP_STOP_CMD)
            self._teleop_clear_axes()
            self._set_teleop_controls_enabled(False)
            self.teleop_btn.setText('交还控制权 ...')
            QTimer.singleShot(1500, self.teleop_proc.stop)   # 留时间走完交还流程

    def _send_teleop_start(self, retries_left):
        if not self.teleop_btn.isChecked():
            return
        rc = self.teleop_proc.exited()
        if rc is not None:
            self.log.error(f'遥控节点进程启动失败 (退出码 {rc}), '
                           f'检查编译: colcon build --packages-select flylab_vision')
            self.teleop_proc.clear()
            self._teleop_reset()
            return
        self.node.publish_cmd(config.TELEOP_START_CMD)
        if retries_left > 0:
            QTimer.singleShot(500, lambda: self._send_teleop_start(retries_left - 1))

    def _check_teleop_status(self):
        """遥控节点状态事件: engaged=接管成功 (解锁摇杆), idle/failed/error=
        已结束 (按钮弹起 + 回收进程)。状态带序号, 只处理新事件。"""
        status, seq = self.node.get_teleop_status()
        if seq == self._last_teleop_seq:
            return
        self._last_teleop_seq = seq
        if not status:
            return
        if status == 'engaged':
            self.teleop_btn.setText('遥控中 (OFFBOARD)')
            self._set_teleop_controls_enabled(True)
        elif status == 'engaging':
            self.teleop_btn.setText('接管中 ...')
        elif status == 'releasing':
            self.teleop_btn.setText('交还控制权 ...')
        else:   # idle / failed / error:*
            if status.startswith('error:') or status == 'failed':
                self.log.error(f'手动遥控: {status}')
            self.teleop_proc.stop()
            self.teleop_proc.clear()
            self._teleop_reset()

    def _teleop_reset(self):
        """按钮/摇杆复位为空闲态 (异常或流程结束调用)。"""
        self._teleop_clear_axes()
        self._set_teleop_controls_enabled(False)
        self.teleop_btn.setChecked(False)
        self.teleop_btn.setText('开启遥控')

    def _set_teleop_controls_enabled(self, enabled):
        for w in self._teleop_widgets:
            w.setEnabled(enabled)

    # ---------- 视觉反控 (工具栏 toggle = ArUco降落, 执行节点 aruco_landing_node) ----------

    def _build_landing_button(self):
        """视觉反控工具按钮 (ArUco 视觉降落): 点击拉起 aruco_landing_node
        并接管 offboard, 按视觉输出对准码并调速下降, 满足条件切 AUTO.LAND;
        再点取消降落交还控制权。仅 ArUco检测运行中可点。"""
        self.landing_btn = QToolButton()
        self.landing_btn.setText('视觉反控')
        self.landing_btn.setCheckable(True)
        self.landing_btn.setEnabled(False)
        self.landing_btn.setStyleSheet(self._TOOL_BTN_STYLE)
        self.landing_btn.setToolTip(
            '根据当前算法的视觉输出控制飞机; '
            '开启手动遥控可随时抢回控制权')
        self.landing_btn.toggled.connect(self._on_landing_toggle)
        return self.landing_btn

    # ---------- 主轮询 (33ms, GUI 线程) ----------

    def _poll_annotations(self):
        """轮询入口兜底: 本方法跑在 Qt 定时器槽里, PyQt5 对槽内未捕获的
        Python 异常会直接 qFatal abort 整个进程 (界面"自己退出"的元凶,
        进程树无人回收)。任何刷新环节抛异常只记日志跳过本拍, 不带崩界面。"""
        try:
            self._poll_once()
        except Exception:
            self._log_poll_error()

    def _log_poll_error(self):
        """轮询异常限频上屏 (同一异常风暴 5s 一条, 防刷屏)。"""
        now = time.monotonic()
        if now - self._poll_err_last < 5.0:
            return
        self._poll_err_last = now
        self.log.error('轮询异常 (已跳过本拍, 界面继续运行):\n'
                       + traceback.format_exc())

    def _poll_once(self):
        # 驱动 ROS 收包 (节点没有独立 spin 线程, 由本定时器在 GUI 线程代转,
        # 回调天然在主线程执行, 无并发问题); 不 spin 则订阅永远收不到消息
        try:
            rclpy.spin_once(self.node, timeout_sec=0)
        except Exception:
            pass
        # 遥测六值条 + 罗盘 + 航线地图刷新 (数据随 0.5s 飞控心跳采样)
        tele = self.veh_ctrl.get_telemetry()
        self.tele_bar.update_values(tele)
        self.compass.set_heading(tele.get('hdg') or 0.0)
        track, home = self.veh_ctrl.get_track()
        self.map_view.update_state(track, home, tele.get('hdg') or 0.0)
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
        self._check_flight_alive()   # 链路进程异常退出则复位按钮
        self._check_px4_alive()      # PX4 进程异常退出则复位按钮
        self._check_link_alive()     # 桥进程异常退出则复位按钮
        # 飞行控制按钮: 需 PX4 SITL 已启动 + 飞控 RPC 已连通才可点
        # (PX4 没起时 AirSim 对该机型报默认状态且命令必失败, 直接置灰);
        # 手动遥控/视觉反控接管中同样置灰 (offboard 期间飞控命令会打断
        # 接管, 想干预先关对应功能交还控制权); 控制权在本按钮时点亮 ——
        # 飞机在空中且未被遥控/视觉反控接管, 落地或接管结束后恢复常态
        veh_ok = (self.px4.is_running() and self.veh_ctrl.is_connected()
                  and not self.teleop_btn.isChecked()
                  and not self.landing_btn.isChecked())
        if self.veh_btn.isEnabled() != veh_ok:
            self.veh_btn.setEnabled(veh_ok)
        self._set_btn_on(self.veh_btn, veh_ok and self.veh_ctrl.is_flying()
                         and not self.teleop_btn.isChecked()
                         and not self.landing_btn.isChecked())
        self._check_landing_status()  # 降落结果反馈 (成功/中止 → 按钮弹起)
        self._check_teleop_status()   # 遥控状态 (接管成功/交还/失败 → 按钮与摇杆联动)
        self._update_landing_btn()   # 降落(视觉反控)/遥控互斥门控
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
        if self.running_flight is not None:
            self._stop_flight()          # 链路随地面站退出一并回收
        if self.landing_btn.isChecked():
            self.node.publish_cmd(config.LANDING_STOP_CMD)
        self.land_proc.stop()
        if self.teleop_btn.isChecked():
            self.node.publish_cmd(config.TELEOP_STOP_CMD)   # 交还控制权再回收
        self.teleop_proc.stop()
        self._stop_px4()             # PX4 SITL 随地面站退出一并回收
        for key in list(self.link_procs):
            self._stop_link(key)
        self.veh_ctrl.stop()         # 飞控 RPC 会话线程
        self.camera_tab.shutdown()
