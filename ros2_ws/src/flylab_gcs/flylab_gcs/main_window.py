#!/usr/bin/env python3
"""地面站主窗口 —— 界面组装与调度中枢。

模块分层 (细节见各文件):
  config.py           全局配置 (话题/端口/机型/算法注册表, 改这里)
  ros_node.py         ROS 节点 (订阅/发布 + TeeLogger 日志分流)
  camera_grabber.py   视频源抓取线程 (AirSim RPC/本机摄像头/文件) + 软件云台
                      (支撑: quat_math.py / gimbal.py / camera_sources.py)
  vehicle_control.py  飞行控制 (独立 RPC 会话: 解锁/起飞/降落/悬停/返航/上锁)
  processes.py        托管子进程 (PX4 SITL/通信链路/算法/降落节点的生命周期)
  log_panel.py        画面下方日志框 (TeeLogger 分流 + PX4 输出接管, 抗刷屏)
  功能域面板 (mixin, 纯搬移自本文件, 见 panels/ 子包):
    panels/px4_panel.py      PX4 SITL 一键启停 (+ AirSim 导出项目联动)
    panels/link_panel.py     通信链路互斥起停 (uXRCE-DDS / MAVROS)
    panels/vehicle_panel.py  飞行控制命令菜单 (AirSim RPC 直发)
    panels/algo_panel.py     视觉算法启停
    panels/flight_panel.py   飞行链路一键 launch
    panels/control_panels.py 接管控制菜单 (视觉反控 ArUco降落 + 手动遥控 offboard 互斥门控)
  main_window.py      本文件: 工具栏/面板组装 + 33ms 轮询调度 + 门控汇总

界面结构:
  顶部工具栏 (QToolBar): 视频源 / PX4 SITL / 飞行控制 / 通信链路 /
  视觉算法 / 飞行链路 / 接管控制, 全部下拉或 toggle 交互
  左侧画面 + 底部日志框 (camera_widget 内部布局)
  右栏: 航线地图 + 罗盘 → 云台 (视场/变焦) → 手动遥控双摇杆
"""
import os
import time
import traceback

import rclpy
from PyQt5.QtCore import QTimer, pyqtSignal
from PyQt5.QtWidgets import QHBoxLayout, QMainWindow, QToolBar, QWidget

from . import config
from .algo_output_panel import AlgoOutputPanel
from .camera_widget import CameraViewWidget
from .compass import Compass
from .log_panel import LogPanel
from .map_view import MapView
from .panels import ui_style
from .panels.algo_panel import AlgoPanelMixin
from .panels.control_panels import ControlPanelsMixin
from .panels.flight_panel import FlightPanelMixin
from .panels.link_panel import LinkPanelMixin
from .panels.px4_panel import Px4PanelMixin
from .panels.vehicle_panel import VehiclePanelMixin
from .processes import ManagedProcess
from .telemetry_bar import TelemetryBar
from .vehicle_control import VehicleController


class MainWindow(Px4PanelMixin, LinkPanelMixin, VehiclePanelMixin,
                 AlgoPanelMixin, FlightPanelMixin, ControlPanelsMixin,
                 QMainWindow):
    """主窗口: 组装各功能域面板 (mixin 提供) + 33ms 主轮询调度,
    功能域方法通过 self.* 访问主窗口共享状态。"""

    # 日志行 (level, text) —— 任意线程可 emit, 经队列投递到 GUI 线程上屏
    log_line = pyqtSignal(str, str)

    # 工具栏按钮样式 (mixin 引用, 常量本体见 panels/ui_style.py)
    _TOOL_BTN_STYLE = ui_style.TOOL_BTN_STYLE

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
        self.airsim_proc = ManagedProcess(self.log, 'AirSim 项目')  # PX4 联动的 UE 导出项目

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
        # 视场角同步: 上次已发布的 FOV 值 (变化才发布, None=尚未发布过)
        self._fov_pub_last = None

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
        self.toolbar.addWidget(self._build_takeover_button())
        self.addToolBar(self.toolbar)

        # 右栏布局 (自上而下): 航线地图 + 罗盘并排一行 → 云台 (视场角
        # 竖排在云台方向键右侧) → 手动遥控
        self.compass = Compass()
        self.map_view = MapView()   # 俯视轨迹, 数据同遥测: 飞控心跳采样
        map_row = QWidget()
        map_lay = QHBoxLayout(map_row)
        map_lay.setContentsMargins(0, 0, 0, 0)
        map_lay.addWidget(self.map_view, stretch=3)
        map_lay.addWidget(self.compass, stretch=1)
        self.camera_tab.add_side_widget(map_row)
        self.camera_tab.add_side_widget(self.camera_tab.gimbal_box)
        self.camera_tab.add_side_widget(self._build_teleop_panel())
        # 算法信息面板 (右栏底部): 默认空白, 运行中算法动态显示对应分组
        # (config.ALGORITHMS 的 'panel' 键 + 视觉反控降落); 带码边长滑杆,
        # 拖动经 latched 话题同步 aruco_node / aruco_landing_node
        self.algo_panel = AlgoOutputPanel(
            self.node.get_aruco_info, self.node.publish_marker_length,
            self.node.publish_landing_yaw_offset)
        self.camera_tab.add_side_widget(self.algo_panel)

        self.setCentralWidget(self.camera_tab)

        # 33ms 主轮询: 标注上屏/进程存活/按钮门控/画面转发 (见 _poll_annotations)
        self.anno_timer = QTimer(self)
        self.anno_timer.timeout.connect(self._poll_annotations)
        self.anno_timer.start(33)

    @staticmethod
    def _set_btn_on(btn, on):
        """工具按钮状态灯: on 动态属性切换点亮/熄灭, 状态不变不重刷。"""
        if bool(btn.property('on') or False) == bool(on):
            return
        btn.setProperty('on', bool(on))
        btn.style().unpolish(btn)
        btn.style().polish(btn)

    # ---------- 主轮询 (33ms, GUI 线程) ----------

    def _poll_annotations(self):
        """轮询兜底: Qt 定时器槽内未捕获异常会 qFatal 杀掉整个进程,
        任何刷新环节抛异常只记日志跳过本拍。"""
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
        # 驱动 ROS 收包 (节点无独立 spin 线程, 回调因此天然在主线程执行)
        try:
            rclpy.spin_once(self.node, timeout_sec=0)
        except Exception:
            pass
        # 遥测条 + 罗盘 + 航线地图刷新 (数据随 0.5s 飞控心跳采样)
        tele = self.veh_ctrl.get_telemetry()
        self.tele_bar.update_values(tele)
        self.compass.set_heading(tele.get('hdg') or 0.0)
        track, home = self.veh_ctrl.get_track()
        self.map_view.update_state(track, home, tele.get('hdg') or 0.0)
        # 标注门控: 仅算法运行中或降落进行中时显示, 其余情况清掉过期标注
        if self.running_algo is None and not self.landing_act.isChecked():
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
        # 算法信息面板: 可见性随运行中算法变化 + 数据刷新 (内部去重);
        # ArUco 降落分组在运行 enables_landing 算法 (ArUco检测) 期间常驻,
        # 未点视觉反控只是无数据显示 '—', 不随降落按钮出现/消失
        active = []
        if self.running_algo is not None and self.running_algo.get('panel'):
            active.append(self.running_algo['panel'])
            if self.running_algo.get('enables_landing'):
                active.append('landing')
        if self.landing_act.isChecked():
            active.append('landing')
        self.algo_panel.set_active(active)
        self.algo_panel.refresh()
        self._check_algo_alive()
        self._check_flight_alive()   # 链路进程异常退出则复位按钮
        self._check_px4_alive()      # PX4 进程异常退出则复位按钮
        self._check_link_alive()     # 桥进程异常退出则复位按钮
        # 飞行控制按钮: 需 PX4 已启动 + RPC 已连通, 且未被遥控/视觉反控
        # 接管 (offboard 期间飞控命令会打断接管); 飞行中未被接管时点亮
        veh_ok = (self.px4.is_running() and self.veh_ctrl.is_connected()
                  and not self.teleop_act.isChecked()
                  and not self.landing_act.isChecked())
        if self.veh_btn.isEnabled() != veh_ok:
            self.veh_btn.setEnabled(veh_ok)
        self._set_btn_on(self.veh_btn, veh_ok and self.veh_ctrl.is_flying()
                         and not self.teleop_act.isChecked()
                         and not self.landing_act.isChecked())
        self._check_landing_status()  # 降落结果反馈 (成功/中止 → 菜单项弹起)
        self._check_teleop_status()   # 遥控状态 (接管成功/交还/失败 → 菜单项与摇杆联动)
        self._update_takeover_gates()   # 降落(视觉反控)/遥控互斥门控
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
        # 视场角同步: 用户调整过 FOV 时变化即发布 (滑杆拖动靠轮询节流);
        # 未调整过时抓取线程连上 AirSim 后已采纳相机真实 FOV (见
        # Grabber._adopt_camera_fov), 这里照常发布, ArUco 等算法节点启动即同步
        fov = self.camera_tab.grabber.get_fov_applied()
        if fov is not None and fov != self._fov_pub_last:
            self._fov_pub_last = fov
            self.node.publish_camera_fov(fov)

    def _publish_display_frame(self):
        """取当前显示帧交给转发线程 (深度伪彩帧跳过)。
        仅 O(1) 引用传递, 序列化/publish 在 DisplayForwarder 线程完成。"""
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
        # 启动后 3s 是 DDS 发现窗口, 计数 0 属正常不告警
        if cnt == 0 and now - self._algo_started_at > 3.0:
            self.log.warning(
                f'画面转发: 近 2 秒 0 帧 (源 {self._last_src or "?"} 无图像?)')

    def shutdown(self):
        """退出: 回收全部托管进程, 停掉后台线程。"""
        if self.running_algo is not None:
            self._stop_algo()
        if self.running_flight is not None:
            self._stop_flight()          # 链路随地面站退出一并回收
        if self.landing_act.isChecked():
            self.node.publish_cmd(config.LANDING_STOP_CMD)
        self.land_proc.stop()
        if self.teleop_act.isChecked():
            self.node.publish_cmd(config.TELEOP_STOP_CMD)   # 交还控制权再回收
        self.teleop_proc.stop()
        self._stop_px4()             # PX4 SITL 随地面站退出一并回收
        for key in list(self.link_procs):
            self._stop_link(key)
        self.veh_ctrl.stop()         # 飞控 RPC 会话线程
        self.camera_tab.shutdown()
