#!/usr/bin/env python3
"""地面站主窗口 —— 左侧画面 + 右侧工具栏 + 模型输出叠加。

布局：画面占主区域, 右栏是输入源选择 + 算法选择 (下拉框 + 运行/取消) + 状态。
模型输出不画在界面里：模型节点往 config.ANNOTATION_TOPIC 发 JSON 标注
(见 overlay.py)，本窗口轮询并叠加到画面上。
加新算法 = 在 config.ALGORITHMS 里加一项, 界面交互对所有算法一致。
算法进程生命周期也由这里管理: 配了 cmd 的算法, 点运行自动拉起进程,
点取消自动回收 (未配 cmd 则假定节点已在外部运行, 只发启停指令)。
"""
import os
import subprocess
import time

import rclpy
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (QComboBox, QHBoxLayout, QMainWindow, QPushButton,
                           QVBoxLayout, QWidget)

from . import config
from .camera_widget import CameraViewWidget


class MainWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.setWindowTitle('FLYLAB 地面站')
        self.node = node

        # 输入源视图 (UE 相机/本机摄像头/视频文件), 右栏布局
        host = os.environ.get('PX4_SIM_HOST_ADDR', '127.0.0.1')
        self.camera_tab = CameraViewWidget(
            self.node.get_logger().info, host, config.AIRSIM_RPC_PORT,
            side_panel=True)

        # 算法选择: 下拉框选算法 + 运行/取消按钮, 插到右栏输入源下方。
        # 同一时刻只跑一个算法; 运行中下拉框锁定, 取消后恢复。
        # 配了 cmd 的算法点运行时自动拉起进程 (用户无需手动 ros2 run)
        self.running_algo = None
        self.proc = None       # 本地面站拉起的算法进程 (Popen), 无则 None
        self.land_proc = None  # ArUco 降落节点进程 (独立于算法进程)
        # 显示帧转发统计 (运行中每 2 秒打一条日志)
        self._fwd_count = 0
        self._fwd_last = time.monotonic()
        self._last_src, self._last_fps = '', 0.0
        self.algo_panel = QWidget()
        algo_lay = QVBoxLayout(self.algo_panel)
        algo_lay.setContentsMargins(0, 0, 0, 0)
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
        # 按下: 拉起 aruco_landing_node 并发 landing_start 触发三态状态机;
        # 抬起: 发 landing_stop 取消降落并交还控制权。
        # 默认禁用, 门控见 _update_landing_btn: 需 ArUco检测运行中 且
        # 云台俯仰转到垂直向下才可点 (降落进行中保持可点, 随时可取消)。
        self.landing_btn = QPushButton('ArUco降落')
        self.landing_btn.setCheckable(True)
        self.landing_btn.setEnabled(False)
        self.landing_btn.setToolTip(
            '需先: 1) 运行 ArUco检测  2) 云台俯仰调到垂直向下 (↓到限位)')
        self.landing_btn.setStyleSheet(self._LAND_BTN_STYLE)
        self.landing_btn.clicked.connect(self._on_landing_toggle)
        algo_lay.addWidget(self.landing_btn)
        self.camera_tab.add_side_widget(self.algo_panel)

        self.setCentralWidget(self.camera_tab)

        # 模型标注叠加: 轮询 ROS 节点缓存的最新标注 (在 ROS 线程更新, 界面线程绘制)
        self.anno_timer = QTimer(self)
        self.anno_timer.timeout.connect(self._poll_annotations)
        self.anno_timer.start(33)

    _BTN_STYLE = """
        QPushButton { min-height: 32px; }
        QPushButton:checked { background-color: #2d8a3e; color: white; }
    """

    _LAND_BTN_STYLE = """
        QPushButton { min-height: 32px; }
        QPushButton:checked { background-color: #b03030; color: white; }
    """

    # ---------- 算法运行/取消 (含进程生命周期管理) ----------

    def _on_run_algo(self):
        """运行选中的算法: 拉起进程 (若配了 cmd) 并发启动指令;
        若已有算法在跑, 先停掉旧的再启动新的 (无缝切换)。"""
        algo = config.ALGORITHMS[self.algo_combo.currentIndex()]
        if self.running_algo is not None and self.running_algo is not algo:
            self._stop_algo()
        self.running_algo = algo
        self.proc = self._launch_process(algo)
        # 新进程的 DDS 发现需要约 1s, 延迟发 start; 节点对重复 start 无副作用
        delay_ms = 1000 if self.proc is not None else 0
        QTimer.singleShot(delay_ms, lambda: self._send_start(algo))
        self._update_algo_state()

    def _launch_process(self, algo):
        """拉起算法进程; 未配 cmd 返回 None (外部已运行模式)。
        子进程日志直通本终端, 便于排查链路问题。"""
        cmd = algo.get('cmd')
        if not cmd:
            return None
        self.node.get_logger().info(f'启动算法进程: {" ".join(cmd)}')
        return subprocess.Popen(cmd)

    def _send_start(self, algo):
        """延迟发送启动指令; 触发前算法被取消/切换/进程已死则跳过。"""
        if self.running_algo is not algo:
            return
        if self.proc is not None and self.proc.poll() is not None:
            self.node.get_logger().error(
                f'算法进程启动失败 (退出码 {self.proc.returncode}), '
                f'请检查 config.ALGORITHMS 里的 cmd')
            self._reset_algo()
            return
        self.node.publish_cmd(algo['start'])

    def _on_cancel_algo(self):
        """取消当前正在运行的算法 (停止的是运行中的, 与下拉框当前选中无关)。"""
        if self.running_algo is None:
            return
        self._stop_algo()
        self._update_algo_state()

    # ---------- ArUco 降落 (独立 toggle 按钮) ----------

    def _update_landing_btn(self):
        """降落按钮门控 (33ms 轮询): ArUco检测运行中 且 云台俯仰到垂直向下
        限位 (绝对 -90°, 即相对俯仰到 GIMBAL_PITCH_MIN_DEG) 才可点。
        降落进行中不参与门控 —— 保持可点, 随时可取消交还控制权。"""
        if self.landing_btn.isChecked():
            return
        ok = False
        algo = self.running_algo
        if algo is not None and algo.get('enables_landing'):
            gim = self.camera_tab.grabber.get_gimbal()
            if gim is not None:
                _, pitch = gim
                ok = pitch <= config.GIMBAL_PITCH_MIN_DEG + 1e-3
        if self.landing_btn.isEnabled() != ok:
            self.landing_btn.setEnabled(ok)

    def _on_landing_toggle(self):
        if self.landing_btn.isChecked():
            self.node.get_logger().info(
                f'启动降落节点进程: {" ".join(config.LANDING_CMD)}')
            self.land_proc = subprocess.Popen(config.LANDING_CMD)
            # 新进程 DDS 发现需要约 1s, 延迟发触发指令 (期间取消则跳过)
            QTimer.singleShot(1000, self._send_landing_start)
        else:
            self._stop_landing()

    def _send_landing_start(self):
        """延迟触发降落; 按钮已抬起或节点进程已死则跳过。"""
        if not self.landing_btn.isChecked():
            return
        if self.land_proc is not None and self.land_proc.poll() is not None:
            self.node.get_logger().error(
                f'降落节点进程启动失败 (退出码 {self.land_proc.returncode}), '
                f'检查编译: colcon build --packages-select flylab_vision')
            self.land_proc = None
            self.landing_btn.setChecked(False)
            return
        self.node.publish_cmd(config.LANDING_START_CMD)

    def _stop_landing(self):
        """取消降落: 发 landing_stop (节点自行切回 LOITER 交还控制权并清标注),
        稍后回收进程, 给节点留出发送停止指令与清屏的时间。"""
        self.node.publish_cmd(config.LANDING_STOP_CMD)
        proc = self.land_proc
        self.land_proc = None

        def _reap():
            if proc is not None and proc.poll() is None:
                proc.terminate()
        QTimer.singleShot(500, _reap)

    def _stop_algo(self):
        """停掉正在运行的算法: 发停止指令 + 界面立即清标注 + 回收进程。"""
        algo = self.running_algo
        if algo is None:
            return
        self.node.publish_cmd(algo['stop'])
        self.node.clear_annotations()
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()
        self.running_algo = None
        self.proc = None

    def _reset_algo(self):
        """算法异常退出时复位状态 (不重复发停止指令)。"""
        self.running_algo = None
        self.proc = None
        self.node.clear_annotations()
        self._update_algo_state()

    def _check_algo_alive(self):
        """轮询里顺带检查进程存活: 异常退出则复位并提示。"""
        if (self.running_algo is not None and self.proc is not None
                and self.proc.poll() is not None):
            self.node.get_logger().error(
                f'算法进程异常退出 (退出码 {self.proc.returncode})')
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

    def _poll_annotations(self):
        # 驱动 ROS 收包 (节点没有独立 spin 线程, 由本定时器在 GUI 线程代转,
        # 回调天然在主线程执行, 无并发问题); 不 spin 则订阅永远收不到消息
        try:
            rclpy.spin_once(self.node, timeout_sec=0)
        except Exception:
            pass
        self.camera_tab.video.set_annotations(self.node.get_annotations())
        # HUD 第二行: 算法输出帧率 (运行中才显示, 停止/无输出自动隐藏)
        fps = self.node.get_algo_fps() if self.running_algo is not None else 0.0
        self.camera_tab.video.set_algo_hud(
            f'解算 {fps:.1f}' if fps > 0 else None)
        self._check_algo_alive()
        self._update_landing_btn()   # 降落按钮门控 (检测运行中 + 云台垂直向下)
        # 显示画面转发: 有算法节点订阅时, 把当前显示的帧发给它
        if config.DISPLAY_IMAGE_HZ > 0:
            self._publish_display_frame()

    def _publish_display_frame(self):
        """取当前显示的帧转发给算法节点 (深度伪彩帧跳过, 无检测意义)。"""
        frame, src, fps = self.camera_tab.grabber.get()
        if frame is None:
            return
        self._last_src, self._last_fps = src, fps
        arr, is_depth = frame
        if is_depth or arr.ndim != 3:
            return
        if self.node.publish_frame(arr):
            self._fwd_count += 1
        self._log_forward_stats()

    def _log_forward_stats(self):
        """运行中每 2 秒汇报一次画面转发情况, 链路断在哪一眼可见。"""
        now = time.monotonic()
        if now - self._fwd_last < 2.0:
            return
        self._fwd_last = now
        cnt, subs = self._fwd_count, self.node.image_pub.get_subscription_count()
        self._fwd_count = 0
        if self.running_algo is None:
            return
        if cnt == 0:
            self.node.get_logger().warning(
                f'画面转发: 近 2 秒 0 帧 (源 {self._last_src or "?"} 无图像?)')
        else:
            # 例行统计日志 (排查链路时再打开):
            # self.node.get_logger().info(
            #     f'画面转发: {self._last_src} @{self._last_fps:.0f}fps, '
            #     f'近 2 秒发 {cnt} 帧, 算法订阅者 {subs}')
            pass

    def shutdown(self):
        """优雅退出: 回收算法进程, 停掉后台线程。"""
        if self.running_algo is not None:
            self._stop_algo()
        if self.land_proc is not None:
            if self.land_proc.poll() is None:
                self.node.publish_cmd(config.LANDING_STOP_CMD)
                self.land_proc.terminate()
            self.land_proc = None
        self.camera_tab.shutdown()
