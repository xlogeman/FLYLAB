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
import signal
import subprocess
import time

import rclpy
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (QComboBox, QGroupBox, QHBoxLayout, QMainWindow,
                           QPushButton, QVBoxLayout, QWidget)

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
        self._last_landing_seq = 0  # 已处理的降落状态事件序号
        # 通信链路进程 (地面站一键拉起, 省开终端): uXRCE-DDS Agent / MAVROS
        self.link_procs = {}   # key -> Popen
        self.link_btns = {}    # key -> QPushButton
        self.link_titles = {}  # key -> 按钮标题 (日志用)
        # 显示帧转发统计 (运行中每 2 秒打一条日志)
        self._fwd_count = 0
        self._fwd_last = time.monotonic()
        self._last_src, self._last_fps = '', 0.0
        # 相机机体系姿态发布节流 (10Hz, 供 aruco_landing 云台泛化映射)
        self._cam_ori_last = 0.0
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
        # 云台俯仰转到垂直向下 且 已选择通信链路 才可点
        # (降落进行中保持可点, 随时可取消)。
        self.landing_btn = QPushButton('ArUco降落')
        self.landing_btn.setCheckable(True)
        self.landing_btn.setEnabled(False)
        self.landing_btn.setToolTip(
            '需先: 1) 运行 ArUco检测  2) 云台俯仰调到垂直向下 (↓到限位)  '
            '3) 下方选择通信链路 (uXRCE-DDS / MAVROS)')
        self.landing_btn.setStyleSheet(self._LAND_BTN_STYLE)
        self.landing_btn.clicked.connect(self._on_landing_toggle)
        algo_lay.addWidget(self.landing_btn)
        # 右栏三个分组框: 输入源 / 云台 (camera_widget) + 算法 / 通信链路 (这里)
        self.camera_tab.add_side_widget(self.algo_panel)
        self.camera_tab.add_side_widget(self._build_link_panel())

        self.setCentralWidget(self.camera_tab)

        # 模型标注叠加: 轮询 ROS 节点缓存的最新标注 (在 ROS 线程更新, 界面线程绘制)
        self.anno_timer = QTimer(self)
        self.anno_timer.timeout.connect(self._poll_annotations)
        self.anno_timer.start(33)

    _BTN_STYLE = """
        QPushButton { min-height: 32px; }
        QPushButton:checked { background-color: #2d8a3e; color: white; }
    """

    # ---------- 通信链路面板 (uXRCE-DDS / MAVROS 一键起停) ----------

    def _build_link_panel(self):
        """通信链路分组: 两个 toggle 按钮并排一行, 互斥 (一次只开一条链路)"""
        box = QGroupBox('通信链路')
        lay = QVBoxLayout(box)
        lay.setContentsMargins(6, 4, 6, 6)
        row = QHBoxLayout()
        row.setSpacing(6)
        for key, text, cmd in (
                ('uxrce', 'uXRCE-DDS', config.UXRCE_CMD),
                ('mavros', 'MAVROS', config.MAVROS_CMD)):
            btn = QPushButton(text)
            btn.setCheckable(True)
            btn.setStyleSheet(self._BTN_STYLE)
            btn.setToolTip(' '.join(cmd))
            # toggled (而非 clicked): 状态变化即处理, 便于互斥时联动另一条链路
            btn.toggled.connect(
                lambda checked, k=key, c=cmd, t=text: self._on_link_toggled(k, checked, c, t))
            row.addWidget(btn, stretch=1)
            self.link_btns[key] = btn
            self.link_titles[key] = text
        lay.addLayout(row)
        return box

    def _on_link_toggled(self, key, checked, cmd, title):
        """链路按钮 toggle: 起/停对应桥进程; 两条链路互斥, 开启一条先停另一条。"""
        if not checked:
            self._stop_link(key, title)
            return
        # 互斥: 开启一条链路前, 把另一条停掉 (按钮一起弹起)
        for other_key, other_btn in self.link_btns.items():
            if other_key != key and other_btn.isChecked():
                self._stop_link(other_key, self.link_titles.get(other_key))
        self._start_link(key, cmd, title)

    def _start_link(self, key, cmd, title):
        """拉起桥进程 (独立进程组, 停止时整组回收, 避免 ros2 run 留下孤儿)。"""
        if key in self.link_procs and self.link_procs[key].poll() is None:
            return
        self.node.get_logger().info(f'启动 {title}: {" ".join(cmd)}')
        try:
            # start_new_session=True: 子进程独立进程组, 停止时 killpg 整组回收
            proc = subprocess.Popen(cmd, start_new_session=True)
        except (OSError, FileNotFoundError) as e:
            self.node.get_logger().error(
                f'{title} 启动失败: {e} (可执行文件是否在 PATH?)')
            self._set_link_btn(key, False)
            return
        self.link_procs[key] = proc
        # 0.5s 后自检: 立即退出说明命令有问题 (端口占用/二进制缺失)
        QTimer.singleShot(500, lambda: self._verify_link(key, title))

    def _verify_link(self, key, title):
        """启动后自检: 进程已退出则报错并复位按钮。"""
        proc = self.link_procs.get(key)
        if proc is None or proc.poll() is None:
            return
        self.node.get_logger().error(
            f'{title} 启动后立即退出 (退出码 {proc.returncode}), '
            f'请检查端口占用 / 二进制是否存在')
        self.link_procs.pop(key, None)
        self._set_link_btn(key, False)

    def _stop_link(self, key, title=None):
        """停止桥进程: SIGTERM 整个进程组, 超时再 SIGKILL。"""
        proc = self.link_procs.pop(key, None)
        self._set_link_btn(key, False)
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except (ProcessLookupError, PermissionError, OSError):
            proc.terminate()
        try:
            proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError, OSError):
                proc.kill()
        self.node.get_logger().info(f'通信链路已停止: {title or key}')

    def _set_link_btn(self, key, checked):
        """同步按钮状态 (blockSignals 避免触发 toggle 回调重启进程)。"""
        btn = self.link_btns.get(key)
        if btn is None:
            return
        btn.blockSignals(True)
        btn.setChecked(checked)
        btn.blockSignals(False)

    def _check_link_alive(self):
        """轮询: 桥进程异常退出则复位按钮 (用户可重新点起)。"""
        for key, proc in list(self.link_procs.items()):
            if proc.poll() is not None:
                self.node.get_logger().error(
                    f'通信链路进程退出 (退出码 {proc.returncode}), 请重新点击启动')
                self.link_procs.pop(key, None)
                self._set_link_btn(key, False)

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

    def _send_start(self, algo, retries_left=6):
        """发送启动指令并按 400ms 间隔重发多次: 新进程的 DDS 发现可能
        超过 1s, 单发一条 start 撞上发现窗口就永远丢掉 (volatile 消息
        不补发, 表现为"点了运行没反应"); 节点对重复 start 幂等, 多发无副作用。
        触发前算法被取消/切换/进程已死则跳过。"""
        if self.running_algo is not algo:
            return
        if self.proc is not None and self.proc.poll() is not None:
            self.node.get_logger().error(
                f'算法进程启动失败 (退出码 {self.proc.returncode}), '
                f'请检查 config.ALGORITHMS 里的 cmd')
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

    # ---------- ArUco 降落 (独立 toggle 按钮) ----------

    def _selected_link(self):
        """当前选中的通信链路 (uXRCE-DDS / MAVROS), 未选返回 None。"""
        for key, btn in self.link_btns.items():
            if btn.isChecked():
                return key
        return None

    def _update_landing_btn(self):
        """降落按钮门控 (33ms 轮询): 需同时满足
        1) ArUco检测运行中
        2) 云台俯仰到垂直向下限位 (绝对 -90°, 即相对俯仰到 GIMBAL_PITCH_MIN_DEG)
        3) 下方已选择通信链路 (uXRCE-DDS / MAVROS 之一已开启)
        降落进行中不参与门控 —— 保持可点, 随时可取消交还控制权;
        同时禁用链路切换按钮 (换桥会断掉正在跑的降落控制)。"""
        if self.landing_btn.isChecked():
            for btn in self.link_btns.values():
                btn.setEnabled(False)
            return
        for btn in self.link_btns.values():
            btn.setEnabled(True)
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
            # 链路参数跟随下方所选链路 (mavros / uxrce)
            link = self._selected_link() or 'mavros'
            cmd = list(config.LANDING_CMD) + ['--ros-args', '-p', f'link:={link}']
            self.node.get_logger().info(
                f'启动降落节点进程 (link={link}): {" ".join(cmd)}')
            self.land_proc = subprocess.Popen(cmd)
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
            self.node.get_logger().info(
                '\033[32mArUco 视觉降落完成, 飞机已上锁\033[0m')
            self._flash_landing_btn('#2d8a3e')      # 绿色高亮闪一下
        elif status == 'idle':
            self.node.get_logger().info('降落流程已结束 (取消/中止), 按钮已复位')
        else:
            return
        # 按钮弹起 (checked = 降落进行中) + 回收降落节点进程
        self.landing_btn.setChecked(False)
        proc = self.land_proc
        self.land_proc = None
        if proc is not None and proc.poll() is None:
            proc.terminate()

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
        if self.land_proc is not None and self.land_proc.poll() is not None:
            self.node.get_logger().error(
                f'降落节点进程启动失败 (退出码 {self.land_proc.returncode}), '
                f'检查编译: colcon build --packages-select flylab_vision')
            self.land_proc = None
            self.landing_btn.setChecked(False)
            return
        self.node.publish_cmd(config.LANDING_START_CMD)
        if retries_left > 0:
            QTimer.singleShot(500, lambda: self._send_landing_start(retries_left - 1))

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
        self._check_link_alive()     # 桥进程异常退出则复位按钮
        self._check_landing_status()  # 降落结果反馈 (成功/中止 → 按钮弹起)
        self._update_landing_btn()   # 降落按钮门控 (检测运行中 + 云台垂直向下 + 已选链路)
        # 显示画面转发: 有算法节点订阅时, 把当前显示的帧发给它
        if config.DISPLAY_IMAGE_HZ > 0:
            self._publish_display_frame()
        # 相机机体系姿态 (10Hz): 供 aruco_landing 做云台泛化的方向映射;
        # 非 AirSim 源 get_cam_orientation() 返回 None, 不发布
        now_mono = time.monotonic()
        if now_mono - self._cam_ori_last >= 0.1:
            self._cam_ori_last = now_mono
            self.node.publish_cam_orientation(
                self.camera_tab.grabber.get_cam_orientation())

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
        """转发完全断流时告警 (正常运行不刷屏)。"""
        now = time.monotonic()
        if now - self._fwd_last < 2.0:
            return
        self._fwd_last = now
        cnt = self._fwd_count
        self._fwd_count = 0
        if self.running_algo is None:
            return
        if cnt == 0:
            self.node.get_logger().warning(
                f'画面转发: 近 2 秒 0 帧 (源 {self._last_src or "?"} 无图像?)')

    def shutdown(self):
        """退出: 回收算法进程, 停掉后台线程。"""
        if self.running_algo is not None:
            self._stop_algo()
        if self.land_proc is not None:
            if self.land_proc.poll() is None:
                self.node.publish_cmd(config.LANDING_STOP_CMD)
                self.land_proc.terminate()
            self.land_proc = None
        for key in list(self.link_procs):
            self._stop_link(key)
        self.camera_tab.shutdown()
