#!/usr/bin/env python3
"""flylab 相机查看器 —— 直连 AirSim RPC 拉取相机画面并显示

特性：
  - 不依赖任何 ROS 链路/传感器节点，UE 开着即可看画面
  - 每帧只向 AirSim 请求当前选中的一路相机，切换即时生效（最高效的拉法）
  - Scene 相机显示彩色；名字含 depth 的相机自动按深度伪彩显示

用法：
  ros2 run flylab_tools camera_view
  ros2 run flylab_tools camera_view --ros-args -p host:=<UE主机IP> -p port:=41451
  host 默认取环境变量 PX4_SIM_HOST_ADDR（跨主机看 UE），否则 127.0.0.1

依赖：pip install msgpack-rpc-python
     cosysairsim 客户端位于 Cosys-AirSim/PythonClient（自动加入 sys.path）
"""
import os
import signal
import sys
import threading
import time

import numpy as np


def _to_np(data, dtype):
    """AirSim msgpack 返回的图像数据可能是 bytes(bin 编码)或 list(array 编码)，
    统一转成 numpy 数组。"""
    if isinstance(data, np.ndarray):
        return data
    if isinstance(data, list):
        return np.asarray(data, dtype=dtype)
    return np.frombuffer(data, dtype=dtype)

try:
    import cosysairsim as airsim
except ImportError:
    sys.path.insert(0, '/root/flylab/Cosys-AirSim/PythonClient')
    import cosysairsim as airsim

import rclpy
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QComboBox, QHBoxLayout, QLabel, QMainWindow, QVBoxLayout,
    QWidget,
)

VEHICLE = 'PX4'
GRAB_HZ = 60        # 拉取频率上限 (实际帧率 = min(该值, 1/RPC往返延迟))
DISPLAY_MS = 33     # 显示刷新周期
DEPTH_MIN_M, DEPTH_MAX_M = 0.2, 20.0

DEFAULT_CAMERAS = ['left_cam', 'right_cam', 'depth_cam']


class Grabber(threading.Thread):
    """后台线程：以 GRAB_HZ 持续拉取当前相机的最新一帧。"""

    def __init__(self, log, host, port):
        super().__init__(daemon=True)
        self.log = log
        self.host, self.port = host, int(port)
        self.lock = threading.Lock()
        self.camera = DEFAULT_CAMERAS[0]
        self.frame = None      # (np.ndarray, is_depth)
        self.fps = 0.0
        self.running = True

    def set_camera(self, name):
        with self.lock:
            self.camera = name
            self.frame = None   # 丢弃旧相机画面，避免切换时闪现错图

    def stop(self):
        self.running = False

    def get(self):
        with self.lock:
            return self.frame, self.camera, self.fps

    def run(self):
        client = None
        n, t_prev = 0, time.time()
        while self.running:
            try:
                t_start = time.monotonic()
                if client is None:
                    client = airsim.VehicleClient(ip=self.host, port=self.port)
                    client.confirmConnection()
                    self.log(f'已连接 AirSim {self.host}:{self.port}')

                with self.lock:
                    cam = self.camera
                is_depth = 'depth' in cam.lower()
                # 深度必须用 DepthPlanar 类型: 若用 Scene+pixels_as_float,
                # UE 端会对 8-bit 彩色渲染目标做浮点读回, 触发 D3D12 的
                # PF_FloatRGBA ensure 断言并返回无效数据 (NaN/Inf)
                if is_depth:
                    req = airsim.ImageRequest(
                        cam, airsim.ImageType.DepthPlanar, True, False)
                else:
                    req = airsim.ImageRequest(
                        cam, airsim.ImageType.Scene, False, False)
                resp = client.simGetImages([req], VEHICLE)

                if resp and resp[0].width > 0:
                    r = resp[0]
                    if is_depth:   # DepthPlanar, 单位米
                        arr = _to_np(
                            r.image_data_float, np.float32).reshape(r.height, r.width)
                    else:          # BGR
                        arr = np.ascontiguousarray(_to_np(
                            r.image_data_uint8, np.uint8).reshape(r.height, r.width, 3))
                    with self.lock:
                        self.frame = (arr, is_depth)
                    n += 1
                    now = time.time()
                    if now - t_prev >= 1.0:
                        self.fps = n / (now - t_prev)
                        n, t_prev = 0, now
                else:
                    time.sleep(0.05)
                # 自适应节流: 扣除 RPC 往返耗时后再补足剩余间隔。
                # 往返超过 1/GRAB_HZ 时不再额外睡眠, 帧率由网络/渲染延迟决定
                time.sleep(max(0.0, 1.0 / GRAB_HZ - (time.monotonic() - t_start)))
            except Exception as e:
                self.log(f'连接中断: {e}，1 秒后重连...')
                client = None
                time.sleep(1.0)


def colorize_depth(depth):
    """深度图(米) -> 伪彩 BGR。"""
    # 天空等无回波区域为 Inf/NaN, 归到最远端, 避免 invalid value in cast 警告
    d = np.nan_to_num(depth, nan=DEPTH_MAX_M, posinf=DEPTH_MAX_M, neginf=DEPTH_MIN_M)
    d = np.clip(d, DEPTH_MIN_M, DEPTH_MAX_M)
    norm = ((d - DEPTH_MIN_M) / (DEPTH_MAX_M - DEPTH_MIN_M) * 255).astype(np.uint8)
    try:
        import cv2
        cmap = getattr(cv2, 'COLORMAP_TURBO', cv2.COLORMAP_JET)
        return cv2.applyColorMap(norm, cmap)
    except ImportError:
        return np.ascontiguousarray(np.repeat(norm[:, :, None], 3, axis=2))


class Viewer(QMainWindow):
    def __init__(self, grabber):
        super().__init__()
        self.setWindowTitle('FLYLAB 相机查看器')
        self.grabber = grabber

        self.combo = QComboBox()
        self.combo.setEditable(True)
        self.combo.addItems(DEFAULT_CAMERAS)
        self.combo.activated.connect(self._on_camera_changed)
        self.combo.lineEdit().returnPressed.connect(self._on_camera_changed)

        self.video = QLabel('等待图像...')
        self.video.setMinimumSize(640, 480)
        self.video.setAlignment(Qt.AlignCenter)

        self.status = QLabel('未连接')
        self.status.setStyleSheet('color: #666; padding: 2px;')

        central = QWidget()
        layout = QVBoxLayout(central)
        row = QHBoxLayout()
        row.addWidget(QLabel('相机:'))
        row.addWidget(self.combo, stretch=1)
        layout.addLayout(row)
        layout.addWidget(self.video)
        layout.addWidget(self.status)
        self.setCentralWidget(central)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(DISPLAY_MS)

    def _on_camera_changed(self):
        name = self.combo.currentText().strip()
        if name:
            self.grabber.set_camera(name)

    def _refresh(self):
        frame, cam, fps = self.grabber.get()
        if frame is None:
            self.status.setText(f'[{cam}] 等待图像...')
            return
        arr, is_depth = frame
        if is_depth:
            arr = colorize_depth(arr)
        h, w = arr.shape[:2]
        img = QImage(arr.data, w, h, w * 3, QImage.Format_BGR888).copy()
        self.video.setPixmap(QPixmap.fromImage(img).scaled(
            self.video.size(), Qt.KeepAspectRatio, Qt.FastTransformation))
        kind = '深度' if is_depth else 'Scene'
        self.status.setText(f'[{cam}] {w}x{h} {kind}  {fps:.1f} fps')


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('camera_view')
    host = node.declare_parameter(
        'host', os.environ.get('PX4_SIM_HOST_ADDR', '127.0.0.1')).value
    port = node.declare_parameter('port', 41451).value

    grabber = Grabber(node.get_logger().info, host, port)
    grabber.start()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    app = QApplication([])
    # 显式指定含中文字形的字体, 避免无中文字体环境下界面显示为方框
    app.setFont(QFont('Noto Sans CJK SC', 10))
    win = Viewer(grabber)
    win.show()

    # Ctrl+C 修复: 主线程阻塞在 Qt 事件循环里, 默认的 KeyboardInterrupt 会被
    # PyQt 的槽回调边界吞掉, 导致进程杀不死。改为收到 SIGINT 直接退出事件循环。
    signal.signal(
        signal.SIGINT, lambda *_: QApplication.instance().quit())

    try:
        ret = app.exec_()
    except KeyboardInterrupt:
        ret = 0
    finally:
        grabber.stop()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    raise SystemExit(ret)


if __name__ == '__main__':
    main()
