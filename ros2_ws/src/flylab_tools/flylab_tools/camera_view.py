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
import sys
import threading
import time

import numpy as np

try:
    import cosysairsim as airsim
except ImportError:
    sys.path.insert(0, '/root/flylab/Cosys-AirSim/PythonClient')
    import cosysairsim as airsim

import rclpy
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QComboBox, QHBoxLayout, QLabel, QMainWindow, QVBoxLayout,
    QWidget,
)

VEHICLE = 'PX4'
GRAB_HZ = 30        # 拉取频率
DISPLAY_MS = 33     # 显示刷新周期
DEPTH_MIN_M, DEPTH_MAX_M = 0.2, 20.0

DEFAULT_CAMERAS = ['left_cam', 'right_cam', 'depth_cam', 'CameraDepth']


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
                if client is None:
                    client = airsim.VehicleClient(ip=self.host, port=self.port)
                    client.confirmConnection()
                    self.log(f'已连接 AirSim {self.host}:{self.port}')

                with self.lock:
                    cam = self.camera
                is_depth = 'depth' in cam.lower()
                req = airsim.ImageRequest(
                    cam, airsim.ImageType.Scene, is_depth, False)
                resp = client.simGetImages([req], VEHICLE)

                if resp and resp[0].width > 0:
                    r = resp[0]
                    if is_depth:   # DepthPlanar, 单位米
                        arr = np.frombuffer(
                            r.image_data_float, np.float32).reshape(r.height, r.width)
                    else:          # BGR
                        arr = np.ascontiguousarray(np.frombuffer(
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
                time.sleep(1.0 / GRAB_HZ)
            except Exception as e:
                self.log(f'连接中断: {e}，1 秒后重连...')
                client = None
                time.sleep(1.0)


def colorize_depth(depth):
    """深度图(米) -> 伪彩 BGR。"""
    d = np.clip(depth, DEPTH_MIN_M, DEPTH_MAX_M)
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
            self.video.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
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
    win = Viewer(grabber)
    win.show()
    ret = app.exec_()

    grabber.stop()
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(ret)


if __name__ == '__main__':
    main()
