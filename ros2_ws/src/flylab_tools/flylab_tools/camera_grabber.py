#!/usr/bin/env python3
"""AirSim 相机抓取器 —— 直连 AirSim RPC 拉图 + 本机 V4L2 摄像头，与界面解耦。

被 camera_view (独立调试工具) 和 ground_station (总界面) 共用。
"""
import glob
import os
import threading
import time

import numpy as np

# /dev/video* 里 metadata 节点探测失败是预期行为, 屏蔽 V4L2 的 WARN 刷屏
# (apt 旧版没有 setLogLevel, 用环境变量; 4.8+ 再用 API 双保险)
os.environ.setdefault('OPENCV_LOG_LEVEL', 'ERROR')
try:
    import cv2
    if hasattr(cv2, 'setLogLevel') and hasattr(cv2, 'LOG_LEVEL_ERROR'):
        cv2.setLogLevel(cv2.LOG_LEVEL_ERROR)
except ImportError:
    cv2 = None

try:
    import cosysairsim as airsim
except ImportError:
    import sys
    sys.path.insert(0, '/root/flylab/Cosys-AirSim/PythonClient')
    import cosysairsim as airsim

VEHICLE = 'PX4'
GRAB_HZ = 60        # AirSim 拉取频率上限 (实际帧率 = min(该值, 1/RPC往返延迟))
LOCAL_HZ = 30       # 本机摄像头拉取频率上限
DEPTH_MIN_M, DEPTH_MAX_M = 0.2, 20.0

DEFAULT_CAMERAS = ['left_cam', 'front_cam', 'right_cam', 'depth_cam']
LOCAL_PREFIX = 'local:'   # 本机摄像头在下拉框里的条目前缀, 如 local:/dev/video0
FILE_PREFIX = 'file:'     # 视频文件源前缀, 如 file:/path/to/demo.mp4 (循环播放)


def _to_np(data, dtype):
    """AirSim msgpack 返回的图像数据可能是 bytes(bin 编码)或 list(array 编码)，
    统一转成 numpy 数组。"""
    if isinstance(data, np.ndarray):
        return data
    if isinstance(data, list):
        return np.asarray(data, dtype=dtype)
    return np.frombuffer(data, dtype=dtype)


def _v4l2_index(dev):
    """'/dev/video0' -> 0。V4L2 后端只能按整数索引打开, 按名字打开会报
    "can't be used to capture by name" 且失败。"""
    return int(dev.rsplit('video', 1)[1])


def list_local_cameras():
    """探测本机可用摄像头：遍历 /dev/video*，能成功 grab 到帧的才算。
    一台相机常暴露多个 video 节点，只有 capture 节点能出帧。"""
    if cv2 is None:
        return []
    cams = []
    for dev in sorted(glob.glob('/dev/video*')):
        try:
            cap = cv2.VideoCapture(_v4l2_index(dev), cv2.CAP_V4L2)
            ok = cap.isOpened() and cap.grab()
            cap.release()
            if ok:
                cams.append(dev)
        except Exception:
            pass
    return cams


def colorize_depth(depth):
    """深度图(米) -> 伪彩 BGR。"""
    # 天空等无回波区域为 Inf/NaN, 归到最远端, 避免 invalid value in cast 警告
    d = np.nan_to_num(depth, nan=DEPTH_MAX_M, posinf=DEPTH_MAX_M, neginf=DEPTH_MIN_M)
    d = np.clip(d, DEPTH_MIN_M, DEPTH_MAX_M)
    norm = ((d - DEPTH_MIN_M) / (DEPTH_MAX_M - DEPTH_MIN_M) * 255).astype(np.uint8)
    if cv2 is not None:
        cmap = getattr(cv2, 'COLORMAP_TURBO', cv2.COLORMAP_JET)
        return cv2.applyColorMap(norm, cmap)
    return np.ascontiguousarray(np.repeat(norm[:, :, None], 3, axis=2))


class Grabber(threading.Thread):
    """后台线程：以 GRAB_HZ/LOCAL_HZ 持续拉取当前相机的最新一帧。"""

    def __init__(self, log, host, port):
        super().__init__(daemon=True)
        self.log = log
        self.host, self.port = host, int(port)
        self.lock = threading.Lock()
        self.camera = DEFAULT_CAMERAS[0]
        self.frame = None      # (np.ndarray, is_depth)
        self.fps = 0.0
        self.n, self.t_prev = 0, time.time()
        self.cap = None        # 本机摄像头/视频文件的 VideoCapture (仅抓取线程访问)
        self.cap_dev = None
        self.file_interval = 1.0 / 30   # 视频文件帧间隔 (打开时按文件 fps 更新)
        # 视频文件播放控制 (标志位由界面线程设置, 抓取线程处理)
        self.file_paused = False
        self.file_stop = False
        self.file_seek = None           # 0..1, 待跳转的相对位置
        self.file_pos, self.file_total = 0.0, 0.0
        self.file_fps = 30.0
        self.running = True

    def set_camera(self, name):
        with self.lock:
            self.camera = name
            self.frame = None   # 丢弃旧相机画面，避免切换时闪现错图

    def stop(self):
        self.running = False

    def release_local(self):
        """释放本机摄像头设备 (须在抓取线程 join 之后调用)。"""
        if self.cap is not None:
            self.cap.release()
            self.cap = self.cap_dev = None

    def get(self):
        with self.lock:
            return self.frame, self.camera, self.fps

    # ---------- 视频文件播放控制 (界面线程调用, 标志位由抓取线程处理) ----------

    def set_file_paused(self, paused):
        self.file_paused = paused

    def stop_file(self):
        """停止播放: 回到第 0 帧并暂停。"""
        self.file_stop = True

    def set_file_position(self, frac):
        """跳转到相对位置 (0..1)，暂停中也可用。"""
        self.file_seek = max(0.0, min(1.0, frac))

    def get_progress(self):
        """(当前帧, 总帧数, 是否暂停, 文件fps)；非文件源返回 None。"""
        with self.lock:
            if self.camera.startswith(FILE_PREFIX) and self.cap_dev is not None:
                return self.file_pos, self.file_total, self.file_paused, self.file_fps
        return None

    def _commit(self, arr, is_depth):
        """写入最新帧并统计 fps。"""
        with self.lock:
            self.frame = (arr, is_depth)
        self.n += 1
        now = time.time()
        if now - self.t_prev >= 1.0:
            self.fps = self.n / (now - self.t_prev)
            self.n, self.t_prev = 0, now

    def _read_local(self, dev):
        """从本机 V4L2 摄像头读一帧 (BGR)；设备只打开一次，失败/换设备时重开。"""
        if cv2 is None:
            self.log('未安装 OpenCV，无法使用本机摄像头 (pip install opencv-python)')
            time.sleep(5.0)
            return None
        if self.cap_dev != dev:
            if self.cap is not None:
                self.cap.release()
                self.cap = None
            cap = cv2.VideoCapture(_v4l2_index(dev), cv2.CAP_V4L2)
            if not cap.isOpened():
                self.log(f'无法打开本机摄像头 {dev}')
                time.sleep(1.0)
                return None
            # 默认常协商到 YUYV 未压缩格式, usbipd 转发下 USB 带宽不足导致
            # 帧尾数据缺失, 解码出来画面底部是绿色。改用 MJPG 压缩格式。
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fcc = int(cap.get(cv2.CAP_PROP_FOURCC))
            fmt = ''.join(chr((fcc >> 8 * i) & 0xFF) for i in range(4))
            self.log(f'已打开本机摄像头 {dev} {w}x{h} {fmt}')
            self.cap, self.cap_dev = cap, dev
        ok, frame = self.cap.read()
        if not ok:              # 设备被拔出等异常，下次循环重开
            self.cap.release()
            self.cap = self.cap_dev = None
            return None
        return frame

    def _read_file(self, path):
        """从视频文件读一帧 (BGR)；播完自动从头循环。"""
        if cv2 is None:
            self.log('未安装 OpenCV，无法播放视频文件')
            time.sleep(5.0)
            return None
        if self.cap_dev != path:
            if self.cap is not None:
                self.cap.release()
                self.cap = None
            cap = cv2.VideoCapture(path)
            if not cap.isOpened():
                self.log(f'无法打开视频文件 {path}')
                time.sleep(1.0)
                return None
            fps = cap.get(cv2.CAP_PROP_FPS)
            self.file_interval = 1.0 / fps if fps > 1e-3 else 1.0 / 30
            self.file_fps = fps if fps > 1e-3 else 30.0
            self.file_total = cap.get(cv2.CAP_PROP_FRAME_COUNT)
            dur = self.file_total / self.file_fps
            self.log(f'已打开视频文件 {path} @ {fps:.1f} fps, '
                     f'{int(self.file_total)} 帧, {int(dur // 60):02d}:{int(dur % 60):02d}')
            self.cap, self.cap_dev = cap, path
        # 播放控制: 停止(回第0帧+暂停) / 跳转 / 暂停, 均在抓取线程内操作设备
        if self.file_stop:
            self.file_stop = False
            self.file_paused = True
            if self.cap is not None:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                self.file_pos = 0.0
            return None
        if self.file_seek is not None:
            frac, self.file_seek = self.file_seek, None
            if self.cap is not None and self.file_total > 0:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, int(frac * self.file_total))
                ok, frame = self.cap.read()   # 立即读一帧, 暂停中也能预览跳转位置
                if ok:
                    self.file_pos = self.cap.get(cv2.CAP_PROP_POS_FRAMES)
                    return frame
        if self.file_paused:
            return None              # 暂停: 不读帧, 画面停留在最后一帧
        ok, frame = self.cap.read()
        if not ok:              # 播完 -> 从头循环
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return None
        self.file_pos = self.cap.get(cv2.CAP_PROP_POS_FRAMES)
        return frame

    def run(self):
        client = None
        while self.running:
            try:
                t_start = time.monotonic()
                with self.lock:
                    cam = self.camera
                if cam.startswith(LOCAL_PREFIX):    # 本机摄像头: 不经 AirSim
                    arr = self._read_local(cam[len(LOCAL_PREFIX):])
                    if arr is not None:
                        self._commit(arr, False)
                    time.sleep(max(0.0, 1.0 / LOCAL_HZ - (time.monotonic() - t_start)))
                    continue
                if cam.startswith(FILE_PREFIX):     # 视频文件: 按文件帧率循环播放
                    arr = self._read_file(cam[len(FILE_PREFIX):])
                    if arr is not None:
                        self._commit(arr, False)
                    time.sleep(max(0.0, self.file_interval - (time.monotonic() - t_start)))
                    continue
                if self.cap_dev is not None:        # 切回 AirSim 相机时释放本机设备
                    self.cap.release()
                    self.cap = self.cap_dev = None
                if client is None:
                    client = airsim.VehicleClient(ip=self.host, port=self.port)
                    client.confirmConnection()
                    self.log(f'已连接 AirSim {self.host}:{self.port}')

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
                    self._commit(arr, is_depth)
                else:
                    time.sleep(0.05)
                # 自适应节流: 扣除 RPC 往返耗时后再补足剩余间隔。
                # 往返超过 1/GRAB_HZ 时不再额外睡眠, 帧率由网络/渲染延迟决定
                time.sleep(max(0.0, 1.0 / GRAB_HZ - (time.monotonic() - t_start)))
            except Exception as e:
                self.log(f'连接中断: {e}，1 秒后重连...')
                client = None
                time.sleep(1.0)
