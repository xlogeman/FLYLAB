#!/usr/bin/env python3
"""本地视频源设备 IO —— 本机 V4L2 摄像头 / 视频文件, 与 AirSim 无关。

LocalFileReader 封装 VideoCapture 的打开/读帧/播放控制, 只应在抓取线程
调用 (非线程安全); 播放控制标志位 (paused/stop_flag/seek) 由界面线程
设置, 抓取线程处理 —— 与原 Grabber 内联实现同一套语义。
"""
import glob
import time

import numpy as np

# /dev/video* 里 metadata 节点探测失败是预期行为, 屏蔽 V4L2 的 WARN 刷屏
# (apt 旧版没有 setLogLevel, 用环境变量; 4.8+ 再用 API 双保险)
import os
os.environ.setdefault('OPENCV_LOG_LEVEL', 'ERROR')
try:
    import cv2
    if hasattr(cv2, 'setLogLevel') and hasattr(cv2, 'LOG_LEVEL_ERROR'):
        cv2.setLogLevel(cv2.LOG_LEVEL_ERROR)
except ImportError:
    cv2 = None

DEPTH_MIN_M, DEPTH_MAX_M = 0.2, 20.0


def v4l2_index(dev):
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
            cap = cv2.VideoCapture(v4l2_index(dev), cv2.CAP_V4L2)
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


class LocalFileReader:
    """本机摄像头/视频文件的设备读取器 (仅抓取线程访问设备)。"""

    def __init__(self, log):
        self.log = log
        self.cap = None          # VideoCapture (打开一次, 换源/异常时重开)
        self.dev = None          # 当前打开的设备路径或文件路径
        self.interval = 1.0 / 30   # 视频文件帧间隔 (打开时按文件 fps 更新)
        # 播放控制 (标志位由界面线程设置, 抓取线程处理)
        self.paused = False
        self.stop_flag = False
        self.seek = None         # 0..1, 待跳转的相对位置
        self.pos, self.total = 0.0, 0.0
        self.fps = 30.0

    def close(self):
        """释放当前设备 (若有)。"""
        if self.cap is not None:
            self.cap.release()
            self.cap = self.dev = None

    def release(self):
        """释放设备 (须在抓取线程 join 之后调用)。"""
        self.close()

    def read_local(self, dev):
        """从本机 V4L2 摄像头读一帧 (BGR)；设备只打开一次，失败/换设备时重开。"""
        if cv2 is None:
            self.log('未安装 OpenCV，无法使用本机摄像头 (pip install opencv-python)')
            time.sleep(5.0)
            return None
        if self.dev != dev:
            self.close()
            cap = cv2.VideoCapture(v4l2_index(dev), cv2.CAP_V4L2)
            if not cap.isOpened():
                self.log(f'无法打开本机摄像头 {dev}')
                time.sleep(1.0)
                return None
            # YUYV 未压缩在 usbipd 转发下 USB 带宽不足, 画面底部发绿,
            # 改用 MJPG 压缩格式
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fcc = int(cap.get(cv2.CAP_PROP_FOURCC))
            fmt = ''.join(chr((fcc >> 8 * i) & 0xFF) for i in range(4))
            self.log(f'已打开本机摄像头 {dev} {w}x{h} {fmt}')
            self.cap, self.dev = cap, dev
        ok, frame = self.cap.read()
        if not ok:              # 设备被拔出等异常，下次循环重开
            self.close()
            return None
        return frame

    def read_file(self, path):
        """从视频文件读一帧 (BGR)；播完自动从头循环。"""
        if cv2 is None:
            self.log('未安装 OpenCV，无法播放视频文件')
            time.sleep(5.0)
            return None
        if self.dev != path:
            self.close()
            cap = cv2.VideoCapture(path)
            if not cap.isOpened():
                self.log(f'无法打开视频文件 {path}')
                time.sleep(1.0)
                return None
            fps = cap.get(cv2.CAP_PROP_FPS)
            self.interval = 1.0 / fps if fps > 1e-3 else 1.0 / 30
            self.fps = fps if fps > 1e-3 else 30.0
            self.total = cap.get(cv2.CAP_PROP_FRAME_COUNT)
            dur = self.total / self.fps
            self.log(f'已打开视频文件 {path} @ {fps:.1f} fps, '
                     f'{int(self.total)} 帧, {int(dur // 60):02d}:{int(dur % 60):02d}')
            self.cap, self.dev = cap, path
        # 播放控制: 停止(回第0帧+暂停) / 跳转 / 暂停, 均在抓取线程内操作设备
        if self.stop_flag:
            self.stop_flag = False
            self.paused = True
            if self.cap is not None:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                self.pos = 0.0
            return None
        if self.seek is not None:
            frac, self.seek = self.seek, None
            if self.cap is not None and self.total > 0:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, int(frac * self.total))
                ok, frame = self.cap.read()   # 立即读一帧, 暂停中也能预览跳转位置
                if ok:
                    self.pos = self.cap.get(cv2.CAP_PROP_POS_FRAMES)
                    return frame
        if self.paused:
            return None              # 暂停: 不读帧, 画面停留在最后一帧
        ok, frame = self.cap.read()
        if not ok:              # 播完 -> 从头循环
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return None
        self.pos = self.cap.get(cv2.CAP_PROP_POS_FRAMES)
        return frame

    def progress(self):
        """(当前帧, 总帧数, 是否暂停, 文件fps)；无已打开设备返回 None。"""
        if self.cap is None:
            return None
        return self.pos, self.total, self.paused, self.fps
