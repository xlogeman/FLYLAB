#!/usr/bin/env python3
"""AirSim 相机抓取器 —— 直连 AirSim RPC 拉图 + 本机 V4L2 摄像头，与界面解耦。

被 camera_view (独立调试工具) 和 ground_station (总界面) 共用。
附带软件云台：经 simSetCameraPose 旋转相机视角 (yaw/pitch 步进 + 回中)。
"""
import glob
import math
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

GIMBAL_PITCH_LIMIT = 90.0  # 云台俯仰限位(度), 正=向上; 偏航不设限, 按 ±180 归一化


def _gimbal_quat(yaw_deg, pitch_deg):
    """云台角度 -> NED 四元数 (相对机体: 先绕 z 偏航, 再绕 y 俯仰)。
    yaw 正=向右 (绕 NED +z, 即向下轴的右手旋转), pitch 正=向上 (绕 +y)。
    回中 (0,0) 即单位四元数 (相对安装姿态不动)。"""
    y1, p2 = math.radians(yaw_deg) / 2, math.radians(pitch_deg) / 2
    c1, s1, c2, s2 = math.cos(y1), math.sin(y1), math.cos(p2), math.sin(p2)
    # qz(yaw) ⊗ qy(pitch) 的 Hamilton 积, 注意 z 分量是 s1*c2 (此前的 c1*s1
    # 是代数错误, 四元数未归一化, 实际转角/轴向全错)
    return airsim.Quaternionr(-s1 * s2, c1 * s2, s1 * c2, c1 * c2)


def _quat_mul(a, b):
    """四元数 Hamilton 积 a⊗b: 相对机体的组合旋转, 先施 b 再施 a。"""
    ax, ay, az, aw = a.x_val, a.y_val, a.z_val, a.w_val
    bx, by, bz, bw = b.x_val, b.y_val, b.z_val, b.w_val
    return airsim.Quaternionr(
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz)


def _quat_conj(q):
    """四元数共轭 (单位四元数下即逆旋转)。"""
    return airsim.Quaternionr(-q.x_val, -q.y_val, -q.z_val, q.w_val)


def _quat_rot_vec(q, v):
    """向量 v 按单位四元数 q 旋转 (v' = q v q*)。"""
    x, y, z = v.x_val, v.y_val, v.z_val
    qx, qy, qz, qw = q.x_val, q.y_val, q.z_val, q.w_val
    tx = 2 * (qy * z - qz * y)
    ty = 2 * (qz * x - qx * z)
    tz = 2 * (qx * y - qy * x)
    return airsim.Vector3r(
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx))


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
        # 软件云台: 全局角度 (yaw, pitch), 同步作用于所有 AirSim 相机
        self._gimbal = (0.0, 0.0)
        self._gimbal_dirty = False
        # 相机名 -> 首改前的安装位姿 (position, orientation)。相机在 settings.json
        # 里常带下俯安装角 (如 front_cam pitch=-36.87°), 云台旋转必须叠加在
        # 安装姿态之上, 回中才等价于恢复安装角; 不能用单位四元数当回中态
        self._gimbal_mount = {}
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

    # ---------- 软件云台 (界面线程调用, 角度由抓取线程经 RPC 写入相机) ----------

    def gimbal_step(self, dyaw, dpitch):
        """步进云台角度 (yaw 正=向右, pitch 正=向上), 同步作用到所有 AirSim
        相机 (left/front/right/depth 一起转)。本机/视频文件源为空操作。"""
        with self.lock:
            if self.camera.startswith((LOCAL_PREFIX, FILE_PREFIX)):
                return
            yaw, pitch = self._gimbal
            yaw = (yaw + dyaw + 180.0) % 360.0 - 180.0
            pitch = max(-GIMBAL_PITCH_LIMIT,
                        min(GIMBAL_PITCH_LIMIT, pitch + dpitch))
            self._gimbal = (yaw, pitch)
            self._gimbal_dirty = True

    def gimbal_reset(self):
        """云台回中 (恢复所有相机挂载原始姿态)。"""
        with self.lock:
            self._gimbal = (0.0, 0.0)
            self._gimbal_dirty = True

    def get_gimbal(self):
        """当前云台角度 (yaw, pitch)；非 AirSim 源返回 None。"""
        with self.lock:
            if self.camera.startswith((LOCAL_PREFIX, FILE_PREFIX)):
                return None
            return self._gimbal

    def _capture_mount(self, client, cam):
        """缓存相机安装位姿 (机体系)。

        注意 AirSim 的坐标系陷阱: simGetCameraInfo 返回的是世界系位姿
        (位置相对出生点、姿态为世界姿态, 见 PIPCamera.cpp getCameraInfo 的
        toLocalNed/toNed), 而 simSetCameraPose 按"机体相对位姿"解释输入
        (SetActorRelativeLocation/Rotation)。直接把 get 的结果 set 回去,
        飞机一旦起飞/偏航/移动, 相机会被瞬移甩歪。必须先取机体当前世界
        位姿 (simGetGroundTruthKinematics), 把相机位姿换算到机体系。"""
        info = client.simGetCameraInfo(cam, VEHICLE)
        kin = client.simGetGroundTruthKinematics(VEHICLE)
        q_b, p_b = kin.orientation, kin.position   # KinematicsState 字段是扁平的
        p_c, q_c = info.pose.position, info.pose.orientation
        # 安装姿态 (机体系): q_mount = q_body⁻¹ ⊗ q_cam_world
        q_rel = _quat_mul(_quat_conj(q_b), q_c)
        # 安装位置 (机体系): p_mount = R_body⁻¹ · (p_cam - p_body)
        d = airsim.Vector3r(p_c.x_val - p_b.x_val,
                            p_c.y_val - p_b.y_val,
                            p_c.z_val - p_b.z_val)
        p_rel = _quat_rot_vec(_quat_conj(q_b), d)
        return p_rel, q_rel

    def _apply_gimbal(self, client, yaw_deg, pitch_deg):
        """把云台角度经 simSetCameraPose 同步写到所有 AirSim 相机。
        姿态 = 云台角度 ⊗ 安装姿态 (角度按机体系解释, 叠加在 settings.json
        的安装角之上), 位置沿用安装偏移; 单个相机不存在不影响其余相机。"""
        try:
            for cam in DEFAULT_CAMERAS:
                if cam not in self._gimbal_mount:
                    try:
                        self._gimbal_mount[cam] = self._capture_mount(client, cam)
                    except Exception as e:
                        self._gimbal_mount[cam] = None   # 该相机不可用, 跳过
                        self.log(f'云台: 获取 {cam} 安装位姿失败, 跳过: {e}')
                mount = self._gimbal_mount[cam]
                if mount is None:
                    continue
                pos, ori = mount
                q = _quat_mul(_gimbal_quat(yaw_deg, pitch_deg), ori)
                client.simSetCameraPose(cam, airsim.Pose(pos, q), VEHICLE)
        except Exception as e:
            # 失败不置回 dirty: 避免每帧重试刷屏; 重连或下次按键会重新下发
            self.log(f'云台控制失败: {e}')
        finally:
            self._gimbal_dirty = False

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
                    # 用户转过云台 (非回中) 时重连后重新下发, 保持视角;
                    # 未动过则完全不碰相机, 保留 settings.json 的安装角
                    self._gimbal_dirty = self._gimbal != (0.0, 0.0)

                is_depth = 'depth' in cam.lower()
                with self.lock:
                    gim_dirty, gim = self._gimbal_dirty, self._gimbal
                if gim_dirty:
                    self._apply_gimbal(client, *gim)
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
