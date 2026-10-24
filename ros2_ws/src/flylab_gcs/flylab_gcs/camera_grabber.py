#!/usr/bin/env python3
"""AirSim 相机抓取器 —— 直连 AirSim RPC 拉图 + 本机 V4L2 摄像头，与界面解耦。

被 camera_view (独立调试工具) 和 ground_station (总界面) 共用。
附带软件云台：经 simSetCameraPose 旋转相机视角 (摇杆角速度 + 回中)，
世界系稳定云台：视线方位/仰角钉在世界系 (方位参考机头朝向)，机体怎么
转、怎么斜都由周期重下发 (GIMBAL_HOLD_HZ) 补偿——垂直向下时原地旋转
仍保持垂直向下，方向键上下左右与机体姿态无关。
"""
import glob
import json
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

from . import config

VEHICLE = 'PX4'
GRAB_HZ = 60        # AirSim 拉取频率上限 (实际帧率 = min(该值, 1/RPC往返延迟))
LOCAL_HZ = 30       # 本机摄像头拉取频率上限
DEPTH_MIN_M, DEPTH_MAX_M = 0.2, 20.0

DEFAULT_CAMERAS = ['left_cam', 'front_cam', 'right_cam', 'depth_cam']
LOCAL_PREFIX = 'local:'   # 本机摄像头在下拉框里的条目前缀, 如 local:/dev/video0
FILE_PREFIX = 'file:'     # 视频文件源前缀, 如 file:/path/to/demo.mp4 (循环播放)

def _clamp_gimbal(yaw, pitch):
    """按 config 限位夹紧云台角度 (config.GIMBAL_*_LIMIT_DEG)。"""
    yaw = max(-config.GIMBAL_YAW_LIMIT_DEG,
              min(config.GIMBAL_YAW_LIMIT_DEG, yaw))
    pitch = max(config.GIMBAL_PITCH_MIN_DEG,
                min(config.GIMBAL_PITCH_MAX_DEG, pitch))
    return yaw, pitch


# 云台状态持久化文件。地面站重启时沿用上次视角与安装基准 (UE 端相机并没
# 被动过); 连接建立时对比相机实际姿态与上次下发值即可判断 UE 是否重新
# play 过 —— play 过则回中重采基准, 没过则保持记忆 (见 _restore_or_reset)。
_GIMBAL_STATE_FILE = os.path.join(os.path.expanduser('~'), '.flylab_gimbal.json')

# 判定 UE 重新 play 的阈值 (度): 相机实际姿态与上次下发值相差超过该值
_GIMBAL_REPLAY_THRESHOLD_DEG = 2.0


def _load_gimbal_state():
    """读取持久化的云台角度/安装位姿/上次下发姿态; 没有或损坏时返回空。"""
    try:
        with open(_GIMBAL_STATE_FILE) as f:
            state = json.load(f)
        gimbal = state.get('gimbal', [0.0, 0.0])
        mounts = {}
        for cam, (p, q) in state.get('mounts', {}).items():
            try:
                mounts[cam] = (airsim.Vector3r(*p), airsim.Quaternionr(*q))
            except Exception:
                pass
        last_applied = {}
        for cam, q in state.get('last_applied', {}).items():
            try:
                last_applied[cam] = airsim.Quaternionr(*q)
            except Exception:
                pass
        return (_clamp_gimbal(float(gimbal[0]), float(gimbal[1])),
                mounts, last_applied)
    except Exception:
        return (0.0, 0.0), {}, {}


def _save_gimbal_state(gimbal, mounts, last_applied):
    """持久化云台状态; 失败静默, 不影响主流程。"""
    try:
        data = {'gimbal': list(gimbal), 'mounts': {}, 'last_applied': {}}
        for cam, mount in mounts.items():
            if mount is None:
                continue
            pos, ori = mount
            data['mounts'][cam] = [
                [pos.x_val, pos.y_val, pos.z_val],
                [ori.x_val, ori.y_val, ori.z_val, ori.w_val]]
        for cam, q in last_applied.items():
            data['last_applied'][cam] = [q.x_val, q.y_val, q.z_val, q.w_val]
        with open(_GIMBAL_STATE_FILE, 'w') as f:
            json.dump(data, f)
    except Exception:
        pass


def _quat_yaw_el(q):
    """四元数前向轴 (机体系 +x) 的 (方位角, 仰角), 单位度。
    方位角 = 前向轴水平投影的北偏东角 (NED, 顺时针/向右为正);
    仰角 = 相对地平线, 上正下负。前向轴接近竖直时方位角退化 (取 0),
    此时方位角只影响画面 roll, 不影响指向。"""
    f = _quat_rot_vec(q, airsim.Vector3r(1.0, 0.0, 0.0))
    h = math.hypot(f.x_val, f.y_val)
    yaw = math.degrees(math.atan2(f.y_val, f.x_val)) if h > 1e-6 else 0.0
    el = math.degrees(math.atan2(-f.z_val, h))
    return yaw, el


def _gimbal_yaw_quat(azimuth_deg):
    """方位角 (度, 北偏东/向右为正) -> NED 四元数, 绕世界竖直轴 (+z)。
    与 Rz⊗Ry 组合成目标姿态后 roll 恒为 0 (地平线水平)。回中链路上
    传 0 即单位四元数。"""
    y = math.radians(azimuth_deg) / 2
    return airsim.Quaternionr(0.0, 0.0, math.sin(y), math.cos(y))


def _gimbal_pitch_quat(elevation_deg):
    """仰角 (度, 上正下负) -> NED 四元数, 绕世界 +y 轴。
    R_y(el) 把前向轴抬到仰角 el (el=-90 即垂直向下)。与 _gimbal_yaw_quat
    组合成世界系目标姿态, 与机体姿态完全解耦。"""
    p = math.radians(elevation_deg) / 2
    return airsim.Quaternionr(0.0, math.sin(p), 0.0, math.cos(p))


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
        self._gen = 0          # 切源代数: 阻塞读取期间切源, 旧源的帧作废
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
        # 软件云台: 全局角度 (yaw, pitch), 同步作用于所有 AirSim 相机。
        # 状态持久化: 地面站重启沿用上次视角; 仅检测到 UE 重新 play 才回中,
        # 见 _restore_or_reset (连接建立时调用)
        (self._gimbal,
         self._gimbal_mount,
         self._gimbal_last) = _load_gimbal_state()
        self._gimbal_anim = None   # 回中动画目标 (yaw, pitch), None=无动画
        self._anim_t_prev = 0.0
        # 摇杆连续转动: 当前角速度 (°/s) 与上次帧时刻 (None=未在转动)
        self._gimbal_rate = (0.0, 0.0)
        self._rate_t_prev = None
        self._gimbal_dirty = False
        # 会话内限位拒绝标记 (方向 -> bool): 仅当按键被限位卡住时标红,
        # 该方向恢复可动即清; 不持久化, 地面站重启后一律不红
        self._gimbal_blocked = {'yaw+': False, 'yaw-': False,
                                'pitch+': False, 'pitch-': False}
        self._gimbal_apply_t = 0.0   # 上次云台下发时刻 (姿态跟随节流用)
        self.running = True
        self._airsim_ok = False     # AirSim RPC 是否已连通 (云台可用性依据)
        self._conn_last_log = 0.0   # AirSim 未连接时的日志节流 (每 10 秒一条)

    def set_camera(self, name):
        with self.lock:
            self.camera = name
            self._gen += 1      # 旧源正在读取中的帧提交时会被本代数拒绝
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

    def gimbal_set_rate(self, yaw_rate, pitch_rate):
        """设置云台连续转动角速度 (°/s, 摇杆控制): 抓取线程每帧按限位
        夹紧步进 (_rate_tick), 回零即停 (位置保持)。限位卡住的方向记入
        _gimbal_blocked 供界面标红。本机/视频文件源为空操作。"""
        with self.lock:
            if self.camera.startswith((LOCAL_PREFIX, FILE_PREFIX)):
                return
            if yaw_rate == 0.0 and pitch_rate == 0.0:
                if any(self._gimbal_rate):
                    _save_gimbal_state(self._gimbal, self._gimbal_mount,
                                       self._gimbal_last)   # 停止时落盘最终角
                self._gimbal_rate = (0.0, 0.0)
                return
            self._gimbal_rate = (yaw_rate, pitch_rate)
            self._gimbal_anim = None   # 摇杆打断回中/摆位动画
            self._rate_t_prev = None   # 下一帧重新计时

    def _rate_tick(self):
        """摇杆连续转动: 每帧按当前角速度步进 (限位夹紧, 被夹住的方向
        标红, 该方向恢复可动即清)。仅在 _gimbal_rate 非零期间生效;
        角度落盘延后到摇杆回零时一次完成 (避免每帧写文件)。"""
        ry, rp = self._gimbal_rate
        if ry == 0.0 and rp == 0.0:
            self._rate_t_prev = None
            return
        now = time.monotonic()
        if self._rate_t_prev is None:
            self._rate_t_prev = now
            return
        dt = min(0.1, now - self._rate_t_prev)
        self._rate_t_prev = now
        yaw, pitch = self._gimbal
        nyaw, npitch = _clamp_gimbal(yaw + ry * dt, pitch + rp * dt)
        # 限位拒绝标记: 该方向被夹住 => 标红; 有实际动作 => 清对向红
        if ry > 0:
            self._gimbal_blocked['yaw+'] = nyaw == yaw
            if nyaw != yaw:
                self._gimbal_blocked['yaw-'] = False
        elif ry < 0:
            self._gimbal_blocked['yaw-'] = nyaw == yaw
            if nyaw != yaw:
                self._gimbal_blocked['yaw+'] = False
        if rp > 0:
            self._gimbal_blocked['pitch+'] = npitch == pitch
            if npitch != pitch:
                self._gimbal_blocked['pitch-'] = False
        elif rp < 0:
            self._gimbal_blocked['pitch-'] = npitch == pitch
            if npitch != pitch:
                self._gimbal_blocked['pitch+'] = False
        if (nyaw, npitch) != (yaw, pitch):
            self._gimbal = (nyaw, npitch)
            self._gimbal_dirty = True   # RPC 下发由节流周期负责

    def gimbal_reset(self):
        """云台回中: 启动平滑回中动画 (非瞬跳), 角速度见
        config.GIMBAL_SLEW_RATE_DEG_S; 摇杆可打断。"""
        with self.lock:
            self._gimbal_blocked = {k: False for k in self._gimbal_blocked}
            if self.camera.startswith((LOCAL_PREFIX, FILE_PREFIX)):
                self._gimbal = (0.0, 0.0)
                return
            self._gimbal_anim = (0.0, 0.0)
            self._anim_t_prev = time.monotonic()

    def gimbal_pitch_bottom(self):
        """俯仰平滑转到垂直向下限位 (视觉反控前自动摆位, 航向保持不变):
        复用回中动画机制, 以 GIMBAL_SLEW_RATE_DEG_S 匀速转动、画面连续
        过渡 (每帧推进并置 dirty 连续下发); 本机/视频文件源为空操作。"""
        with self.lock:
            if self.camera.startswith((LOCAL_PREFIX, FILE_PREFIX)):
                return
            yaw, _ = self._gimbal
            self._gimbal_blocked = {k: False for k in self._gimbal_blocked}
            self._gimbal_anim = (yaw, config.GIMBAL_PITCH_MIN_DEG)
            self._anim_t_prev = time.monotonic()

    def airsim_connected(self):
        """AirSim RPC 是否已连通 (断线/未 play 时为 False)。"""
        with self.lock:
            return self._airsim_ok

    def get_gimbal(self):
        """当前云台角度 (yaw, pitch)；非 AirSim 源返回 None。"""
        with self.lock:
            if self.camera.startswith((LOCAL_PREFIX, FILE_PREFIX)):
                return None
            return self._gimbal

    def get_gimbal_blocked(self):
        """会话内被限位卡住的方向 (按键不再动作), 界面标红用;
        非 AirSim 源恒为全 False。"""
        with self.lock:
            return dict(self._gimbal_blocked)

    def get_cam_orientation(self):
        """当前显示相机的机体系姿态四元数 (x, y, z, w)；非 AirSim 源或尚无
        位姿数据返回 None。

        语义: 相机 NED 体约定 (x 视轴, y 图右, z 图下) 相对机体 FRD 的旋转,
        取最近一次 simSetCameraPose 实际下发的机体相对姿态 (_gimbal_last,
        世界系稳定云台会随飞行周期重下发而持续更新); 云台从未动过时回退
        安装姿态基准。供 aruco_landing 做图像方向 -> 机体系的泛化映射,
        不依赖"图像上边朝机头"假设。"""
        with self.lock:
            if self.camera.startswith((LOCAL_PREFIX, FILE_PREFIX)):
                return None
            q = self._gimbal_last.get(self.camera)
            if q is None:
                mount = self._gimbal_mount.get(self.camera)
                if mount is None:
                    return None
                q = mount[1]
            return (q.x_val, q.y_val, q.z_val, q.w_val)

    def _anim_tick(self):
        """回中动画推进一步 (仅抓取线程调用, 须持锁): 以恒定角速度朝目标
        (0,0) 推进, 时间驱动不受帧率影响; 到位后停用动画并持久化。"""
        if self._gimbal_anim is None:
            return
        ty, tp = self._gimbal_anim
        yaw, pitch = self._gimbal
        dy, dp = ty - yaw, tp - pitch
        dist = math.hypot(dy, dp)
        now = time.monotonic()
        dt = min(0.1, now - self._anim_t_prev)
        self._anim_t_prev = now
        max_d = config.GIMBAL_SLEW_RATE_DEG_S * dt
        if dist <= max_d:
            self._gimbal = (ty, tp)
            self._gimbal_anim = None
            _save_gimbal_state(self._gimbal, self._gimbal_mount,
                               self._gimbal_last)
        else:
            s = max_d / dist
            self._gimbal = _clamp_gimbal(yaw + dy * s, pitch + dp * s)
        self._gimbal_dirty = True

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
        # 机体明显倾斜时, 两次 RPC 之间的姿态变化会污染安装姿态基准
        # (z 轴偏离竖直超过 ~5.7° 时告警); 建议静止平放时再首次使用云台
        z_world = _quat_rot_vec(q_b, airsim.Vector3r(0.0, 0.0, 1.0))
        if z_world.z_val < 0.995:
            self.log('云台: 机体姿态倾斜中, 安装基准可能不准, 建议平放静止时'
                     '使用云台 (重连一次可重采基准)')
        p_c, q_c = info.pose.position, info.pose.orientation
        # 安装姿态 (机体系): q_mount = q_body⁻¹ ⊗ q_cam_world
        q_rel = _quat_mul(_quat_conj(q_b), q_c)
        # 安装位置 (机体系): p_mount = R_body⁻¹ · (p_cam - p_body)
        d = airsim.Vector3r(p_c.x_val - p_b.x_val,
                            p_c.y_val - p_b.y_val,
                            p_c.z_val - p_b.z_val)
        p_rel = _quat_rot_vec(_quat_conj(q_b), d)
        return p_rel, q_rel

    def _restore_or_reset(self, client):
        """连接建立时判断 UE 是否重新 play 过: 对比各相机当前机体系姿态与
        上次下发值 (机体相对姿态不随飞机移动/姿态变化, 比对可靠)。
        全部一致 => 地面站只是重启/重连, 相机没动过, 沿用记忆不碰相机;
        有不一致 => UE 重玩过相机已复位, 云台回中并重采安装基准。"""
        try:
            with self.lock:
                last = dict(self._gimbal_last)
            if not last:
                return                      # 首次运行, 无记忆可恢复
            q_b = client.simGetGroundTruthKinematics(VEHICLE).orientation
            replayed = False
            for cam, q_cmd in last.items():
                try:
                    info = client.simGetCameraInfo(cam, VEHICLE)
                except Exception:
                    continue                # 相机不存在, 跳过比对
                q_cur = _quat_mul(_quat_conj(q_b), info.pose.orientation)
                # 四元数夹角: |dot| = cos(θ/2), q 与 -q 同旋转
                dot = abs(q_cur.w_val * q_cmd.w_val
                          + q_cur.x_val * q_cmd.x_val
                          + q_cur.y_val * q_cmd.y_val
                          + q_cur.z_val * q_cmd.z_val)
                if dot < math.cos(math.radians(
                        _GIMBAL_REPLAY_THRESHOLD_DEG) / 2.0):
                    replayed = True
                    break
            with self.lock:
                if replayed:
                    self.log('检测到 UE 重新 play, 云台回中')
                    self._gimbal = (0.0, 0.0)
                    self._gimbal_mount = {}
                    self._gimbal_dirty = True
                    self._gimbal_blocked = {k: False
                                            for k in self._gimbal_blocked}
                else:
                    self._gimbal_dirty = False   # 相机未动, 记忆仍然有效
        except Exception as e:
            self.log(f'云台状态检查失败, 保持当前状态: {e}')

    def _apply_gimbal(self, client, yaw_deg, pitch_deg):
        """把云台角度经 simSetCameraPose 同步写到所有 AirSim 相机。

        simSetCameraPose 按"机体相对位姿"解释输入, 而机体在悬停/飞行中带着
        姿态倾斜: 若把云台角直接叠加在机体系安装姿态上, 偏航绕的是机体 z 轴
        而非竖直轴 —— 绕斜轴连续步进会沿斜锥面进动, 起初看着在左转, 转过
        一定角度视角就甩到天上/地下, 俯仰也随之变斜、地平线滚动。

        这里改为世界系稳定云台: 目标视线方向用 (方位角, 仰角) 在世界系直接
        重建 —— 方位 = 机体前向方位 + 云台偏航 + 安装偏航, 仰角 = 安装仰角 +
        云台俯仰, roll 恒为 0 (地平线水平)。机体怎么转、怎么斜, 视线都钉在
        世界系目标方向上 (垂直向下时原地旋转/倾斜仍保持垂直向下), 方向键的
        上下左右与机体姿态完全无关。历次错误做法的教训: 两角合成一个四元数
        世界系左乘会把俯仰轴钉死在世界 East 轴 (航向一变上下变侧滚/反向);
        都机体系叠加则偏航绕斜轴进动、俯仰随姿态倾斜漂移。
        云台偏航的参考是当前机头朝向, 回中恢复安装方向的水平重建。"""
        try:
            q_b = client.simGetGroundTruthKinematics(VEHICLE).orientation
            applied = {}
            for cam in DEFAULT_CAMERAS:
                if cam not in self._gimbal_mount:
                    try:
                        self._gimbal_mount[cam] = self._capture_mount(client, cam)
                    except Exception as e:
                        self._gimbal_mount[cam] = None   # 该相机不可用, 跳过
                        self.log(f'云台: 获取 {cam} 安装位姿失败, 跳过: {e}')
                        continue
                mount = self._gimbal_mount[cam]
                if mount is None:
                    continue
                pos, ori = mount
                # 世界系稳定云台: 方位 = 机体前向方位 + 云台偏航 + 安装偏航,
                # 仰角 = 安装仰角 + 云台俯仰, 按世界系方位/仰角重建目标姿态
                # (roll=0), 再反算回机体系下发
                yaw_b, _ = _quat_yaw_el(q_b)
                yaw_m, el_m = _quat_yaw_el(ori)
                q_des_world = _quat_mul(
                    _gimbal_yaw_quat(yaw_b + yaw_deg + yaw_m),
                    _gimbal_pitch_quat(el_m + pitch_deg))
                q_cmd = _quat_mul(_quat_conj(q_b), q_des_world)
                try:
                    client.simSetCameraPose(cam, airsim.Pose(pos, q_cmd), VEHICLE)
                    applied[cam] = q_cmd
                except Exception as e:
                    self.log(f'云台: {cam} 下发失败: {e}')
            if applied:
                # 记录本次实际下发的相机姿态 (供 UE 重玩检测比对) 并持久化
                with self.lock:
                    self._gimbal_last = applied
                _save_gimbal_state(self._gimbal, self._gimbal_mount, applied)
        except Exception as e:
            # 失败不置回 dirty: 避免每帧重试刷屏; 重连或下次按键会重新下发
            self.log(f'云台控制失败: {e}')
        finally:
            self._gimbal_dirty = False

    def _commit(self, arr, is_depth, gen):
        """写入最新帧并统计 fps。gen 与当前切源代数不符 (读取阻塞期间
        发生了切源) 时丢弃 —— 否则旧源最后一帧会在切源后回填, 界面清屏
        被覆盖, 表现为"卡在旧画面, 要再切一次才出现等待提示"。"""
        with self.lock:
            if gen != self._gen:
                return
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
                    gen = self._gen   # 本轮帧的切源代数, 提交时校验
                if cam.startswith(LOCAL_PREFIX):    # 本机摄像头: 不经 AirSim
                    arr = self._read_local(cam[len(LOCAL_PREFIX):])
                    if arr is not None:
                        self._commit(arr, False, gen)
                    time.sleep(max(0.0, 1.0 / LOCAL_HZ - (time.monotonic() - t_start)))
                    continue
                if cam.startswith(FILE_PREFIX):     # 视频文件: 按文件帧率循环播放
                    arr = self._read_file(cam[len(FILE_PREFIX):])
                    if arr is not None:
                        self._commit(arr, False, gen)
                    time.sleep(max(0.0, self.file_interval - (time.monotonic() - t_start)))
                    continue
                if self.cap_dev is not None:        # 切回 AirSim 相机时释放本机设备
                    self.cap.release()
                    self.cap = self.cap_dev = None
                if client is None:
                    # 连接探测必须用短超时探针: VehicleClient 默认 RPC 超时
                    # 长达 3600s (client.py timeout_value=3600), UE 未 play 时
                    # confirmConnection 里的同步 ping 会把抓取线程阻塞在
                    # 这里, 切到本机摄像头/文件源也回不到循环顶 —— 直到
                    # UE play ping 通才恢复。探针 2s ping 通才建正式连接。
                    try:
                        probe = airsim.VehicleClient(
                            ip=self.host, port=self.port, timeout_value=0.5)
                        probe.ping()
                    except Exception as e:
                        # 未连接状态持续重试: 节流到每 10 秒一条, 不刷屏
                        if time.monotonic() - self._conn_last_log > 10.0:
                            self.log(f'等待 AirSim {self.host}:{self.port}: {e}')
                            self._conn_last_log = time.monotonic()
                        with self.lock:
                            self._airsim_ok = False
                        time.sleep(1.0)
                        continue
                    client = airsim.VehicleClient(ip=self.host, port=self.port)
                    with self.lock:
                        self._airsim_ok = True
                    self.log(f'已连接 AirSim {self.host}:{self.port}')
                    # 判断 UE 是否重新 play: play 过则回中, 没过则沿用记忆
                    self._restore_or_reset(client)

                is_depth = 'depth' in cam.lower()
                with self.lock:
                    self._rate_tick()    # 摇杆连续转动: 每帧按角速度推进
                    self._anim_tick()    # 回中动画: 每帧推进并置 dirty
                    gim_dirty, gim = self._gimbal_dirty, self._gimbal
                # 云台角只在按键瞬间按当时机体姿态换算一次, 相机随后刚性跟
                # 随机体; 飞行中机体姿态一变 (前飞俯身/转向偏航), 记忆的
                # (yaw,pitch) 就与实际视角脱钩, 下次按键会把积累的姿态差一
                # 次性归零 —— 方向键不再纯上下/左右。因此除 dirty 立即下发
                # 外, 还周期性按当前机体姿态重下发保持基准新鲜 (回中动画
                # 每帧置 dirty 本就连续下发, 方向键路径在此补齐)。
                now = time.monotonic()
                if gim_dirty or (
                        config.GIMBAL_HOLD_HZ > 0 and gim != (0.0, 0.0)
                        and now - self._gimbal_apply_t
                        >= 1.0 / config.GIMBAL_HOLD_HZ):
                    self._apply_gimbal(client, *gim)
                    self._gimbal_apply_t = now
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
                    self._commit(arr, is_depth, gen)
                else:
                    time.sleep(0.05)
                # 自适应节流: 扣除 RPC 往返耗时后再补足剩余间隔。
                # 往返超过 1/GRAB_HZ 时不再额外睡眠, 帧率由网络/渲染延迟决定
                time.sleep(max(0.0, 1.0 / GRAB_HZ - (time.monotonic() - t_start)))
            except Exception as e:
                self.log(f'连接中断: {e}，1 秒后重连...')
                client = None
                time.sleep(1.0)
