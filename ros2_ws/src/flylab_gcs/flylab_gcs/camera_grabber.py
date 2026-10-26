#!/usr/bin/env python3
"""AirSim 相机抓取器 —— 直连 RPC 拉图 + 本机 V4L2 摄像头, 与界面解耦。

被 camera_view (独立调试) 和 ground_station 共用。附带软件云台: 经
simSetCameraPose 旋转相机视角; 世界系稳定云台把视线方位/仰角钉在世界系,
机体转动/倾斜由周期重下发 (GIMBAL_HOLD_HZ) 补偿。

支撑模块 (本文件只留抓取线程与云台运行时状态):
  quat_math.py      四元数数学工具 (纯函数)
  gimbal.py         云台限位/状态持久化/安装基准采集
  camera_sources.py 本机摄像头/视频文件设备 IO (colorize_depth 也在此)
"""
import math
import threading
import time

import numpy as np

# /dev/video* 的 metadata 节点探测失败是预期行为, 屏蔽 V4L2 WARN 刷屏
import os
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
from .camera_sources import (LocalFileReader, colorize_depth,   # noqa: F401 (re-export)
                             list_local_cameras)
from .gimbal import (APPLY_TOL_DEG, REPLAY_THRESHOLD_DEG, clamp_gimbal,
                     capture_mount, load_gimbal_state, save_gimbal_state)
from .quat_math import (gimbal_pitch_quat, gimbal_yaw_quat, quat_conj,
                        quat_mul, quat_yaw_el)

VEHICLE = 'PX4'
GRAB_HZ = 60        # AirSim 拉取频率上限 (实际帧率 = min(该值, 1/RPC往返延迟))
LOCAL_HZ = 30       # 本机摄像头拉取频率上限

DEFAULT_CAMERAS = ['left_cam', 'front_cam', 'right_cam', 'depth_cam']
DEFAULT_CAMERA = 'front_cam'   # 启动默认源 (AirSim)
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


class Grabber(threading.Thread):
    """后台线程：以 GRAB_HZ/LOCAL_HZ 持续拉取当前相机的最新一帧。"""

    def __init__(self, log, host, port):
        super().__init__(daemon=True)
        self.log = log
        self.host, self.port = host, int(port)
        self.lock = threading.Lock()
        self.camera = DEFAULT_CAMERA
        self.frame = None      # (np.ndarray, is_depth)
        self._gen = 0          # 切源代数: 阻塞读取期间切源, 旧源的帧作废
        self.fps = 0.0
        self.n, self.t_prev = 0, time.time()
        # 本机摄像头/视频文件设备 IO (见 camera_sources.LocalFileReader;
        # 设备仅在抓取线程访问, 播放控制标志位由界面线程设置)
        self.reader = LocalFileReader(log)
        # 软件云台: 全局角度 (yaw, pitch), 作用于所有 AirSim 相机。
        # 状态持久化, 重启沿用; 仅 UE 重新 play 才回中 (见 _restore_or_reset)
        (self._gimbal, self._gimbal_mount, self._gimbal_last,
         self._fov) = load_gimbal_state()
        # 全局视场角 (度): 所有相机共用 (None=从未调整, 保持 settings.json 原值)
        self._fov_saved = self._fov   # 上次落盘值 (变化才写盘)
        self._fov_dirty = False       # 视场角待下发标记
        # 数字变焦 (抓取层中心裁切, 图传/转发/标注同一像素系);
        # _raw 留未变焦原帧, 档位变化时重提交
        self._zoom = 1.0
        self._raw = None
        self._gimbal_anim = None   # 回中动画目标 (yaw, pitch), None=无动画
        self._anim_t_prev = 0.0
        # 摇杆连续转动: 当前角速度 (°/s) 与上次帧时刻 (None=未在转动)
        self._gimbal_rate = (0.0, 0.0)
        self._rate_t_prev = None
        self._gimbal_dirty = False
        # 云台稳定 (follow 模式): 云台角按 GIMBAL_HOLD_HZ 世界系持续重下发
        # (俯仰/横滚不随机体晃动, 偏航跟随机头); 关闭为刚性挂载 (FPV 手感)。
        # UE 原生 Gimbal.Stabilization 与 simSetCameraPose 互斥, 故在地面站侧实现
        self._gimbal_stab = True
        # 上次持久化时的云台角 (连续下发期间避免频繁写盘)
        self._gimbal_saved = None
        # 上次成功下发的 (机体姿态, 云台角, 相机) 指纹: 悬停抖动在容差内
        # 则跳过下发 RPC, 省下的串行等待直接转化为图传帧率
        self._gimbal_apply_key = None
        # Scene 帧走 PNG 压缩传输 (线上体积约 1/3); 服务端不支持时回退原始传输
        self._scene_compress = config.DISPLAY_SCENE_COMPRESS and cv2 is not None
        # 会话内限位拒绝标记 (方向 -> bool), 按键被限位卡住时标红, 不持久化
        self._gimbal_blocked = {'yaw+': False, 'yaw-': False,
                                'pitch+': False, 'pitch-': False}
        self._gimbal_apply_t = 0.0   # 上次云台下发时刻 (姿态跟随节流用)
        self.running = True
        self._airsim_ok = False     # AirSim RPC 是否已连通 (云台可用性依据)
        self._conn_last_log = 0.0   # 未连接日志节流 (每 10 秒一条)
        # 云台专用 RPC 连接 (惰性创建): 与拉图分离, 下发不阻塞拉图往返
        self._gimbal_client = None

    def set_camera(self, name):
        with self.lock:
            self.camera = name
            self._gen += 1      # 旧源读取阻塞期间的帧提交时被代数拒绝
            self.frame = None   # 丢弃旧相机画面，避免切换时闪现错图
            # 切源后视角/视场已过时, 置 dirty 让下一拍立即补下发
            self._gimbal_dirty = True
            self._fov_dirty = True

    def stop(self):
        self.running = False

    def release_local(self):
        """释放本机摄像头设备 (须在抓取线程 join 之后调用)。"""
        self.reader.release()

    def get(self):
        with self.lock:
            return self.frame, self.camera, self.fps

    # ---------- 视频文件播放控制 (界面线程调用, 标志位由抓取线程处理) ----------

    def set_file_paused(self, paused):
        self.reader.paused = paused

    def stop_file(self):
        """停止播放: 回到第 0 帧并暂停。"""
        self.reader.stop_flag = True

    def set_file_position(self, frac):
        """跳转到相对位置 (0..1)，暂停中也可用。"""
        self.reader.seek = max(0.0, min(1.0, frac))

    def get_progress(self):
        """(当前帧, 总帧数, 是否暂停, 文件fps)；非文件源返回 None。"""
        with self.lock:
            if self.camera.startswith(FILE_PREFIX):
                return self.reader.progress()
        return None

    # ---------- 软件云台 (界面线程调用, 角度由抓取线程经 RPC 写入相机) ----------

    def gimbal_set_rate(self, yaw_rate, pitch_rate):
        """设置云台连续转动角速度 (°/s, 摇杆控制), 每帧按限位夹紧步进,
        回零即停; 限位卡住的方向记入 _gimbal_blocked 供标红。"""
        with self.lock:
            if self.camera.startswith((LOCAL_PREFIX, FILE_PREFIX)):
                return
            if yaw_rate == 0.0 and pitch_rate == 0.0:
                if any(self._gimbal_rate):
                    save_gimbal_state(self._gimbal, self._gimbal_mount,
                                      self._gimbal_last, self._fov)   # 停止时落盘最终角
                self._gimbal_rate = (0.0, 0.0)
                return
            self._gimbal_rate = (yaw_rate, pitch_rate)
            self._gimbal_anim = None   # 摇杆打断回中/摆位动画
            self._rate_t_prev = None   # 下一帧重新计时

    def _rate_tick(self):
        """摇杆连续转动: 每帧按角速度步进 (限位夹紧并标红);
        角度落盘延后到摇杆回零时一次完成。"""
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
        nyaw, npitch = clamp_gimbal(yaw + ry * dt, pitch + rp * dt)
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
        """俯仰平滑转到垂直向下限位 (视觉反控前自动摆位, 航向不变),
        复用回中动画机制匀速转动。"""
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

    def get_gimbal_stab(self):
        """云台稳定 (follow 模式) 是否开启 (界面开关初始态)。"""
        with self.lock:
            return self._gimbal_stab

    def set_gimbal_stab(self, on):
        """开关云台稳定: 开启立即补一拍下发 (视角马上稳定);
        关闭即停在当前姿态转为刚性跟随。"""
        with self.lock:
            if self._gimbal_stab == on:
                return
            self._gimbal_stab = on
            self._gimbal_dirty = True

    # ---------- 视场角 (界面线程调用, 由抓取线程经 RPC 写入相机) ----------

    def set_fov(self, deg):
        """设置全局视场角 (度): 记忆 + 置 dirty, 下一拍下发到当前相机。
        滑杆高频调用, 节流靠抓取帧率 (每拍最多一次 RPC)。"""
        with self.lock:
            if self.camera.startswith((LOCAL_PREFIX, FILE_PREFIX)):
                return
            deg = max(config.GIMBAL_FOV_MIN_DEG,
                      min(config.GIMBAL_FOV_MAX_DEG, float(deg)))
            if self._fov == deg:
                return
            self._fov = deg
            self._fov_dirty = True

    def get_fov(self):
        """全局视场角 (度); 未调整过返回默认 (界面滑杆显示用)。
        本机/视频文件源返回 None (界面置灰依据)。"""
        with self.lock:
            if self.camera.startswith((LOCAL_PREFIX, FILE_PREFIX)):
                return None
            return self._fov if self._fov is not None \
                else config.GIMBAL_FOV_DEFAULT_DEG

    def get_fov_applied(self):
        """实际生效的全局视场角 (度); 用户从未调整过返回 None
        (视场角同步发布依据: 不拿界面默认值冒充相机真实 FOV)。
        本机/视频文件源返回 None。"""
        with self.lock:
            if self.camera.startswith((LOCAL_PREFIX, FILE_PREFIX)):
                return None
            return self._fov

    # ---------- 数字变焦 (界面滚轮/双击/滑杆, 裁切由抓取线程在 _commit 做) ----------

    def set_zoom(self, factor):
        """设置数字变焦倍数 (1.0=原图, 上限 config.DISPLAY_ZOOM_MAX):
        立即用缓存的未变焦原帧重提交一帧, 暂停中的文件源也能即时看到;
        之后每帧按该倍数中心裁切。"""
        with self.lock:
            factor = max(1.0, min(config.DISPLAY_ZOOM_MAX, float(factor)))
            if self._zoom == factor:
                return
            self._zoom = factor
            raw, gen = self._raw, self._gen
        if raw is None:
            return
        arr, is_depth = raw
        if factor > 1.0:
            arr = self._zoom_center(arr, factor)
        with self.lock:
            if gen == self._gen:
                self.frame = (arr, is_depth)

    def get_zoom(self):
        with self.lock:
            return self._zoom

    @staticmethod
    def _zoom_center(arr, z):
        """中心裁切 1/z 区域并放回原分辨率 (图传/转发/标注坐标不变)。"""
        h, w = arr.shape[:2]
        cw, ch = max(1, int(w / z)), max(1, int(h / z))
        x0, y0 = (w - cw) // 2, (h - ch) // 2
        crop = np.ascontiguousarray(arr[y0:y0 + ch, x0:x0 + cw])
        if cv2 is None:          # 无 OpenCV 只裁不缩, 显示端等比放大兜底
            return crop
        return cv2.resize(crop, (w, h), interpolation=cv2.INTER_LINEAR)

    def get_cam_orientation(self):
        """当前显示相机的机体系姿态四元数 (x, y, z, w)；非 AirSim 源返回 None。

        语义: 相机 NED 体约定 (x 视轴, y 图右, z 图下) 相对机体 FRD 的旋转,
        取最近一次实际下发的机体相对姿态, 云台未动过时回退安装基准。
        供 aruco_landing 做图像方向 -> 机体系的泛化映射。"""
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
            save_gimbal_state(self._gimbal, self._gimbal_mount,
                              self._gimbal_last, self._fov)
        else:
            s = max_d / dist
            self._gimbal = clamp_gimbal(yaw + dy * s, pitch + dp * s)
        self._gimbal_dirty = True

    def _restore_or_reset(self, client):
        """连接建立时判断 UE 是否重新 play 过: 对比各相机当前机体系姿态与
        上次下发值 (机体相对姿态不随飞机移动, 比对可靠)。
        一致 => 沿用记忆; 不一致 => UE 重玩过, 云台回中并重采安装基准。"""
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
                q_cur = quat_mul(quat_conj(q_b), info.pose.orientation)
                # 四元数夹角: |dot| = cos(θ/2), q 与 -q 同旋转
                dot = abs(q_cur.w_val * q_cmd.w_val
                          + q_cur.x_val * q_cmd.x_val
                          + q_cur.y_val * q_cmd.y_val
                          + q_cur.z_val * q_cmd.z_val)
                if dot < math.cos(math.radians(REPLAY_THRESHOLD_DEG) / 2.0):
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

    def _apply_gimbal(self, client, yaw_deg, pitch_deg, cam_cur):
        """把云台角度经 simSetCameraPose 同步写到 AirSim 相机。

        只持续下发当前相机; 其余相机仅在首次同步时下发一次 (供 UE 重玩
        检测), 切到它时指纹含相机名必失配, 下一拍立即补发。飞行中每拍
        只有 1 次 kinematics + 1 次 simSetCameraPose, 且走云台专用 RPC
        连接, 不阻塞拉图。

        simSetCameraPose 按"机体相对位姿"解释输入, 若把云台角直接叠加在
        机体系安装姿态上, 偏航绕的是机体 z 轴 (斜轴), 连续步进会沿斜锥面
        进动、视角甩偏。改为世界系稳定云台: 目标视线用 (方位角, 仰角) 在
        世界系直接重建 —— 方位 = 机体前向方位 + 云台偏航 + 安装偏航,
        仰角 = 安装仰角 + 云台俯仰, roll=0。视线钉在世界系, 方向键与机体
        姿态完全无关。"""
        try:
            q_b = client.simGetGroundTruthKinematics(VEHICLE).orientation
            # 指纹 = (机体姿态, 云台角, 当前相机), 姿态按四元数夹角容差判定
            # (|dot| = cos(θ/2)): 悬停抖动在容差内跳过下发, 真实机动照常补偿
            last = self._gimbal_apply_key
            if last is not None:
                lq, lyaw, lpitch, lcam = last
                dot = abs(q_b.x_val * lq.x_val + q_b.y_val * lq.y_val
                          + q_b.z_val * lq.z_val + q_b.w_val * lq.w_val)
                if (lcam == cam_cur and lyaw == yaw_deg and lpitch == pitch_deg
                        and dot >= math.cos(math.radians(APPLY_TOL_DEG) / 2.0)):
                    return
            applied = {}
            for cam in DEFAULT_CAMERAS:
                if cam != cam_cur and cam in self._gimbal_last:
                    continue   # 非当前相机且已同步过: 切到它时才会再下发
                if cam not in self._gimbal_mount:
                    try:
                        self._gimbal_mount[cam] = capture_mount(
                            client, cam, self.log)
                    except Exception as e:
                        self._gimbal_mount[cam] = None   # 该相机不可用, 跳过
                        self.log(f'云台: 获取 {cam} 安装位姿失败, 跳过: {e}')
                        continue
                mount = self._gimbal_mount[cam]
                if mount is None:
                    continue
                pos, ori = mount
                # 世界系重建目标姿态 (方位/仰角, roll=0), 再反算回机体系下发
                yaw_b, _ = quat_yaw_el(q_b)
                yaw_m, el_m = quat_yaw_el(ori)
                q_des_world = quat_mul(
                    gimbal_yaw_quat(yaw_b + yaw_deg + yaw_m),
                    gimbal_pitch_quat(el_m + pitch_deg))
                q_cmd = quat_mul(quat_conj(q_b), q_des_world)
                try:
                    client.simSetCameraPose(cam, airsim.Pose(pos, q_cmd), VEHICLE)
                    applied[cam] = q_cmd
                except Exception as e:
                    self.log(f'云台: {cam} 下发失败: {e}')
            if applied:
                # 指纹仅在成功下发后更新; 持久化仅在云台角变化时进行
                # (稳定模式连续下发, 角度不变的周期写盘就是纯 IO 浪费)
                with self.lock:
                    self._gimbal_last = applied
                    changed = (yaw_deg, pitch_deg) != self._gimbal_saved
                    if changed:
                        self._gimbal_saved = (yaw_deg, pitch_deg)
                if changed:
                    save_gimbal_state(self._gimbal, self._gimbal_mount,
                                      applied, self._fov)
        except Exception as e:
            # 失败不置回 dirty (避免每帧重试刷屏); 置空云台连接下拍惰性重建
            self.log(f'云台控制失败: {e}')
            self._gimbal_client = None
        finally:
            self._gimbal_dirty = False

    def _apply_fov(self, client, cam):
        """把全局视场角经 simSetCameraFov 写入当前相机 (走云台专用 RPC)。
        只在 dirty 时对齐一次, 不逐帧重发; 从未调过 (None) 保持原值。"""
        with self.lock:
            fov = self._fov
            if fov is None:
                self._fov_dirty = False
                return
        try:
            client.simSetCameraFov(cam, fov, VEHICLE)
            self._fov_dirty = False
            with self.lock:
                if self._fov_saved != fov:
                    self._fov_saved = fov
                    save_gimbal_state(self._gimbal, self._gimbal_mount,
                                      self._gimbal_last, self._fov)
        except Exception as e:
            self.log(f'视场角下发失败: {e}')
            self._fov_dirty = False
            self._gimbal_client = None

    def _commit(self, arr, is_depth, gen):
        """写入最新帧并统计 fps。切源代数不符 (读取阻塞期间切了源) 时
        丢弃, 否则旧源最后一帧会在切源后回填。"""
        with self.lock:
            if gen != self._gen:
                return
            zoom = self._zoom
            self._raw = (arr, is_depth)   # 未变焦原帧 (变焦档变化时重提交)
        if zoom > 1.0:
            arr = self._zoom_center(arr, zoom)   # 裁切放锁外, 不阻塞 get()
        with self.lock:
            if gen != self._gen:
                return
            self.frame = (arr, is_depth)
        self.n += 1
        now = time.time()
        if now - self.t_prev >= 1.0:
            self.fps = self.n / (now - self.t_prev)
            self.n, self.t_prev = 0, now

    def run(self):
        client = None
        while self.running:
            try:
                t_start = time.monotonic()
                with self.lock:
                    cam = self.camera
                    gen = self._gen   # 本轮帧的切源代数, 提交时校验
                if cam.startswith(LOCAL_PREFIX):    # 本机摄像头: 不经 AirSim
                    arr = self.reader.read_local(cam[len(LOCAL_PREFIX):])
                    if arr is not None:
                        self._commit(arr, False, gen)
                    time.sleep(max(0.0, 1.0 / LOCAL_HZ - (time.monotonic() - t_start)))
                    continue
                if cam.startswith(FILE_PREFIX):     # 视频文件: 按文件帧率循环播放
                    arr = self.reader.read_file(cam[len(FILE_PREFIX):])
                    if arr is not None:
                        self._commit(arr, False, gen)
                    time.sleep(max(0.0, self.reader.interval - (time.monotonic() - t_start)))
                    continue
                self.reader.close()                 # 切回 AirSim 相机时释放本机设备
                if client is None:
                    # 连接探测必须用短超时探针: VehicleClient 默认 RPC 超时
                    # 3600s, UE 未 play 时 ping 会把抓取线程长期阻塞在这里
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
                    # 正式连接带 3s 超时: 默认 3600s 下 UE 卡顿会把抓图线程
                    # 冻结成图传假死, 超时抛异常走重连
                    client = airsim.VehicleClient(
                        ip=self.host, port=self.port, timeout_value=3.0)
                    self._gimbal_client = airsim.VehicleClient(
                        ip=self.host, port=self.port, timeout_value=3.0)
                    with self.lock:
                        self._airsim_ok = True
                    self.log(f'已连接 AirSim {self.host}:{self.port}')
                    # 判断 UE 是否重新 play: play 过则回中, 没过则沿用记忆
                    self._restore_or_reset(client)
                    # 视场角对齐: 重启/重 play 后按记忆恢复
                    with self.lock:
                        if self._fov is not None:
                            self._fov_dirty = True

                is_depth = 'depth' in cam.lower()
                with self.lock:
                    self._rate_tick()    # 摇杆连续转动: 每帧按角速度推进
                    self._anim_tick()    # 回中动画: 每帧推进并置 dirty
                    gim_dirty, gim = self._gimbal_dirty, self._gimbal
                    stab = self._gimbal_stab
                # 云台角只在下发瞬间按当时机体姿态换算, 机体姿态一变记忆
                # 角就与实际视角脱钩, 故除 dirty 立即下发外还周期重下发;
                # 稳定模式下居中角 (0,0) 也持续重下发 (视角保持水平),
                # 非稳定模式居中=刚性挂载 (省 RPC)
                now = time.monotonic()
                if self._gimbal_client is None:
                    # 云台连接惰性重建: 构造失败 (UE 未 play 等) 不拖累拉图
                    try:
                        self._gimbal_client = airsim.VehicleClient(
                            ip=self.host, port=self.port, timeout_value=3.0)
                    except Exception:
                        pass
                if self._gimbal_client is not None and (
                        gim_dirty or (
                            config.GIMBAL_HOLD_HZ > 0
                            and (stab or gim != (0.0, 0.0))
                            and now - self._gimbal_apply_t
                            >= 1.0 / config.GIMBAL_HOLD_HZ)):
                    self._apply_gimbal(self._gimbal_client, *gim, cam)
                    self._gimbal_apply_t = now
                # 视场角: dirty 时按当前相机记忆值对齐一次 (与云台共用
                # 专用 RPC 连接, 下发不阻塞拉图)
                if self._fov_dirty and self._gimbal_client is not None:
                    self._apply_fov(self._gimbal_client, cam)
                # 深度必须用 DepthPlanar: Scene+float 读回会触发 D3D12
                # 断言并返回 NaN/Inf
                if is_depth:
                    req = airsim.ImageRequest(
                        cam, airsim.ImageType.DepthPlanar, True, False)
                else:
                    # Scene 走 PNG 压缩传输 (线上体积约 1/3, RPC 往返变短直接转化为帧率)
                    req = airsim.ImageRequest(
                        cam, airsim.ImageType.Scene, False, self._scene_compress)
                resp = client.simGetImages([req], VEHICLE)

                if resp and resp[0].width > 0:
                    r = resp[0]
                    if is_depth:   # DepthPlanar, 单位米
                        arr = _to_np(
                            r.image_data_float, np.float32).reshape(r.height, r.width)
                    else:          # BGR (compress=False) 或 PNG (compress=True)
                        data = _to_np(r.image_data_uint8, np.uint8)
                        arr = None
                        if self._scene_compress:
                            arr = cv2.imdecode(data, cv2.IMREAD_COLOR)
                            if arr is None:
                                # 服务端没按 PNG 返回 (版本差异/异常数据):
                                # 本会话回退原始传输, 下一拍走原始解码路径
                                self._scene_compress = False
                                self.log('图传: 服务端未按 PNG 返回, 回退原始传输')
                        if arr is None and not self._scene_compress:
                            # 服务端原始字节是 RGB 序, 显示约定 BGR —— 翻转通道
                            arr = np.ascontiguousarray(
                                data.reshape(r.height, r.width, 3)[..., ::-1])
                    if arr is not None:
                        self._commit(arr, is_depth, gen)
                    else:
                        time.sleep(0.05)
                # 自适应节流: 扣除 RPC 往返后补足剩余间隔
                time.sleep(max(0.0, 1.0 / GRAB_HZ - (time.monotonic() - t_start)))
            except Exception as e:
                self.log(f'连接中断: {e}，1 秒后重连...')
                client = None
                self._gimbal_client = None
                time.sleep(1.0)
