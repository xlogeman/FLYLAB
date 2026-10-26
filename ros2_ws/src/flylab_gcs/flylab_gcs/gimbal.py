#!/usr/bin/env python3
"""云台支撑逻辑 —— 角度限位、状态持久化、安装基准采集。

云台运行时状态 (当前角/动画/摇杆角速度/下发指纹等) 仍在 Grabber 里
(与抓取循环共用一把锁、一帧一拍推进); 本模块只放可独立描述的部分:
限位夹紧、~/.flylab_gimbal.json 的读写、相机安装位姿采集。
"""
import json
import math
import os

try:
    import cosysairsim as airsim
except ImportError:
    import sys
    sys.path.insert(0, '/root/flylab/Cosys-AirSim/PythonClient')
    import cosysairsim as airsim

from . import config
from .quat_math import quat_conj, quat_mul, quat_rot_vec

# 云台状态持久化文件。地面站重启时沿用上次视角与安装基准 (UE 端相机并没
# 被动过); 连接建立时对比相机实际姿态与上次下发值即可判断 UE 是否重新
# play 过 —— play 过则回中重采基准, 没过则保持记忆 (见 Grabber._restore_or_reset)。
STATE_FILE = os.path.join(os.path.expanduser('~'), '.flylab_gimbal.json')

# 判定 UE 重新 play 的阈值 (度): 相机实际姿态与上次下发值相差超过该值
REPLAY_THRESHOLD_DEG = 2.0

# 云台稳定重下发的姿态容差 (度): 机体姿态与上次下发时相对转角在该阈值内
# 视为"没变" 跳过整批下发; 0.25° 视角偏移肉眼不可见, 却能滤掉大半悬停
# 抖动 (真实机动远超阈值照常下发补偿)。若飞行中发现视角有可见的缓慢
# 漂移再调回 0.1
APPLY_TOL_DEG = 0.25


def clamp_gimbal(yaw, pitch):
    """按 config 限位夹紧云台角度 (config.GIMBAL_*_LIMIT_DEG)。"""
    yaw = max(-config.GIMBAL_YAW_LIMIT_DEG,
              min(config.GIMBAL_YAW_LIMIT_DEG, yaw))
    pitch = max(config.GIMBAL_PITCH_MIN_DEG,
                min(config.GIMBAL_PITCH_MAX_DEG, pitch))
    return yaw, pitch


def load_gimbal_state():
    """读取持久化的云台角度/安装位姿/上次下发姿态; 没有或损坏时返回空。"""
    try:
        with open(STATE_FILE) as f:
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
        fov_v = state.get('fov')
        fov = float(fov_v) if isinstance(fov_v, (int, float)) else None
        return (clamp_gimbal(float(gimbal[0]), float(gimbal[1])),
                mounts, last_applied, fov)
    except Exception:
        return (0.0, 0.0), {}, {}, None


def save_gimbal_state(gimbal, mounts, last_applied, fov=None):
    """持久化云台状态 (fov=全局视场角, 与云台同一文件); 失败静默。"""
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
        data['fov'] = fov
        with open(STATE_FILE, 'w') as f:
            json.dump(data, f)
    except Exception:
        pass


def capture_mount(client, cam, log):
    """采集相机安装位姿 (机体系)。

    注意 AirSim 的坐标系陷阱: simGetCameraInfo 返回的是世界系位姿
    (位置相对出生点、姿态为世界姿态, 见 PIPCamera.cpp getCameraInfo 的
    toLocalNed/toNed), 而 simSetCameraPose 按"机体相对位姿"解释输入
    (SetActorRelativeLocation/Rotation)。直接把 get 的结果 set 回去,
    飞机一旦起飞/偏航/移动, 相机会被瞬移甩歪。必须先取机体当前世界
    位姿 (simGetGroundTruthKinematics), 把相机位姿换算到机体系。"""
    info = client.simGetCameraInfo(cam, 'PX4')
    kin = client.simGetGroundTruthKinematics('PX4')
    q_b, p_b = kin.orientation, kin.position   # KinematicsState 字段是扁平的
    # 机体明显倾斜时, 两次 RPC 之间的姿态变化会污染安装姿态基准
    # (z 轴偏离竖直超过 ~5.7° 时告警); 建议静止平放时再首次使用云台
    z_world = quat_rot_vec(q_b, airsim.Vector3r(0.0, 0.0, 1.0))
    if z_world.z_val < 0.995:
        log('云台: 机体姿态倾斜中, 安装基准可能不准, 建议平放静止时'
            '使用云台 (重连一次可重采基准)')
    p_c, q_c = info.pose.position, info.pose.orientation
    # 安装姿态 (机体系): q_mount = q_body⁻¹ ⊗ q_cam_world
    q_rel = quat_mul(quat_conj(q_b), q_c)
    # 安装位置 (机体系): p_mount = R_body⁻¹ · (p_cam - p_body)
    d = airsim.Vector3r(p_c.x_val - p_b.x_val,
                        p_c.y_val - p_b.y_val,
                        p_c.z_val - p_b.z_val)
    p_rel = quat_rot_vec(quat_conj(q_b), d)
    return p_rel, q_rel
