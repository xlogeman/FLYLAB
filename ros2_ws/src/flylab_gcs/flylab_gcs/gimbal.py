#!/usr/bin/env python3
"""云台支撑逻辑 —— 角度限位、安装基准采集。

云台运行时状态 (当前角/动画/摇杆角速度/下发指纹等) 仍在 Grabber 里
(与抓取循环共用一把锁、一帧一拍推进); 本模块只放可独立描述的部分:
限位夹紧、相机安装位姿采集。无持久化: 每次连接 AirSim 从头开始
(云台回中、安装基准重采、FOV 查询真实值)。
"""
import math

try:
    import cosysairsim as airsim
except ImportError:
    import sys
    sys.path.insert(0, '/root/flylab/Cosys-AirSim/PythonClient')
    import cosysairsim as airsim

from . import config
from .quat_math import quat_conj, quat_mul, quat_rot_vec

# 云台稳定重下发的姿态容差 (度): 姿态相对转角在阈值内视为"没变"跳过
# 整批下发, 滤掉大半悬停抖动 (0.25° 偏移肉眼不可见)
APPLY_TOL_DEG = 0.25

# 安装基准采集就绪阈值:
#   机体 z 轴与竖直夹角余弦 (0.998 ≈ 3.6°) —— PX4 SITL 启动弹跳/自旋期
#   两次 RPC 之间机体在动, 姿态基准会被污染, 必须等机体稳定
#   安装位置偏移上限 (米) —— 机体原点到相机通常 <0.5m; 运动学未就绪时
#   kinematics 返回零位姿, 偏移=相机世界坐标 (几十米), 一眼识别
MOUNT_LEVEL_COS = 0.998
MOUNT_OFFSET_MAX_M = 1.0


class MountNotReady(Exception):
    """相机安装基准暂时不可采 (机体未稳/运动学未就绪), 调用方稍后重试。"""


def clamp_gimbal(yaw, pitch):
    """按 config 限位夹紧云台角度 (config.GIMBAL_*_LIMIT_DEG)。"""
    yaw = max(-config.GIMBAL_YAW_LIMIT_DEG,
              min(config.GIMBAL_YAW_LIMIT_DEG, yaw))
    pitch = max(config.GIMBAL_PITCH_MIN_DEG,
                min(config.GIMBAL_PITCH_MAX_DEG, pitch))
    return yaw, pitch


def capture_mount(client, cam):
    """采集相机安装位姿 (机体系)。基准未就绪时抛 MountNotReady,
    调用方跳过本拍、下一拍重采 (污染的基准会让相机错位拍到机体)。

    注意: simGetCameraInfo 返回世界系位姿, 而 simSetCameraPose 按
    "机体相对位姿"解释输入 —— 直接把 get 的结果 set 回去, 飞机一移动
    相机会被瞬移甩歪。必须先取机体世界位姿, 换算到机体系。"""
    info = client.simGetCameraInfo(cam, 'PX4')
    kin = client.simGetGroundTruthKinematics('PX4')
    q_b, p_b = kin.orientation, kin.position   # KinematicsState 字段是扁平的
    # 机体倾斜/弹跳中: 两次 RPC 之间的姿态变化会污染安装姿态基准, 不采
    z_world = quat_rot_vec(q_b, airsim.Vector3r(0.0, 0.0, 1.0))
    if z_world.z_val < MOUNT_LEVEL_COS:
        raise MountNotReady('机体姿态未稳 (倾斜/弹跳中)')
    p_c, q_c = info.pose.position, info.pose.orientation
    # 安装位置 (机体系): p_mount = R_body⁻¹ · (p_cam - p_body)
    d = airsim.Vector3r(p_c.x_val - p_b.x_val,
                        p_c.y_val - p_b.y_val,
                        p_c.z_val - p_b.z_val)
    p_rel = quat_rot_vec(quat_conj(q_b), d)
    # 偏移量级校验: kinematics 未就绪 (零位姿) 时 p_rel = 相机世界坐标,
    # 几十米量级; 之后 simSetCameraPose 会把相机放到离机身高几十米处,
    # 云台一动就拍到飞机自身
    offset = math.sqrt(p_rel.x_val ** 2 + p_rel.y_val ** 2 + p_rel.z_val ** 2)
    if offset > MOUNT_OFFSET_MAX_M:
        raise MountNotReady(f'安装偏移异常 ({offset:.1f}m, 运动学基准未就绪?)')
    # 安装姿态 (机体系): q_mount = q_body⁻¹ ⊗ q_cam_world
    q_rel = quat_mul(quat_conj(q_b), q_c)
    return p_rel, q_rel
