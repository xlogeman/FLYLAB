#!/usr/bin/env python3
"""四元数数学工具 (AirSim NED 约定) —— 纯函数, 无状态。

供云台姿态换算使用: 输入/输出均为 cosysairsim 的 Vector3r/Quaternionr
(只按属性访问, 不依赖其方法)。
"""
import math

try:
    import cosysairsim as airsim
except ImportError:
    import sys
    sys.path.insert(0, '/root/flylab/Cosys-AirSim/PythonClient')
    import cosysairsim as airsim


def quat_yaw_el(q):
    """四元数前向轴 (机体系 +x) 的 (方位角, 仰角), 单位度。
    方位角 = 前向轴水平投影的北偏东角 (NED, 顺时针/向右为正);
    仰角 = 相对地平线, 上正下负。前向轴接近竖直时方位角退化 (取 0),
    此时方位角只影响画面 roll, 不影响指向。"""
    f = quat_rot_vec(q, airsim.Vector3r(1.0, 0.0, 0.0))
    h = math.hypot(f.x_val, f.y_val)
    yaw = math.degrees(math.atan2(f.y_val, f.x_val)) if h > 1e-6 else 0.0
    el = math.degrees(math.atan2(-f.z_val, h))
    return yaw, el


def gimbal_yaw_quat(azimuth_deg):
    """方位角 (度, 北偏东/向右为正) -> NED 四元数, 绕世界竖直轴 (+z)。
    与 Rz⊗Ry 组合成目标姿态后 roll 恒为 0 (地平线水平)。回中链路上
    传 0 即单位四元数。"""
    y = math.radians(azimuth_deg) / 2
    return airsim.Quaternionr(0.0, 0.0, math.sin(y), math.cos(y))


def gimbal_pitch_quat(elevation_deg):
    """仰角 (度, 上正下负) -> NED 四元数, 绕世界 +y 轴。
    R_y(el) 把前向轴抬到仰角 el (el=-90 即垂直向下)。与 gimbal_yaw_quat
    组合成世界系目标姿态, 与机体姿态完全解耦。"""
    p = math.radians(elevation_deg) / 2
    return airsim.Quaternionr(0.0, math.sin(p), 0.0, math.cos(p))


def quat_mul(a, b):
    """四元数 Hamilton 积 a⊗b: 相对机体的组合旋转, 先施 b 再施 a。"""
    ax, ay, az, aw = a.x_val, a.y_val, a.z_val, a.w_val
    bx, by, bz, bw = b.x_val, b.y_val, b.z_val, b.w_val
    return airsim.Quaternionr(
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz)


def quat_conj(q):
    """四元数共轭 (单位四元数下即逆旋转)。"""
    return airsim.Quaternionr(-q.x_val, -q.y_val, -q.z_val, q.w_val)


def quat_rot_vec(q, v):
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
