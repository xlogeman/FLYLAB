#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation as R

class PureImuIntegrator(Node):
    """
    基于 IMU 数据进行二次积分生成轨迹的节点。
    要求输入数据符合 ROS 标准：参考系为 ENU (东-北-天)，机体系为 FLU (前-左-上)。
    """
    def __init__(self):
        super().__init__('pure_imu_integrator')
        
        # 1. 订阅 C++ 转换后发布的 ROS2 标准 IMU 话题
        self.sub = self.create_subscription(
            Imu, 
            #'/airsim/imu_ros2/data', 
            '/airsim_node/PX4/imu/Imu_flu',
            self.imu_callback, 
            100
        )
        
        # 2. 发布 Path 消息，用于在 RViz2 中显示轨迹
        self.pub = self.create_publisher(Path, '/pure_imu_path', 10)
        
        # 3. 状态变量初始化
        self.pos = np.zeros(3)      # 当前世界坐标系下的位置 [x, y, z] (单位: m)
        self.vel = np.zeros(3)      # 当前世界坐标系下的速度 [vx, vy, vz] (单位: m/s)
        self.last_time = None       # 用于计算 dt 的上一帧时间戳
        
        # 初始化轨迹 Path 消息
        self.path_msg = Path()
        self.path_msg.header.frame_id = "world" # 设置为 RViz 中的 Fixed Frame
        
        # 4. 物理常量适配 (针对 ENU 坐标系)
        # 在 ENU 系统中，Z轴向上。重力加速度作为矢量，方向向下，因此为负值。
        self.g_norm = 9.80665
        self.g_vector = np.array([0.0, 0.0, -self.g_norm]) # 指向 Z 轴负方向
        
        self.get_logger().info("Pure IMU Integrator (ENU/FLU) 已启动，等待数据...")
        self.count = 0

    def imu_callback(self, msg):
        # --- A. 时间处理 ---
        # 获取当前 IMU 消息的时间戳（单位：秒）
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # 第一帧数据处理：仅记录时间，不进行积分计算
        if self.last_time is None:
            self.last_time = current_time
            return
            
        dt = current_time - self.last_time
        if dt <= 0:
            return # 忽略时间戳回退或重复的数据
            
        # --- B. 坐标变换 (Body -> World) ---
        # 1. 获取四元数姿态：描述从 ENU 世界系到 FLU 机体系的旋转
        # Scipy 使用的顺序是 [x, y, z, w]
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        rot_matrix = R.from_quat(quat)
        
        # 2. 获取机体系(FLU)下的原始加速度读数
        a_body = np.array([
            msg.linear_acceleration.x, 
            msg.linear_acceleration.y, 
            msg.linear_acceleration.z
        ])
        
        # 3. 将机体加速度旋转投影到 ENU 世界坐标系中
        # a_world_raw 此时包含了运动加速度和重力支持力
        a_world_raw = rot_matrix.apply(a_body)
        
        # --- C. 剔除重力影响 ---
        # a_real = 测得的加速度 + 重力矢量
        # 在静止状态下：[0, 0, 9.8] + [0, 0, -9.8] = [0, 0, 0]
        a_real = a_world_raw + self.g_vector
        
        # --- D. 数值积分 (计算位置与速度) ---
        # 使用牛顿运动学公式进行一阶积分
        # 1. 更新位置：P_new = P_old + V*dt + 0.5*a*dt^2
        self.pos = self.pos + self.vel * dt + 0.5 * a_real * (dt ** 2)
        
        # 2. 更新速度：V_new = V_old + a*dt
        self.vel = self.vel + a_real * dt
        
        self.last_time = current_time
        
        # --- E. 轨迹消息封装与发布 ---
        # 构造轨迹点
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = "world"
        
        # 填充积分算出的位置
        pose.pose.position.x = float(self.pos[0])
        pose.pose.position.y = float(self.pos[1])
        pose.pose.position.z = float(self.pos[2])
        
        # 直接透传当前姿态，让路径点也带有朝向信息
        pose.pose.orientation = msg.orientation
        
        # 加入轨迹列表
        self.path_msg.poses.append(pose)
        
        # 限制轨迹长度，防止内存占用过高及 RViz 卡顿 (保留最新 5000 个点)
        if len(self.path_msg.poses) > 5000:
            self.path_msg.poses.pop(0)
            
        # 发布轨迹话题
        self.path_msg.header.stamp = msg.header.stamp
        self.pub.publish(self.path_msg)
        
        # --- F. 日志打印 (每隔100帧，约1秒) ---
        self.count += 1
        if self.count % 100 == 0:
            self.get_logger().info(
                f"Pos (ENU): X={self.pos[0]:.2f}, Y={self.pos[1]:.2f}, Z={self.pos[2]:.2f} | "
                f"Vel: {np.linalg.norm(self.vel):.2f} m/s"
            )

def main(args=None):
    rclpy.init(args=args)
    node = PureImuIntegrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
