#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import math
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from sensor_msgs.msg import Range, Image
from hw_interface.msg import HWSimpleKeyboardInfo
from quadrotor_msgs.msg import PositionCommand
from geometry_msgs.msg import PoseStamped
import struct

class Px4EgoTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__('px4_ego_tracker_node')

        # 和PX4通讯的QOS配置文件
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ---------------- 参数 ----------------
        self.declare_parameter('goal_z_height', 2.0)

        # ---------------- 发布者 ----------------
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.goal_3d_publisher = self.create_publisher(PoseStamped, '/goal_pose_3d', 10)

        # ---------------- 订阅者 ----------------
        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.sub_vehicle_pos = self.create_subscription(VehicleLocalPosition,'/fmu/out/vehicle_local_position', self.vehicle_position_callback, qos_profile)
        self.hw_position_subscriber = self.create_subscription(HWSimpleKeyboardInfo, '/hw_insight/keyboard_velocity', self.hw_velocity_callback, 1)
        self.rviz_goal_subscriber = self.create_subscription(PoseStamped, '/goal_pose', self.rviz_goal_callback, 10)
        
        # 订阅 EGO-Planner 的轨迹输出
        self.ego_pos_cmd_subscriber = self.create_subscription(PositionCommand, '/drone_0_planning/pos_cmd', self.ego_pos_cmd_callback, 10)

        # ---------------- 状态变量 ----------------
        self.vehicle_status = VehicleStatus()               # 无人机状态信息
        self.hw_keyboard_control = HWSimpleKeyboardInfo()   # 键盘控制信息
        self.vehicle_position = VehicleLocalPosition()      # 无人机当前位置

        # EGO-Planner 相关变量
        self.has_ego_cmd = False                            # 是否收到过EGO的指令
        self.ego_setpoint = TrajectorySetpoint()            # 转换后的EGO轨迹点

        # 定时器处理函数 (50Hz，仅用于维护 PX4 Offboard 心跳和状态机逻辑)
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.get_logger().info("PX4 + EGO 混合控制节点已启动！(开启 100Hz 零延迟转发与全状态前馈)")

# =============================================================================================================================
# ================= 各种订阅的回调函数 ========================================================================================
# =============================================================================================================================
    def vehicle_status_callback(self, vehicle_status):
        self.vehicle_status = vehicle_status

    def hw_velocity_callback(self, hw_keyboard_control):
        self.hw_keyboard_control = hw_keyboard_control

    def vehicle_position_callback(self, vehicle_position):
        self.vehicle_position = vehicle_position

    # 接收 EGO-Planner 传来的位置指令，转换成PX4的NED坐标系，然后同步发送给PX4
    def ego_pos_cmd_callback(self, msg):
        self.has_ego_cmd = True
        # 坐标系转换：ROS(ENU) -> PX4(NED)
        # 1. 位置转换
        self.ego_setpoint.position = [msg.position.y, msg.position.x, -msg.position.z]
        # 2. 速度转换
        self.ego_setpoint.velocity = [msg.velocity.y, msg.velocity.x, -msg.velocity.z]
        # 3. 加速度转换
        self.ego_setpoint.acceleration = [msg.acceleration.y, msg.acceleration.x, -msg.acceleration.z]
        # 4. 偏航角 (Yaw) 转换
        self.ego_setpoint.yaw = math.pi / 2.0 - msg.yaw
        # 5. 偏航角速度 (Yaw Rate) 转换
        self.ego_setpoint.yawspeed = -msg.yaw_dot

        # 事件驱动：只要处于 EGO 避障模式下，收到一帧就发一帧，实现PX4和EGO同步
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.hw_keyboard_control.track == 1:
            self.ego_setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.trajectory_setpoint_publisher.publish(self.ego_setpoint)


# =============================================================================================================================
# ================= 目标高度更换 ==============================================================================================
# =============================================================================================================================
    def rviz_goal_callback(self, msg):
        target_z = self.get_parameter('goal_z_height').value
        msg.pose.position.z = float(target_z)
        self.get_logger().info(f"拦截到RViz目标点，已将其高度修改为 Z = {target_z} 米，并转发至 EGO-Planner")
        self.goal_3d_publisher.publish(msg)


# =============================================================================================================================
# ================= 无人机状态切换的命令 ======================================================================================
# =============================================================================================================================
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)


# =============================================================================================================================
# ================= 控制命令发布 ==============================================================================================
# =============================================================================================================================
    # 发布 offboard 心跳，解锁 PX4 的速度和加速度前馈控制权
    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        
        if self.hw_keyboard_control.track == 1:
            # EGO 自动避障模式
            msg.position = True
            msg.velocity = True
            msg.acceleration = True
        else:
            # 键盘手动模式
            msg.position = False
            msg.velocity = True
            msg.acceleration = False
            
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    # 手动模式：发布速度指令
    def publish_velocity_setpoint_speed(self, x: float, y: float, z: float, yawspeed: float):
        msg = TrajectorySetpoint()
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [x, y, z]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float('nan')
        msg.yawspeed = yawspeed
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    # 悬停保护：发布当前位置为期望位置
    def publish_hover_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [0.0, 0.0, 0.0]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = self.vehicle_position.heading
        msg.yawspeed = float('nan')
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)


# =============================================================================================================================
# ================= 定时处理函数，主循环 ======================================================================================
# =============================================================================================================================
    def timer_callback(self):
        # 1. 始终高频发布 Offboard 心跳
        self.publish_offboard_control_heartbeat_signal()

        # 2. 状态机逻辑
        if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
            # 未解锁状态，收到上升命令，则解锁并起飞，切 OFFBOARD
            if self.hw_keyboard_control.z < -15:
                self.engage_offboard_mode()
                self.arm()
                self.publish_velocity_setpoint_speed(self.hw_keyboard_control.x, self.hw_keyboard_control.y, self.hw_keyboard_control.z, self.hw_keyboard_control.yaw)
                
        elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            # 已经解锁，且在 OFFBOARD 状态
            
            if self.hw_keyboard_control.track == 1:
                # 【EGO 自动避障模式】
                if not self.has_ego_cmd:
                    # 安全保护：开启了自动模式，但 EGO 还没算出轨迹时，原地悬停
                    self.publish_hover_setpoint()
                # 提示：正常运行时的轨迹下发已移交给 ego_pos_cmd_callback 实现 100Hz 零延时驱动
                    
            elif self.hw_keyboard_control.z > 35 :
                # 触发降落指令
                self.land()
                
            else:
                # 【键盘手动控制飞行模式】
                forward_x = self.hw_keyboard_control.x * math.cos(self.vehicle_position.heading)
                forward_y = self.hw_keyboard_control.x * math.sin(self.vehicle_position.heading)
                left_x = -self.hw_keyboard_control.y * math.sin(self.vehicle_position.heading)
                left_y = self.hw_keyboard_control.y * math.cos(self.vehicle_position.heading)
                
                self.publish_velocity_setpoint_speed(forward_x + left_x, forward_y + left_y, self.hw_keyboard_control.z, self.hw_keyboard_control.yaw)
                
        elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            # 降落状态下打断降落
            if self.hw_keyboard_control.z < -5:
                self.engage_offboard_mode()
                self.publish_velocity_setpoint_speed(self.hw_keyboard_control.x, self.hw_keyboard_control.y, self.hw_keyboard_control.z, self.hw_keyboard_control.yaw)
        else:
            # 其他未知状态，触发降落保护
            self.land()


def main(args=None) -> None:
    print('Starting PX4 EGO Tracker Node...')
    rclpy.init(args=args)
    node = Px4EgoTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()