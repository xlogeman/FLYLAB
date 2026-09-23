#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from hw_interface.msg import HWSimpleKeyboardInfo
from sensor_msgs.msg import Range


class MoveVelocity(Node):
    def __init__(self) -> None:
        super().__init__('move_velocity')

        # 和PX4通讯的QOS配置文件
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 切换模式用，包括起飞，着陆，返回，offboard等
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        # 发布offboard控制模式，设置为速度控制模式
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        # 发布速度值
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        # 订阅无人机状态信息
        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        # 订阅向下的距离传感器，用来降落
        self.distance_sensor_subscriber = self.create_subscription(Range, '/airsim_node/PX4/distance/DistanceDown', self.distance_sensor_callback, 1)
        # 订阅键盘控制信息
        self.hw_position_subscriber = self.create_subscription(HWSimpleKeyboardInfo, '/hw_insight/keyboard_velocity', self.hw_velocity_callback, 1)

        # 初始化变量
        self.vehicle_status = VehicleStatus()
        self.distance_sensor = Range()
        self.hw_velocity = HWSimpleKeyboardInfo()

        # 定时器处理函数
        self.timer = self.create_timer(0.1, self.timer_callback)


    # 获取无人机状态
    def vehicle_status_callback(self, vehicle_status):
        self.vehicle_status = vehicle_status

    # 获取向下的距离传感器信息
    def distance_sensor_callback(self, distance_sensor):
        self.distance_sensor = distance_sensor

    # 获取键盘控制信息
    def hw_velocity_callback(self, hw_velocity):
        self.hw_velocity = hw_velocity


    # 解锁命令
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    # 加锁命令
    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    # 切换到offboard模式
    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    # 降落命令
    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    # 返回命令
    def return_home(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)

    # 设置为offboard速度控制模式
    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    # 设置各个方向速度
    def publish_velocity_setpoint(self, x: float, y: float, z: float, yawspeed: float):
        msg = TrajectorySetpoint()
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [x, y, z]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float('nan')
        msg.yawspeed = yawspeed
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    # 发布命令
    def publish_vehicle_command(self, command, **params) -> None:
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


    # 主逻辑，根据键盘信息控制无人机
    def timer_callback(self) -> None:
        # offboard心跳
        self.publish_offboard_control_heartbeat_signal()
        #print(f"nav_state = {self.vehicle_status.nav_state}, arm = {self.vehicle_status.arming_state}, velocity={[self.hw_velocity.x, self.hw_velocity.y, self.hw_velocity.z, self.hw_velocity.yaw]}")

        if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
            # 未解锁状态，收到上升命令，则解锁并起飞，切换到OFFBOARD状态
            if self.hw_velocity.z < -5:
                self.engage_offboard_mode()
                self.arm()
                self.publish_velocity_setpoint(self.hw_velocity.x, self.hw_velocity.y, self.hw_velocity.z, self.hw_velocity.yaw)
        elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            # 已经解锁，且在OFFBOARD状态，则根据订阅/hw_insight/keyboard_velocity得到的值控制无人机
            if self.hw_velocity.z > 5 and self.distance_sensor.range < 5 :
                # 降低到一定高度，出发降落指令
                self.land()
            else:
                # 按照命令飞行
                self.publish_velocity_setpoint(self.hw_velocity.x, self.hw_velocity.y, self.hw_velocity.z, self.hw_velocity.yaw)
        elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            # 降落状态下，如果收到上升指令，则结束降落状态，切换到OFFBOARD，并重新飞行
            if self.hw_velocity.z < -5:
                self.engage_offboard_mode()
                self.publish_velocity_setpoint(self.hw_velocity.x, self.hw_velocity.y, self.hw_velocity.z, self.hw_velocity.yaw)
        else:
            # 其他状态 (QGC 手动控制的 POSCTL / AUTO_TAKEOFF / AUTO_LOITER 等) 属于
            # 地面站合法操作, 不干预, 否则会以 10Hz 持续发 NAV_LAND 与 QGC 抢控制权
            pass

def main(args=None) -> None:
    print('Starting move velocity node...')
    rclpy.init(args=args)
    move_velocity = MoveVelocity()
    rclpy.spin(move_velocity)
    move_velocity.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
