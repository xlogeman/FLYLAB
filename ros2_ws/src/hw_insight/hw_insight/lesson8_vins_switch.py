#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from hw_interface.msg import HWSimpleKeyboardInfo
from sensor_msgs.msg import Range


class VINSSwitch(Node):
    def __init__(self) -> None:
        super().__init__('lesson8_vins_switch')

        # 用来发布 VINS FUSION 控制话题
        self.cam_pub = self.create_publisher(Bool, '/vins_cam_switch', 10)   # 摄像头开关
        self.imu_pub = self.create_publisher(Bool, '/vins_imu_switch', 10)   # IMU开关
        self.restart_pub = self.create_publisher(Bool, '/vins_restart', 10)  # 重置动作

        # 和PX4通讯的QOS配置文件
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 订阅无人机状态信息
        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # 初始化变量
        self.vehicle_status = VehicleStatus()
        self.vehicle_status.arming_state = VehicleStatus.ARMING_STATE_ARMED


    # 辅助发话题函数
    def publish_bool(self, publisher, value):
        msg = Bool()
        msg.data = value
        publisher.publish(msg)


    # 获取无人机状态
    def vehicle_status_callback(self, vehicle_status):
        if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED and vehicle_status.arming_state==VehicleStatus.ARMING_STATE_ARMED:
            # 检测到起飞，重启VINS FUSION
            self.publish_bool(self.imu_pub, True)
            self.publish_bool(self.cam_pub, True)
            self.publish_bool(self.restart_pub, True)
            self.get_logger().info("[YTJY]: Vehicle armed, open and reset VINS Fusion")
        elif self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED and vehicle_status.arming_state==VehicleStatus.ARMING_STATE_DISARMED:
            # 检测到降落，关闭VINS FUSION
            self.publish_bool(self.imu_pub, False)
            self.publish_bool(self.cam_pub, False)
            self.publish_bool(self.restart_pub, True)
            self.get_logger().info("[YTJY]: Vehicle disarmed, close VINS Fusion")
        self.vehicle_status = vehicle_status


def main(args=None) -> None:
    print('Start VINS FUSION switch logic...')
    rclpy.init(args=args)
    lesson8_vins_switch = VINSSwitch()
    rclpy.spin(lesson8_vins_switch)
    lesson8_vins_switch.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
