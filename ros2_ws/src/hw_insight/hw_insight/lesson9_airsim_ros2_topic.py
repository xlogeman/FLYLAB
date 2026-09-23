#!/usr/bin/env python3
"""
AirSim 全传感器高频数据发布节点 (为 VINS-Fusion 深度优化)
特性：
1. 多线程并发：IMU 200Hz 与 Camera 80+Hz 互不阻塞
2. 独立TCP通道：解决 Tornado IOLoop 多线程冲突问题
3. 严格时间戳：双目时间戳严格一致，供 VIO 使用
"""
import os
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image, Imu, NavSatFix, FluidPressure, MagneticField
from std_msgs.msg import Header
import airsim
import numpy as np
import cv2
import sys
from cv_bridge import CvBridge

class AirSimSensorPublisher(Node):
    def __init__(self, airsim_ip):
        super().__init__('lesson9_airsim_ros2_topic')
        
        self.get_logger().info('正在初始化 AirSim 全传感器并发节点...')
        
        # ---------------------------------------------------------
        # 【核心修改】：为 3 个不同的线程分配 3 个独立的 AirSim 客户端
        # 这样底层的 Tornado IOLoop 和 TCP Socket 就不会互相打架了
        # ---------------------------------------------------------
        self.get_logger().info('正在建立多通道连接...')
        try:
            self.client_img = airsim.MultirotorClient(ip=airsim_ip)
            self.client_img.confirmConnection()
            
            self.client_imu = airsim.MultirotorClient(ip=airsim_ip)
            self.client_imu.confirmConnection()
            
            self.client_low = airsim.MultirotorClient(ip=airsim_ip)
            self.client_low.confirmConnection()
            
            self.get_logger().info('3条独立数据通道均已成功连接到AirSim！')
        except Exception as e:
            self.get_logger().error(f'连接失败: {e}')
            raise RuntimeError("AirSim Connection Failed")

        self.bridge = CvBridge()
        self.vehicle_name = "PX4"

        # QoS 配置
        # 实测reliable比best effort的FPS要高
        #self.qos_fast = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        self.qos_fast = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        self.qos_slow = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=5)

        # 创建发布者
        self.pub_img_left = self.create_publisher(Image, '/airsim/left_cam/image_raw', self.qos_fast)
        self.pub_img_right = self.create_publisher(Image, '/airsim/right_cam/image_raw', self.qos_fast)
        self.pub_imu = self.create_publisher(Imu, '/airsim/imu/data', self.qos_fast)
        self.pub_gps = self.create_publisher(NavSatFix, '/airsim/gps/data', self.qos_slow)
        self.pub_baro = self.create_publisher(FluidPressure, '/airsim/baro/data', self.qos_slow)
        self.pub_mag = self.create_publisher(MagneticField, '/airsim/mag/data', self.qos_slow)

        # 创建多线程回调组
        self.cb_group_img = MutuallyExclusiveCallbackGroup()
        self.cb_group_imu = MutuallyExclusiveCallbackGroup()
        self.cb_group_low = MutuallyExclusiveCallbackGroup()

        # 创建定时器
        self.timer_img = self.create_timer(0.01, self.publish_images, callback_group=self.cb_group_img)
        self.timer_imu = self.create_timer(0.005, self.publish_imu, callback_group=self.cb_group_imu)
        self.timer_low = self.create_timer(0.02, self.publish_low_freq_sensors, callback_group=self.cb_group_low)

        self.get_logger().info('节点启动成功！多线程独立通道并发采集中...')

    # ================= 线程 1：双目图像发布 (使用 client_img) =================
    def publish_images(self):
        try:
            responses = self.client_img.simGetImages([
                airsim.ImageRequest("left_cam", airsim.ImageType.Scene, False, False),
                airsim.ImageRequest("right_cam", airsim.ImageType.Scene, False, False)
            ], self.vehicle_name)
            
            if responses and len(responses) == 2:
                common_stamp = self.get_clock().now().to_msg()
                
                # 左目
                if responses[0].image_data_uint8:
                    img_l = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8).reshape(responses[0].height, responses[0].width, 3)
                    msg_l = self.bridge.cv2_to_imgmsg(cv2.cvtColor(img_l, cv2.COLOR_BGR2GRAY), encoding="mono8")
                    msg_l.header.stamp = common_stamp
                    msg_l.header.frame_id = "left_cam_optical_frame"
                    self.pub_img_left.publish(msg_l)
                
                # 右目
                if responses[1].image_data_uint8:
                    img_r = np.frombuffer(responses[1].image_data_uint8, dtype=np.uint8).reshape(responses[1].height, responses[1].width, 3)
                    msg_r = self.bridge.cv2_to_imgmsg(cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY), encoding="mono8")
                    msg_r.header.stamp = common_stamp
                    msg_r.header.frame_id = "right_cam_optical_frame"
                    self.pub_img_right.publish(msg_r)
        except Exception as e:
            self.get_logger().debug(f'图像获取异常: {e}') # 降级为debug避免刷屏

    # ================= 线程 2：极高频 IMU 发布 (使用 client_imu) =================
    def publish_imu(self):
        try:
            imu_data = self.client_imu.getImuData("Imu", self.vehicle_name)
            
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "imu_link"
            
            msg.angular_velocity.x = imu_data.angular_velocity.x_val
            msg.angular_velocity.y = imu_data.angular_velocity.y_val
            msg.angular_velocity.z = imu_data.angular_velocity.z_val
            
            msg.linear_acceleration.x = imu_data.linear_acceleration.x_val
            msg.linear_acceleration.y = imu_data.linear_acceleration.y_val
            msg.linear_acceleration.z = imu_data.linear_acceleration.z_val
            
            msg.orientation.w = imu_data.orientation.w_val
            msg.orientation.x = imu_data.orientation.x_val
            msg.orientation.y = imu_data.orientation.y_val
            msg.orientation.z = imu_data.orientation.z_val
            
            self.pub_imu.publish(msg)
        except Exception:
            pass

    # ================= 线程 3：低频传感器发布 (使用 client_low) =================
    def publish_low_freq_sensors(self):
        try:
            now_stamp = self.get_clock().now().to_msg()
            
            # GPS
            gps_data = self.client_low.getGpsData("Gps", self.vehicle_name)
            msg_gps = NavSatFix()
            msg_gps.header.stamp = now_stamp
            msg_gps.header.frame_id = "gps_link"
            msg_gps.latitude = gps_data.gnss.geo_point.latitude
            msg_gps.longitude = gps_data.gnss.geo_point.longitude
            msg_gps.altitude = gps_data.gnss.geo_point.altitude
            self.pub_gps.publish(msg_gps)
            
            # Baro
            baro_data = self.client_low.getBarometerData("Barometer", self.vehicle_name)
            msg_baro = FluidPressure()
            msg_baro.header.stamp = now_stamp
            msg_baro.header.frame_id = "baro_link"
            msg_baro.fluid_pressure = baro_data.pressure
            self.pub_baro.publish(msg_baro)
            
            # Mag
            mag_data = self.client_low.getMagnetometerData("Magnetometer", self.vehicle_name)
            msg_mag = MagneticField()
            msg_mag.header.stamp = now_stamp
            msg_mag.header.frame_id = "mag_link"
            msg_mag.magnetic_field.x = mag_data.magnetic_field_body.x_val
            msg_mag.magnetic_field.y = mag_data.magnetic_field_body.y_val
            msg_mag.magnetic_field.z = mag_data.magnetic_field_body.z_val
            self.pub_mag.publish(msg_mag)

        except Exception as e:
            self.get_logger().debug(f'低频传感器获取异常: {e}')

def main(args=None):
    rclpy.init(args=args)
    airsim_ip = os.environ.get('PX4_SIM_HOST_ADDR', '172.28.144.1')
    try:
        node = AirSimSensorPublisher(airsim_ip)
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except RuntimeError as e:
        print(f"节点初始化失败: {e}")
    except KeyboardInterrupt:
        print('收到关闭信号...')
    finally:
        if 'node' in locals():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
