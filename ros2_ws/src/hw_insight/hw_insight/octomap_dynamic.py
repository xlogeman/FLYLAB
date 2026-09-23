#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from octomap_msgs.srv import BoundingBoxQuery


class OctomapDynamic(Node):
    def __init__(self) -> None:
        super().__init__('octomap_dynamic')

        # 范围参数
        self.declare_parameter('range_around_drone', 5.0)  # 无人机周围的范围大小（米）
        self.range_around_drone = self.get_parameter('range_around_drone').value
        print(f"range is {self.range_around_drone}")
        self.declare_parameter('map_range', 200.0)  # 地图大小（米）
        self.map_range = self.get_parameter('map_range').value
        
        # 创建BBOX服务客户端
        self.clear_bbox_client = self.create_client(BoundingBoxQuery, '/octomap_server/clear_bbox')
        # 等待服务可用
        while not self.clear_bbox_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /octomap_server/clear_bbox 服务...')

        # 订阅里程计信息
        self.odom_subscriber = self.create_subscription(Odometry, '/airsim_node/PX4/odometry', self.odometry_callback, 1)

        # 记录坐标数据
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.idx = 0;
        
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self) -> None:
        if self.idx == 0:
            bbox = [self.x+self.range_around_drone, self.map_range, -self.map_range, self.map_range, -self.map_range, self.map_range]
        elif self.idx == 1:
            bbox = [-self.map_range,self.x-self.range_around_drone, -self.map_range, self.map_range, -self.map_range, self.map_range]
        elif self.idx == 2:
            bbox = [-self.map_range, self.map_range, self.y+self.range_around_drone, self.map_range, -self.map_range, self.map_range]
        elif self.idx == 3:
            bbox = [-self.map_range, self.map_range, -self.map_range,self.y-self.range_around_drone, -self.map_range, self.map_range]
        elif self.idx == 4:
            bbox = [-self.map_range, self.map_range, -self.map_range, self.map_range, self.z+self.range_around_drone, self.map_range]
        elif self.idx == 5:
            bbox = [-self.map_range, self.map_range, -self.map_range, self.map_range, self.z-self.range_around_drone,-self.map_range]
        self.octomap_clear_bbox(bbox)

        self.idx = (self.idx + 1) % 6
        print("loop")


    # 里程计回调函数，根据无人机位置信息清理地图
    def odometry_callback(self, msg: Odometry):
        # 获取无人机位置
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

    def octomap_clear_bbox(self, bbox):
        req = BoundingBoxQuery.Request()
        req.min.x = bbox[0]  # 最小值x坐标
        req.max.x = bbox[1]  # 最大值x坐标
        req.min.y = bbox[2]  # 最小值y坐标
        req.max.y = bbox[3]  # 最大值y坐标
        req.min.z = bbox[4]  # 最小值z坐标
        req.max.z = bbox[5]  # 最大值z坐标

        # 调用服务
        future = self.clear_bbox_client.call_async(req)



def main(args=None) -> None:
    print('Start octomap dynamic config node...')
    rclpy.init(args=args)
    octomap_dynamic = OctomapDynamic()
    rclpy.spin(octomap_dynamic)
    octomap_dynamic.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
