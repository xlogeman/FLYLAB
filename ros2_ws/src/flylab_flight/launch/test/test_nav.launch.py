# test_nav —— 导航传感器单测 (GPS/气压计/磁力计, 一条 RPC 通道)
#
# 用法:
#   ros2 launch flylab_flight test_nav.launch.py
#
# 说明:
#   - 这三路话题 RViz 不便直观显示, 本 launch 不启动 RViz, 用命令行检查:
#       ros2 topic hz /airsim/gps    # ~50Hz
#       ros2 topic echo /airsim/gps --once     # 经纬度/海拔
#       ros2 topic echo /airsim/baro --once    # 气压 Pa (~101325 海平面)
#       ros2 topic echo /airsim/mag --once     # FLU 机体系磁场
#   - 数据全零: settings.json 里没配 Gps1/Barometer1/Magnetometer1
import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    nav_node = Node(
            package='flylab_flight', executable='airsim_nav', name='airsim_nav',
            output='screen',
            parameters=[{'period_ms': 20}])  # 50Hz

    ld = LaunchDescription()
    ld.add_action(nav_node)
    return ld
