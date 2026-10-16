# test_lidar —— LiDAR 单测 (仅一条点云 RPC 通道)
#
# 用法 (参数必须与 UE 端 settings.json 的 Lidar1 一致):
#   ros2 launch flylab test_lidar.launch.py                       # GPU 雷达 + RViz
#   ros2 launch flylab test_lidar.launch.py gpu_lidar:=false      # CPU 雷达 (SensorType 6)
#   ros2 launch flylab test_lidar.launch.py lidar_sensor_name:=Lidar2      # 雷达名不同时
#   ros2 launch flylab test_lidar.launch.py odom:=true            # 叠加真值里程计, 点云随无人机移动
#
# TF 说明 (测试专用, 不代表真实安装位姿):
#   - odom:=false (默认): 静态 TF world -> lidar_ned 零位, 点云固定在原点
#   - odom:=true: 启动 airsim_odom 提供 world -> base_link TF, 静态 TF 改为
#     base_link -> lidar_ned, 点云随无人机一起移动, 便于对照真值检查点云位置
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('flylab')

    declare_rviz = DeclareLaunchArgument('rviz', default_value='true',
            description='是否启动 RViz')
    declare_rviz_cfg = DeclareLaunchArgument('rviz_cfg', default_value=os.path.join(
            pkg_share, 'rviz', 'test', 'test_lidar.rviz'))
    rviz_node = Node(package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_cfg')],
            output='screen', condition=IfCondition(LaunchConfiguration('rviz')))

    declare_odom = DeclareLaunchArgument(
            'odom', default_value='false',
            description='是否叠加真值里程计 (点云随无人机移动)')
    declare_gpu_lidar = DeclareLaunchArgument(
            'gpu_lidar', default_value='true',
            description='UE 端是否使用 GPULidar (SensorType 8)')
    declare_num_channels = DeclareLaunchArgument(
            'num_channels', default_value='64',
            description='UE 端 Lidar1 NumberOfChannels')
    declare_rotation_hz = DeclareLaunchArgument(
            'rotation_frequency', default_value='10.0',
            description='UE 端 Lidar1 RotationsPerSecond')
    declare_lidar_name = DeclareLaunchArgument(
            'lidar_sensor_name', default_value='Lidar1',
            description='UE 端雷达传感器名 (须与 settings.json 一致)')

    lidar_node = Node(
            package='flylab', executable='airsim_lidar', name='airsim_lidar',
            output='screen',
            parameters=[{'lidar_sensor_name': LaunchConfiguration('lidar_sensor_name'),
                         'use_gpu_lidar': LaunchConfiguration('gpu_lidar'),
                         'num_channels': LaunchConfiguration('num_channels'),
                         'rotation_frequency': LaunchConfiguration('rotation_frequency')}])

    # 可选: 真值里程计 (TF world -> base_link)
    odom_node = Node(
            package='flylab', executable='airsim_odom', name='airsim_odom',
            output='screen', condition=IfCondition(LaunchConfiguration('odom')),
            parameters=[{'odom_period_ms': 10, 'enable_tf': True}])

    # 测试专用静态 TF, 两种模式互斥:
    #   odom 开: 点云挂到机体系 (随无人机移动); odom 关: 直接挂 world (固定原点)
    lidar_tf_on_body = Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='lidar_test_tf_body',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'lidar_ned'],
            condition=IfCondition(LaunchConfiguration('odom')))
    lidar_tf_on_world = Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='lidar_test_tf_world',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'world', '--child-frame-id', 'lidar_ned'],
            condition=UnlessCondition(LaunchConfiguration('odom')))

    ld = LaunchDescription()
    ld.add_action(declare_rviz)
    ld.add_action(declare_rviz_cfg)
    ld.add_action(declare_odom)
    ld.add_action(declare_gpu_lidar)
    ld.add_action(declare_num_channels)
    ld.add_action(declare_rotation_hz)
    ld.add_action(declare_lidar_name)
    ld.add_action(lidar_node)
    ld.add_action(odom_node)
    ld.add_action(lidar_tf_on_body)
    ld.add_action(lidar_tf_on_world)
    ld.add_action(rviz_node)
    return ld
