# sim_fly —— 仿真飞行基础场景
# 内容: airsim_node(传感器数据通路) + move_velocity(手动速度控制) + 双 rviz
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # offboard 速度控制器 (订阅 /flylab/cmd_vel, 标准 Twist)
    move_velocity_node = Node(
        package='flylab',
        executable='move_velocity',
        name='move_velocity',
        output='screen')

    # rviz 配置
    pkg_share = get_package_share_directory('flylab')
    depth_rviz_path = os.path.join(pkg_share, 'rviz/fly/depth_cloud.rviz')
    image_lidar_rviz_path = os.path.join(pkg_share, 'rviz/fly/image_lidar.rviz')

    rviz_depth_node = Node(
        package='rviz2',
        executable='rviz2',
        name='depth_rviz2',
        arguments=['-d', depth_rviz_path]
    )

    rviz_image_lidar_node = Node(
        package='rviz2',
        executable='rviz2',
        name='image_lidar_rviz2',
        arguments=['-d', image_lidar_rviz_path]
    )

    # 用 airsim_ros_pkgs 原包
    airsim_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('airsim_ros_pkgs'), 'launch/airsim_node.launch.py')
        )
    )

    ld = LaunchDescription()
    ld.add_action(airsim_node_launch)
    ld.add_action(move_velocity_node)
    ld.add_action(rviz_depth_node)
    ld.add_action(rviz_image_lidar_node)
    return ld
