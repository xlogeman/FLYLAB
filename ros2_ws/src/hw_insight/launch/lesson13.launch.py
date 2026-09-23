import os
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    drone_id = LaunchConfiguration('drone_id', default=0)

    advanced_param_include = IncludeLaunchDescription(
        '/home/hw/sitl/hw-ros2/ros2/src/hw_insight/ego_planner/lesson13/speed.launch.py',
        launch_arguments={
            'drone_id': drone_id,           # 无人机编号，如果是单机通常为 0；多机集群时用来区分不同飞机
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('drone_id', default_value='0', description='Drone ID'))
    ld.add_action(advanced_param_include)
    return ld
