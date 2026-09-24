import os
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory 
from launch.actions import ExecuteProcess

def generate_launch_description():
    pkg_share = get_package_share_directory('hw_insight')

    # RVIZ2显示配置
    lesson10_rviz_path = '/root/fucker/ros2_ws/src/hw_insight/rviz/lesson10.rviz'
    lesson10_rviz_node = launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', lesson10_rviz_path]
    )

    # 启动 ytjy_airsim_fast 快速发布各种ROS2话题
    ytjy_airsim_fast = Node(
            package='airsim_ros_pkgs',
            executable='ytjy_airsim_fast',
            name='ytjy_airsim_fast',
            output='screen')
  
    # Create the launch description and populate
    ld = LaunchDescription()

    # Create the launch description and populate
    #ld.add_action(ytjy_airsim_fast)
    ld.add_action(lesson10_rviz_node)
    return ld

