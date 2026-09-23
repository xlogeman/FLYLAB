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
    hw_move_velocity_node = Node(
            package='hw_insight',
            executable='move_velocity',
            name='move_velocity',
            output='screen')

    pkg_share = get_package_share_directory('hw_insight')
    depth_rviz_path = os.path.join(pkg_share, 'rviz/depth_cloud.rviz')
    image_lidar_rviz_path = os.path.join(pkg_share, 'rviz/image_lidar.rviz')

    hw_rviz_depth_node = launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='depth_rviz2',
            arguments=['-d', depth_rviz_path]
    )

    hw_rviz_image_lidar_node = launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='image_lidar_rviz2',
            arguments=['-d', image_lidar_rviz_path]
    )

    airsim_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('airsim_ros_pkgs'), 'launch/airsim_node.launch.py')
        )
    )
    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(airsim_node_launch)
    ld.add_action(hw_move_velocity_node)
    ld.add_action(hw_rviz_depth_node)
    ld.add_action(hw_rviz_image_lidar_node)
    return ld
