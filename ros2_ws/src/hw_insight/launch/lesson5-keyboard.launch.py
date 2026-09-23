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
    lesson5_rviz_path = os.path.join(pkg_share, 'rviz/lesson5.rviz')
    lensson5_rviz_node = launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='depth_rviz2',
            arguments=['-d', lesson5_rviz_path]
    )

    # 启动AirSim节点
    airsim_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('airsim_ros_pkgs'), 'launch/airsim_node.launch.py')
        )
    )

    # 多机启动
    multi_move_velocity_node1 = launch_ros.actions.Node(
            package='hw_insight',
            executable='multi_move_velocity',
            name='multi_move_velocity1',
            parameters=[
                {'px4_instance': '1'}
            ]
    )

    multi_move_velocity_node2 = launch_ros.actions.Node(
            package='hw_insight',
            executable='multi_move_velocity',
            name='multi_move_velocity2',
            parameters=[
                {'px4_instance': '2'}
            ]
    )

    multi_move_velocity_node3 = launch_ros.actions.Node(
            package='hw_insight',
            executable='multi_move_velocity',
            name='multi_move_velocity3',
            parameters=[
                {'px4_instance': '3'}
            ]
    )

    multi_move_velocity_node4 = launch_ros.actions.Node(
            package='hw_insight',
            executable='multi_move_velocity',
            name='multi_move_velocity4',
            parameters=[
                {'px4_instance': '4'}
            ]
    )

    multi_move_velocity_node5 = launch_ros.actions.Node(
            package='hw_insight',
            executable='multi_move_velocity',
            name='multi_move_velocity5',
            parameters=[
                {'px4_instance': '5'}
            ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(airsim_node_launch)
    ld.add_action(multi_move_velocity_node1)
    ld.add_action(multi_move_velocity_node2)
    ld.add_action(multi_move_velocity_node3)
    ld.add_action(multi_move_velocity_node4)
    ld.add_action(multi_move_velocity_node5)
    ld.add_action(lensson5_rviz_node)

    return ld
