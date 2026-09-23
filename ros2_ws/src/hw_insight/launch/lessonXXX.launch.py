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
    lessonXXX_rviz_path = os.path.join(pkg_share, 'rviz/rgbd.rviz')
    lenssonXXX_rviz_node = launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='depth_rviz2',
            arguments=['-d', lessonXXX_rviz_path]
    )

    # 启动AirSim节点
    airsim_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('airsim_ros_pkgs'), 'launch/airsim_node.launch.py')
        )
    )

    # 启动RTAB MAP节点
    rtabmap_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('hw_insight'), 'launch/rtabmap.launch.py')
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


    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(airsim_node_launch)
    ld.add_action(rtabmap_node_launch)
    #ld.add_action(lensson6_rviz_node)
    #ld.add_action(multi_move_velocity_node1)


    return ld
