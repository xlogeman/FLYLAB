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
    lesson11_rviz_path = os.path.join(pkg_share, 'rviz/lesson11.rviz')
    lesson11_rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', lesson11_rviz_path],
        output='screen'
    )

    # 启动AirSim节点（自定义话题桥接节点）
    airsim_ros_topic_node = launch_ros.actions.Node(
        package='airsim_ros_pkgs',
        executable='ytjy_airsim_ros_topic',
        name='ytjy_airsim_ros_topic',
        output='screen'
    )

    # 启动 ORB SLAM3 双目+IMU
    orb_vocab_path = "/root/fucker/ros2_ws/src/orb_slam3/vocabulary/ORBvoc.txt"
    orb_config_path = "/root/fucker/ros2_ws/src/hw_insight/orb_slam3/lesson11/stereo-inertial.yaml"
    orbslam3_node = launch_ros.actions.Node(
        package='orbslam3',
        executable='stereo-inertial',
        name='orbslam3',
        output='screen',
        arguments=[
            orb_vocab_path,
            orb_config_path,
            'true'  # 不显示pangolin可视化窗口
        ],
        remappings=[
            ('/camera/left', '/airsim/left_cam/image_raw'),
            ('/camera/right', '/airsim/right_cam/image_raw'),
            ('/imu', '/airsim/imu_ros2/data')
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(airsim_ros_topic_node)
    ld.add_action(lesson11_rviz_node)
    ld.add_action(orbslam3_node)
    return ld
