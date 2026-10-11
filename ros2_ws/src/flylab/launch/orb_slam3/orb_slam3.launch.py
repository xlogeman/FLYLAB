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
    pkg_share = get_package_share_directory('flylab')

    # RVIZ2显示配置
    orb_slam3_rviz_path = os.path.join(pkg_share, 'rviz/orb_slam3/orb_slam3.rviz')
    orb_slam3_rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', orb_slam3_rviz_path],
        output='screen'
    )

    # AirSim 直连传感器节点 (可插拔): 双目灰度相机 + FLU IMU, 供 ORB-SLAM3 使用
    airsim_camera = launch_ros.actions.Node(
        package='flylab',
        executable='airsim_camera',
        name='airsim_camera_stereo',
        output='screen',
        parameters=[{'camera_names': 'left_cam,right_cam', 'grayscale': True}]
    )
    airsim_imu = launch_ros.actions.Node(
        package='flylab',
        executable='airsim_imu',
        name='airsim_imu',
        output='screen',
        parameters=[{'period_ms': 5}]
    )

    # 启动 ORB SLAM3 双目+IMU
    orb_vocab_path = "/root/flylab/ros2_ws/src/orb_slam3/vocabulary/ORBvoc.txt"
    orb_config_path = os.path.join(pkg_share, 'configs/orb_slam3/stereo-inertial.yaml')
    orbslam3_node = launch_ros.actions.Node(
        package='orbslam3',
        executable='stereo-inertial',
        name='orbslam3',
        output='screen',
        arguments=[
            orb_vocab_path,
            orb_config_path,
            'false'  # 不显示pangolin可视化窗口；AirSim 理想平行双目无需预校正 (do_rectify)
        ],
        remappings=[
            ('/camera/left', '/airsim/cam/left_cam/image_raw'),
            ('/camera/right', '/airsim/cam/right_cam/image_raw'),
            ('/imu', '/airsim/imu/data')
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(airsim_camera)
    ld.add_action(airsim_imu)
    ld.add_action(orb_slam3_rviz_node)
    ld.add_action(orbslam3_node)
    return ld
