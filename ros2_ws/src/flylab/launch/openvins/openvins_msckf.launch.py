import os
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('flylab')

    # RVIZ2显示配置
    openvins_msckf_rviz_path = os.path.join(pkg_share, 'rviz', 'openvins', 'openvins_msckf.rviz')
    openvins_msckf_rviz_node = launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', openvins_msckf_rviz_path]
    )

    # OpenVINS MSCKF 估计器
    # 配置目录: flylab/configs/openvins/
    #   estimator_config.yaml 中 relative_config_imu / relative_config_imucam
    #   为同目录相对路径, 只需传 estimator_config.yaml 的绝对路径
    default_config_path = os.path.join(
            pkg_share, 'configs', 'openvins', 'estimator_config.yaml')
    config_path = LaunchConfiguration('config_path')
    declare_config_path = DeclareLaunchArgument(
            'config_path', default_value=default_config_path,
            description='OpenVINS estimator_config.yaml 路径')

    run_subscribe_msckf = Node(
            package='ov_msckf',
            executable='run_subscribe_msckf',
            name='run_subscribe_msckf',
            output='screen',
            parameters=[{'config_path': config_path}])

    # VIO 链路数据源 (可插拔): 传感器节点 + 门控桥接
    #   - airsim_camera: 双目灰度图 (grayscale=true, 单次 RPC 硬件同步)
    #   - airsim_imu: FRD 原生 IMU (/airsim/imu/data_raw, 与 kalibr 配置对齐)
    #   - bridge_vio: 断桨门控 (降落停发/解锁恢复), 输出 /vio/* 供 OpenVINS 订阅
    airsim_camera = Node(
            package='flylab',
            executable='airsim_camera',
            name='airsim_camera_stereo',
            output='screen',
            parameters=[{'camera_names': 'left_cam,right_cam', 'grayscale': True}])
    airsim_imu = Node(
            package='flylab',
            executable='airsim_imu',
            name='airsim_imu',
            output='screen',
            parameters=[{'period_ms': 5}])
    bridge_vio = Node(
            package='flylab',
            executable='bridge_vio',
            name='bridge_vio',
            output='screen')

    # Create the launch description and populate
    ld = LaunchDescription()

    # Create the launch description and populate
    ld.add_action(declare_config_path)
    ld.add_action(airsim_camera)
    ld.add_action(airsim_imu)
    ld.add_action(bridge_vio)
    ld.add_action(run_subscribe_msckf)
    ld.add_action(openvins_msckf_rviz_node)
    return ld
