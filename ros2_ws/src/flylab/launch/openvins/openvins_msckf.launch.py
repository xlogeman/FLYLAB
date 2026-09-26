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

    # 启动 VIO 链路 AirSim 传感器节点: 双目相机+IMU, 供 MSCKF 使用
    airsim_sensor_vio = Node(
            package='flylab',
            executable='airsim_sensor_vio',
            name='airsim_sensor_vio',
            output='screen')

    # Create the launch description and populate
    ld = LaunchDescription()

    # Create the launch description and populate
    ld.add_action(declare_config_path)
    ld.add_action(airsim_sensor_vio)
    ld.add_action(run_subscribe_msckf)
    ld.add_action(openvins_msckf_rviz_node)
    return ld
