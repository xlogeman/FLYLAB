import os
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('flylab')

    # RViz2 显示配置: flylab 适配版 fast_lio.rviz
    # (话题 /cloud_registered /cloud_registered_body /Odometry /path, TF camera_init -> body)
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    declare_rviz = DeclareLaunchArgument(
            'rviz', default_value='true',
            description='是否启动 RViz 监视建图结果')
    declare_rviz_cfg = DeclareLaunchArgument(
            'rviz_cfg', default_value=os.path.join(
                pkg_share, 'rviz', 'fast_lio', 'fast_lio.rviz'))
    fast_lio_rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            condition=IfCondition(rviz_use))

    # FAST-LIO 估计器
    # 配置目录: flylab/configs/fast_lio/
    #   fast_lio.yaml: lid_topic=/airsim/lidar/points, imu_topic=/airsim/imu/data
    #   lidar_type=2 (velodyne), timestamp_unit=0 (秒), 零位外参
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    declare_config_path = DeclareLaunchArgument(
            'config_path', default_value=os.path.join(pkg_share, 'configs', 'fast_lio'),
            description='fast_lio yaml 配置文件所在目录')
    declare_config_file = DeclareLaunchArgument(
            'config_file', default_value='fast_lio.yaml',
            description='fast_lio 配置文件名')
    # UE 端雷达类型: true=GPU 雷达 (SensorType 8, 深度渲染, 异步采集), false=CPU 雷达 (SensorType 6)
    declare_gpu_lidar = DeclareLaunchArgument(
            'gpu_lidar', default_value='true',
            description='UE 端是否使用 GPULidar (需 settings.json 中 Lidar1 SensorType=8)')
    run_fast_lio = Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='fastlio_mapping',
            output='screen',
            parameters=[PathJoinSubstitution([config_path, config_file])])

    # 启动 FAST-LIO 链路 AirSim 传感器节点: 3D LiDAR(单时刻瞬时快照, 无伪逐点时间) + IMU
    # 参数默认值与 configs/fast_lio/settings.json 一致 (16线/1800点每转/10Hz)
    # gpu_lidar:=true 需 UE 端 settings.json 中 Lidar1 SensorType=8
    airsim_sensor_lidar = Node(
            package='flylab',
            executable='airsim_sensor_lidar',
            name='airsim_sensor_lidar',
            output='screen',
            parameters=[{'use_gpu_lidar': LaunchConfiguration('gpu_lidar')}])

    ld = LaunchDescription()

    ld.add_action(declare_rviz)
    ld.add_action(declare_rviz_cfg)
    ld.add_action(declare_config_path)
    ld.add_action(declare_config_file)
    ld.add_action(declare_gpu_lidar)
    ld.add_action(airsim_sensor_lidar)
    ld.add_action(run_fast_lio)
    ld.add_action(fast_lio_rviz_node)
    return ld
