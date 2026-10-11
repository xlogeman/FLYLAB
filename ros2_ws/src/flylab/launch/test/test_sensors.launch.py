# test_sensors —— 传感器数据可视化测试链路 (算法无关)
#
# 目的: 可插拔启动各传感器节点 (src/sensors/), 在 RViz 里直接查看进入算法
#       之前的原始数据是否正常 (图像/点云/IMU/里程计), 不启动任何算法/桥接。
#
# 用法 (需先启动 PX4 SITL + Windows 端 UE, 且 source env.sh 拿到 PX4_SIM_HOST_ADDR):
#   ros2 launch flylab test_sensors.launch.py                     # 全部传感器
#   ros2 launch flylab test_sensors.launch.py lidar:=false        # 只看相机+IMU+odom
#   ros2 launch flylab test_sensors.launch.py camera:=false       # 只看点云+IMU+odom
#   ros2 launch flylab test_sensors.launch.py image_type:=2       # 相机改采深度图
#   ros2 launch flylab test_sensors.launch.py rviz:=false         # 只发数据不启动 RViz
#
# 话题速查:
#   /airsim/cam/<相机名>/image_raw   图像 (Scene=bgr8, 深度类=32FC1 米)
#   /airsim/imu/data_raw            IMU FRD 原生 200Hz
#   /airsim/imu/data                IMU FLU 标准 200Hz
#   /airsim/odom                    真值里程计 ENU 100Hz + TF world->base_link
#   /airsim/lidar/points_raw        原始点云 (传感器系 NED, frame_id=lidar_ned)
#   /airsim/gps /airsim/baro /airsim/mag   导航传感器 50Hz (RViz 不便显示, 用
#                                   `ros2 topic echo` 或 `ros2 topic hz` 检查)
#
# 检查命令:
#   ros2 topic hz /airsim/imu/data          # 期望 ~200Hz
#   ros2 topic hz /airsim/lidar/points_raw  # 期望 ~rotation_frequency (10Hz)
#   ros2 topic hz /airsim/odom              # 期望 ~100Hz
#   ros2 topic echo /airsim/gps --once
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('flylab')

    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    declare_rviz = DeclareLaunchArgument(
            'rviz', default_value='true',
            description='是否启动 RViz 可视化窗口')
    declare_rviz_cfg = DeclareLaunchArgument(
            'rviz_cfg', default_value=os.path.join(
                pkg_share, 'rviz', 'test', 'test_sensors.rviz'))
    test_rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen',
            condition=IfCondition(rviz_use))

    # ---- 可插拔开关 (不想看哪路就关哪路, 减小 UE 端采集压力) ----
    declare_camera = DeclareLaunchArgument(
            'camera', default_value='true',
            description='是否启动相机采集节点')
    declare_imu = DeclareLaunchArgument(
            'imu', default_value='true',
            description='是否启动 IMU 采集节点')
    declare_odom = DeclareLaunchArgument(
            'odom', default_value='true',
            description='是否启动真值里程计节点 (RViz 的 TF 依赖它)')
    declare_lidar = DeclareLaunchArgument(
            'lidar', default_value='true',
            description='是否启动 LiDAR 采集节点')
    declare_nav = DeclareLaunchArgument(
            'nav', default_value='false',
            description='是否启动 GPS/气压计/磁力计节点 (RViz 不显示, 用 topic echo 看)')

    # ---- 相机参数 ----
    # camera_names 须与 UE 端 settings.json 中的相机名一致
    # image_type: 0=Scene 1=DepthPlanar 2=DepthPerspective 3=DepthVis ...
    declare_camera_names = DeclareLaunchArgument(
            'camera_names', default_value='left_cam,right_cam',
            description='相机名列表, 逗号分隔 (须与 UE 端 settings.json 一致)')
    declare_image_type = DeclareLaunchArgument(
            'image_type', default_value='0',
            description='AirSim ImageType 枚举值 (0=Scene 1=DepthPlanar 2=DepthPerspective)')
    declare_grayscale = DeclareLaunchArgument(
            'grayscale', default_value='false',
            description='Scene 图转灰度 mono8 输出')
    airsim_camera = Node(
            package='flylab',
            executable='airsim_camera',
            name='airsim_camera',
            output='screen',
            condition=IfCondition(LaunchConfiguration('camera')),
            parameters=[{'camera_names': LaunchConfiguration('camera_names'),
                         'image_type': LaunchConfiguration('image_type'),
                         'grayscale': LaunchConfiguration('grayscale'),
                         'period_ms': 33}])

    # ---- IMU (200Hz, FLU 标准话题供 RViz 显示姿态) ----
    airsim_imu = Node(
            package='flylab',
            executable='airsim_imu',
            name='airsim_imu',
            output='screen',
            condition=IfCondition(LaunchConfiguration('imu')),
            parameters=[{'period_ms': 5}])

    # ---- 真值里程计 + world->base_link TF (RViz 各显示的坐标系来源) ----
    airsim_odom = Node(
            package='flylab',
            executable='airsim_odom',
            name='airsim_odom',
            output='screen',
            condition=IfCondition(LaunchConfiguration('odom')),
            parameters=[{'odom_period_ms': 10,
                         'enable_tf': True}])

    # ---- LiDAR (参数必须与 UE 端 settings.json 的 Lidar1 一致) ----
    declare_gpu_lidar = DeclareLaunchArgument(
            'gpu_lidar', default_value='true',
            description='UE 端是否使用 GPULidar (SensorType 8)')
    declare_num_channels = DeclareLaunchArgument(
            'num_channels', default_value='64',
            description='UE 端 Lidar1 NumberOfChannels')
    declare_rotation_hz = DeclareLaunchArgument(
            'rotation_frequency', default_value='10.0',
            description='UE 端 Lidar1 RotationsPerSecond')
    airsim_lidar = Node(
            package='flylab',
            executable='airsim_lidar',
            name='airsim_lidar',
            output='screen',
            condition=IfCondition(LaunchConfiguration('lidar')),
            parameters=[{'use_gpu_lidar': LaunchConfiguration('gpu_lidar'),
                         'num_channels': LaunchConfiguration('num_channels'),
                         'rotation_frequency': LaunchConfiguration('rotation_frequency')}])

    # ---- 测试专用静态 TF: base_link -> lidar_ned ----
    # 正式链路里雷达外参由 bridge_fastlio / bridge_super 挂载; 测试链路没有桥接,
    # 原始点云 frame_id=lidar_ned 无 TF 可查, RViz 会报错。这里用零位变换把点云
    # 挂到机体系上 (仅用于显示, 不代表真实安装位姿), 点云随无人机一起移动。
    lidar_test_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_test_tf',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link',
                       '--child-frame-id', 'lidar_ned'],
            condition=IfCondition(LaunchConfiguration('lidar')))

    # ---- 导航传感器 (GPS/气压计/磁力计, 默认关闭) ----
    airsim_nav = Node(
            package='flylab',
            executable='airsim_nav',
            name='airsim_nav',
            output='screen',
            condition=IfCondition(LaunchConfiguration('nav')))

    ld = LaunchDescription()

    ld.add_action(declare_rviz)
    ld.add_action(declare_rviz_cfg)
    ld.add_action(declare_camera)
    ld.add_action(declare_imu)
    ld.add_action(declare_odom)
    ld.add_action(declare_lidar)
    ld.add_action(declare_nav)
    ld.add_action(declare_camera_names)
    ld.add_action(declare_image_type)
    ld.add_action(declare_grayscale)
    ld.add_action(declare_gpu_lidar)
    ld.add_action(declare_num_channels)
    ld.add_action(declare_rotation_hz)
    ld.add_action(airsim_camera)
    ld.add_action(airsim_imu)
    ld.add_action(airsim_odom)
    ld.add_action(airsim_lidar)
    ld.add_action(lidar_test_tf)
    ld.add_action(airsim_nav)
    ld.add_action(test_rviz_node)
    return ld
