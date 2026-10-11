import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# EGO 避障飞行一键启动 (支持 tracker 参数选择控制链路):
#   ros2 launch flylab fly_ego.launch.py                     # 默认 uXRCE-DDS 链路
#   ros2 launch flylab fly_ego.launch.py tracker:=mavros     # MAVROS 链路
# 等价于同时启动以下组件:
#   1. airsim_camera/airsim_imu/airsim_odom/airsim_nav
#                               AirSim 直连传感器节点 (可插拔: 深度图/IMU/里程计/GPS/气压计/磁力计)
#   2. ego_planner.launch.py     EGO-Planner 规划器 + 轨迹服务器
#   3. tracker:=mavros 时自动启动 MAVROS 桥 (UDP 14542, PX4 中 MAVROS 专属实例);
#      tracker:=px4 时可用 xrce:=true 启动 MicroXRCEAgent
#      注意: PX4 SITL 的 14540/14580 onboard 实例对端会被 AirSim 控制通道 (UnrealEditor, UDP 14580) 抢占,
#      MAVROS 必须使用 14582↔14542 专属实例 (见 px4-rc.mavlink)
#   4. tracker_mavros / tracker_px4  轨迹跟踪节点 (按 tracker 参数二选一, RViz 点击目标点接管, 到达后交还 QGC)
#   5. rviz2                     EGO 链路 RViz 可视化
# 可选:
#   tracker:=px4|mavros          轨迹跟踪链路选择 (默认 px4, 即 uXRCE-DDS 直连)
#   xrce:=true                   同时启动 MicroXRCEAgent udp4 -p 8888 (tracker:=px4 时需要, 已手动启动则保持 false)
#   mavros:=false                不启动 MAVROS (tracker:=mavros 且已手动启动过时使用)
#   fcu_url:=udp://:14542@       MAVROS 连接串, 默认指向 PX4 的 MAVROS 专属实例 (14582→14542)
#   rviz:=false                  不启动 RViz


def generate_launch_description():
    pkg_share = get_package_share_directory('flylab')

    ego_launch_file = os.path.join(pkg_share, 'launch', 'ego_planner', 'ego_planner.launch.py')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'ego_planner', 'ego_planner.rviz')

    use_rviz = LaunchConfiguration('rviz', default='true')
    start_xrce = LaunchConfiguration('xrce', default='false')
    tracker = LaunchConfiguration('tracker', default='px4')
    start_mavros = LaunchConfiguration('mavros', default='true')
    fcu_url = LaunchConfiguration('fcu_url', default='udp://:14542@')
    drone_id = LaunchConfiguration('drone_id', default='0')

    # MAVROS 启动条件: tracker 选了 mavros 且 mavros 参数未关闭
    need_mavros = PythonExpression([
        "'", tracker, "' == 'mavros' and '", start_mavros, "' == 'true'"
    ])

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='是否启动 RViz'),
        DeclareLaunchArgument('xrce', default_value='false', description='是否启动 MicroXRCEAgent (tracker:=px4 时需要, 已手动启动则保持 false)'),
        DeclareLaunchArgument('tracker', default_value='px4', choices=['mavros', 'px4'],
                              description='轨迹跟踪链路: px4=uXRCE-DDS 直连 (默认) / mavros=MAVROS 桥'),
        DeclareLaunchArgument('mavros', default_value='true', description='是否随 launch 启动 MAVROS (tracker:=mavros 且已手动启动则设为 false)'),
        DeclareLaunchArgument('fcu_url', default_value='udp://:14542@', description='MAVROS 飞控连接串 (PX4 MAVROS 专属实例 14582→14542)'),
        DeclareLaunchArgument('drone_id', default_value='0', description='Drone ID'),

        # 可选: MicroXRCEAgent (PX4 与 ROS2 的 uXRCE-DDS 桥)
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='screen',
            condition=IfCondition(start_xrce),
        ),

        # 1. AirSim 直连传感器节点 (可插拔, 按需启动):
        #    深度相机 (image_type=1 DepthPlanar) / IMU / 真值里程计(含TF) / 低频导航
        Node(
            package='flylab',
            executable='airsim_camera',
            name='airsim_camera_depth',
            output='screen',
            parameters=[{'camera_names': 'depth_cam', 'image_type': 1}],
        ),
        Node(
            package='flylab',
            executable='airsim_imu',
            name='airsim_imu',
            output='screen',
            parameters=[{'period_ms': 5}],
        ),
        Node(
            package='flylab',
            executable='airsim_odom',
            name='airsim_odom',
            output='screen',
            parameters=[{'odom_period_ms': 10}],
        ),
        Node(
            package='flylab',
            executable='airsim_nav',
            name='airsim_nav',
            output='screen',
            parameters=[{'period_ms': 20}],
        ),

        # 2. EGO-Planner 规划器 + 轨迹服务器 (参数见 configs/ego_planner/ego_planner.yaml)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ego_launch_file),
            launch_arguments={'drone_id': drone_id}.items(),
        ),

        # 3. MAVROS 桥 (仅 tracker:=mavros 时启动, mavros:=false 可关闭适用于已手动启动的情况)
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[{
                'fcu_url': fcu_url,
                'gcs_url': '',
                'tgt_system': 1,
                'tgt_component': 1,
            }],
            condition=IfCondition(need_mavros),
        ),

        # 4. 轨迹跟踪节点 (按 tracker 参数二选一; pos_cmd_topic 对接 EGO-Planner 输出)
        Node(
            package='flylab',
            executable='tracker_mavros',
            name='tracker_mavros_node',
            output='screen',
            parameters=[{'pos_cmd_topic': PythonExpression(
                ["'/drone_' + '", drone_id, "' + '_planning/pos_cmd'"])}],
            condition=IfCondition(PythonExpression(["'", tracker, "' == 'mavros'"])),
        ),
        Node(
            package='flylab',
            executable='tracker_px4',
            name='tracker_px4_node',
            output='screen',
            parameters=[{'pos_cmd_topic': PythonExpression(
                ["'/drone_' + '", drone_id, "' + '_planning/pos_cmd'"])}],
            condition=IfCondition(PythonExpression(["'", tracker, "' == 'px4'"])),
        ),

        # 5. RViz 可视化 (rviz:=false 可关闭)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            condition=IfCondition(use_rviz),
        ),
    ])
