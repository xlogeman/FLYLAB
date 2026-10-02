import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# SUPER 避障飞行一键启动 (支持 tracker 参数选择控制链路):
#   ros2 launch flylab fly_super.launch.py                     # 默认 uXRCE-DDS 链路
#   ros2 launch flylab fly_super.launch.py tracker:=mavros     # MAVROS 链路
# 等价于同时启动以下组件:
#   1. airsim_sensor_super       AirSim 直连传感器节点 (LiDAR -> world 系点云 + 真值里程计)
#   2. fsm_node                  SUPER 规划器 (hku-mars), 目标点来自 tracker 转发的 /goal_pose_3d
#   3. super_cmd_bridge          SUPER 输出桥接: mars PositionCommand -> 标准 PositionCommand
#   4. tracker:=mavros 时自动启动 MAVROS 桥 (UDP 14542, PX4 中 MAVROS 专属实例);
#      tracker:=px4 时可用 xrce:=true 启动 MicroXRCEAgent
#   5. tracker_mavros / tracker_px4  轨迹跟踪节点 (按 tracker 参数二选一, RViz 点击目标点接管, 到达后交还 QGC)
#   6. rviz2                     SUPER 链路 RViz 可视化
# 前提: PX4 SITL + UE/AirSim 已启动; UE 端 settings.json 需含 Lidar1 16 线雷达
# 可选:
#   tracker:=px4|mavros          轨迹跟踪链路选择 (默认 px4, 即 uXRCE-DDS 直连)
#   xrce:=true                   同时启动 MicroXRCEAgent udp4 -p 8888 (tracker:=px4 时需要)
#   mavros:=false                不启动 MAVROS (tracker:=mavros 且已手动启动过时使用)
#   fcu_url:=udp://:14542@       MAVROS 连接串, 默认指向 PX4 的 MAVROS 专属实例 (14582→14542)
#   rviz:=false                  不启动 RViz


def generate_launch_description():
    pkg_share = get_package_share_directory('flylab')

    rviz_config_file = os.path.join(pkg_share, 'rviz', 'super_planner', 'super_planner.rviz')

    use_rviz = LaunchConfiguration('rviz', default='true')
    start_xrce = LaunchConfiguration('xrce', default='false')
    tracker = LaunchConfiguration('tracker', default='px4')
    start_mavros = LaunchConfiguration('mavros', default='true')
    fcu_url = LaunchConfiguration('fcu_url', default='udp://:14542@')

    # MAVROS 启动条件: tracker 选了 mavros 且 mavros 参数未关闭
    need_mavros = PythonExpression([
        "'", tracker, "' == 'mavros' and '", start_mavros, "' == 'true'"
    ])

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='是否启动 RViz'),
        DeclareLaunchArgument('xrce', default_value='false', description='是否启动 MicroXRCEAgent (tracker:=px4 时需要)'),
        DeclareLaunchArgument('tracker', default_value='px4', choices=['mavros', 'px4'],
                              description='轨迹跟踪链路: px4=uXRCE-DDS 直连 (默认) / mavros=MAVROS 桥'),
        DeclareLaunchArgument('mavros', default_value='true', description='是否随 launch 启动 MAVROS'),
        DeclareLaunchArgument('fcu_url', default_value='udp://:14542@', description='MAVROS 飞控连接串'),

        # 可选: MicroXRCEAgent (PX4 与 ROS2 的 uXRCE-DDS 桥)
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='screen',
            condition=IfCondition(start_xrce),
        ),

        # 1. AirSim 直连传感器节点: LiDAR -> world 系点云 + 真值里程计
        Node(
            package='flylab',
            executable='airsim_sensor_super',
            name='airsim_sensor_super_node',
            output='screen',
        ),

        # 2. SUPER 规划器 (配置真源: flylab/configs/super_planner/flylab_super.yaml,
        #    编译后安装到 share, 此处以绝对路径传入, 绕过 fsm_node 的源码树默认查找)
        Node(
            package='super_planner',
            executable='fsm_node',
            name='super_fsm_node',
            output='screen',
            parameters=[{'config_name': os.path.join(
                pkg_share, 'configs', 'super_planner', 'flylab_super.yaml')}],
        ),

        # 3. SUPER 输出桥接: /super/pos_cmd (mars) -> /planning/pos_cmd (标准, tracker 订阅)
        Node(
            package='flylab',
            executable='super_cmd_bridge',
            name='super_cmd_bridge_node',
            output='screen',
        ),

        # 4. MAVROS 桥 (仅 tracker:=mavros 时启动, mavros:=false 可关闭)
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

        # 5. 轨迹跟踪节点 (按 tracker 参数二选一; 订阅 /planning/pos_cmd 默认值)
        Node(
            package='flylab',
            executable='tracker_mavros',
            name='tracker_mavros_node',
            output='screen',
            condition=IfCondition(PythonExpression(["'", tracker, "' == 'mavros'"])),
        ),
        Node(
            package='flylab',
            executable='tracker_px4',
            name='tracker_px4_node',
            output='screen',
            condition=IfCondition(PythonExpression(["'", tracker, "' == 'px4'"])),
        ),

        # 6. RViz 可视化 (rviz:=false 可关闭)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            condition=IfCondition(use_rviz),
        ),
    ])
