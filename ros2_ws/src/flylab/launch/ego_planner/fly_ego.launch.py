import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# EGO 避障飞行一键启动:
#   ros2 launch flylab fly_ego.launch.py
# 等价于同时启动以下组件:
#   1. airsim_sensor_ego         AirSim 直连传感器节点 (深度图/IMU/里程计/GPS/气压计/磁力计 + TF)
#   2. ego_planner.launch.py     EGO-Planner 规划器 + 轨迹服务器
#   3. px4_ego_tracker           PX4 轨迹跟踪节点 (RViz 点击目标点接管, 到达后交还 QGC)
#   4. rviz2                     EGO 链路 RViz 可视化
# 可选:
#   xrce:=true                   同时启动 MicroXRCEAgent udp4 -p 8888 (若已手动启动过会端口冲突, 保持默认 false)
#   rviz:=false                  不启动 RViz


def generate_launch_description():
    pkg_share = get_package_share_directory('flylab')

    ego_launch_file = os.path.join(pkg_share, 'launch', 'ego_planner', 'ego_planner.launch.py')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'ego_planner', 'ego_planner.rviz')

    use_rviz = LaunchConfiguration('rviz', default='true')
    start_xrce = LaunchConfiguration('xrce', default='false')
    drone_id = LaunchConfiguration('drone_id', default='0')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='是否启动 RViz'),
        DeclareLaunchArgument('xrce', default_value='false', description='是否启动 MicroXRCEAgent (已手动启动则保持 false)'),
        DeclareLaunchArgument('drone_id', default_value='0', description='Drone ID'),

        # 可选: MicroXRCEAgent (PX4 与 ROS2 的 uXRCE-DDS 桥)
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='screen',
            condition=IfCondition(start_xrce),
        ),

        # 1. AirSim 直连传感器节点
        Node(
            package='flylab',
            executable='airsim_sensor_ego',
            name='airsim_sensor_ego_node',
            output='screen',
        ),

        # 2. EGO-Planner 规划器 + 轨迹服务器 (参数见 configs/ego_planner/ego_planner.yaml)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ego_launch_file),
            launch_arguments={'drone_id': drone_id}.items(),
        ),

        # 3. PX4 轨迹跟踪节点
        Node(
            package='flylab',
            executable='px4_ego_tracker',
            name='px4_ego_tracker_node',
            output='screen',
        ),

        # 4. RViz 可视化 (rviz:=false 可关闭)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            condition=IfCondition(use_rviz),
        ),
    ])
