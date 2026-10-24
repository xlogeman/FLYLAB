# test_odom —— 真值里程计单测 (仅一条 odom RPC 通道)
#
# 用法:
#   ros2 launch flylab_flight test_odom.launch.py
#
# 说明:
#   - airsim_odom 自带 TF world -> base_link 广播, 无需额外静态 TF
#   - RViz 里 Odom_truth 显示位姿轴, TF 显示坐标系树; 手动拖动无人机
#     (UE 中或 API) 时轴应实时跟随
#   - `ros2 topic hz /airsim/odom` 期望 ~100Hz
#   - 检查数值: `ros2 topic echo /airsim/odom --once`, 位置 (x,y,z) 应与
#     UE 中无人机世界坐标一致 (NED->ENU 已换算: ENU z 向上)
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('flylab_flight')

    declare_rviz = DeclareLaunchArgument('rviz', default_value='true',
            description='是否启动 RViz')
    declare_rviz_cfg = DeclareLaunchArgument('rviz_cfg', default_value=os.path.join(
            pkg_share, 'rviz', 'test', 'test_odom.rviz'))
    rviz_node = Node(package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_cfg')],
            output='screen', condition=IfCondition(LaunchConfiguration('rviz')))

    odom_node = Node(
            package='flylab_flight', executable='airsim_odom', name='airsim_odom',
            output='screen',
            parameters=[{'odom_period_ms': 10, 'enable_tf': True}])

    ld = LaunchDescription()
    ld.add_action(declare_rviz)
    ld.add_action(declare_rviz_cfg)
    ld.add_action(odom_node)
    ld.add_action(rviz_node)
    return ld
