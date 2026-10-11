# test_imu —— IMU 单测 (仅一条 IMU RPC 通道)
#
# 用法:
#   ros2 launch flylab test_imu.launch.py                     # 默认
#   ros2 launch flylab test_imu.launch.py inject_bias:=true   # 注入校准残余偏置
#
# 说明:
#   - airsim_imu 不发布 TF, 这里用静态 TF world -> imu_link_flu (零位) 把
#     IMU 姿态轴挂到 RViz 原点显示; 静止时轴与 world 重合 (水平姿态),
#     手势/起飞转动时轴应实时跟随
#   - RViz 里 Imu 显示用 FLU 标准话题 /airsim/imu/data; FRD 原生话题
#     /airsim/imu/data_raw 供算法用, 数值验证建议用 plotjuggler:
#     ros2 run plotjuggler plotjuggler  (订阅 /airsim/imu/data_raw 画曲线)
#   - 200Hz 数据流, `ros2 topic hz /airsim/imu/data` 期望 ~200Hz
#   - UE 过载时仿真变慢, 节点按 time_stamp 去重, hz 会低于 200, 属正常
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('flylab')

    declare_rviz = DeclareLaunchArgument('rviz', default_value='true',
            description='是否启动 RViz')
    declare_rviz_cfg = DeclareLaunchArgument('rviz_cfg', default_value=os.path.join(
            pkg_share, 'rviz', 'test', 'test_imu.rviz'))
    rviz_node = Node(package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_cfg')],
            output='screen', condition=IfCondition(LaunchConfiguration('rviz')))

    declare_bias = DeclareLaunchArgument(
            'inject_bias', default_value='false',
            description='注入模拟校准残余偏置 (依据 settings.json 的 ARW/VRW 定级)')
    imu_node = Node(
            package='flylab', executable='airsim_imu', name='airsim_imu',
            output='screen',
            parameters=[{'period_ms': 5,
                         'inject_bias': LaunchConfiguration('inject_bias')}])

    # 测试专用静态 TF: world -> imu_link_flu (零位, 仅用于 RViz 显示)
    imu_test_tf = Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='imu_test_tf',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'world', '--child-frame-id', 'imu_link_flu'])

    ld = LaunchDescription()
    ld.add_action(declare_rviz)
    ld.add_action(declare_rviz_cfg)
    ld.add_action(declare_bias)
    ld.add_action(imu_node)
    ld.add_action(imu_test_tf)
    ld.add_action(rviz_node)
    return ld
