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
    pkg_share = get_package_share_directory('hw_insight')

    # RVIZ2显示配置
    lesson8_rviz_path = os.path.join(pkg_share, 'rviz/lesson8.rviz')
    lesson8_rviz_node = launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', lesson8_rviz_path]
    )

    # map to world
    static_tf_map_to_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_world',
        arguments=[
            '0', '0', '0',  # x y z 平移量（米）
            '0', '0', '0',  # roll pitch yaw 旋转量（弧度）
            'world',         # 父坐标系
            'map'            # 子坐标系
        ],
        output='screen'
    )

    # 启动 ytjy_airsim_fast 快速发布各种ROS2话题
    ytjy_airsim_fast = Node(
            package='airsim_ros_pkgs',
            executable='ytjy_airsim_fast',
            name='ytjy_airsim_fast',
            output='screen')

    # 启动 vins 切换节点
    hw_vins_switch_node = Node(
            package='hw_insight',
            executable='lesson8_vins_switch',
            name='lesson8_vins_switch',
            output='screen')

    # 启动 vins_estimator 原始数据（绿色轨迹）
    config_path = "/home/hw/sitl/hw-ros2/ros2/src/vins/config/ytjy/lesson9_extrinsics.yaml"
    vins_estimator_node = launch_ros.actions.Node(
            package='vins',
            executable='vins_node',
            name='vins_estimator',
            namespace='vins_estimator',
            output='screen',
            parameters=[{'config_file': config_path}],
            arguments=[config_path]
    )

    # 启动 loop_fusion 回环检测（红色轨迹）
    loop_fusion_node = launch_ros.actions.Node(
            package='loop_fusion',
            executable='loop_fusion_node',
            name='loop_fusion',
            namespace='loop_fusion',
            output='screen',
            parameters=[{'config_file': config_path}],
            arguments=[config_path]
    )

    # 启动 global_fusion GPS融合（白色轨迹）
    global_fusion_node = launch_ros.actions.Node(
            package='global_fusion',
            executable='global_fusion_node',
            name='global_fusion',
            namespace='global_fusion',
            output='screen',
            parameters=[
                {'gps_type': 0}  # 0: GPS数据是经纬度(NavSatFix), 1: 已经是XY坐标
            ],
            remappings=[
                ('/gps', '/airsim/gps/data')
            ]
    )
  
    # Create the launch description and populate
    ld = LaunchDescription()

    # Create the launch description and populate
    #ld.add_action(ytjy_airsim_fast)
    ld.add_action(static_tf_map_to_world)
    ld.add_action(lesson8_rviz_node)
    ld.add_action(hw_vins_switch_node)
    #ld.add_action(vins_estimator_node)
    #ld.add_action(loop_fusion_node)
    #ld.add_action(global_fusion_node)
    return ld

