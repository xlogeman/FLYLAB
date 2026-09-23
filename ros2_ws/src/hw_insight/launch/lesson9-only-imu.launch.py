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
    lesson9_only_imu_rviz_path = os.path.join(pkg_share, 'rviz/lesson9_only_imu.rviz')
    lesson9_only_imu_rviz_node = launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', lesson9_only_imu_rviz_path]
    )

    # ================= IMU 坐标系转换逻辑 =================
    # 1. 静态变换：定义从 FRD 到 FLU 的物理关系 (绕 X 轴旋转 180 度)
    # 参数顺序: x y z yaw pitch roll frame_id child_frame_id
    static_tf_frd_to_flu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_frd_to_flu',
        arguments=[
            '0', '0', '0',          # x y z 平移
            '0', '0', '3.1415926',  # yaw pitch roll (绕X轴转180度)
            'PX4/base_link_frd',    # 原始 Frame ID
            'PX4/base_link_flu'     # 新的 Frame ID
        ],
        output='screen'
    )

  
    # Create the launch description and populate
    ld = LaunchDescription()

    # Create the launch description and populate
    ld.add_action(static_tf_frd_to_flu)
    ld.add_action(lesson9_only_imu_rviz_node)
    return ld

