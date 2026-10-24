# test_camera —— 相机单测 (左右 Scene 彩图 + 深度图, 共两条图像 RPC 通道)
#
# 用法 (需先启动 Windows 端 UE, source env.sh 拿到 PX4_SIM_HOST_ADDR):
#   ros2 launch flylab_flight test_camera.launch.py                  # 左右彩图 + 深度图全开
#   ros2 launch flylab_flight test_camera.launch.py depth:=false     # 只看左右彩图
#   ros2 launch flylab_flight test_camera.launch.py grayscale:=true  # 彩图转灰度 mono8
#
# 说明:
#   - 一个 airsim_camera 实例只能采一种 image_type, 所以场景图 (0=Scene) 与
#     深度图 (1=DepthPlanar) 各起一个实例, 各占一条 RPC 通道, 互不影响
#   - 默认相机名与 configs/settings.json 一致:
#       left_cam / right_cam (Scene, 640x480), depth_cam (DepthPlanar, 400x300)
#   - 图像显示不需要 TF, RViz Fixed Frame 随意也不会影响 Image 窗口;
#     RViz 布局: 左侧 Displays 面板, 右侧三个图像窗口纵向铺满
#   - 缺相机/名字不对: 终端每 2 秒一条 "返回空图像" 警告, UE 端 settings.json
#     核对相机名; 图像黑屏且无警告 -> 检查 image_type 与 pixels_as_float 组合
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
            pkg_share, 'rviz', 'test', 'test_camera.rviz'))
    rviz_node = Node(package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_cfg')],
            output='screen', condition=IfCondition(LaunchConfiguration('rviz')))

    # ---- 场景彩图实例 (left_cam / right_cam, Scene) ----
    declare_camera_names = DeclareLaunchArgument(
            'camera_names', default_value='left_cam,front_cam,right_cam',
            description='彩图相机名列表, 逗号分隔 (须与 UE 端 settings.json 一致)')
    declare_image_type = DeclareLaunchArgument(
            'image_type', default_value='0',
            description='彩图 AirSim ImageType 枚举值 (0=Scene)')
    declare_grayscale = DeclareLaunchArgument(
            'grayscale', default_value='false',
            description='Scene 图转灰度 mono8 输出')
    camera_node = Node(
            package='flylab_flight', executable='airsim_camera', name='airsim_camera',
            output='screen',
            parameters=[{'camera_names': LaunchConfiguration('camera_names'),
                         'image_type': LaunchConfiguration('image_type'),
                         'grayscale': LaunchConfiguration('grayscale'),
                         'period_ms': 33}])

    # ---- 深度图实例 (depth_cam, DepthPlanar, 32FC1 单位米) ----
    declare_depth = DeclareLaunchArgument(
            'depth', default_value='true',
            description='是否启动深度图采集实例')
    declare_depth_names = DeclareLaunchArgument(
            'depth_camera_names', default_value='depth_cam',
            description='深度相机名 (须与 UE 端 settings.json 一致)')
    declare_depth_type = DeclareLaunchArgument(
            'depth_image_type', default_value='1',
            description='深度图 ImageType (1=DepthPlanar 2=DepthPerspective 3=DepthVis)')
    depth_camera_node = Node(
            package='flylab_flight', executable='airsim_camera', name='airsim_camera_depth',
            output='screen', condition=IfCondition(LaunchConfiguration('depth')),
            parameters=[{'camera_names': LaunchConfiguration('depth_camera_names'),
                         'image_type': LaunchConfiguration('depth_image_type'),
                         'period_ms': 33}])

    ld = LaunchDescription()
    ld.add_action(declare_rviz)
    ld.add_action(declare_rviz_cfg)
    ld.add_action(declare_camera_names)
    ld.add_action(declare_image_type)
    ld.add_action(declare_grayscale)
    ld.add_action(declare_depth)
    ld.add_action(declare_depth_names)
    ld.add_action(declare_depth_type)
    ld.add_action(camera_node)
    ld.add_action(depth_camera_node)
    ld.add_action(rviz_node)
    return ld
