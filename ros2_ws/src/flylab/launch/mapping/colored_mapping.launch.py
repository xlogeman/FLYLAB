# colored_mapping —— 彩色点云(xyzrgb)版建图链路（原课程 lesson4 变体）
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
    pkg_share = get_package_share_directory('flylab')
    rviz_path = os.path.join(pkg_share, 'rviz/mapping/colored_mapping.rviz')
    lensson4_rviz_node = launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='depth_rviz2',
            arguments=['-d', rviz_path]
    )

    lensson4_depth_image_proc_node = launch_ros.actions.Node(
            package='depth_image_proc',
            executable='point_cloud_xyzrgb_node',
            name='depth_to_cloud',
            remappings=[
                ('depth_registered/image_rect', '/airsim_node/PX4/depth_cam_DepthPlanar/image'),
                ('depth_registered/camera_info', '/airsim_node/PX4/depth_cam_DepthPlanar/camera_info'),
                ('rgb/image_rect_color', '/airsim_node/PX4/depth_cam_Scene/image'),
                ('rgb/camera_info', '/airsim_node/PX4/depth_cam_Scene/camera_info'),
                ('points', '/point_cloud_xyzrgb_node/depth_color_cloud')
            ]
    )

    lensson4_octomap_server_node = launch_ros.actions.Node(
            package='octomap_server',          # 包名
            executable='octomap_server_node',  # 可执行文件名
            name='octomap_server',             # 节点名称
            parameters=[
                {'frame_id': 'world'},            # 地图坐标系 (官方版 ENU)
                {'base_frame_id': 'world'},       # 过滤地面时，使用该坐标
                {'resolution': 0.25},             # 地图分辨率，单位：米
                {'dynamic': True},                # 启用动态边界扩展
                {'sensor_model.max_range': 50.0}, # 传感器最大有效距离
                {'compress_map': False},          # 是否压缩地图，启用后会减少网络数据传输，但会加大CPU计算量。用于分布式协同系统
                {'colored_map': True},
                {'ground_filter': True},          # 过滤地面，即不显示地面，降低计算量
                {'ground_filter.distance': 0.2},  # 小于多少米都算作地面
                {'ground_filter.plane_distance': 0.21},
                {'publish_free_space': False},    # 是否发布空的地点值(无阻挡的空间)，避障逻辑会使用
                {'point_cloud_min_z': 0.0},       # ENU: 过滤起飞点高度以下的点 (原 NED: max_z<=0)
                {'latch': False}                  # default: True for a static map, false if no initial map is given
            ],
            remappings=[
                ('cloud_in', '/point_cloud_xyzrgb_node/depth_color_cloud')  # 输入点云话题重映射
            ]
    )

    airsim_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('airsim_ros_pkgs'), 'launch/airsim_node.launch.py')
        )
    )


    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(airsim_node_launch)
    ld.add_action(lensson4_depth_image_proc_node)
    ld.add_action(lensson4_octomap_server_node)
    ld.add_action(lensson4_rviz_node)
    return ld
