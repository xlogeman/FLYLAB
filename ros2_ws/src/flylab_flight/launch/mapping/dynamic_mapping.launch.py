# dynamic_mapping —— 完整建图链路（原课程 lesson4 变体）
import os
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory 

def generate_launch_description():
    pkg_share = get_package_share_directory('flylab_flight')

	# RVIZ2显示配置
    rviz_path = os.path.join(pkg_share, 'rviz/mapping/depth_mapping.rviz')
    lensson4_rviz_node = launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='depth_rviz2',
            arguments=['-d', rviz_path]
    )

	# 将深度摄像头的信息转换为点云信息，再将点云信息发送给octomap_server
    lensson4_depth_image_proc_node = launch_ros.actions.Node(
            package='depth_image_proc',
            executable='point_cloud_xyz_node',
            name='depth_to_cloud',
            remappings=[
                ('image_rect', '/airsim_node/PX4/depth_cam_DepthPlanar/image'),
                ('camera_info', '/airsim_node/PX4/depth_cam_DepthPlanar/camera_info'),
                ('points', '/depth_camera/pointers')
            ]
    )


    camera_max_range_arg = launch.actions.DeclareLaunchArgument('sensor_max_range', default_value='10.0', description=' ')
    camera_max_range = launch.substitutions.LaunchConfiguration('sensor_max_range')

    map_resolution_arg = launch.actions.DeclareLaunchArgument('map_resolution', default_value='0.05', description=' ')
    map_resolution = launch.substitutions.LaunchConfiguration('map_resolution')

    # 用上面depth_image_proc转换出的点云信息生成为地图
    lensson4_octomap_server_node = launch_ros.actions.Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            parameters=[
                {'frame_id': 'world'},                       # 地图坐标系 (官方版 ENU)
                {'base_frame_id': 'PX4/depth_cam_optical'},  # 备选: PX4/depth_cam_body, PX4/depth_cam_body/static
                {'resolution': map_resolution},
                {'compress_map': False},                     # 压缩减网络传输但增 CPU (分布式协同用)
                {'ground_filter': False},                    # 关地面过滤, 用 point_cloud_max_z 代替
                {'ground_filter.distance': 0.05},
                {'ground_filter.plane_distance': 0.06},
                {'sensor_model.max_range': camera_max_range},
                {'publish_free_space': False},               # 发布空闲体, 避障逻辑会使用
                {'use_height_map': True},
                {'point_cloud_max_x': 200.0},
                {'point_cloud_min_x': -200.0},
                {'point_cloud_max_y': 200.0},
                {'point_cloud_min_y': -200.0},
                {'point_cloud_max_z': 50.0},                 # ENU: 0.17~50m 高于起飞点 (原 NED: -50~-0.17)
                {'point_cloud_min_z': 0.17},
                {'occupancy_max_z': 50.0},
                {'occupancy_min_z': 0.17},
                {'min_x_size': 0.0},
                {'min_y_size': 0.0},
                {'latch': False}                             # 非静态地图, 增量更新更快
            ],
            remappings=[
                ('cloud_in', '/depth_camera/pointers')  # 输入点云话题重映射
            ]
    )

	# 启动AirSim节点
    airsim_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('airsim_ros_pkgs'), 'launch/airsim_node.launch.py')
        )
    )


    ld = LaunchDescription()
    ld.add_action(camera_max_range_arg)
    ld.add_action(map_resolution_arg)
    ld.add_action(airsim_node_launch)
    ld.add_action(lensson4_depth_image_proc_node)
    ld.add_action(lensson4_octomap_server_node)
    ld.add_action(lensson4_rviz_node)
    return ld
