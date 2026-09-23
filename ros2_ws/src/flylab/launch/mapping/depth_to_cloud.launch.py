# depth_to_cloud —— 分段调试: 仅深度图→点云, 不建图（原课程 lesson4 分段）
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
                ('image_rect', '/airsim_node/PX4/CameraDepth/DepthPlanar'),
                ('camera_info', '/airsim_node/PX4/CameraDepth/camera_info'),
                ('points', '/depth_camera/pointers')
            ]
    )

    # 用上面depth_image_proc转换出的点云信息生成为地图
    lensson4_octomap_server_node = launch_ros.actions.Node(
            package='octomap_server',          # 包名
            executable='octomap_server_node',  # 可执行文件名
            name='octomap_server',             # 节点名称
            parameters=[
                {'frame_id': 'world_ned'},         # 地图坐标系，以世界地图为坐标
                {'base_frame_id': 'PX4/CameraDepth_optical'},  #无人机参考坐标 # PX4/CameraDepth_optical, PX4/CameraDepth_body, PX4/CameraDepth_body/static
                {'resolution': 0.25},              # 地图分辨率，单位：米，当前0.25米的分辨率
                {'compress_map': False},           # 是否压缩地图，启用后会减少网络数据传输，但会加大CPU计算量。用于分布式协同系统
                {'ground_filter': False},          # 过滤地面相关的配置，使用point_cloud_max_z来代替
                {'ground_filter.distance': 0.05},
                {'ground_filter.plane_distance': 0.06},
                {'sensor_model.max_range': 25.0},  # 传感器最大有效距离，也就是深度摄像头的能力
                {'publish_free_space': False},     # 是否发布空的地点值(无阻挡的空间)，避障逻辑会使用
                {'use_height_map': True},          # 地图是否用不同的颜色标记高度
                {'dynamic': False},                # 不启用动态边界扩展。下面的设置就是边界设置，超出边界的数据都会丢弃
                {'point_cloud_max_x': 200.0},
                {'point_cloud_min_x': -200.0},
                {'point_cloud_max_y': 200.0},
                {'point_cloud_min_y': -200.0},
                {'point_cloud_max_z': -0.17},
                {'point_cloud_min_z': -50.0},
                {'occupancy_max_z': -0.17},
                {'occupancy_min_z': -50.0},
                {'min_x_size': 0.0},
                {'min_y_size': 0.0},
                {'latch': False}                  # True for a static map, false if no initial map is given。设置为false能够加快处理速度
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


    # Create the launch description and populate
    ld = LaunchDescription()
    #ld.add_action(airsim_node_launch)
    ld.add_action(lensson4_depth_image_proc_node)
    #ld.add_action(lensson4_octomap_server_node)
    #ld.add_action(lensson4_rviz_node)
    return ld
