import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# EGO-Planner 启动文件: ego_planner_node (规划器) + traj_server (轨迹服务器)
# 全部可调参数集中在 configs/ego_planner/ego_planner.yaml, 本文件只留少量 launch 参数


def generate_launch_description():
    drone_id = LaunchConfiguration('drone_id', default=0)                # 单机 0; 多机集群区分飞机
    flight_type = LaunchConfiguration('flight_type', default=1)          # 1=目标点飞行, 2=预设航点巡航

    # 话题参数
    odometry_topic = LaunchConfiguration('odometry_topic', default='/airsim/odom')               # 里程计 (真值; 后续可换 VINS/ORB)
    depth_topic = LaunchConfiguration('depth_topic', default='/airsim/cam/depth_cam/image_raw')  # 深度相机
    camera_pose_topic = LaunchConfiguration('camera_pose_topic', default='/nouse/camera_pose')   # 不使用, 相机位置走 TF
    cloud_topic = LaunchConfiguration('cloud_topic', default='/nouse/cloud')                     # 不使用, 走深度图

    # 全参数文件 (随 colcon 安装到 share/flylab_flight)
    params_file = os.path.join(
        get_package_share_directory('flylab_flight'),
        'configs', 'ego_planner', 'ego_planner.yaml')

    # ---------- 1. 规划器 ----------
    ego_planner_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name=['drone_', drone_id, '_ego_planner_node'],   # 动态节点名, 便于多机协同
        output='screen',
        parameters=[
            params_file,
            # launch 参数覆盖 YAML 默认值
            {'fsm/flight_type': flight_type},
            {'manager/drone_id': drone_id},
        ],
        remappings=[
            # 单机阶段先去掉飞机编号前缀, 直接收绝对话题; 集群时再加编号
            ('odom_world', odometry_topic),
            ('grid_map/odom', odometry_topic),
            ('grid_map/depth', depth_topic),
            ('grid_map/cloud', cloud_topic),
            ('grid_map/pose', camera_pose_topic),

            # flight_type=1 手动目标点: RViz "2D Nav Goal" 已被映射为 3D 目标点
            ('/move_base_simple/goal', '/goal_pose_3d'),
            # ('/traj_start_trigger', '/goal_pose_3d'),   # flight_type=2 自动航点

            # 内部话题保留飞机 ID 前缀, 避免多机冲突
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline']),
            ('planning/data_display', ['drone_', drone_id, '_planning/data_display']),

            # 多机防碰撞广播
            ('planning/broadcast_bspline_from_planner', '/broadcast_bspline'),
            ('planning/broadcast_bspline_to_planner', '/broadcast_bspline'),

            # 规划可视化
            ('goal_point', ['drone_', drone_id, '_plan_vis/goal_point']),
            ('global_list', ['drone_', drone_id, '_plan_vis/global_list']),
            ('init_list', ['drone_', drone_id, '_plan_vis/init_list']),
            ('optimal_list', ['drone_', drone_id, '_plan_vis/optimal_list']),
            ('a_star_list', ['drone_', drone_id, '_plan_vis/a_star_list']),

            # 膨胀占据栅格地图可视化
            ('grid_map/occupancy_inflate', ['drone_', drone_id, '_grid/grid_map/occupancy_inflate'])
        ],
    )

    # ---------- 2. 轨迹服务器: B 样条 -> PositionCommand, 供 tracker 订阅 ----------
    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['drone_', drone_id, '_traj_server'],
        output='screen',

        remappings=[
            ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline'])
        ],

        parameters=[params_file]
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('drone_id', default_value='0', description='Drone ID'))
    ld.add_action(DeclareLaunchArgument('flight_type', default_value='1', description='1: manual goal, 2: waypoints'))
    ld.add_action(DeclareLaunchArgument('odometry_topic', default_value='/airsim/odom'))
    ld.add_action(DeclareLaunchArgument('depth_topic', default_value='/airsim/cam/depth_cam/image_raw'))
    ld.add_action(DeclareLaunchArgument('camera_pose_topic', default_value='/nouse/camera_pose'))
    ld.add_action(DeclareLaunchArgument('cloud_topic', default_value='/nouse/cloud'))
    ld.add_action(ego_planner_node)
    ld.add_action(traj_server_node)

    return ld
