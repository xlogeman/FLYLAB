import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# EGO-Planner 启动文件：启动 ego_planner_node (规划器) 和 traj_server (轨迹服务器)
# 全部可调参数集中在 configs/ego_planner/ego_planner.yaml 中维护，本文件只保留少量 launch 参数


def generate_launch_description():
    drone_id = LaunchConfiguration('drone_id', default=0)                                     # 无人机编号，单机为 0；多机集群时用来区分不同飞机
    flight_type = LaunchConfiguration('flight_type', default=1)                               # 飞行模式：1为目标点飞行，2为预设航点巡航

    # ============================= 话题参数 =============================
    odometry_topic = LaunchConfiguration('odometry_topic', default='/airsim/true_odom/data')  # 里程计，目前使用真值里程计，后续更换为ORB-SLAM3或者VINS等提供的里程计
    depth_topic = LaunchConfiguration('depth_topic', default='/airsim/depth_cam/image_raw')   # 深度相机话题
    camera_pose_topic = LaunchConfiguration('camera_pose_topic', default='/nouse/camera_pose')# 相机位置话题，这里不使用。通过tf获取相机位置
    cloud_topic = LaunchConfiguration('cloud_topic', default='/nouse/cloud')                  # 点云话题。我们使用深度相机，不使用点云

    # EGO-Planner 全参数文件（configs/ego_planner/ego_planner.yaml，随 colcon 安装到 share/flylab）
    params_file = os.path.join(
        get_package_share_directory('flylab'),
        'configs', 'ego_planner', 'ego_planner.yaml')

    # =========================================================
    # 1. 规划器 (大脑节点，负责接收里程计/深度图建图并规划避障轨迹)
    # =========================================================
    ego_planner_node = Node(
        package='ego_planner',             # 节点所在的 ROS 2 功能包名称
        executable='ego_planner_node',     # 可执行文件的名称
        name=['drone_', drone_id, '_ego_planner_node'], # 动态生成节点名，例如 'drone_0_ego_planner_node'，便于多机协同
        output='screen',                   # 将节点的标准输出打印到终端屏幕
        parameters=[
            params_file,                                   # 全量参数来自 YAML 配置文件
            # 以下参数由 launch 参数覆盖 YAML 中的默认值
            {'fsm/flight_type': flight_type},              # 飞行模式：1为目标点飞行，2为预设航点巡航
            {'manager/drone_id': drone_id},                # 飞机 ID
        ],
        remappings=[                       # 话题重映射（将代码内部的默认话题名替换为实际系统中的话题名）
            # 目前先去掉飞机编号前缀，直接接收绝对路径话题。后续要做集群避障的时候再把飞机编号加入进来
            ('odom_world', odometry_topic),         # 接收无人机的全局里程计（位置、姿态、速度）
            ('grid_map/odom', odometry_topic),      # 建图模块使用的里程计
            ('grid_map/depth', depth_topic),        # 接收深度相机的深度图（用于感知障碍物）
            ('grid_map/cloud', cloud_topic),        # 接收点云数据（可选，通常深度图和点云二选一输入）
            ('grid_map/pose', camera_pose_topic),   # 相机在世界坐标系下的位姿

            # 不同flight_type模式下的目标点话题。在其他代码中会将RViz中的"2D Nav Goal"点击事件话题映射为3D目标点，EGO将订阅这个三维目标点
            ('/move_base_simple/goal', '/goal_pose_3d'), # 这个是flight_type=1 手动目标点模式时使用的话题
            #('/traj_start_trigger', '/goal_pose_3d'), # 这个是flight_type=2 自动航点模式时使用的话题

            # 以下内部话题保留带飞机 ID 的前缀，避免多机之间的话题冲突
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline']), # 本机生成的 B 样条轨迹
            ('planning/data_display', ['drone_', drone_id, '_planning/data_display']), # 调试数据

            # 广播话题，用于多机集群间的防碰撞通信（将本机的 B 样条广播给其他飞机）
            ('planning/broadcast_bspline_from_planner', '/broadcast_bspline'),
            ('planning/broadcast_bspline_to_planner', '/broadcast_bspline'),

            # 规划可视化话题，用于在 RViz 中显示轨迹的不同阶段
            ('goal_point', ['drone_', drone_id, '_plan_vis/goal_point']),       # 目标点可视化
            ('global_list', ['drone_', drone_id, '_plan_vis/global_list']),     # 全局路径可视化
            ('init_list', ['drone_', drone_id, '_plan_vis/init_list']),         # 前端初始路径（A* 或 启发式寻路）可视化
            ('optimal_list', ['drone_', drone_id, '_plan_vis/optimal_list']),   # 后端优化后的最终光滑轨迹可视化
            ('a_star_list', ['drone_', drone_id, '_plan_vis/a_star_list']),     # A* 搜索扩展节点可视化

            # 膨胀后的占据栅格地图可视化
            ('grid_map/occupancy_inflate', ['drone_', drone_id, '_grid/grid_map/occupancy_inflate'])
        ],
    )

    # =========================================================
    # 2. 轨迹服务器 (小脑节点，负责把抽象的数学曲线变成具体的控制指令)
    # =========================================================
    traj_server_node = Node(
        package='ego_planner',              # 节点所在的 ROS2 包名
        executable='traj_server',           # 运行的可执行文件名
        name=['drone_', drone_id, '_traj_server'], # 节点名称，例如 drone_0_traj_server
        output='screen',                    # 将该节点的日志打印到终端屏幕上

        remappings=[
            # 【输出接口】：将默认的 position_cmd 重映射为特定的格式，我们的 px4_ego_tracker 监听的就是这个话题
            ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),

            # 【输入接口】：监听由大脑 (ego_planner_node) 发出的 B-样条数学曲线参数
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline'])
        ],

        parameters=[params_file]            # 全量参数来自 YAML 配置文件
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('drone_id', default_value='0', description='Drone ID'))
    ld.add_action(DeclareLaunchArgument('flight_type', default_value='1', description='1: manual goal, 2: waypoints'))
    ld.add_action(DeclareLaunchArgument('odometry_topic', default_value='/airsim/true_odom/data'))
    ld.add_action(DeclareLaunchArgument('depth_topic', default_value='/airsim/depth_cam/image_raw'))
    ld.add_action(DeclareLaunchArgument('camera_pose_topic', default_value='/nouse/camera_pose'))
    ld.add_action(DeclareLaunchArgument('cloud_topic', default_value='/nouse/cloud'))
    ld.add_action(ego_planner_node)
    ld.add_action(traj_server_node)

    return ld
