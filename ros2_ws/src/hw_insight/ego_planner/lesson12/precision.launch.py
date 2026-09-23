import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ============================= 话题参数 ================================
    drone_id = LaunchConfiguration('drone_id', default=0)                                           # 无人机编号，如果是单机通常为 0；多机集群时用来区分不同飞机
    odometry_topic = LaunchConfiguration('odometry_topic', default='/airsim/true_odom/data')        # 里程计，目前使用真值里程计，后续更换为ORB-SLAM3或者VINS等提供的里程计
    depth_topic = LaunchConfiguration('depth_topic', default='/airsim/depth_cam/image_raw')         # 深度相机话题
    camera_pose_topic = LaunchConfiguration('camera_pose_topic', default='/nouse/camera_pose')      # 相机位置话题，这里不使用。通过tf获取相机位置
    cloud_topic = LaunchConfiguration('cloud_topic', default='/nouse/cloud')                        # 点云话题。我们使用深度相机，不使用点云

    # ============================= 整体地图参数，相机参数 ================================
    # 总地图大小，以无人机起点为中心的总地图大小。非常占用内存。无人机飞出此范围EGO会挂掉。
    # ！！！！！！注意：理论上全局地图x和y可以不相等，但是由于代码bug导致x，y不相等时会出问题，所以目前还是将x，y设置为相同大小。后续再改代码bug。
    # ！！！！！！注意：局部地图大小也有类似问题，先确保局部地图也是x和y相等。
    # ！！！！！！所以在分辨率很细，地图很大的情况下，会消耗海量内存，注意调整wsl和windows的内存分配，避免内存不够用。当前配制至少给WSL分配11G物理
    map_size_x = LaunchConfiguration('map_size_x', default=260.0)       # EGO 全局静态内存数组的 X 轴物理边界大小 (米)
    map_size_y = LaunchConfiguration('map_size_y', default=260.0)       # EGO 全局静态内存数组的 Y 轴物理边界大小 (米)
    map_size_z = LaunchConfiguration('map_size_z', default=7.0)         # EGO 全局静态内存数组的 Z 轴物理边界大小 (米)
    cx = LaunchConfiguration('cx', default=200.0)                       # 默认内参改为 AirSim 配置 (W400, H300, FOV90)
    cy = LaunchConfiguration('cy', default=150.0)
    fx = LaunchConfiguration('fx', default=200.0)
    fy = LaunchConfiguration('fy', default=200.0)
    flight_type = LaunchConfiguration('flight_type', default=1)         # 飞行模式：1为目标点飞行，2为预设航点巡航

    # ============================= 局部地图参数参数 ================================
    # 局部更新范围，ego在此范围内规划路径
    resolution = 0.1                        # 局部地图和全局地图分辨率(米)。这个值越小，精度越高，占用内存空间越多。
    local_update_range_x = 22.0              # 局部地图X轴长度(米)。local_update_range_x除以 resolution必须小于250，否则需要修改planner_manager.cpp中initGridMap(grid_map_, Eigen::Vector3i(250, 250, 250))
    local_update_range_y = 22.0              # 局部地图Y轴长度(米)。local_update_range_y除以 resolution必须小于250
    local_update_range_z = 7.0              # 局部地图Z轴长度(米)。local_update_range_z除以 resolution必须小于250
    local_map_margin = 10                   # 局部地图的边界余量
    # 在文件 planner_manager.cpp 中调用函数 initGridMap(grid_map_, Eigen::Vector3i(250, 250, 250)) 设置了局部地图最大占据 250x250x250 个格子
    # X轴占据的格子数等于 local_update_range_x / resolution + local_map_margin * 2 ，上面的配置算出来是 23/0.1 + 10*2 = 250，正好没有超过。
    # Y和Z轴也是同样的算法。X和Y比较临界，有点危险，考虑将initGridMap设置为270x270x270。局部地图的格子使用结构struct GridNode定义，其大小是48个字节。
    # 因此修改后的局部地图总共占用 250x250x250x48 = 715M字节左右
    # 全局地图也是类似的计算方法，只有没有最后的margin，并且一个格子使用的内存空间要小很多。

    # ============================= 路径规划参数 ================================
    # 速度和加速度相关配置
    manager_max_vel = 3.0                              # 规划器最大速度
    manager_max_acc = 3.0                              # 规划器最大加速度
    optimization_max_vel = manager_max_vel - 0.0        # 优化器最大速度
    optimization_max_acc = manager_max_acc - 0.0        # 优化器最大加速度
    bspline_limit_vel = manager_max_vel                 # B样条速度硬性限制
    bspline_limit_acc = manager_max_acc                 # B样条加速度硬性限制
    bspline_limit_ratio = 1.1                           # B样条速度/加速度松弛比例（允许瞬间超出限制的 10%）

    # 最大加加速度 (Jerk, m/s^3)，控制平滑度，越大飞机加速度变化的越快，也就是飞机角度改变的越猛。在PX4中看到的上限是8.0，为了适应高速转向，设置为6.0
    # manager_max_acc / manager_max_jerk 即可得到无人机从0加速度到最大加速度所需的时间
    manager_max_jerk = 7.0

    # 动力学可行性检查的容差。manager 实际将 manager_max_vel 和 manager_max_acc 乘以 1+feasibility_tolerance 后，再来判定速度和加速度是否合法
    # 目前PX4最大速度是12m/s，最大加速度是5m/s^2。而目前设置的manager_max_vel=5，manager_max_acc = 4.5
    # 所以速度最大溢出是 (12-5) / 5 > 100%，加速度最大溢出是 (5-4.5) / 4.5 = 1/9 = 11.1%，所以这个值不能超过0.11
    # 因此先在 0.08 ~ 0.10 之间进行调试
    feasibility_tolerance = 0.0

    # 规划路径长度，时间，分割等配置
    planning_horizon = local_update_range_x/2 - 0.5       # 每次 A* 搜索和轨迹优化的最大前瞻距离(米) (必须小于局部地图x和y的大小）
    planning_horizen_time = -1.0                         # 每次局部规划的前瞻时间（秒）
    thresh_replan_time = 1.0 / manager_max_vel          # 触发重规划的时间阈值（当前轨迹执行时间超过此值且未达终点时重规划），速度越快该值越小。也就是 max_vel * fsm_thresh_replan_time 最好在1到2米之间
    emergency_time = 0.0                                # 紧急停止/避障的反应时间。该时间乘以 manager_max_acc 必须大于 manager_max_vel，并且要有余量，这样才能规划出一条合理的急停轨迹。
    control_points_distance = 0.1 * manager_max_vel     # B 样条控制点之间的平均间距 (米)。它除以manager_max_vel实际上代表控制点的时间间隔，这个间隔最好在0.1~0.3之间。这里取得0.18，也就是大约每 0.18 秒的飞行距离放置一个控制点

    # 膨胀和排斥配置
    airsim_radius = 0.6                                 # 实际测量出的AirSim无人机半径（米）
    obstacles_inflation = airsim_radius+0.05             # 障碍物膨胀半径（米），通常略大于无人机半径。该值越小，无人机可以越贴近障碍物，配合极小的速度设置，能够在极端复杂的环境中缓慢前行
    optimization_dist0 = obstacles_inflation * 1.5      # 安全距离余量（当轨迹距离障碍物小于此值时，产生排斥力），比膨胀半径要大。如果碰撞会导致极其严重的后果，则需要设置为远大于膨胀半径

    # 前瞻时间设置
    traj_time_forward = 0.7

    # 规划多条路线，选择最优
    use_distinctive_trajs = True

    # 重要权重设置
    lambda_smooth = 1.0
    lambda_collision = 5.0
    lambda_feasibility = 1.0
    lambda_fitness = 1.0

    # 深度图滤波
    use_depth_filter = True                             # 启用深度图滤波。
    depth_filter_tolerance = 0.3                        # 滤波容差
    depth_filter_maxdist = local_update_range_x/2 - 0.1   # 深度图有效最大距离（超过此值的数据截断）
    depth_filter_mindist = 0.2                          # 深度图有效最小距离（过滤机身遮挡或近处噪点）

    # 光线投射
    max_ray_length = local_update_range_x/2 - 0.2         # 光线投射最大长度
    min_ray_length = 0.1                                # 光线投射起点偏移（防自身遮挡）


    # Ego Planner Node 定义
    ego_planner_node = Node(
        package='ego_planner',             # 节点所在的 ROS 2 功能包名称
        executable='ego_planner_node',     # 可执行文件的名称
        name=['drone_', drone_id, '_ego_planner_node'], # 动态生成节点名，例如 'drone_0_ego_planner_node'，便于多机协同
        output='screen',                   # 将节点的标准输出打印到终端屏幕
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
        parameters=[
            # ========== 深度相机到机身的外参矩阵 (Extrinsic) ==========
            # 平移: 相机在 X 轴(正前方) 0.4 米处
            {'grid_map/extrinsic_T': [0.4, 0.0, 0.0]},

            # 旋转: 将光学坐标系(Z前, X右, Y下) 映射为 机身坐标系(X前, Y左, Z上)
            # Row 1: 光学 Z 映射为 机身 X (1.0)
            # Row 2: 光学 X 映射为 机身 -Y (-1.0)
            # Row 3: 光学 Y 映射为 机身 -Z (-1.0)
            {'grid_map/extrinsic_R': [ 0.0,  0.0,  1.0,
                                      -1.0,  0.0,  0.0,
                                       0.0, -1.0,  0.0]},

            # ========== FSM (有限状态机) 参数 ==========
            {'fsm/flight_type': flight_type},                           # 飞行模式：1为目标点飞行，2为预设航点巡航
            {'fsm/thresh_replan_time': thresh_replan_time},             # 触发重规划的时间阈值（当前轨迹执行时间超过此值且未达终点时重规划）
            {'fsm/thresh_no_replan_meter': 1.0},                        # 距离终点小于此距离（米）时，不再进行重规划
            {'fsm/planning_horizon': planning_horizon},                 # 每次局部规划的前瞻距离（米），每次 A* 搜索和轨迹优化的最大前瞻距离(米) (必须小于局部地图大小)
            {'fsm/planning_horizen_time': planning_horizen_time},       # 每次局部规划的前瞻时间（秒）
            {'fsm/emergency_time': emergency_time},                     # 紧急停止/避障的反应时间
            {'fsm/realworld_experiment': False},                        # 是否为真实世界实验（True 时会加入一些针对真机的安全冗余）
            {'fsm/fail_safe': False},                                    # 开启故障安全模式（规划失败时悬停）

            # 预设航点参数 (flight_type=2 时使用)
            {'fsm/waypoint_num': 0},           # 航点总数。下面是每个航点的具体坐标值。目前设置了5个航点，可以根据需求增加或删除。
            {'fsm/waypoint0_x': 0.0}, {'fsm/waypoint0_y': 0.0}, {'fsm/waypoint0_z': 0.0}, # 航点 0 坐标
            {'fsm/waypoint1_x': 0.0}, {'fsm/waypoint1_y': 0.0}, {'fsm/waypoint1_z': 0.0}, # 航点 1 坐标
            {'fsm/waypoint2_x': 0.0}, {'fsm/waypoint2_y': 0.0}, {'fsm/waypoint2_z': 0.0}, # 航点 2 坐标
            {'fsm/waypoint3_x': 0.0}, {'fsm/waypoint3_y': 0.0}, {'fsm/waypoint3_z': 0.0}, # 航点 3 坐标
            {'fsm/waypoint4_x': 0.0}, {'fsm/waypoint4_y': 0.0}, {'fsm/waypoint4_z': 0.0}, # 航点 4 坐标

            # ========== Grid Map (占据栅格地图) 参数 ==========
            {'grid_map/resolution': resolution},       # 栅格地图分辨率（0.1米/格）
            {'grid_map/map_size_x': map_size_x},       # 全局地图X大小
            {'grid_map/map_size_y': map_size_y},       # 全局地图Y大小
            {'grid_map/map_size_z': map_size_z},       # 全局地图Z大小
            {'grid_map/local_update_range_x': local_update_range_x},    # 局部地图在X轴的更新范围（米）
            {'grid_map/local_update_range_y': local_update_range_y},    # 局部地图在Y轴的更新范围（米）
            {'grid_map/local_update_range_z': local_update_range_z},    # 局部地图在Z轴的更新范围（米）
            {'grid_map/obstacles_inflation': obstacles_inflation},      # 障碍物膨胀半径（米），通常略大于无人机半径。实测airsim无人机半径0.6米
            {'grid_map/local_map_margin': local_map_margin},            # 局部地图的边界余量
            {'grid_map/ground_height': 0.02},           # 地面高度（小于此高度的所有物体都被忽略，也就是说必须保证飞行的最低高度大于这个值）

            # 相机内参匹配 AirSim 配置 (W400, H300, FOV90)
            {'grid_map/cx': cx},
            {'grid_map/cy': cy},
            {'grid_map/fx': fx},
            {'grid_map/fy': fy},

            # 深度图滤波去噪参数
            {'grid_map/use_depth_filter': use_depth_filter},                # 启用深度图滤波
            {'grid_map/depth_filter_tolerance': depth_filter_tolerance},    # 滤波容差
            {'grid_map/depth_filter_maxdist': depth_filter_maxdist},        # 深度图有效最大距离（超过此值的数据截断）
            {'grid_map/depth_filter_mindist': depth_filter_mindist},        # 深度图有效最小距离（过滤机身遮挡或近处噪点）
            {'grid_map/depth_filter_margin': 2},                            # 图像边缘过滤像素范围
            {'grid_map/k_depth_scaling_factor': 1.0},                       # 真实相机深度单位通常是毫米，需设为1000缩小为米。AirSim深度相图单位就是米，所以设为1.0
            {'grid_map/skip_pixel': 2},                                     # 深度图降采样步长（跳过2个像素处理1个，降低计算量）

            # ========== 局部地图融合与光线投射 (Raycasting) 概率参数 ==========
            # 使用对数几率(Log-odds)更新栅格概率
            {'grid_map/p_hit': 0.85},                   # 击中障碍物时的概率增量
            {'grid_map/p_miss': 0.15},                  # 未击中（空闲）时的概率减量
            {'grid_map/p_min': 0.12},                   # 栅格最小概率下限
            {'grid_map/p_max': 0.90},                   # 栅格最大概率上限
            {'grid_map/p_occ': 0.80},                   # 判定栅格为"占据"的概率阈值
            {'grid_map/min_ray_length': min_ray_length},# 光线投射起点偏移（防自身遮挡）
            {'grid_map/max_ray_length': max_ray_length},# 光线投射最大长度

            # 飞行包线与高度限制
            # 【重要修改】：虚拟天花板。超过这个高度会被强制视为有障碍物，防止飞机飞出边界或由于屋顶没有点云而无限上升。
            {'grid_map/virtual_ceil_height': 10.0},
            {'grid_map/visualization_truncate_height': 10.0}, # 可视化时的截断高度
            {'grid_map/show_occ_time': True},          # 是否打印地图占据更新的耗时

            # 【重要修改】：相机位姿获取方式。2 表示不依赖单独的 pose 话题，而是通过 TF 树和里程计推算相机相对世界的位姿
            {'grid_map/pose_type': 2},
            {'grid_map/frame_id': "world"},             # 全局参考坐标系

            # ========== 规划器管理器 (Planner Manager) 运动学限制 ==========
            # 【动力学约束】：限制 EGO 规划出的轨迹不能超过飞机的物理极限。该极限由PX4和airsim共同决定。
             # 后续考虑减小airsim无人机的质量，让无人机能够做出快速变向
            {'manager/max_vel': manager_max_vel},       # 全局最大速度限制 (m/s)
            {'manager/max_acc': manager_max_acc},       # 全局最大加速度限制 (m/s^2)
            {'manager/max_jerk': manager_max_jerk},     # 最大加加速度 (Jerk, m/s^3)，控制平滑度
            {'manager/control_points_distance': control_points_distance},   # B 样条控制点之间的平均间距 (米)
            {'manager/feasibility_tolerance': feasibility_tolerance},       # 动力学可行性检查的容差
            {'manager/planning_horizon': planning_horizon},# 管理器前瞻距离
            {'manager/use_distinctive_trajs': use_distinctive_trajs},   # 是否生成拓扑不同的备选轨迹（用于应对局部死胡同），开启后遇到障碍物时，算法会同时尝试寻找多条不同的绕行路线并选最优。关闭后只会规划一条路线。
            {'manager/drone_id': drone_id},             # 飞机ID

            # ========== 轨迹优化 (Trajectory Optimization) 权重参数 ==========
            {'optimization/lambda_smooth': lambda_smooth},          # 平滑项权重（惩罚过大的速度、加速度、Jerk）
            {'optimization/lambda_collision': lambda_collision},    # 避障项权重（惩罚轨迹侵入障碍物膨胀区）
            {'optimization/lambda_feasibility': lambda_feasibility},# 动力学可行性权重（惩罚超速、超加速度）
            {'optimization/lambda_fitness': lambda_fitness},        # 目标牵引权重（保证轨迹不要偏离初始 A* 路径太远）
            {'optimization/dist0': optimization_dist0}, # 安全距离余量（当轨迹距离障碍物小于此值时，产生排斥力）
            {'optimization/swarm_clearance': 0.5},      # 多机集群时的机间安全避让距离
            {'optimization/max_vel': optimization_max_vel},# 优化器使用的最大速度
            {'optimization/max_acc': optimization_max_acc},# 优化器使用的最大加速度

            # ========== B-Spline (B样条) 参数 ==========
            {'bspline/limit_vel': bspline_limit_vel},   # B样条速度硬约束
            {'bspline/limit_acc': bspline_limit_acc},   # B样条加速度硬约束
            {'bspline/limit_ratio': bspline_limit_ratio},# 速度/加速度松弛比例（允许瞬间超出限制的 10%）

            # ========== 动态障碍物预测参数 ==========
            {'prediction/obj_num': 0},                  # 集群避障预测参数：假设环境中最多有多少个动态障碍物/其他无人机。单无人机静态地图，可设置为0
            {'prediction/lambda': 1.0},                 # 预测衰减系数
            {'prediction/predict_rate': 1.0}            # 预测频率
        ]
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

        parameters=[
            # 前瞻时间 (秒)
            {'traj_server/time_forward': traj_time_forward}
        ]
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(ego_planner_node)
    ld.add_action(traj_server_node)

    return ld