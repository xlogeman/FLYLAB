# flylab

PX4 + AirSim SITL 无人机仿真工具包。从 [ytjy-sitl] 课程体系（hw_insight / hw_interface）逐步解构移植而来，用 C++ 重写、去除了原作者的个人化封装，形成一个结构清晰、可分享、可持续扩展的自主实验项目。

## 目录结构

```bash
flylab/
├── CMakeLists.txt
├── package.xml
├── configs/      # 各链路的仿真与算法参数
│   ├── fly.json        # sim_fly 用：全套相机+雷达
│   ├── mapping.json    # 建图链路用：仅 CameraDepth 深度相机
│   ├── settings.json   # 双目基础配置（left_cam/right_cam）
│   ├── vins_mono/      # vins_mono 用：AirSim settings.json + VINS 参数 + 相机内参
│   ├── vins_stereo/    # vins_stereo 用：VINS 双目外参 + 内参
│   ├── orb_slam3/      # orb_slam3 用：ORB-SLAM3 双目+IMU 参数
│   ├── ego_planner/    # EGO 避障链路：ego_planner.yaml（规划器全参数）+ settings.json（AirSim 深度相机配置）
│   └── openvins/       # OpenVINS MSCKF 链路：estimator_config.yaml（估计器主参数）+ kalibr IMU 链/相机-IMU 外参
│                        # 注意: estimator_config.yaml 中 relative_config_imu / relative_config_imucam
│                        #       为同目录相对路径, 3 个文件必须保持在同一目录
├── src/          # C++ 节点
│   ├── move_velocity.cpp        # 通用 PX4 offboard 速度控制器（Twist 输入）
│   ├── octomap_dynamic.cpp      # octomap 动态清理（建图链路）
│   ├── vins_arming_switch.cpp   # VINS 起飞联动开关
│   ├── airsim_sensor_ego.cpp   # AirSim 直连传感器节点（EGO 链路: 深度图+真值里程计+IMU）
│   ├── airsim_sensor_vio.cpp   # AirSim 直连传感器节点（VIO 链路: 双目相机+IMU, 供 OpenVINS/VINS）
│   └── px4_ego_tracker.cpp      # EGO 轨迹跟踪 + PX4 混控节点（EGO 链路）
├── log/          # VINS 运行输出（vio.csv / pose_graph/），不入库
├── launch/       # 按功能域分目录（ros2 launch 会递归搜索）
│   ├── fly/            # 飞行控制类
│   │   └── sim_fly.launch.py            # 仿真飞行基础场景：airsim_node + 手动 offboard 速度控制 + 双 rviz
│   ├── mapping/        # 感知建图类
│   │   └── depth_mapping.launch.py      # 感知建图完整链路：深度图 → 点云 → 占据地图 + rviz
│   │   └── depth_to_cloud.launch.py     # 分段调试：仅 深度图→点云
│   │   └── cloud_to_map.launch.py       # 分段调试：仅 点云→占据地图
│   │   └── dynamic_mapping.launch.py    # 完整建图 + octomap_dynamic 动态清理节点
│   │   └── colored_mapping.launch.py    # 彩色点云（xyzrgb）版建图
│   ├── ego_planner/    # EGO 避障飞行类
│   │   ├── ego_planner.launch.py        # EGO 规划器 + 轨迹服务器（参数来自 configs/ego_planner/ego_planner.yaml）
│   │   └── fly_ego.launch.py            # 一键启动整条 EGO 链路（传感器 + 规划 + 跟踪 + RViz）
│   ├── openvins/       # 视觉惯性里程计类（OpenVINS MSCKF，原课程 lesson10）
│   │   └── openvins_msckf.launch.py     # 一键启动 VIO 链路（airsim_sensor_vio + run_subscribe_msckf + RViz）
│   ├── vins_mono/      # 视觉惯性里程计类（单目）
│   │   └── vins_mono.launch.py
│   ├── vins_stereo/    # 视觉惯性里程计类（双目）
│   │   └── vins_stereo.launch.py
│   └── orb_slam3/      # SLAM 类
│       └── orb_slam3.launch.py
└── rviz/          # 与 launch/ 分目录对应
    ├── fly/
    │   ├── depth_cloud.rviz
    │   └── image_lidar.rviz
    ├── mapping/
    │   ├── depth_mapping.rviz
    │   └── colored_mapping.rviz
    ├── ego_planner/
    │   └── ego_planner.rviz
    ├── openvins/
    │   └── openvins_msckf.rviz
    ├── vins_mono/
    │   └── vins_mono.rviz
    ├── vins_stereo/
    │   └── vins_stereo.rviz
    └── orb_slam3/
        └── orb_slam3.rviz
```

## 环境依赖

| 依赖 | 说明 |
|---|---|
| ROS2 Humble | 基础框架 |
| px4_msgs | PX4 uXRCE-DDS 消息定义 |
| PX4 SITL | v1.15.2，机型 `none_iris` |
| AirSim (Cosys fork) | UE 端仿真；EGO 链路不依赖 `airsim_ros_pkgs`，由 `airsim_sensor_ego` 直连 RPC 取数 |
| MicroXRCEAgent | `udp4 -p 8888`，PX4 ↔ ROS2 DDS 桥 |
| ego_planner | EGO-Planner 规划器 ROS2 包（工作空间内移植版），提供 `ego_planner_node` / `traj_server` |
| quadrotor_msgs | EGO-Planner 轨迹消息 `PositionCommand` |
| ov_msckf (OpenVINS) | MSCKF 视觉惯导估计器，提供 `run_subscribe_msckf`（工作空间 `openvins/` 源码包，ov_init 有一处日志级别补丁） |
| orbslam3 + ORB_SLAM3 库 | `orbslam3` wrapper 包提供 `stereo-inertial` 可执行文件；运行时库在 `/root/fucker/ORB_SLAM3_FIXED`（`LD_LIBRARY_PATH` 见工作区 `env.sh`） |

## 编译 & 运行

```bash
source /opt/ros/humble/setup.bash
export MAKEFLAGS=-j2                        # WSL 下必须限制并行度
cd <workspace>                              # colcon 工作空间根目录
colcon build --parallel-workers 2
source install/setup.bash

# 先启动 UE/AirSim 场景。不同场景需要不同的传感器配置，见 configs/ 目录。把对应 json 复制为 Windows 端 Documents\AirSim\settings.json 后重启 UE

# 运行 px4
bash /root/fucker/scripts/bare/fucker-run-px4.sh

# 启动 ROS2 与 PX4 的通信代理
MicroXRCEAgent udp4 -p 8888
```

## 各链路一键启动

```bash
source install/setup.bash

# EGO 避障飞行链路（需深度相机场景配置）
ros2 launch flylab fly_ego.launch.py            # 可选: xrce:=true 同时拉起 MicroXRCEAgent, rviz:=false 关闭可视化

# OpenVINS MSCKF 视觉惯导链路（需双目相机场景配置，即 configs/openvins + settings.json）
# 前置: UE/AirSim 已启动、PX4 SITL 已运行；跨主机连接需 export PX4_SIM_HOST_ADDR=<AirSim IP>
ros2 launch flylab openvins_msckf.launch.py     # 可选: config_path:=<自定义 estimator_config.yaml 路径>

# 单独运行某个传感器/控制节点
ros2 run flylab airsim_sensor_ego               # EGO 链路传感器（深度图+真值里程计+IMU）
ros2 run flylab airsim_sensor_vio               # VIO 链路传感器（双目相机+IMU）
ros2 run flylab px4_ego_tracker                 # EGO 轨迹跟踪 + PX4 混控
```

MSCKF 初始化提示：启动后静止时终端可能短暂打印初始化等待信息，让无人机**平移/转动**（视差 ≥15、陀螺变化 ≥7°）即可完成初始化并输出里程计。

## 节点说明补充

- `px4_ego_tracker`：终端日志带颜色状态提示——黄色 = 未收到飞控数据 / 链路中断，绿色 = 链路正常 / 到达目标点。断链检测阈值由参数 `link_timeout_sec` 控制（默认 3 秒；节点启动后同样静默 3 秒再开始报警）：
  ```bash
  ros2 run flylab px4_ego_tracker --ros-args -p link_timeout_sec:=5.0
```



