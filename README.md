# FLYLAB —— PX4 + AirSim SITL 无人机仿真实验平台

WSL2 (Ubuntu 22.04) + ROS2 Humble 仿真环境，围绕 PX4 + Cosys-AirSim 做无人机自主飞行研究：感知建图、视觉惯导（VINS / OpenVINS / ORB-SLAM3）、避障规划（EGO-Planner）与 offboard 控制全链路打通。

## 工作区总体

| 目录 | 说明 |
|---|---|
| `ros2_ws/` | ROS2 工作空间，**核心是 `src/flylab` 包**，其余为各算法移植包（openvins / vins / orb_slam3 / planner / mavros 等） |
| `Cosys-AirSim/` | UE 端仿真器（Cosys fork，含深度相机/激光雷达支持） |
| `px4.1.15.2/` | PX4 SITL v1.15.2 源码（机型 `none_iris`） |
| `ORB_SLAM3_FIXED/` | ORB-SLAM3 运行时库 |
| `mavros/` `libmavconn/` `mavros_msgs/` | MAVROS 通信栈 |
| `scripts/env.sh` | 启动脚本与环境变量 |

---

## flylab 核心包

### 功能链路一览

| 链路 | 一键启动 | 场景配置 | 说明 |
|---|---|---|---|
| 仿真飞行 | `ros2 launch flylab sim_fly.launch.py` | `configs/fly.json` | 基础场景：手动 offboard 速度控制 |
| 感知建图 | `ros2 launch flylab depth_mapping.launch.py` | `configs/mapping.json` | 深度图 → 点云 → 占据地图 |
| EGO 避障飞行 | `ros2 launch flylab fly_ego.launch.py` | `configs/ego_planner/` | RViz 点目标点 → EGO 规划 → PX4 跟踪，到达后自动交还 QGC |
| OpenVINS MSCKF | `ros2 launch flylab openvins_msckf.launch.py` | `configs/openvins/` + `settings.json` | 双目 VIO，断桨联动启停 |
| VINS 单目/双目 | `ros2 launch flylab vins_mono.launch.py` / `vins_stereo.launch.py` | `configs/vins_*/` | VINS-Fusion 航迹 |
| ORB-SLAM3 | `ros2 launch flylab orb_slam3.launch.py` | `configs/orb_slam3/` | 双目+IMU SLAM |
| FAST-LIO 建图 | `ros2 launch flylab fast_lio.launch.py` | `configs/fast_lio/` | LiDAR-惯性里程计（16 线仿真雷达），`/map_save` 保存 PCD |

### C++ 节点（flylab/src）

| 节点 | 功能 |
|---|---|
| `airsim_sensor_ego` | EGO 链路传感器：深度图 + IMU + 真值里程计 + GPS/气压计/磁力计 + TF |
| `airsim_sensor_vio` | VIO 链路传感器：双目相机 + IMU；断桨自动停发、解锁自动恢复 |
| `airsim_sensor_lidar` | FAST-LIO 链路传感器：3D LiDAR（按方位角合成逐点时间戳）+ IMU，NED→FLU |
| `px4_ego_tracker` | EGO 轨迹跟踪 + PX4 混控，链路状态彩色提示，断链自动检测（默认 3s） |
| `move_velocity` | 通用 offboard 速度控制器（Twist 输入） |
| `octomap_dynamic` | 建图链路 octomap 动态清理 |
| `vins_arming_switch` | VINS-Fusion 起飞联动开关 |

### 环境依赖

| 依赖 | 说明 |
|---|---|
| ROS2 Humble + PX4 SITL v1.15.2 | PX4 消息经 MicroXRCEAgent（`udp4 -p 8888`）桥接 |
| Cosys-AirSim 5.8 | UE 端仿真器，场景按链路选用 `configs/` 下的 json |
| ego_planner / ov_msckf / orbslam3 / fast_lio | 工作空间内移植的算法包（fast_lio 依赖 livox_ros_driver2，需先 source） |

---

## 快速开始

```bash
# 1. 编译
source /opt/ros/humble/setup.bash
export MAKEFLAGS=-j2                    # WSL 下必须限制并行度
cd ~/flylab/ros2_ws && colcon build --parallel-workers 2
source install/setup.bash

# 2. 启动仿真环境
#    UE/AirSim 场景配置: 把 configs/ 下对应 json 复制到 Windows 端 Documents\AirSim\settings.json 后重启 UE
bash /root/flylab/scripts/run-px4.sh        # PX4 SITL
MicroXRCEAgent udp4 -p 8888                        # PX4 <-> ROS2 桥

# 3. 启动链路（跨主机连接 AirSim 需先 export PX4_SIM_HOST_ADDR=<AirSim IP>）
ros2 launch flylab fly_ego.launch.py        # EGO 避障
ros2 launch flylab openvins_msckf.launch.py # OpenVINS 双目 VIO
```

## 注意事项

- **MSCKF 初始化**：静止时不会初始化，起飞后让无人机平移/转动（视差 ≥15、陀螺变化 ≥7°）即可
- **链路状态颜色**：黄色日志 = 飞控数据未收到/断链，绿色 = 链路正常/到达目标点
- **airsim_ros_pkgs 为官方原版**：话题为官方命名（如 `PX4/left_cam_Scene/image`、`PX4/odom_local`），数据为 ENU/FLU 语义，世界坐标系 `world`；旧课程话题命名（`world_ned`、`PX4/odometry`）不再有效。两个包的本地改动仅有：`AIRSIM_ROOT` 绝对路径、Humble 的 `cv_bridge.h` 头文件、launch 默认参数（`PX4_SIM_HOST_ADDR` 环境变量 / `is_vulkan:=False`）
- 各节点详细话题/参数说明见 `ros2_ws/src/flylab/src/*.cpp` 文件头注释
