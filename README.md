# FLYLAB —— PX4 + AirSim SITL 无人机仿真实验平台

WSL2 (Ubuntu 22.04) + ROS2 Humble 仿真环境，围绕 PX4 + Cosys-AirSim 做无人机自主飞行研究：感知建图、视觉惯导、避障规划与 offboard 控制全链路打通。

## 工作区总体

| 目录 | 说明 |
|---|---|
| `ros2_ws/` | ROS2 工作空间，核心是 `src/flylab` 包，其余为各算法移植包  |
| `Cosys-AirSim/` | UE 端仿真器 |
| `px4.1.15.2/` | PX4 SITL v1.15.2 源码（机型 `none_iris`） |
| `ORB_SLAM3_FIXED/` | ORB-SLAM3 运行时库 |
| `mavros/` `libmavconn/` `mavros_msgs/` | MAVROS 通信栈 |
| `scripts/env.sh` | 启动脚本与环境变量 |

---

## flylab 核心包

### 功能链路一览

| 链路 | 一键启动 | 场景配置 | 说明 |
|---|---|---|---|
| 感知建图 | `ros2 launch flylab depth_mapping.launch.py` | `configs/mapping.json` | 深度图 → 点云 → 占据地图 |
| EGO 避障飞行 | `ros2 launch flylab fly_ego.launch.py` | `configs/ego_planner/` | RViz 点目标点 → EGO 规划 → PX4 跟踪，到达后自动交还 QGC；`tracker:=mavros\|px4` 选择跟踪链路 |
| OpenVINS MSCKF | `ros2 launch flylab openvins_msckf.launch.py` | `configs/openvins/` + `settings.json` | 双目 VIO，断桨联动启停 |
| VINS 单目/双目 | `ros2 launch flylab vins_mono.launch.py` / `vins_stereo.launch.py` | `configs/vins_*/` | VINS-Fusion 航迹 |
| ORB-SLAM3 | `ros2 launch flylab orb_slam3.launch.py` | `configs/orb_slam3/` | 双目+IMU SLAM |
| FAST-LIO 建图 | `ros2 launch flylab fast_lio.launch.py` | `configs/fast_lio/` | LiDAR-惯性里程计（GPULidar，参数对齐 Livox Mid-360） |
| SUPER 避障飞行 | `ros2 launch flylab fly_super.launch.py` | `configs/super_planner/` | RViz 点目标点 → SUPER 规划（ROG-Map 占据地图）→ 桥接 → PX4 跟踪；与 EGO 共用 tracker 链路，`tracker:=mavros\|px4` 选择跟踪链路 |

### C++ 节点（airsim 传感器可插拔架构 + control = 飞行控制）

传感器层 (`src/sensors/`) 与算法完全解耦：每种传感器一份代码，哪个算法需要什么数据就由其 launch 启动对应实例；算法特化的适配（坐标系/门控/时间对齐）下沉到桥接层 (`src/bridges/`)，纯 ROS 实现、不依赖 AirSim。

| 传感器节点 | 发布话题 | 说明 |
|---|---|---|
| `airsim_camera` | `/airsim/cam/<名>/image_raw` | 通用相机（深度/双目/单目参数化），多相机同次 RPC 硬件同步；`grayscale:=true` 输出 mono8；`image_type` 选 AirSim 图像类型 |
| `airsim_imu` | `/airsim/imu/data_raw`（FRD 原生）<br>`/airsim/imu/data`（FLU 标准） | 200Hz；时间戳统一映射仿真时钟；重复采样去重；`inject_bias` 可选偏置注入 |
| `airsim_odom` | `/airsim/odom`（world 系 ENU）+ TF | 真值里程计 100Hz；时间戳与 LiDAR/IMU 同一时间轴（点云位姿插值的前提） |
| `airsim_nav` | `/airsim/gps` `/airsim/baro` `/airsim/mag` | 低频导航传感器 50Hz |
| `airsim_lidar` | `/airsim/lidar/points_raw` | 原始点云（传感器系 NED）+ 逐点时间模型（CPU 快照 / GPU 段级真实渲染时刻），坐标变换交给桥接 |

| 桥接节点 | 输入 → 输出 | 说明 |
|---|---|---|
| `bridge_vio` | 相机+原始 IMU → `/vio/*` | OpenVINS 断桨门控：降落停发、解锁恢复；数据原样转发 |
| `bridge_fastlio` | `points_raw` → `/airsim/lidar/points` | NED→FLU + 雷达挂载外参静态 TF（FAST-LIO 链路） |
| `bridge_super` | `points_raw`+odom → `/cloud_registered` `/lidar_slam/odom` | 点云→world 系 ENU：位姿按点云时刻插值对齐 + 逐点去畸变 + 机身回波过滤（SUPER 链路） |

其他节点：`tracker_px4`/`tracker_mavros`（轨迹跟踪+PX4 混控，`tracker:=mavros\|px4` 选择链路）、`super_cmd_bridge`（mars→标准 PositionCommand）、`octomap_dynamic`（建图动态清理）、`vins_arming_switch`（VINS 起飞联动）。

### 环境依赖

| 依赖 | 说明 |
|---|---|
| ROS2 Humble + PX4 SITL v1.15.2 | PX4 消息经 MicroXRCEAgent（`udp4 -p 8888`）桥接 |
| Cosys-AirSim 5.8 | UE 端仿真器，场景按链路选用 `configs/` 下的 json |
| ego_planner / ov_msckf / orbslam3 / fast_lio / super_planner | 工作空间内移植的算法包（fast_lio 依赖 livox_ros_driver2） |

---

## 快速开始

```bash
# 1. 编译
source /opt/ros/humble/setup.bash
export MAKEFLAGS=-j2                    # WSL 下需限制并行度
cd ~/flylab/ros2_ws && colcon build --parallel-workers 2
source install/setup.bash

# 2. 启动仿真环境
#    UE/AirSim 场景配置: 把 configs/ 下对应 json 复制到 Windows 端 Documents\AirSim\settings.json 后重启 UE
bash /root/flylab/scripts/run-px4.sh        # PX4 SITL
MicroXRCEAgent udp4 -p 8888                 # PX4 <-> ROS2 桥 (tracker:=px4 链路需要)

# 3. 启动链路（跨主机连接 AirSim 需先 export PX4_SIM_HOST_ADDR=<AirSim IP>）
ros2 launch flylab fly_ego.launch.py                 # EGO 避障 (默认 uXRCE-DDS 跟踪)
ros2 launch flylab fly_ego.launch.py tracker:=mavros # EGO 避障 (MAVROS 跟踪, 用专属端口 14542)
ros2 launch flylab fly_super.launch.py               # SUPER 避障 (GPULidar, 需 configs/fast_lio/settings.json)
ros2 launch flylab fast_lio.launch.py                # FAST-LIO 建图 (GPULidar)
ros2 launch flylab openvins_msckf.launch.py          # OpenVINS 双目 VIO
```

---

## flylab_tools 视觉算法实验区

独立小工具（`ros2_ws/src/flylab_tools`），**直连 AirSim RPC** 实时查看任意相机画面，不依赖 ROS 链路/传感器节点，与算法链路互不干扰。

- **按需拉取**：每帧只向 AirSim 请求当前选中的那 1 路相机（30Hz），切换下拉框即时生效，其余相机零开销
- **深度伪彩**：名字含 `depth` 的相机自动伪彩显示（红近蓝远，0.2~20m）
- **断线重连**：UE 重启后自动接上；只保留最新帧，不积压
- 前提：相机需在 UE 端 `settings.json` 中配置过（`configs/settings.json` 已含全部相机）

```bash
ros2 run flylab_tools camera_view                                # 本机 UE / 已设 PX4_SIM_HOST_ADDR
ros2 run flylab_tools camera_view --ros-args -p host:=<UE主机IP>  # 跨主机指定
```

依赖：`pip install msgpack-rpc-python`；AirSim 客户端复用 `Cosys-AirSim/PythonClient`。

## SITL 链路与端口分配（重要踩坑记录）

PX4 SITL + Cosys-AirSim + ROS2 之间的全部通信端口（配置位于 `px4.1.15.2/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink` 与 Windows 端 `settings.json`）：

| 端口 | 用途 | 对端 |
|---|---|---|
| TCP 4560 | HIL 物理仿真链路（传感器→PX4，执行器→UE） | UnrealEditor（Windows） |
| UDP 14580→14540 | onboard 实例 | **被 AirSim 控制通道抢占**（见下） |
| UDP 14582→14542 | **MAVROS 专属实例**（本次新增） | MAVROS `fcu_url:=udp://:14542@` |
| UDP 18570→14550 | GCS 实例 | QGC 等 |
| UDP 8888 | uXRCE-DDS（`tracker:=px4` 链路） | MicroXRCEAgent |
| TCP 41451 | AirSim RPC API（传感器/相机直连） | airsim_sensor_* / flylab_tools |

**坑 ：14580 onboard 实例会被 AirSim 抢占。** 即使 `settings.json` 里 `UseTcp: true`，Cosys-AirSim 仍会从 Windows 向 `ControlIp:ControlPortRemote`（即 WSL 的 14580）发 1 Hz 心跳；PX4 的 MAVLink 实例只锁定一次对端（`mavlink_receiver.cpp`），AirSim 先到先得，把整条链路的数据流全部导向 UE，导致 MAVROS `/mavros/state.connected` 永远为 false。因此 MAVROS 必须使用 14582↔14542 专属实例，不要动 14580。

## 注意事项

- **GPULidar 与 CPU 雷达配置**：`configs/fast_lio/settings.json` 为 GPU 雷达（`SensorType: 8`，参数对齐 Livox Mid-360），FAST-LIO 与 SUPER 链路共用，对应 launch 默认 `gpu_lidar:=true`；`configs/settings.json` 为 CPU 雷达（`SensorType: 6`，**必须带 `LimitPoints: false`**，否则每 tick 扫描被 5000 点上限截断，一圈分片采样导致帧内拼接错位且实际帧率掉到 ~1.7Hz）。仿真无法复现 Mid-360 的非重复菱形扫描，以每帧均匀网格 ~2 万点近似
- **雷达时间戳约定**：AirSim 点云是单时刻瞬时快照（整帧同一位姿一次生成，无扫描内运动畸变），传感器节点统一填 `time = 1/rotation_hz` 并以 `LidarData.time_stamp` 标定 ROS 偏移，LiDAR/IMU 映射到同一时间轴；逐点合成时间会反向注入运动剪切导致平移漂移，勿改回
- **MSCKF 初始化**：静止时不会初始化，起飞后让无人机平移/转动（视差 ≥15、陀螺变化 ≥7°）即可
- **链路状态颜色**：黄色日志 = 飞控数据未收到/断链，绿色 = 链路正常/到达目标点
- **airsim_ros_pkgs 为官方原版**：话题为官方命名（如 `PX4/left_cam_Scene/image`、`PX4/odom_local`），数据为 ENU/FLU 语义，世界坐标系 `world`。
- 各节点详细话题/参数说明见 `ros2_ws/src/flylab/src/`（`sensors/`、`bridges/`、`control/`）下各 `.cpp` 文件头注释
- **传感器时间轴约定**：`airsim_imu`/`airsim_lidar`/`airsim_odom` 的时间戳统一经 `SimClockSync` 映射（ros = offset + 仿真时间，见 `src/sensors/airsim_common.hpp`），三路数据落同一时间轴；跨节点混用（如 bridge_super 对点云插值 odom 位姿、FAST-LIO 融合雷达+IMU）都依赖这一约定，自定义新传感器节点请保持
