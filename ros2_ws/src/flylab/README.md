# flylab

PX4 + AirSim SITL 无人机仿真工具包。从 [ytjy-sitl] 课程体系（hw_insight / hw_interface）逐步解构移植而来，用 C++ 重写、去除了原作者的个人化封装，目标是形成一个结构清晰、可分享、可持续扩展的自主项目。

## 特性

- **C++ 实现**：控制类节点全部基于 `rclcpp`，无 Python 运行时开销
- **标准消息接口**：不依赖自定义消息包，控制入口统一使用 `geometry_msgs/Twist`
- **QGC 友好**：节点不干预地面站的手动控制（POSCTL / AUTO_TAKEOFF 等状态下不发送任何指令）
- **坐标系透明**：对外统一 REP-103 惯例（FLU：x 前 y 左 z 上），内部自动转换 PX4 的 NED 系

## 环境依赖

| 依赖 | 说明 |
|---|---|
| ROS2 Humble | 基础框架 |
| px4_msgs | PX4 uXRCE-DDS 消息定义 |
| PX4 SITL | v1.15.2，机型 `none_iris` |
| AirSim (Cosys fork) | UE 端仿真，`airsim_ros_pkgs` 提供传感器数据通路 |
| MicroXRCEAgent | `udp4 -p 8888`，PX4 ↔ ROS2 DDS 桥 |
| orbslam3 + ORB_SLAM3 库 | `orbslam3` wrapper 包提供 `stereo-inertial` 可执行文件；运行时库在 `/root/fucker/ORB_SLAM3_FIXED`（`LD_LIBRARY_PATH` 见工作区 `env.sh`） |

## 编译

```bash
source /opt/ros/humble/setup.bash
export MAKEFLAGS=-j2                        # WSL 下必须限制并行度
cd <workspace>                              # colcon 工作空间根目录
colcon build --parallel-workers 2
source install/setup.bash
```

## 运行

```bash
#先启动 UE/AirSim 场景

# 运行 px4
bash /root/fucker/scripts/bare/fucker-run-px4.sh
# 或
make px4_sitl_default none_iris

# 启动 ROS2 与 PX4 的通信代理（好像不需要也能通）
MicroXRCEAgent udp4 -p 8888

ros2 launch flylab sim_fly.launch.py

ros2 launch flylab depth_mapping.launch.py

ros2 launch flylab orb_slam3.launch.py
```

> **AirSim 端配置**：不同场景需要不同的传感器配置，见 `configs/` 目录。把对应 json 复制为 Windows 端 `Documents\AirSim\settings.json` 后**重启 UE**（AirSim 只在启动时读取一次该文件）。

launch 内容（`sim_fly` = 仿真飞行基础场景，对应原课程 lesson3）：`airsim_node`（借用 airsim_ros_pkgs 原包）+ `move_velocity` + 两个 rviz（深度点云 / 相机+雷达）。

## launch 一览（按功能域分目录）

```
launch/
├── fly/          # 飞行控制类
│   └── sim_fly.launch.py
├── mapping/      # 感知建图类
│   ├── depth_mapping.launch.py
│   ├── depth_to_cloud.launch.py
│   ├── cloud_to_map.launch.py
│   ├── dynamic_mapping.launch.py
│   └── colored_mapping.launch.py
├── vins_mono/    # 视觉惯性里程计类
│   └── vins_mono.launch.py
├── vins_stereo/  # 视觉惯性里程计类（双目）
│   └── vins_stereo.launch.py
└── orb_slam3/    # SLAM 类
    └── orb_slam3.launch.py
```

| launch | 用途 |
|---|---|
| `fly/sim_fly.launch.py` | 仿真飞行基础场景：airsim_node + 手动 offboard 速度控制 + 双 rviz |
| `mapping/depth_mapping.launch.py` | 感知建图完整链路：深度图 →(depth_image_proc)→ 点云 →(octomap_server)→ 占据地图 + rviz |
| `mapping/depth_to_cloud.launch.py` | 分段调试：仅 深度图→点云 |
| `mapping/cloud_to_map.launch.py` | 分段调试：仅 点云→占据地图 |
| `mapping/dynamic_mapping.launch.py` | 完整建图 + `octomap_dynamic` 动态清理节点 |
| `mapping/colored_mapping.launch.py` | 彩色点云（xyzrgb）版建图 |
| `vins_mono/vins_mono.launch.py` | 单目 VIO + GPS 全局融合（原课程 lesson7），见下文 |
| `vins_stereo/vins_stereo.launch.py` | 双目 VIO + GPS 全局融合（原课程 lesson8），见下文 |
| `orb_slam3/orb_slam3.launch.py` | ORB-SLAM3 双目+IMU SLAM（原课程 lesson11），见下文 |

> `ros2 launch flylab <文件名>` 会递归搜索子目录，调用时**不需要带目录前缀**。

### vins_mono —— 单目 VIO + GPS 全局融合（原课程 lesson7）

VINS-Fusion 单目+IMU 里程计，`loop_fusion` 回环，`global_fusion` 融合降频 GPS，三色轨迹（绿=VIO / 红=回环 / 白=GPS 融合）。

**前置条件**：UE 场景已启动、PX4 SITL 已运行（`fucker-run-px4.sh`）、飞机起飞并有移动激励（VIO 初始化需要加速度激励，悬停不运动不初始化）。

```bash
ros2 launch flylab vins_mono.launch.py
```

**launch 内容**：`airsim_node`（借用 airsim_ros_pkgs）+ `topic_tools/throttle`（GPS 100Hz→5Hz）+ 静态 TF（world→map、map→PX4_odom、world_ned→world）+ `vins_node` / `loop_fusion_node` / `global_fusion_node` + `vins_arming_switch`（起飞联动，`use_stereo:=False` 单目模式，见 vins_stereo 小节）+ rviz。

**包内配置**（`configs/vins_mono/`，随包分发、自包含）：

| 文件 | 用途 |
|---|---|
| `euroc_mono_imu_config.yaml` | VINS 参数：话题、IMU 噪声、外参等 |
| `cam0_mei.yaml` | left_cam 内参（Mei 模型，400x300 FOV90） |
| `settings.json` | AirSim 端参数（双目相机定义 + IMU/GPS 传感器），**用本链路时复制到 Windows 端 `Documents\AirSim\` 后重启 UE** |

**关键参数（已按实际装机校准）**：

- 图像订阅 `/airsim_node/PX4/left_cam/Scene`（相机实际名是 `left_cam`，不是 `CameraImage`）
- 外参 `body_T_cam0`：平移 前 0.4m / 左 0.08m，旋转含 36.87° 下俯（sin=0.6, cos=0.8），NED 体坐标系
- `estimate_extrinsic: 0`（外参锁定）。若轨迹异常，可改为 `1` 让 VINS 在线微调外参兜底
- VIO 输出日志在 `flylab/log/`（`vio.csv`、`pose_graph/`），目录需已存在

### vins_stereo —— 双目 VIO + GPS 全局融合（原课程 lesson8）

VINS-Fusion 双目+IMU 里程计，`loop_fusion` 回环，`global_fusion` 融合 GPS，三色轨迹（绿=VIO / 红=回环 / 白=GPS 融合），比 vins_mono 多一个 `vins_arming_switch` 联动节点。

**前置条件**：与 vins_mono 相同（UE 场景、PX4 SITL、飞机起飞后有移动激励）。

```bash
ros2 launch flylab vins_stereo.launch.py
```

**launch 内容**：`airsim_node` + `topic_tools/throttle`（GPS→1Hz）+ 静态 TF（world_enu→world）+ `vins_node` / `loop_fusion_node` / `global_fusion_node` + `vins_arming_switch`（C++ 节点，见下）+ rviz。

**vins_arming_switch**：起飞/降落联动开关。监听 `/fmu/out/vehicle_status`，解锁时向 VINS 发送"开 IMU + 开相机 + 重启"，降落时发送"关 IMU + 关相机 + 重置"，保证 VINS 工作区间与飞行区间对齐、初始化状态干净。参数 `use_stereo` 控制起飞时相机开关的值（双目 true / 单目 false）。

**包内配置**（`configs/vins_stereo/`，随包分发、自包含）：

| 文件 | 用途 |
|---|---|
| `stereo_extrinsics.yaml` | VINS 参数：话题、IMU 噪声、双目外参（内参文件须与其同目录） |
| `stereo_intrinsics.yaml` | left_cam/right_cam 共用内参（PINHOLE，fx=fy=200，400x300 FOV90） |

双目端 AirSim 参数沿用 `configs/settings.json`（left_cam/right_cam 双目基础配置），复制到 Windows 端后重启 UE。

**关键参数（已按实际装机校准）**：

- 图像订阅 `/airsim_node/PX4/left_cam/Scene` 与 `/airsim_node/PX4/right_cam/Scene`
- 双目基线 0.16m（Y=∓0.08m），外参与 AirSim 端相机位姿（Pitch -36.87°）严格对应，`estimate_extrinsic: 0` 锁定

### orb_slam3 —— ORB-SLAM3 双目+IMU（原课程 lesson11）

ORB-SLAM3 特征点法双目+惯性 SLAM（2020 版，含 Atlas 多地图），输出位姿与稀疏点云地图，与 vins_stereo（优化式 VIO）、OpenVINS（滤波式）形成方案对比。

**前置条件**：UE 场景已启动（双目相机配置，沿用 `configs/settings.json`）、PX4 SITL 已运行、飞机有移动激励（静止时无视差，SLAM 无法初始化）。

```bash
ros2 launch flylab orb_slam3.launch.py
```

**launch 内容**：`ytjy_airsim_ros_topic`（airsim_ros_pkgs 内的 C++ 直连桥）+ `stereo-inertial`（orbslam3 包可执行文件）+ rviz。

> Pangolin 可视化窗口在该 ROS 节点中固定开启（`System` 构造参数写死为 true），launch 参数控制不了它；软件渲染下会多占资源，介意可改 orbslam3 源码重编。

**包内配置**（`configs/orb_slam3/`，随包分发、自包含）：

| 文件 | 用途 |
|---|---|
| `stereo-inertial.yaml` | ORB-SLAM3 参数：内参（fx=fy=200，400x300）、双目基线（bf=32）、Tbc 外参、IMU 噪声、ORB 特征提取参数 |

**外部依赖（未随包分发）**：

- ORB 词表 `ORBvoc.txt`（约 52MB）：launch 中以绝对路径引用 `/root/fucker/ros2_ws/src/orb_slam3/vocabulary/ORBvoc.txt`，不宜复制进 flylab
- `orbslam3` wrapper 包：`package.xml` 已声明 `exec_depend`

**关键参数（已按实际装机校准）**：

- 图像/IMU remap：`/camera/left → /airsim/left_cam/image_raw`、`/camera/right → /airsim/right_cam/image_raw`、`/imu → /airsim/imu_ros2/data`
- 外参 `Tbc` 直接搬运 VINS 的 `body_T_cam0`（前 0.4m / 左 0.08m，36.87° 下俯），与 vins_stereo 同一装机
- 启动第 3 个参数 `do_rectify := 'false'`：AirSim 是理想平行双目、零畸变，无需图像预校正（ORB-SLAM3 内部会用 yaml 的 `Tlr` 自行处理双目几何）。若改 `'true'`，节点要求 yaml 提供 `LEFT.K/D/R/P` + `RIGHT.K/D/R/P` 校正矩阵，缺了会直接报 "Calibration parameters to rectify stereo are missing!" 崩溃退出

## 节点说明

### move_velocity —— 通用 PX4 offboard 速度控制器

订阅 `/flylab/cmd_vel`（`geometry_msgs/Twist`），驱动 PX4 offboard 速度控制。

**控制约定**：

| 动作 | 触发条件 |
|---|---|
| 解锁 + 起飞 | 未解锁时收到 `linear.z > 5` |
| 速度飞行 | OFFBOARD 状态下按 Twist 指令持续跟踪 |
| 自动降落 | OFFBOARD 中 `linear.z < -5` 且离地 < 5m |
| 取消降落 | AUTO_LAND 中收到 `linear.z > 5` |
| QGC 手动飞行 | 其余状态不干预，无命令发送 |

**手动测试**：

```bash
ros2 topic pub -r 10 /flylab/cmd_vel geometry_msgs/msg/Twist "{linear: {z: 6.0}}"   # 起飞
ros2 topic pub -r 10 /flylab/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"   # 前飞 1 m/s
ros2 topic pub -r 10 /flylab/cmd_vel geometry_msgs/msg/Twist "{linear: {z: -6.0}}"  # 下降触发降落
```

> 坐标系说明：Twist 输入按 FLU 解释，节点内部转换为 NED 后发给 PX4（y、z 取反，角速度取反）。
> 与 PX4 通信的 QoS 为 `BEST_EFFORT + TRANSIENT_LOCAL`（uXRCE-DDS 标配），请勿修改。

## 目录结构

```
flylab/
├── CMakeLists.txt
├── package.xml
├── configs/            # AirSim 端 settings.json
│   ├── fly.json        #   sim_fly 用：全套相机+雷达
│   ├── mapping.json    #   建图链路用：仅 CameraDepth 深度相机
│   ├── settings.json   #   双目基础配置（left_cam/right_cam）
│   ├── vins_mono/      #   vins_mono 用：AirSim settings.json + VINS 参数 + 相机内参
│   ├── vins_stereo/    #   vins_stereo 用：VINS 双目外参 + 内参
│   └── orb_slam3/      #   orb_slam3 用：ORB-SLAM3 双目+IMU 参数
├── src/                # C++ 节点
│   ├── move_velocity.cpp
│   ├── octomap_dynamic.cpp
│   └── vins_arming_switch.cpp
├── log/                # VINS 运行输出（vio.csv / pose_graph/），不入库
├── launch/             # 按功能域分目录（ros2 launch 会递归搜索）
│   ├── fly/            #   飞行控制类
│   │   └── sim_fly.launch.py
│   ├── mapping/        #   感知建图类
│   │   └── depth_mapping.launch.py 等 5 个
│   ├── vins_mono/      #   视觉惯性里程计类（单目）
│   │   └── vins_mono.launch.py
│   ├── vins_stereo/    #   视觉惯性里程计类（双目）
│   │   └── vins_stereo.launch.py
│   └── orb_slam3/      #   SLAM 类
│       └── orb_slam3.launch.py
└── rviz/               # 与 launch/ 分目录对应
    ├── fly/
    │   ├── depth_cloud.rviz
    │   └── image_lidar.rviz
    ├── mapping/
    │   ├── depth_mapping.rviz
    │   └── colored_mapping.rviz
    ├── vins_mono/
    │   └── vins_mono.rviz
    ├── vins_stereo/
    │   └── vins_stereo.rviz
    └── orb_slam3/
        └── orb_slam3.rviz
```

## 话题接口约定（提前冻结）

为 lesson12/13 的 EGO 跟踪器移植预留的接口，定于 2026-09-23，后续节点必须遵守：

```
/flylab/cmd_vel            (geometry_msgs/Twist)   手动速度源（已实现，move_velocity 消费）
/flylab/track_mode         (std_msgs/Bool)         自动模式开关（预留，true=EGO 自动飞行）
/drone_0_planning/pos_cmd  (quadrotor_msgs/PositionCommand)  EGO 轨迹源（EGO 发，我们只订阅）
```

设计要点：

- **模式切换 = setpoint 数据源切换**：飞机始终处于 OFFBOARD，`track_mode` 只决定 tracker 订阅键盘速度还是 EGO 轨迹，不做 MAVLink 模式切换（原作者 T 键机制的去键盘化版本）
- 切换逻辑属于未来的 `ego_tracker` 节点，`move_velocity` 保持纯手动职责不变
- `track_mode=true` 但 EGO 尚未产出轨迹时，tracker 必须悬停等待（沿用原作者的安全保护设计）
- 保留原作者 lesson12 的大阈值起飞防误触思路（z 通道阈值比 lesson3 更高）

## 移植进度

| 来源 (hw_insight) | 去向 | 状态 |
|---|---|---|
| move_velocity.py | `src/move_velocity.cpp`（去键盘化，改 Twist 输入） | ✅ |
| rviz/depth_cloud.rviz, image_lidar.rviz | `rviz/` | ✅ |
| lesson3.launch.py | `launch/sim_fly.launch.py`（更名 sim_fly，airsim_node 暂借用原包） | ✅ |
| keyboard_velocity.py | —（弃用，由 Twist 话题取代） | ➖ |
| lesson4 感知链路 (octomap_dynamic 等) | `src/octomap_dynamic.cpp` + `launch/depth_mapping.launch.py` 等 5 个 + `rviz/depth_mapping.rviz` 等 | ✅ |
| lesson7 VINS+GPS 融合 | `launch/vins_mono/` + `rviz/vins_mono/` + `configs/vins_mono/`（VINS 配置随包分发，外参按实际装机校准） | ✅ |
| lesson5 多机 (multi_move_velocity) | 待定 | ⬜ |
| 双目 VINS (vins_stereo) | `launch/vins_stereo/` + `rviz/vins_stereo/` + `configs/vins_stereo/` + `src/vins_arming_switch.cpp`（settings.json 双目模式） | ✅ |
| lesson11 ORB-SLAM3 双目+IMU | `launch/orb_slam3/` + `rviz/orb_slam3/` + `configs/orb_slam3/stereo-inertial.yaml`（词表沿用 orbslam3 包，不随包分发） | ✅ |
| lesson12/13 EGO 跟踪器 | `src/ego_tracker.cpp`（待建，按上方接口约定实现） | ⬜ |
| airsim_node (C++) | 暂不移植，include airsim_ros_pkgs | ⬜ |

## 许可

Apache-2.0
