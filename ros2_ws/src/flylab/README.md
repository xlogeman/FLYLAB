# flylab

PX4 + AirSim SITL 无人机仿真工具包。从 [ytjy-sitl] 课程体系（hw_insight / hw_interface）逐步解构移植而来，用 C++ 重写、去除了原作者的个人化封装，形成一个结构清晰、可分享、可持续扩展的自主实验项目。

## 目录结构

```bash
flylab/
├── CMakeLists.txt
├── package.xml
├── configs/      # AirSim 端 settings.json
│   ├── fly.json        # sim_fly 用：全套相机+雷达
│   ├── mapping.json    # 建图链路用：仅 CameraDepth 深度相机
│   ├── settings.json   # 双目基础配置（left_cam/right_cam）
│   ├── vins_mono/      # vins_mono 用：AirSim settings.json + VINS 参数 + 相机内参
│   ├── vins_stereo/    # vins_stereo 用：VINS 双目外参 + 内参
│   └── orb_slam3/      # orb_slam3 用：ORB-SLAM3 双目+IMU 参数
├── src/          # C++ 节点
│   ├── move_velocity.cpp
│   ├── octomap_dynamic.cpp
│   └── vins_arming_switch.cpp
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
| AirSim (Cosys fork) | UE 端仿真，`airsim_ros_pkgs` 提供传感器数据通路 |
| MicroXRCEAgent | `udp4 -p 8888`，PX4 ↔ ROS2 DDS 桥 |
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

# 启动 ROS2 与 PX4 的通信代理（好像不需要也能通）
MicroXRCEAgent udp4 -p 8888
```

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

ros2 run hw_insight lesson12_px4_ego_tracker --ros-args -p goal_z_height:=3.0

```

> 坐标系说明：Twist 输入按 FLU 解释，节点内部转换为 NED 后发给 PX4（y、z 取反，角速度取反）。
> 与 PX4 通信的 QoS 为 `BEST_EFFORT + TRANSIENT_LOCAL`（uXRCE-DDS 标配），请勿修改。

## 话题接口约定（提前冻结）

为 lesson12/13 的 EGO 跟踪器移植预留的接口，后续节点必须遵守：

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

