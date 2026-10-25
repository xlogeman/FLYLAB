# FLYLAB —— PX4 + AirSim SITL 无人机仿真实验平台

WSL2 (Ubuntu 22.04) + ROS2 Humble，围绕 PX4 + Cosys-AirSim 做无人机自主飞行研究：感知建图、视觉惯导、避障规划与 offboard 控制

## 1. 工作区总体

| 目录 | 说明 |
|---|---|
| `ros2_ws/` | ROS2 工作空间，核心是 `src/flylab_flight`（飞行链路）、`src/flylab_vision`（视觉算法）、`src/flylab_gcs`（地面站），其余为算法移植包 |
| `Cosys-AirSim/` | UE 端仿真器 |
| `px4.1.15.2/` | PX4 SITL v1.15.2 源码（自定义机型见 §6） |
| `ORB_SLAM3_FIXED/` | ORB-SLAM3 运行时库 |
| `mavros/` `libmavconn/` `mavros_msgs/` | MAVROS 通信栈 |
| `scripts/` | `env.sh`（环境总入口）/ `run-px4.sh`（PX4 启动）/ `sync-airsim-ip.sh`（同步 WSL IP） |

---

## 2. flylab_flight 核心包

### 2.1. 功能链路一览

| 链路 | 一键启动 | 场景配置 | 说明 |
|---|---|---|---|
| 感知建图 | `ros2 launch flylab_flight depth_mapping.launch.py` | octomap 参数内联 | 深度图 → 点云 → 占据地图 |
| EGO 避障 | `ros2 launch flylab_flight fly_ego.launch.py` | `configs/ego_planner/` | RViz 点目标 → EGO 规划 → PX4 跟踪，到达后交还 QGC；`tracker:=mavros\|uxrce` |
| OpenVINS | `ros2 launch flylab_flight openvins_msckf.launch.py` | `configs/openvins/` | 双目 VIO，断桨联动启停 |
| VINS 单/双目 | `vins_mono.launch.py` / `vins_stereo.launch.py` | `configs/vins_*/` | VINS-Fusion 航迹 |
| ORB-SLAM3 | `ros2 launch flylab_flight orb_slam3.launch.py` | `configs/orb_slam3/` | 双目+IMU SLAM |
| FAST-LIO | `ros2 launch flylab_flight fast_lio.launch.py` | `configs/fast_lio/` | LiDAR-惯性里程计 |
| SUPER 避障 | `ros2 launch flylab_flight fly_super.launch.py` | `configs/super_planner/` | RViz 点目标 → SUPER 规划 → PX4 跟踪；`tracker:=mavros\|uxrce` |

### 2.2. 节点一览

传感器层（`src/sensors/`）与算法解耦，算法特化适配（坐标系/门控/时间对齐）在桥接层（`src/bridges/`）

| 传感器节点 | 发布话题 | 说明 |
|---|---|---|
| `airsim_camera` | `/airsim/cam/<名>/image_raw` | 通用相机（深度/双目/单目参数化），多相机同次 RPC 硬件同步 |
| `airsim_imu` | `/airsim/imu/data_raw`(FRD) / `data`(FLU) | 200Hz，时间戳映射仿真时钟，重复采样去重 |
| `airsim_odom` | `/airsim/odom` + TF | 真值里程计 100Hz，与 IMU/LiDAR 同一时间轴 |
| `airsim_nav` | `/airsim/gps` `/airsim/baro` `/airsim/mag` | 低频导航传感器 50Hz |
| `airsim_lidar` | `/airsim/lidar/points_raw` | 原始点云（NED），单时刻瞬时快照时间模型 |

| 桥接节点 | 输入 → 输出 | 说明 |
|---|---|---|
| `bridge_vio` | 相机+IMU → `/vio/*` | OpenVINS 断桨门控（降落停发/解锁恢复），数据原样转发 |
| `bridge_fastlio` | `points_raw` → `/airsim/lidar/points` | NED→FLU + 挂载外参静态 TF |
| `bridge_super` | `points_raw`+odom → `/cloud_registered` `/lidar_slam/odom` | 位姿按点云时刻插值对齐 + 逐点去畸变 + 机身回波过滤 |

其他节点：`tracker_uxrce`/`tracker_mavros`（轨迹跟踪+混控，`tracker:=` 选链路）、`super_cmd_bridge`（mars→PositionCommand）、`vins_arming_switch`（VINS 起飞联动）

### 2.3. 环境依赖

ROS2 Humble + PX4 SITL v1.15.2（消息经 MicroXRCEAgent `udp4 -p 8888` 桥接）；Cosys-AirSim 5.8；ego_planner / ov_msckf / orbslam3 / fast_lio（依赖 livox_ros_driver2）/ super_planner 均为工作空间内移植包

---

## 3. 快速开始

```bash
# 1. 编译 (WSL 需限并行度)
source /opt/ros/humble/setup.bash
export MAKEFLAGS=-j2
cd ~/flylab/ros2_ws && colcon build --parallel-workers 2 && source install/setup.bash

# 2. 场景配置: configs/ 下对应 json 复制到 Windows 端 Documents\AirSim\settings.json，重启 UE
bash /root/flylab/scripts/run-px4.sh                                # PX4 SITL (默认 none_iris)
bash /root/flylab/scripts/run-px4.sh none_fucker_3120g_tw23 --clean # 指定机型 (--clean 清参数缓存)
bash /root/flylab/scripts/sync-airsim-ip.sh                         # WSL IP 变了先跑这个

MicroXRCEAgent udp4 -p 8888
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14542@ -p tgt_system:=1 -p tgt_component:=1

# 3. 启动链路 (跨主机需先 export PX4_SIM_HOST_ADDR=<AirSim IP>)
ros2 launch flylab_flight fly_ego.launch.py    # EGO 避障 (默认 MAVROS 跟踪)

# 4. 地面站 (工具栏一键起 PX4/链路/飞控，算法启停 / ArUco降落 / 日志框)
ros2 run flylab_gcs ground_station
```

---

## 4. 视觉算法实验区

### 4.1. flylab_gcs 地面站

`ros2_ws/src/flylab_gcs`，按功能域分层：`main_window` 只做界面组装与轮询调度，业务细节在各模块

| 模块 | 说明 |
|---|---|
| `config.py` | 全局配置：话题、端口、机型/链路/飞控命令/算法注册表，改这里 |
| `main_window.py` | 主窗口：顶部工具栏（四个下拉按钮）+ 右栏算法面板 + 33ms 轮询与按钮门控 |
| `ros_node.py` | ROS 节点 + `TeeLogger`（终端日志照常，同时分流到界面日志框） |
| `camera_grabber.py` | 视频源抓取线程（AirSim RPC / 本机摄像头 / 视频文件）+ 软件云台（世界系稳定） |
| `vehicle_control.py` | 飞行控制线程：独立 AirSim RPC 会话（解锁/起飞/降落/悬停/返航/上锁） |
| `processes.py` | 托管子进程生命周期：独立进程组、SIGTERM→SIGKILL 整组回收、输出接管 |
| `log_panel.py` | 日志框：日志分流上屏，缓冲批量渲染抗日志风暴 |
| `camera_widget.py` | 输入源视图：源菜单 + 播放控制条 + 云台方向键 |
| `video_label.py` | 视频控件：等比显示 / 深度伪彩 / 拖拽框选 / 标注叠加 |
| `overlay.py` | 标注 schema 与分组解析 |

界面布局：顶部工具栏（视频源 / PX4 SITL / 飞行控制 / 通信链路 / 视觉算法 / 飞行链路 / 视觉反控，运行中亮绿灯）+ 左画面（下方日志框）+ 右栏（地图 / 云台罗盘 / 手动遥控）。窗口大小 `config.WINDOW_SIZE_RATIO`。

**顶部工具栏**（四个按钮全部下拉交互：点选项生效、再点取消）：
- **视频源**：UE 相机 / 本机摄像头 / 视频文件；菜单展开时重扫本机摄像头（热插拔可见）
- **PX4 SITL**：机型菜单（`config.PX4_MODELS`，✓=运行中）——点机型即启动，再点即停，选其他机型停旧换新；输出接管进日志框
- **通信链路**（互斥，一次只开一条）：uXRCE-DDS Agent / MAVROS（**必须用 14542 专属实例**，见 §7）；降落进行中锁定不可切换，所选链路作为降落节点 `link` 参数
- **飞行控制**：解锁/起飞/降落/悬停/返航/上锁（`config.VEHICLE_CMDS`），经 AirSim API → MAVLink → PX4；仅 RPC 已连通可点；异步指令，结果看 QGC / mavros state

**日志框**（画面下方）：rclpy 日志与 PX4 进程输出全部上屏（时间戳 + 级别着色）；缓冲批量渲染限速，PX4 控制台刷屏不拖垮界面。

**显示画面转发**：当前显示帧（深度伪彩除外）以 30Hz 原分辨率发到 `/flylab/display/image_raw`，算法节点订阅即处理「用户正在看的画面」，输入源切换对算法透明；算法要吃原始话题用 `-p image_topic:=...` 覆盖。

**标注接口**（`overlay.py`）：按算法分组，各组独立替换/清屏；坐标一律原图像素，停止前发空 items 清自己的组：

```json
{"algo": "yolo", "items": [
  {"type": "bbox",     "box": [x, y, w, h], "label": "car 0.92", "color": [0,255,0]},
  {"type": "point",    "point": [x, y],     "label": "..."},
  {"type": "polyline", "points": [[x,y]],   "close": true},
  {"type": "text",     "pos": [x, y],       "text": "..."},
  {"type": "image",    "pos": [x,y], "w": 640, "alpha": 0.45, "data": "<base64 JPEG>"}]}
```

**其他**：名字含 `depth` 的源自动伪彩（0.2~20m）；UE 断线自动重连，只保留最新帧；相机需在 UE 端 `settings.json` 配置过（`configs/settings.json` 已含全部）。依赖 `msgpack-rpc-python`、PyQt5、OpenCV（GTK 后端，勿装 pip 版 opencv-python——Qt 与 PyQt5 冲突）。

### 4.2. flylab_vision 算法包

`ros2_ws/src/flylab_vision`（ament_cmake）：算法库纯 OpenCV 不依赖 ROS，展示节点继承 `DisplayVisionNode` 基类只填算法差异，通信/启停/标注/诊断全部下沉到公共层。

```
flylab_vision/
├── include/common/               # DisplayVisionNode 基类 + 公共工具（订阅/启停/标注/诊断全封装）
├── include/aruco/                # ArUco 检测库（检测 + IPPE_SQUARE 位姿，兼容 OpenCV 4.5/4.7+）
├── include/yolo/                 # 七个 YOLO 任务库接口（检测/分类/深度/OBB/姿态/实例分割/语义分割，pimpl 隔离 onnxruntime）
├── include/control/              # PX4 执行层：px4_offboard_client 双链路组件（MAVROS / uXRCE）
├── src/aruco/                    # aruco_node（检测展示）+ aruco_landing_node（降落状态机）
├── src/yolo/<task>/              # 每任务一个子目录：算法库 + 展示节点成对放置
├── thirdparty/onnxruntime/       # 预编译推理库（系统 OpenCV 4.5 DNN 不支持 yolo26）
└── models/<task>/                # 预训练权重
```

```bash
ros2 run flylab_vision aruco_node --ros-args \
    -p image_topic:=/flylab/display/image_raw \   # 算法图像输入 (默认地面站转发帧)
    -p dict_name:=DICT_4X4_1000 \                 # Prometheus 6x6 码用 DICT_6X6_250
    -p marker_length:=0.2 \                       # 码物理边长(米)
    -p calib:=/path/to/calib.yaml                 # 相机标定 (可选; 无则按 FOV 估算)
```

**新增算法**（展示型）：建 `include/<算法>/` + `src/<算法>/`（算法库不碰 ROS）→ 节点继承 `DisplayVisionNode` 实现 `process()` / `clear_json()`，通信零编写 → CMakeLists 两行模板（`flylab_algo` + `flylab_node`，参考 `yolo_obb_node.cpp`）→ `config.ALGORITHMS` 注册即出现在地面站下拉框。

**独立调试 CLI**：根目录 `python3 aruco_detector.py --image/--video/--camera/--gen`。

## 5. 载具物理参数配置化（PhysicsParams）

多旋翼物理参数在 settings.json 载具段下配置 `PhysicsParams`，改完**无需重编译**，重启仿真生效；`Model` 字段为基础机架模板，`PhysicsParams` 在其上覆盖（改动在 Windows 侧 UE 项目 `BeimingSim/Plugins/AirSim/`）。

```json
"Vehicles": {
    "PX4": {
        "VehicleType": "PX4Multirotor",
        "PhysicsParams": {
            "Mass": 3.120,
            "RotorCount": 4,
            "ArmLengths": 0.225,
            "CT": 0.130,
            "CP": 0.055,
            "MaxRPM": 6800,
            "PropDiameter": 0.3048,
            "DragCoefficient": 0.4
        }
    }
}
```

| 量 | 公式 |
|---|---|
| 单桨最大推力 | `CT × 1.225 × (MaxRPM/60)² × PropDiameter⁴` [N] |
| 推重比 | `RotorCount × max_thrust / (Mass × 9.81)` |
| 悬停油门 | `MPC_THR_HOVER = 1/推重比`（推力与油门线性，见 `RotorActuator.hpp`） |

- 挂载荷：`Mass` 加重量 + 重算 `MPC_THR_HOVER`，缺一悬停漂移
- `DragCoefficient` 是绝对值（默认机架 0.325、Flamewheel 系 1.3）
- 限制：仅对称 X 型（QuadX/HexX/OctoX）、机臂统一长度

## 6. 机型库与 PX4 机型文件

自研机型参数档案在 `ros2_ws/src/flylab_flight/configs/vehicles/`（每 json 一架，含 PhysicsParams 与 PX4 侧配套参数）；PX4 机型文件在 `px4.1.15.2/ROMFS/px4fmu_common/init.d-posix/airframes/`，与 PhysicsParams 严格联动（电机坐标 `±ArmLengths/√2`、`KM = CP/CT × D/2π`、`MPC_THR_HOVER = 1/推重比`）。

| 配置文件 | 构型 | 推重比 | 悬停油门 | PX4 机型 |
|---|---|---|---|---|
| `airsim_default_1000g_tw17.json` | F450，GWS 9x5 桨+MT2212，1.0kg | 1.70 | 0.59 | `10016_none_iris` |
| `fucker_3120g_tw23.json` | 450 轴距，2814 电机+12寸桨(4S)，3.12kg | 2.31 | 0.43 | `70003_fucker_3120g_tw23` |
| `fucker_2.00kg_tw15.json` | 450 轴距，2212 电机+10寸桨(3S)，2.0kg | 1.50 | 0.67 | `70004_fucker_2000g_tw15` |

**新增机型**：① `airframes/700XX_<机型名>`（**文件名禁止含点号**——rcS 会把带点文件当 `xxx.post` 忽略，小数点去掉如 `tw23`）② `airframes/CMakeLists.txt` 注册 ③ `simulator_mavlink/CMakeLists.txt` 加 `add_custom_target(none_<机型名>)` ④ `configs/vehicles/` 存档 ⑤ `cd px4.1.15.2 && make px4_sitl_default`

## 7. SITL 链路与端口分配

| 端口 | 用途 | 对端 |
|---|---|---|
| TCP 4560 | HIL 物理仿真链路 | UnrealEditor（Windows） |
| UDP 14580→14540 | onboard 实例 | **被 AirSim 控制通道抢占** |
| UDP 14582→14542 | **MAVROS 专属实例** | MAVROS `fcu_url:=udp://:14542@` |
| UDP 18570→14550 | GCS 实例 | QGC 等 |
| UDP 8888 | uXRCE-DDS（`tracker:=uxrce`） | MicroXRCEAgent |
| TCP 41451 | AirSim RPC API | airsim_sensor_* / flylab_gcs |

**坑：14580 会被 AirSim 抢占。** Cosys-AirSim 以 1Hz 向 `ControlIp:ControlPortRemote`（WSL 的 14580）发心跳，而 PX4 的 MAVLink 实例只锁定一次对端，AirSim 先到先得导致 MAVROS 永远连不上。MAVROS 必须走 14582↔14542 专属实例，勿动 14580；WSL 重启后跑 `scripts/sync-airsim-ip.sh` 更新 ControlIp。

## 8. 注意事项

- **改 PhysicsParams 质量后**：同步重算 `MPC_THR_HOVER`，否则悬停漂移；settings.json 改动复制到 Windows 端 `Documents\AirSim\` 后重启 UE 生效
- **雷达配置**：`configs/fast_lio/settings.json` 为 GPU 雷达（`SensorType: 8`，对齐 Mid-360），FAST-LIO/SUPER 共用；`configs/settings.json` 为 CPU 雷达（`SensorType: 6`，**必须 `LimitPoints: false`**，否则被 5000 点上限截断、帧内拼接错位且帧率掉到 ~1.7Hz）
- **雷达时间戳**：AirSim 点云是单时刻瞬时快照，统一填 `time = 1/rotation_hz` 并以 `LidarData.time_stamp` 标定 ROS 偏移；逐点合成时间会反向注入运动剪切导致漂移，勿改回
- **传感器时间轴**：IMU/LiDAR/odom 统一经 `SimClockSync` 映射（`src/sensors/airsim_common.hpp`），三路同一时间轴；bridge_super 去畸变、FAST-LIO 融合都依赖此约定，新传感器节点请保持
- **WSL 时钟漂移**：高负载下系统时钟漂移可达秒级，timesyncd 步进纠偏会导致时间戳整体跳变（去畸变拖影/FAST-LIO reset）。**已用 chrony slew 模式根治**（时钟永不跳变），timesyncd 已 disable；验证 `chronyc tracking` 的 System time 应为毫秒级
- **点云抗卡顿**：实时流不允许积压回放——`airsim_lidar` 发布 QoS `KeepLast(2) reliable`，bridge 与 RViz 点云 Queue Size 全部限深 1~2；GPU 雷达过滤跨卡顿帧（段时刻跨度 > 3× 扫描周期直接丢弃，否则去畸变整帧拖影）。诊断：`ros2 topic hz` 的 **min 为负 = 时钟被步进**，max 飙秒级 = 发布端真断流
