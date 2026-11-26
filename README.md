# FLYLAB —— PX4 + AirSim SITL 无人机仿真实验平台

WSL2 (Ubuntu 22.04) + ROS2 Humble，围绕 PX4 + Cosys-AirSim 做无人机自主飞行研究：感知建图、视觉惯导、避障规划与 offboard 控制

> **拿到 Docker 镜像复现的看 [§9 Docker 镜像复现](#9-docker-镜像复现)**，其余部分与本机 WSL 开发流程一一对应

## 1. 工作区总体

| 目录 | 说明 |
|---|---|
| `ros2_ws/` | ROS2 工作空间，核心是 `src/flylab_flight`（飞行链路）、`src/flylab_vision`（视觉算法）、`src/flylab_gcs`（地面站），其余为算法移植包 |
| `Cosys-AirSim/` | UE 端仿真器 |
| `px4.1.15.2/` | PX4 SITL v1.15.2 源码（自定义机型见 §6） |
| `ORB_SLAM3_FIXED/` | ORB-SLAM3 运行时库 |
| `mavros/` 等 | MAVROS 通信栈 |
| `scripts/` | `env.sh`（环境总入口）/ `run-px4.sh` / `sync-airsim-ip.sh` |

## 2. flylab_flight 核心包

### 2.1. 功能链路一览

| 链路 | 一键启动 | 场景配置 | 说明 |
|---|---|---|---|
| 感知建图 | `depth_mapping.launch.py` | octomap 参数内联 | 深度图 → 点云 → 占据地图 |
| EGO 避障 | `fly_ego.launch.py` | `configs/ego_planner/` | RViz 点目标 → 规划 → PX4 跟踪，到达交还 QGC；`tracker:=mavros\|uxrce` |
| OpenVINS | `openvins_msckf.launch.py` | `configs/openvins/` | 双目 VIO，断桨联动启停 |
| VINS 单/双目 | `vins_mono/stereo.launch.py` | `configs/vins_*/` | VINS-Fusion 航迹 |
| ORB-SLAM3 | `orb_slam3.launch.py` | `configs/orb_slam3/` | 双目+IMU SLAM |
| FAST-LIO | `fast_lio.launch.py` | `configs/fast_lio/` | LiDAR-惯性里程计 |
| SUPER 避障 | `fly_super.launch.py` | `configs/super_planner/` | RViz 点目标 → 规划 → PX4 跟踪；`tracker:=mavros\|uxrce` |

（launch 均为 `ros2 launch flylab_flight <名>`）

### 2.2. 节点一览

传感器层（`src/sensors/`）与算法解耦，特化适配（坐标系/门控/时间对齐）在桥接层（`src/bridges/`）

| 传感器节点 | 发布话题 | 说明 |
|---|---|---|
| `airsim_camera` | `/airsim/cam/<名>/image_raw` | 深度/双目/单目参数化，多相机 RPC 硬件同步 |
| `airsim_imu` | `/airsim/imu/data_raw`(FRD) / `data`(FLU) | 200Hz，仿真时钟映射 + 重复采样去重 |
| `airsim_odom` | `/airsim/odom` + TF | 真值里程计 100Hz，同一时间轴 |
| `airsim_nav` | `/airsim/gps` `/baro` `/mag` | 导航传感器 50Hz |
| `airsim_lidar` | `/airsim/lidar/points_raw` | NED 原始点云，单时刻瞬时快照 |

| 桥接节点 | 输入 → 输出 | 说明 |
|---|---|---|
| `bridge_vio` | 相机+IMU → `/vio/*` | 数据原样转发 + OpenVINS 断桨门控 |
| `bridge_fastlio` | `points_raw` → `/airsim/lidar/points` | NED→FLU + 外参静态 TF |
| `bridge_super` | `points_raw`+odom → `/cloud_registered` `/lidar_slam/odom` | 位姿按点云时刻插值对齐 + 去畸变 + 机身回波过滤 |

其他节点：`tracker_uxrce`/`tracker_mavros`（轨迹跟踪+混控）、`super_cmd_bridge`（mars→PositionCommand）、`vins_arming_switch`（VINS 起飞联动）

### 2.3. 环境依赖

ROS2 Humble + PX4 SITL v1.15.2（消息经 MicroXRCEAgent `udp4 -p 8888` 桥接）；
Cosys-AirSim 5.8；

## 3. 快速开始

```bash
# 1. 编译 (WSL 需限并行度)
source /opt/ros/humble/setup.bash
export MAKEFLAGS=-j2
cd ~/flylab/ros2_ws && colcon build --parallel-workers 2 && source install/setup.bash

# 2. 场景配置: configs/ 下对应 json 复制到 Windows 端 Documents\AirSim\settings.json，重启 UE
bash scripts/run-px4.sh                                # PX4 SITL (默认 none_iris)
bash scripts/run-px4.sh none_fucker_x450 --clean       # 指定机型 (--clean 清参数缓存)
bash scripts/sync-airsim-ip.sh                         # WSL IP 变了先跑这个

MicroXRCEAgent udp4 -p 8888
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14542@ -p tgt_system:=1

# 3. 启动链路 (跨主机需先 export PX4_SIM_HOST_ADDR=<AirSim IP>)
ros2 launch flylab_flight fly_ego.launch.py

# 4. 地面站 (工具栏一键起 PX4/链路/飞控，算法启停 / ArUco降落 / 日志框)
ros2 run flylab_gcs ground_station

# 5. 可略过 AirSim，绑定本机摄像头做视觉实验
usbipd attach --busid 1-8 --wsl U-22.04-fuck
```

## 4. 视觉算法实验区

### 4.1. flylab_gcs 地面站

`ros2_ws/src/flylab_gcs`，按功能域分层：`main_window` 只做界面组装与轮询调度，业务细节在各模块

| 模块 | 说明 |
|---|---|
| `config/` | 全局配置包：`topics`(话题) / `commands`(命令) / `gimbal`(云台控制律) / `ui_ranges`(控件值域) / `registry`(算法/链路/机型注册表)，按域改对应子模块 |
| `main_window.py` | 主窗口：工具栏 + 右栏面板组装 + 33ms 轮询与按钮门控 |
| `ros_node.py` | ROS 节点 + 日志分流上屏 |
| `camera_grabber.py` | 视频源抓取线程（AirSim RPC / 摄像头 / 视频文件）+ 数字变焦 |
| `gimbal_controller.py` | 软件云台（世界系稳定）+ FOV + 航向跟随，独立 RPC 线程 |
| `vehicle_control.py` | 飞行控制线程：独立 RPC 会话（解锁/起飞/降落/悬停/返航/上锁） |
| `processes.py` | 子进程托管：独立进程组、SIGTERM→SIGKILL 整组回收 |
| `log_panel.py` | 日志框：缓冲批量渲染抗日志风暴 |
| `camera_widget.py` | 源菜单 + 播放控制条 + 云台方向键 |
| `video_label.py` | 等比显示 / 深度伪彩 / 拖拽框选 / 标注叠加 |
| `overlay.py` | 标注 schema 与分组解析 |

界面布局：顶部工具栏 + 左画面（下方日志框）+ 右栏（地图 / 云台罗盘 / 双摇杆遥控 / 算法信息面板）。

**工具栏**（下拉交互：点选项生效、再点取消）：
- **视频源**：UE 相机 / 本机摄像头 / 视频文件，展开菜单时重扫（热插拔可见）
- **PX4 SITL**：机型菜单，点启 / 再点停 / 换机停旧起新，输出接管进日志框
- **通信链路**（互斥）：uXRCE-DDS / MAVROS（**必须 14542 专属实例**，见 §7）
- **飞行控制**：解锁/起飞/降落/悬停/返航/上锁，经 AirSim API → PX4，需 RPC 已连通
- **接管控制**：视觉反控 / 手动遥控二选一（都走 offboard）；反控中可开遥控抢回

**显示画面转发**：当前显示帧（深度伪彩除外）以 30Hz 原分辨率发到 `/flylab/display/image_raw`，算法节点订阅即处理「用户正在看的画面」；算法要吃原始话题用 `-p image_topic:=...` 覆盖。

**标注接口**（`overlay.py`）：按算法分组独立替换/清屏，坐标一律原图像素，停止前发空 items 清自己的组：

```json
{"algo": "yolo", "items": [
  {"type": "bbox",     "box": [x, y, w, h], "label": "car 0.92", "color": [0,255,0]},
  {"type": "point",    "point": [x, y],     "label": "..."},
  {"type": "polyline", "points": [[x,y]],   "close": true},
  {"type": "text",     "pos": [x, y],       "text": "..."},
  {"type": "image",    "pos": [x,y], "w": 640, "alpha": 0.45, "data": "<base64 JPEG>"}]}
```

**算法信息面板**：算法发 `{"src": "<key>", ...}` JSON 到 `/flylab/algo_info`，`panel_sections.SECTIONS` 注册字段表 + `config.ALGORITHMS` 加 `'panel'` 键即随算法显隐；稠密输出（深度/分割）可带 `"img"`（base64 JPEG）进分组顶部图片区。

**其他**：名字含 `depth` 的源自动伪彩（0.2~20m）；UE 断线自动重连只保留最新帧；相机需在 UE 端 `settings.json` 配置过（`configs/settings.json` 已含全部）；OpenCV 用 GTK 后端（勿装 pip 版 opencv-python，Qt 与 PyQt5 冲突）。

### 4.2. flylab_vision 算法包

算法库纯 OpenCV 不依赖 ROS，展示节点继承 `DisplayVisionNode` 基类只填算法差异，通信/启停/标注/诊断全在公共层：

```
flylab_vision/
├── include/common/               # DisplayVisionNode 基类 + 公共工具
├── include/aruco/                # ArUco 检测 + IPPE_SQUARE 位姿
├── include/yolo/                 # 七个 YOLO 任务库接口（pimpl 隔离 onnxruntime）
├── include/control/              # PX4 执行层：offboard client 双链路（MAVROS / uXRCE）
├── src/aruco/                    # aruco_node（检测展示）+ aruco_landing_node（降落状态机）
├── src/yolo/<task>/              # 算法库 + 展示节点成对放置
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

**新增算法**：建 `include/<算法>/` + `src/<算法>/`（算法库不碰 ROS）→ 节点继承 `DisplayVisionNode` 实现 `process()` / `clear_json()`，通信零编写 → CMakeLists 两行模板（参考 `yolo_obb_node.cpp`）→ `config.ALGORITHMS` 注册即出现在地面站下拉框（要上面板加 `'panel'` 键 + 发 `{"src": ...}` JSON，见 §4.1）。

**独立调试 CLI**：根目录 `python3 aruco_detector.py --image/--video/--camera/--gen`。

## 5. 载具物理参数配置化（PhysicsParams）

多旋翼物理参数在 settings.json 载具段下配置 `PhysicsParams`，改完**无需重编译**，重启仿真生效；`Model` 为基础机架模板，`PhysicsParams` 在其上覆盖（改动在 Windows 侧 UE 项目 `BeimingSim/Plugins/AirSim/`）。

```json
"PhysicsParams": {
    "Mass": 3.120, "RotorCount": 4, "ArmLengths": 0.225,
    "CT": 0.130, "CP": 0.055, "MaxRPM": 6800,
    "PropDiameter": 0.3048, "DragCoefficient": 0.4
}
```

| 量 | 公式 |
|---|---|
| 单桨最大推力 | `CT × 1.225 × (MaxRPM/60)² × PropDiameter⁴` [N] |
| 推重比 | `RotorCount × max_thrust / (Mass × 9.81)` |
| 悬停油门 | `MPC_THR_HOVER = 1/推重比`（推力与油门线性，见 `RotorActuator.hpp`） |

- 挂载荷：`Mass` 加重 + 重算 `MPC_THR_HOVER`，缺一悬停漂移
- `DragCoefficient` 是绝对值（默认机架 0.325、Flamewheel 系 1.3）
- 仅对称 X 型（QuadX/HexX/OctoX）、机臂统一长度

## 6. 机型库与 PX4 机型文件

自研机型参数档案在 `configs/vehicles/`（每 json 一架，含 PhysicsParams 与 PX4 侧配套参数）；PX4 机型文件在 `px4.1.15.2/ROMFS/px4fmu_common/init.d-posix/airframes/`，与 PhysicsParams 严格联动（电机坐标 `±ArmLengths/√2`、`KM = CP/CT × D/2π`）。悬停油门：物理值 = `1/推重比`，写入 PX4 时按 `normalizeRotorControls` 的 idle 映射换算 `MPC_THR_HOVER = (物理值 − 0.2) / 0.8`。

| 配置文件 | 构型 | 推重比 | 物理悬停油门 | PX4 机型（MPC_THR_HOVER） |
|---|---|---|---|---|
| `fucker_coax_x050.json` | 共轴双桨，0.35m 桨，1.5kg | 3.12 | 0.32 | `70003_fucker_coax_x050`（0.15） |
| `fucker_coax_x100.json` | 同 X050 物理，调参档更新（HTE/速度域放宽） | 3.12 | 0.32 | `70004_fucker_coax_x100`（0.15） |
| `fucker_coax_x300.json` | 共轴双桨，0.4064m 桨，4.0kg | 2.40 | 0.42 | `70005_fucker_coax_x300`（0.275） |
| `fucker_x450.json` | QuadX，450 轴距，28系电机+12寸桨(4S)，3.12kg | 2.31 | 0.43 | `70006_fucker_x450`（0.30） |

另有 `70001_ytjy_fpv1` / `70002_ytjy_fpv2`（实机 FPV 机）仅 PX4 机型文件，无 vehicles 档案。

**新增机型**：① `airframes/700XX_<机型名>`（**文件名禁止含点号**——rcS 把带点文件当 `xxx.post` 忽略）② `airframes/CMakeLists.txt` 注册 ③ `simulator_mavlink/CMakeLists.txt` 加 `add_custom_target` ④ `configs/vehicles/` 存档 ⑤ `make px4_sitl_default`

## 7. SITL 链路与端口分配

| 端口 | 用途 | 对端 |
|---|---|---|
| TCP 4560 | HIL 物理仿真链路 | UnrealEditor（Windows） |
| UDP 14580→14540 | onboard 实例 | **被 AirSim 控制通道抢占** |
| UDP 14582→14542 | **MAVROS 专属实例** | MAVROS `fcu_url:=udp://:14542@` |
| UDP 18570→14550 | GCS 实例 | QGC 等 |
| UDP 8888 | uXRCE-DDS（`tracker:=uxrce`） | MicroXRCEAgent |
| TCP 41451 | AirSim RPC API | airsim_sensor_* / flylab_gcs |

**坑：14580 会被 AirSim 抢占**——AirSim 以 1Hz 心跳先到先得抢占实例对端锁定，MAVROS 永远连不上。MAVROS 必须走 14582↔14542 专属实例；WSL 重启后跑 `scripts/sync-airsim-ip.sh` 更新 ControlIp。

## 8. 注意事项

- **改质量后**：同步重算 `MPC_THR_HOVER`，否则悬停漂移；settings.json 复制到 Windows 端 `Documents\AirSim\` 后重启 UE 生效
- **雷达配置**：`configs/fast_lio/settings.json` 为 GPU 雷达（`SensorType: 8`，对齐 Mid-360）；`configs/settings.json` 为 CPU 雷达（`SensorType: 6`，**必须 `LimitPoints: false`**，否则被 5000 点上限截断、帧内拼接错位且帧率掉到 ~1.7Hz）
- **雷达时间戳**：AirSim 点云是单时刻瞬时快照，统一填 `time = 1/rotation_hz` 并以 `LidarData.time_stamp` 标定 ROS 偏移；逐点合成时间会反向注入运动剪切导致漂移，勿改回
- **传感器时间轴**：IMU/LiDAR/odom 统一经 `SimClockSync` 映射到同一时间轴（`src/sensors/airsim_common.hpp`），bridge_super 去畸变、FAST-LIO 融合都依赖此约定
- **WSL 时钟漂移**：高负载下漂移可达秒级，timesyncd 步进纠偏会导致时间戳整体跳变（去畸变拖影/FAST-LIO reset），已用 chrony slew 模式根治；验证 `chronyc tracking` 的 System time 应为毫秒级
- **点云抗卡顿**：实时流不允许积压回放——发布 QoS `KeepLast(2) reliable`，下游 Queue Size 全部限深 1~2；GPU 雷达过滤跨卡顿帧（段时刻跨度 > 3× 扫描周期直接丢弃）。诊断：`ros2 topic hz` 的 min 为负 = 时钟被步进，max 飙秒级 = 发布端真断流

## 9. Docker 镜像复现

已导出根文件系统镜像 `flylab-rootfs.tar`（约 32G，gzip 后 10~13G），ROS2 Humble + 全部依赖 + `install/` 编译产物内置。分发：`gzip -1 -k flylab-rootfs.tar`（接收方可 `sha256sum` 校验）。

### 9.1. 导入与运行

```bash
docker import flylab-rootfs.tar flylab:v1

# 三个参数缺一不可
docker run -it --rm \
  --shm-size=8g \                # ① Fast DDS SHM 必需，默认 64M 会让 30Hz 大图转发失败
  --network host \               # ② 与 AirSim 主机 TCP 4560/41451 直连
  -e DISPLAY=$DISPLAY \          # ③ GUI：地面站/RViz（WSLg 或 Linux 宿主 X11）
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  flylab:v1 bash

source /root/flylab/scripts/env.sh   # ROS2 + install/ + 库路径 + Fast DDS SHM 配置
```

- `--shm-size=8g`：`env.sh` 挂的 `fastrtps_shm.xml` 让 900KB 级显示帧走 SHM 零拷贝，`/dev/shm` 满了报 `Failed to create segment` 回退 UDP 掉帧
- `--network host`：PX4 SITL 连 AirSim TCP 4560、传感器/GCS 连 TCP 41451；host 网络下 `env.sh` 自动取 `PX4_SIM_HOST_ADDR` 为默认网关，与 WSL 行为一致
- `-e DISPLAY`：flylab_gcs（PyQt5）和 RViz 是 GUI 程序

### 9.2. 复现飞行实验（Windows + Docker 组合）

与 WSL 架构一致：**Windows 跑 UE 仿真器，容器跑 PX4 + ROS2**。

1. **Windows 侧**：装 Cosys-AirSim 的 UE 项目 `BeimingSim`（不在镜像内需单独分发），`configs/settings.json` 复制到 `Documents\AirSim\`，`ControlIp` 改为**容器所在主机可达的 IP**（UE 以 1Hz 向该地址发心跳）
2. **容器内**：按 §3 流程起 PX4 / 通信链路 / 链路 / 地面站；MAVROS 14542 专属实例的坑（§7）同样存在

### 9.3. 网络拓扑与排障

- `--network host` 时 AirSim 与 Docker 同机则天然可达；跨机 `export PX4_SIM_HOST_ADDR=<AirSim主机IP>` 覆盖
- **Docker Desktop 没有真正的 host 网络**：改端口映射 `-p 4560:4560 -p 41451:41451 -p 14542:14542/udp -p 8888:8888/udp`，并手动 `export PX4_SIM_HOST_ADDR=host.docker.internal`、UE 端 `ControlIp` 填宿主机 IP。强烈建议 Linux 宿主上用 host 网络跑
- 验证连通：容器内 `nc -zv $PX4_SIM_HOST_ADDR 4560`（HIL）、`nc -zv $PX4_SIM_HOST_ADDR 41451`（RPC）

### 9.4. 与 WSL 原环境的差异

| 项 | WSL 原环境 | Docker 容器 |
|---|---|---|
| 时钟同步 | chrony slew 模式（§8） | 用宿主内核时钟，无漂移，无需配置 |
| 摄像头直通 | `usbipd attach`（§3 第 5 步） | `--device=/dev/video0` 挂载 |
| X11 | WSLg 自带 | `-e DISPLAY` + socket 挂载；纯服务器无显示器跑不了 GUI |
| 改代码重编 | `colcon build`（WSL 限 `-j2`） | 同，可放开 `MAKEFLAGS=-j$(nproc)` |
