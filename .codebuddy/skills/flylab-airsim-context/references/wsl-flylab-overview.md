# WSL 端 flylab 项目全貌

> 一句话：PX4 + Cosys-AirSim SITL 无人机仿真实验平台，WSL2 (Ubuntu 22.04) + ROS2 Humble，围绕感知建图、视觉惯导、避障规划与 offboard 控制做自主飞行研究。完整文档见 WSL 端 `~/flylab/README.md`（本文件是其跨端摘要）。

## 目录组成（`~/flylab/`）

| 目录 | 说明 |
|---|---|
| `ros2_ws/` | ROS2 工作区，核心是三个自研包（见下），其余为算法移植与通信栈 |
| `Cosys-AirSim/` | UE 端仿真器源码（5.8），与 Windows 端 BeimingSim 项目配套 |
| `px4.1.15.2/` | PX4 SITL v1.15.2 源码，含多架自研机型 airframes |
| `ORB_SLAM3_FIXED/` | ORB-SLAM3 运行时库 |
| `mavros/`、`libmavconn`、`mavros_msgs`、`px4_msgs`、`airsim_ros_pkgs`/`airsim_interfaces` | MAVROS + uXRCE-DDS + AirSim ROS 封装 |
| `scripts/` | `env.sh`（环境总入口）/ `run-px4.sh`（起 SITL，可换机型/清参数缓存）/ `sync-airsim-ip.sh`（同步 WSL IP 到 UE 端配置）/ SHM 清理 |
| 算法移植包 | `FAST_LIO_ROS2`、`SUPER`(super_planner)、`ego_planner_swarm`、`openvins`、`orb_slam3`、`vins_fusion`、`livox_ros_driver2` |

## 三大自研包（`ros2_ws/src/`）

### flylab_flight（飞行链路）
分层设计：`src/sensors/` 传感器层（`airsim_camera`/`airsim_imu`/`airsim_odom`/`airsim_nav`/`airsim_lidar`，统一经 `SimClockSync` 映射到同一时间轴；`airsim_lidar` 优先调自研碰撞射线雷达 RPC `getRaycastLidarData`（UE 端 SensorType 12，点云 5 floats/点与 GPULidar 同布局），服务端不可用时自动回退 `getGPULidarData`，走快照时间模型）→ `src/bridges/` 桥接层（坐标系 NED↔FLU 转换、断桨门控、时间对齐、点云去畸变）→ `src/control/` 轨迹跟踪（`tracker_mavros`/`tracker_uxrce` 双链路）。

一键 launch（`ros2 launch flylab_flight <名>`）：

| 链路 | launch | 说明 |
|---|---|---|
| 深度建图 | `depth_mapping.launch.py` | 深度图→点云→octomap |
| EGO 避障 | `fly_ego.launch.py` | RViz 点目标→规划→PX4 跟踪 |
| SUPER 避障 | `fly_super.launch.py` | 同上，LiDAR 路线 |
| OpenVINS | `openvins_msckf.launch.py` | 双目 VIO，断桨联动启停 |
| VINS / ORB-SLAM3 / FAST-LIO | `vins_mono/stereo`、`orb_slam3`、`fast_lio` | 各里程计/SLAM 航迹 |

### flylab_vision（视觉算法）
算法库纯 OpenCV 不依赖 ROS；节点继承 `DisplayVisionNode` 基类只写算法差异。任务覆盖 detect/obb/pose/segment/classify/semantic/face/track/marker(ArUco 降落状态机)/depth/wilor 等；onnxruntime 预编译推理（`thirdparty/`）+ 预训练权重（`models/`）；`src/control/` 提供 MAVROS / uXRCE 双链路 offboard 执行层，用于视觉反控（目标追踪、机体跟随、ArUco 降落等）。

### flylab_gcs（地面站）
PyQt5 地面站（`ros2 run flylab_gcs ground_station`）：视频源切换（UE 相机/本机摄像头/视频文件）、PX4 SITL 与通信链路一键启停、飞行控制（经 AirSim RPC→PX4：解锁/起飞/降落/悬停/返航）、云台控制、算法标注 overlay、算法信息面板；当前显示帧 30Hz 转发到 `/flylab/display/image_raw` 供算法节点消费（算法吃原始话题用 `-p image_topic:=` 覆盖）。

## 配置体系（与 Windows 端相关）

- `configs/settings.json` — AirSim 全量场景模板（全部相机/IMU/LiDAR/GPS/载具 PhysicsParams），**复制到 Windows 端 `Documents\AirSim\settings.json` 重启 UE 生效**
- `configs/vehicles/*.json` — 每架自研机型一份档案（PhysicsParams + PX4 侧配套参数），如 `fucker_coax_x050/x100/x300`、`fucker_x450`
- `configs/<链路>/` — 各算法链路的场景参数（ego_planner、super_planner、openvins、fast_lio 等）

### PhysicsParams ↔ PX4 参数联动（改质量必读）

- 单桨最大推力 `= CT × 1.225 × (MaxRPM/60)² × PropDiameter⁴` [N]；推重比 `= RotorCount × max_thrust / (Mass × 9.81)`
- 物理悬停油门 `= 1/推重比`；写入 PX4 按 idle 映射换算：`MPC_THR_HOVER = (物理值 − 0.2) / 0.8`
- 改 `PhysicsParams.Mass` 后必须同步重算 PX4 侧 `MPC_THR_HOVER`（否则悬停漂移），且重启 UE 生效
- PX4 机型文件在 `px4.1.15.2/ROMFS/px4fmu_common/init.d-posix/airframes/700XX_*`，与 vehicles json 严格联动（电机坐标 `±ArmLengths/√2`、`KM = CP/CT × D/2π`）

## WSL 端快速命令

```bash
source ~/flylab/scripts/env.sh          # ROS2 + install/ + Fast DDS SHM
cd ~/flylab/ros2_ws && colcon build --parallel-workers 2   # WSL 限并行度
bash scripts/run-px4.sh                 # PX4 SITL（默认 none_iris）
bash scripts/run-px4.sh none_fucker_x450 --clean   # 换机型
bash scripts/sync-airsim-ip.sh          # WSL IP 变了先跑这个
MicroXRCEAgent udp4 -p 8888             # uXRCE 链路
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14542@ -p tgt_system:=1
```
