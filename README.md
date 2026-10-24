# FLYLAB —— PX4 + AirSim SITL 无人机仿真实验平台

WSL2 (Ubuntu 22.04) + ROS2 Humble 仿真环境，围绕 PX4 + Cosys-AirSim 做无人机自主飞行研究：感知建图、视觉惯导、避障规划与 offboard 控制

## 1. 工作区总体

| 目录 | 说明 |
|---|---|
| `ros2_ws/` | ROS2 工作空间，核心是 `src/flylab_flight` 包（飞行链路）与 `src/flylab_vision` 包（视觉算法），其余为各算法移植包  |
| `Cosys-AirSim/` | UE 端仿真器 |
| `px4.1.15.2/` | PX4 SITL v1.15.2 源码（默认机型 `none_iris`，自定义机型见下方"机型库与 PX4 机型文件"） |
| `ORB_SLAM3_FIXED/` | ORB-SLAM3 运行时库 |
| `mavros/` `libmavconn/` `mavros_msgs/` | MAVROS 通信栈 |
| `scripts/env.sh` | 启动脚本与环境变量 |

---

## 2. flylab_flight 核心包

### 2.1. 功能链路一览

| 链路 | 一键启动 | 场景配置 | 说明 |
|---|---|---|---|
| 感知建图 | `ros2 launch flylab_flight depth_mapping.launch.py` | octomap 参数内联于 launch | 深度图 → 点云 → 占据地图 |
| EGO 避障飞行 | `ros2 launch flylab_flight fly_ego.launch.py` | `configs/ego_planner/` | RViz 点目标点 → EGO 规划 → PX4 跟踪，到达后自动交还 QGC；`tracker:=mavros\|uxrce` 选择跟踪链路 |
| OpenVINS MSCKF | `ros2 launch flylab_flight openvins_msckf.launch.py` | `configs/openvins/` + `settings.json` | 双目 VIO，断桨联动启停 |
| VINS 单目/双目 | `ros2 launch flylab_flight vins_mono.launch.py` / `vins_stereo.launch.py` | `configs/vins_*/` | VINS-Fusion 航迹 |
| ORB-SLAM3 | `ros2 launch flylab_flight orb_slam3.launch.py` | `configs/orb_slam3/` | 双目+IMU SLAM |
| FAST-LIO 建图 | `ros2 launch flylab_flight fast_lio.launch.py` | `configs/fast_lio/` | LiDAR-惯性里程计 |
| SUPER 避障飞行 | `ros2 launch flylab_flight fly_super.launch.py` | `configs/super_planner/` | RViz 点目标点 → SUPER 规划 → 桥接 → PX4 跟踪；`tracker:=mavros\|uxrce` 选择跟踪链路 |

### 2.2. 节点一览

传感器层 (`src/sensors/`) 与算法解耦：算法特化的适配（坐标系/门控/时间对齐）下沉到桥接层 (`src/bridges/`)

| 传感器节点 | 发布话题 | 说明 |
|---|---|---|
| `airsim_camera` | `/airsim/cam/<名>/image_raw` | 通用相机（深度/双目/单目参数化），多相机同次 RPC 硬件同步；`grayscale:=true` 输出 mono8；`image_type` 选 AirSim 图像类型 |
| `airsim_imu` | `/airsim/imu/data_raw`（FRD 原生）<br>`/airsim/imu/data`（FLU 标准） | 200Hz；时间戳统一映射仿真时钟；重复采样去重；`inject_bias` 可选偏置注入 |
| `airsim_odom` | `/airsim/odom`（world 系 ENU）+ TF | 真值里程计 100Hz；时间戳与 LiDAR/IMU 同一时间轴 |
| `airsim_nav` | `/airsim/gps` `/airsim/baro` `/airsim/mag` | 低频导航传感器 50Hz |
| `airsim_lidar` | `/airsim/lidar/points_raw` | 原始点云（传感器系 NED）+ 逐点时间模型 |

| 桥接节点 | 输入 → 输出 | 说明 |
|---|---|---|
| `bridge_vio` | 相机+原始 IMU → `/vio/*` | OpenVINS 断桨门控：降落停发、解锁恢复；数据原样转发 |
| `bridge_fastlio` | `points_raw` → `/airsim/lidar/points` | NED→FLU + 挂载外参静态 TF `base_link→lidar_link` + 恒等 TF `body→base_link` |
| `bridge_super` | `points_raw`+odom → `/cloud_registered` `/lidar_slam/odom` | 点云→world 系 ENU：位姿按点云时刻插值对齐 + 逐点去畸变 + 机身回波过滤 |

其他节点：`tracker_uxrce`/`tracker_mavros`（轨迹跟踪+PX4 混控，`tracker:=mavros\|uxrce` 选择链路，uxrce = uXRCE-DDS 直连）、`super_cmd_bridge`（mars→标准 PositionCommand）、`octomap_dynamic`（建图动态清理）、`vins_arming_switch`（VINS 起飞联动）

### 2.3. 环境依赖

| 依赖 | 说明 |
|---|---|
| ROS2 Humble + PX4 SITL v1.15.2 | PX4 消息经 MicroXRCEAgent（`udp4 -p 8888`）桥接 |
| Cosys-AirSim 5.8 | UE 端仿真器，场景按链路选用 `configs/` 下的 json |
| ego_planner / ov_msckf / orbslam3 / fast_lio / super_planner | 工作空间内移植的算法包（fast_lio 依赖 livox_ros_driver2） |

---

## 3. 快速开始

```bash
# 1. 编译
source /opt/ros/humble/setup.bash
export MAKEFLAGS=-j2                    # WSL 下需限制并行度防一手
cd ~/flylab/ros2_ws && colcon build --parallel-workers 2
source install/setup.bash

# 2. 启动仿真环境
#    UE/AirSim 场景配置: 把 configs/ 下对应 json 复制到 Windows 端 Documents\AirSim\settings.json 后重启 UE
bash /root/flylab/scripts/run-px4.sh                          # PX4 SITL (默认机型 none_iris)
bash /root/flylab/scripts/run-px4.sh none_fucker_3120g_tw23 --clean   # 指定机型启动 (换机型用--clean 清缓存)

MicroXRCEAgent udp4 -p 8888
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14542@ -p tgt_system:=1 -p tgt_component:=1

# 3. 启动链路（跨主机连接 AirSim 需先 export PX4_SIM_HOST_ADDR=<AirSim IP>）
ros2 launch flylab_flight fly_ego.launch.py                 # EGO 避障 (默认 MAVROS 跟踪, 专属端口 14542)
ros2 launch flylab_flight fly_ego.launch.py tracker:=uxrce  # EGO 避障 (uXRCE-DDS 跟踪)
ros2 launch flylab_flight fly_super.launch.py               # SUPER 避障 (GPULidar)

# 4. 地面站
ros2 run flylab_gcs ground_station          # 地面站总界面 (算法启停 / ArUco降落按钮)
```

---

## 4. 视觉算法实验区

### 4.1. flylab_gcs 地面站

可视化与算法实验工具集（`ros2_ws/src/flylab_gcs`），模块化结构：界面 / 抓取 / ROS 三层解耦

| 模块 | 说明 |
|---|---|
| `config.py` | 全局配置：算法注册表（`ALGORITHMS`）、话题、端口，改这里 |
| `video_label.py` | 视频控件：等比显示 / 深度伪彩 / 拖拽框选 / 模型标注叠加（bbox/point/polyline/text 四种图元） |
| `camera_grabber.py` | 抓取线程：AirSim RPC / 本机 V4L2 摄像头 / 视频文件，三源统一 |
| `camera_widget.py` | 输入源控件：下拉选源 + 播放控制条 + 云台方向键 |
| `ros_node.py` | ROS 节点：算法启停指令发布、显示画面转发、按算法分组的标注订阅 |
| `overlay.py` | 模型输出叠加接口（按算法分组的消息 + 标注 schema） |
| `main_window.py` | 主窗口：左画面 + 右侧面板（算法下拉框 + 运行/取消 + ArUco降落按钮 + 通信链路按钮），管理算法/降落/桥进程生命周期 |

```bash
ros2 run flylab_gcs ground_station   # 地面站总界面
```

**新算法接入**：在 `config.ALGORITHMS` 加一项 `{name, cmd, start, stop}`——节点遵循「收到 start 开始、收到 stop 停止、标注按组发到 `/flylab/annotations`」即可出现在下拉框；`cmd` 不填则假定节点已在外部运行，只发启停指令。

**通信链路按钮**（右侧「通信链路」分组，免开终端）：
- `uXRCE-DDS Agent (8888)` → `MicroXRCEAgent udp4 -p 8888`（供 `tracker:=uxrce` / `aruco_landing(link:=uxrce)` 用）
- `MAVROS 桥 (14542)` → `mavros_node --ros-args -p fcu_url:=udp://:14542@ -p tgt_system:=1 -p tgt_component:=1`（**必须用 14542 专属实例**，14580 会被 AirSim 抢占）
- 按钮是 toggle：点一下起、再点一下停；桥进程以独立进程组启动，停止时整组回收（不留孤儿进程）；进程异常退出时按钮自动复位并报错
- 两条链路**互斥**：开启一条先停另一条
- **「ArUco降落」按钮的前置条件包含"已选择链路"**：未选链路时按钮置灰不可点；所选链路同时作为降落节点的 `link` 参数（`link:=mavros` / `link:=uxrce`）下发；降落进行中禁止切换链路（换桥会断掉正在跑的降落控制）

**显示画面转发**：地面站把当前显示帧（深度伪彩帧除外）以 30Hz 原分辨率发到 `/flylab/display/image_raw`，算法节点订阅它即可处理「用户正在看的画面」——无论输入源是 UE 相机、本机摄像头还是视频文件，切换对算法透明。无订阅者时自动跳过序列化不占 CPU；算法要吃原始相机话题时用 `-p image_topic:=/airsim/cam/front_cam/image_raw` 覆盖。

**标注接口**（`overlay.py`）：消息按算法分组，各组独立替换/清屏，多算法同时叠加互不干扰。坐标一律原图像素，算法停止前发 `{"algo": ..., "items": []}` 清自己的组；旧格式纯列表仍向后兼容：

```json
{"algo": "yolo", "items": [
  {"type": "bbox",     "box": [x, y, w, h],   "label": "car 0.92", "color": [0,255,0]},
  {"type": "point",    "point": [x, y],       "label": "..."},
  {"type": "polyline", "points": [[x,y]],     "close": true, "color": [0,255,0]},
  {"type": "text",     "pos": [x, y],         "text": "..."}]}
```

稠密图像类输出（如深度估计）用 `image` 图元：`{"type": "image", "pos": [x,y], "w": 640, "alpha": 0.45, "data": "<base64 JPEG>"}`——pos 为原图左上角、w 为绘制宽度（原图像素，高按纵横比），alpha 叠加不透明度，地面站对同一份 data 只解码一次。

**其他**：名字含 `depth` 的源自动伪彩（0.2~20m）；UE 断线自动重连，只保留最新帧不积压；相机需在 UE 端 `settings.json` 配置过（`configs/settings.json` 已含全部）。依赖 `msgpack-rpc-python`、PyQt5、OpenCV（GTK 后端，勿装 pip 版 opencv-python——其自带 Qt 与 PyQt5 冲突）。

### 4.2. flylab_vision 算法包

ament_cmake 包（`ros2_ws/src/flylab_vision`），算法与地面站彻底解耦：核心库纯 OpenCV 不依赖 ROS，节点继承 `DisplayVisionNode` 基类只填算法差异（`process()`/`clear_json()`），通信/启停/标注/诊断链路全部下沉到公共层。

```
flylab_vision/
├── include/common/                          # ★ 节点公共层
│   ├── image_utils.hpp                      #   to_bgr / 默认模型路径解析 (vision_common 库)
│   └── display_vision_node.hpp              #   展示型节点基类: 地面站链路 (订阅/启停/标注/诊断) 全封装
├── include/aruco/aruco_detector.hpp         # 核心检测库接口 (无 ROS 依赖)
├── include/yolo/yolo_detector.hpp           # YOLO 目标检测库接口 (pimpl 隔离 onnxruntime)
├── include/yolo/yolo_classifier.hpp         # YOLO 图像分类库接口 (同上)
├── include/yolo/yolo_depth.hpp              # YOLO 单目深度估计库接口 (同上)
├── include/yolo/yolo_obb.hpp                # YOLO OBB 旋转框检测库接口 (同上)
├── include/yolo/yolo_pose.hpp               # YOLO 姿态估计库接口 (同上)
├── include/yolo/yolo_segment.hpp            # YOLO 实例分割库接口 (同上)
├── include/yolo/yolo_semantic.hpp           # YOLO 语义分割库接口 (同上)
├── include/control/                         # PX4 执行层 (可复用组件, 业务无关)
│   ├── px4_offboard_client.hpp              #   总入口 (使用文档 + 兼容别名)
│   ├── px4_offboard_client_base.hpp         #   公共接口 (基类 + 坐标约定: FRD 速度 / NED 位置)
│   ├── px4_offboard_client_mavros.hpp       #   MAVROS 链路实现 (与 tracker_mavros 同一套话题/服务)
│   └── px4_offboard_client_uxrce.hpp        #   uXRCE-DDS 直连实现 (与 tracker_uxrce 同一套 /fmu 话题)
├── src/common/image_utils.cpp               # 公共工具实现 (仅节点链接, 算法库不依赖)
├── src/aruco/
│   ├── aruco_detector.cpp                   # 检测 / IPPE_SQUARE 位姿
│   ├── aruco_node.cpp                       # 检测展示节点
│   └── aruco_landing_node.cpp               # 降落业务节点 (三态状态机)
├── src/yolo/                               # 每任务一个子目录 (算法库 + 展示节点成对放置)
│   ├── detector/
│   │   ├── yolo_detector.cpp               # onnxruntime 推理 (letterbox/解码/NMS)
│   │   └── yolo_detector_node.cpp          # 检测展示节点 (yolo_detector_node)
│   ├── classifier/
│   │   ├── yolo_classifier.cpp             # 分类 onnxruntime 推理 + top-k
│   │   └── yolo_cls_node.cpp               # 分类展示节点 (yolo_cls_node)
│   ├── depth/
│   │   ├── yolo_depth.cpp                  # 深度 onnxruntime 推理 + 伪彩 JPEG 编码
│   │   └── yolo_depth_node.cpp             # 深度展示节点 (yolo_depth_node, image 图元)
│   ├── obb/
│   │   ├── yolo_obb.cpp                    # OBB 旋转框检测: onnxruntime 推理 + 旋转框 NMS (rotatedRectangleIntersection)
│   │   └── yolo_obb_node.cpp               # OBB 展示节点 (yolo_obb_node, 4 角点闭合多边形 + 文字标注)
│   ├── pose/
│   │   ├── yolo_pose.cpp                   # 姿态估计: onnxruntime 推理 + 17 关键点解码 + 水平框 NMS
│   │   └── yolo_pose_node.cpp              # 姿态展示节点 (yolo_pose_node, 骨架连线 + 可见关键点)
│   ├── segment/
│   │   ├── yolo_segment.cpp                # 实例分割: onnxruntime 推理 + 掩码轮廓提取
│   │   └── yolo_segment_node.cpp           # 实例分割展示节点 (yolo_segment_node, 掩码多边形 + 检测框)
│   └── semantic/
│       ├── yolo_semantic.cpp               # 语义分割: onnxruntime 推理 + cityscapes 配色 + 中值滤波
│       └── yolo_semantic_node.cpp          # 语义分割展示节点 (yolo_semantic_node, 配色画中画 + 图例)
├── thirdparty/onnxruntime/                  # 预编译推理库 (系统 OpenCV 4.5.x DNN 不支持 yolo26)
├── models/<task>/                           # 预训练权重按任务分类
└── CMakeLists.txt / package.xml
```

| 目标 | 说明 |
|---|---|
| `libvision_common` | 节点层公共工具（`to_bgr` 图像转换 / 默认模型路径解析）；仅供节点链接，算法库保持无 ROS 依赖 |
| `DisplayVisionNode`（基类） | 展示型节点公共链路：订阅地面站显示帧、`start/stop` 启停与清屏、标注发布（base64 大图自动限深 QoS）、2s 无帧自检告警、首帧/首帧推理日志；子类只填指令词/分组/文案 + 实现 `process()`/`clear_json()` |
| `libaruco_detector` | 纯算法库：检测 + 位姿解算（等价 legacy `estimatePoseSingleMarkers`）+ 分组标注 JSON；兼容 OpenCV 4.5（Humble 系统源）与 4.7+ 双 API |
| `aruco_node` | 订阅显示帧话题 + 启停指令，检测结果转 JSON 标注回传；每 2s 自检——收不到帧主动告警（地面站未运行/画面源无图/话题不一致） |
| `aruco_landing_node` | 视觉降落节点（见 4.3）；执行层经 `px4_offboard_client` 双链路组件，`link` 参数选 MAVROS / uXRCE |

```bash
ros2 run flylab_vision aruco_node --ros-args \
    -p image_topic:=/flylab/display/image_raw \   # 算法图像输入 (默认地面站转发帧)
    -p dict_name:=DICT_4X4_1000 \                 # 码型字典 (Prometheus 6x6 码用 DICT_6X6_250)
    -p marker_length:=0.2 \                       # 码物理边长(米), 影响 3D 位姿尺度
    -p calib:=/path/to/calib.yaml                 # 相机标定 (可选; 无则按 FOV 估算, 位姿仅供参考)
```

节点可视化输出（每个码 7 个图元）：绿色四边形轮廓（贴码真实形状）+ 红/蓝/绿三条位姿轴 + 红中心点 + 红首角点 + 黄色 `id=z距离` 文字，全部经统一标注接口叠加到地面站画面。

**新增算法**（展示型，接入地面站）：
1. 并列建 `include/<算法>/` + `src/<算法>/`——算法库纯 OpenCV 不碰 ROS，标注序列化函数放库里
2. 节点继承 `DisplayVisionNode`，构造函数声明参数并加载算法后调 `log_ready()`，实现 `process()`（逐帧推理 → 标注 JSON）与 `clear_json()`（停止清屏）即可，通信代码零编写
3. CMakeLists 两行模板：`flylab_algo(<库名> src/<算法>/<库>.cpp ${ORT_LIB})` + `flylab_node(<节点名> src/<算法>/<节点>.cpp <库名>)`
4. `config.ALGORITHMS` 注册即出现在地面站下拉框，体验与 ArUco 完全一致

参考实现：`yolo_obb_node.cpp`（最简展示节点，~110 行）。

**独立调试 CLI**：根目录 `python3 aruco_detector.py --image/--video/--camera/--gen`（图像/视频/摄像头检测与测试图生成），功能与包内节点共用同一套检测参数约定。


## 5. 载具物理参数配置化（PhysicsParams）

多旋翼物理参数已从源码硬编码改为 settings.json 配置：在载具段下加 `PhysicsParams` 即可换机型/挂载荷/调推重比，改完**无需重编译**，重启仿真生效。改动只存在于 Windows 侧 UE 项目（`BeimingSim/Plugins/AirSim/`），`Model` 字段继续作为基础机架模板，`PhysicsParams` 在其上覆盖，未写的键沿用模板默认值，向后兼容。

```json
"Vehicles": {
    "PX4": {
        "VehicleType": "PX4Multirotor",
	    "PhysicsParams": {
    		"Mass": 3.120,
            "MotorAssemblyWeight": 0.090,
            "RotorCount": 4,
    		"ArmLengths": 0.225,
            "RotorZ": 0.030,
    		"BodyBoxX": 0.200,
            "BodyBoxY": 0.200,
            "BodyBoxZ": 0.100,
    		"CT": 0.130,
            "CP": 0.055,
            "MaxRPM": 6800,
    		"PropDiameter": 0.3048,
            "DragCoefficient": 0.4
  		}
	}
}
```

配套换算（改后 PX4 侧须联动）：

| 量 | 公式 |
|---|---|
| 单桨最大推力 | `CT × 1.225 × (MaxRPM/60)² × PropDiameter⁴` [N] |
| 推重比 | `RotorCount × max_thrust / (Mass × 9.81)` |
| 质量上限 | `Mass < RotorCount × max_thrust / 10`（即推重比必须 > 1） |
| 悬停油门 | `MPC_THR_HOVER ≈ Mass × 9.81 / (RotorCount × max_thrust)` = **1/推重比** |

- **推力与油门是线性关系**（Cosys-AirSim 源码 `RotorActuator.hpp`：`thrust = control_signal × max_thrust`），所以 `MPC_THR_HOVER` 直接取 1/推重比
- 挂载荷只改两处：`Mass` 加载荷重量 + 按上式重算 `MPC_THR_HOVER`，缺一会导致悬停漂移
- `DragCoefficient` 是绝对值（默认机架 0.325、Flamewheel 系 1.3），不是倍数写法
- 限制：转子布局仅支持对称 X 型（QuadX/HexX/OctoX）、机臂统一长度；六/八轴配置时 UE 外观仍是四旋翼模型

## 6. 机型库与 PX4 机型文件

自研机型的完整参数档案存放在 `ros2_ws/src/flylab_flight/configs/vehicles/`，每个 json 一架飞机，含 `PhysicsParams`、PX4 侧配套参数和推导公式：

| 配置文件 | 构型 | 推重比 | 悬停油门 | PX4 机型 |
|---|---|---|---|---|
| `airsim_default_1000g_tw17.json` | AirSim 默认：F450 构型，GWS 9x5 桨+MT2212 电机，1.0kg | 1.70 | 0.59 | `10016_none_iris` |
| `fucker_3120g_tw23.json` | 450 轴距，2814 电机+12寸桨（4S），3.12kg | 2.31 | 0.43 | `70003_fucker_3120g_tw23` |
| `fucker_2.00kg_tw15.json` | 450 轴距，2212 电机+10寸桨（3S），2.0kg | 1.50 | 0.67 | `70004_fucker_2000g_tw15` |

PX4 侧机型文件在 `px4.1.15.2/ROMFS/px4fmu_common/init.d-posix/airframes/`，与 `PhysicsParams` 严格联动：电机坐标 `±ArmLengths/√2`、反扭矩系数 `KM = CP/CT × D/2π`、`MPC_THR_HOVER = 1/推重比`，速度/加速度限制按推重比折算。

**新增一架机型的步骤**：

1. 新建机型文件 `airframes/700XX_fucker_<重量>_<推重比>`（**文件名禁止含点号 `.`**——rcS 会把带点的文件当 `xxx.post` 附加文件忽略，导致启动报 "no autostart file found"；小数点一律去掉，如 `tw23` 而非 `tw2.3`）
2. 在 `airframes/CMakeLists.txt` 的 `px4_add_romfs_files` 中注册（不注册不会拷入 `build/etc`，运行时找不到）
3. 在 `src/modules/simulation/simulator_mavlink/CMakeLists.txt` 加 `add_custom_target(none_<机型名> ...)`（参考现有 70003/70004 写法，`PX4_SYS_AUTOSTART=700XX`）
4. 在 `configs/vehicles/` 存一份参数 json 存档

5. 改完任一机型文件/CMakeLists 后需重新构建：

   ```bash
   cd px4.1.15.2 && make px4_sitl_default
   ```

## 7. SITL 链路与端口分配

PX4 SITL + Cosys-AirSim + ROS2 之间的全部通信端口（配置位于 `px4.1.15.2/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink` 与 Windows 端 `settings.json`）：

| 端口 | 用途 | 对端 |
|---|---|---|
| TCP 4560 | HIL 物理仿真链路（传感器→PX4，执行器→UE） | UnrealEditor（Windows） |
| UDP 14580→14540 | onboard 实例 | **被 AirSim 控制通道抢占** |
| UDP 14582→14542 | **MAVROS 专属实例** | MAVROS `fcu_url:=udp://:14542@` |
| UDP 18570→14550 | GCS 实例 | QGC 等 |
| UDP 8888 | uXRCE-DDS（`tracker:=uxrce` 链路） | MicroXRCEAgent |
| TCP 41451 | AirSim RPC API（传感器/相机直连） | airsim_sensor_* / flylab_gcs |

**坑 ：14580 onboard 实例会被 AirSim 抢占。** 即使 `settings.json` 里 `UseTcp: true`，Cosys-AirSim 仍会从 Windows 向 `ControlIp:ControlPortRemote`（即 WSL 的 14580）发 1 Hz 心跳；PX4 的 MAVLink 实例只锁定一次对端（`mavlink_receiver.cpp`），AirSim 先到先得，把整条链路的数据流全部导向 UE，导致 MAVROS `/mavros/state.connected` 永远为 false。因此 MAVROS 必须使用 14582↔14542 专属实例，不要动 14580

## 8. 注意事项

- **PhysicsParams 改质量后**：需同步重算 PX4 的 `MPC_THR_HOVER`，否则悬停漂移；settings.json 改动复制到 Windows 端 `Documents\AirSim\` 后重启 UE 生效
- **GPULidar 与 CPU 雷达配置**：`configs/fast_lio/settings.json` 为 GPU 雷达（`SensorType: 8`，参数对齐 Livox Mid-360），FAST-LIO 与 SUPER 链路共用，对应 launch 默认 `gpu_lidar:=true`；`configs/settings.json` 为 CPU 雷达（`SensorType: 6`，**必须带 `LimitPoints: false`**，否则每 tick 扫描被 5000 点上限截断，一圈分片采样导致帧内拼接错位且实际帧率掉到 ~1.7Hz）。仿真无法复现 Mid-360 的非重复菱形扫描，以每帧均匀网格 ~2 万点近似
- **雷达时间戳约定**：AirSim 点云是单时刻瞬时快照（整帧同一位姿一次生成，无扫描内运动畸变），传感器节点统一填 `time = 1/rotation_hz` 并以 `LidarData.time_stamp` 标定 ROS 偏移，LiDAR/IMU 映射到同一时间轴；逐点合成时间会反向注入运动剪切导致平移漂移，勿改回
- **传感器时间轴约定**：`airsim_imu`/`airsim_lidar`/`airsim_odom` 的时间戳统一经 `SimClockSync` 映射（ros = offset + 仿真时间，见 `src/sensors/airsim_common.hpp`），三路数据落同一时间轴；跨节点混用（如 bridge_super 对点云插值 odom 位姿、FAST-LIO 融合雷达+IMU）都依赖这一约定，自定义新传感器节点请保持
- **WSL 系统时钟漂移**：WSL2 高负载下系统时钟漂移可达秒级（`chronyc tracking` 实测 RMS offset ~4.3s），`systemd-timesyncd` 纠偏采用"步进"跳变（实测一次向后跳 2.29s），导致 IMU/LiDAR/odom 时间戳整体跳变且三路重标定时刻不一致 → bridge_super 去畸变整帧拖影、FAST-LIO 收到倒退时间戳强制 reset、RViz 卡顿后涌出积压旧帧。**已改用 chrony slew 模式根治**（渐变调时钟速率，时钟永不跳变，`makestep 1 3` 仅启动初期允许步进），timesyncd 已 disable+mask。验证：`chronyc tracking` 的 `System time` 应为毫秒级
- **点云链路抗卡顿设计**：实时传感器流不允许积压回放——`airsim_lidar` 发布 QoS 为 `KeepLast(2) reliable`（下游卡顿只丢帧不补发旧帧），两个 bridge 订阅/发布同样限深 2；GPU 雷达过滤**跨卡顿帧**（UE 卡顿恢复后同一旋转周期累积停顿前后两段渲染，段时刻跨度 > 3× 扫描周期的帧直接丢弃，否则逐点去畸变按停顿间隔摆点 → 整帧拖影）；RViz 全部点云显示 `Queue Size: 1`（默认 0 为无界，渲染一卡就积压回放）。诊断依据：`ros2 topic hz` 的 **min 为负 = 系统时钟被步进**（接收间隔不可能为负），max 飙到秒级 = 发布端真断流
