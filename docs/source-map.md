# 源码地图 —— 每个"本质东西"的代码在哪

> 所有路径均在本机，全部可直接打开读源码。

## 1. 编排层（壳）

| 东西 | 位置 | 说明 |
|---|---|---|
| 全部别名 | `~/.bashrc:119-162` | `ytjy-*` 别名定义 |
| 各课窗格布局 | `~/.config/terminator/config` | `[layouts]` 段，每课一个 `[[ytjy-sitl-lessonN]]` |
| 多机 PX4 脚本 | `/home/hw/sitl/px4.1.15.2/ytjy/lesson5.sh` + `killpx4.sh` | 副本在 `origins/` |

## 2. PX4 侧

| 东西 | 位置 |
|---|---|
| PX4 源码本体 | `/home/hw/sitl/px4.1.15.2/` (v1.15.2) |
| SITL 机型目录 | `ROMFS/px4fmu_common/init.d-posix/airframes/` |
| 默认机型 none_iris | 同上 `10016_none_iris`（副本 `origins/airframes/`） |
| 自研 FPV1 | 同上 `70001_ytjy_fpv1`（副本 `origins/airframes/`） |
| 自研 FPV2 (lesson12/13 用) | 同上 `70002_ytjy_fpv2`（副本+中文注释 `origins/airframes/`） |
| PX4 参数缓存 | `build/px4_sitl_default/rootfs/parameters*`（init-env 删的就是它） |

## 3. ROS2 工作空间

工作空间：`/home/hw/sitl/hw-ros2/ros2`（colcon 构建，`.bashrc` 已 source `install/local_setup.bash`）

### hw_insight 包（课程主包）
`src/hw_insight/`

**Python 节点**（`src/hw_insight/hw_insight/*.py`，与 `setup.py` 的 entry_points 对应）：

| 节点 | 用在 | 作用 |
|---|---|---|
| `keyboard_velocity.py` | 3/5/6/12/13 键盘窗格 | 按键→NED 速度消息 |
| `move_velocity.py` | lesson3 | 速度控制→PX4 offboard |
| `multi_move_velocity.py` | lesson5 | 多机版速度控制（按 px4_instance 分流） |
| `keyboard_position.py` / `move_position.py` | 备选 | 位置版 |
| `octomap_dynamic.py` | lesson4 变体 | 动态 octomap |
| `lesson6_yolo.py` | lesson6 | YOLOv8 图像检测 |
| `lesson6_track.py` | lesson6 | 检测框→追踪速度指令 |
| `lesson8_vins_switch.py` | lesson8/9 | VINS 配置切换胶水节点 |
| `lesson9_airsim_ros2_topic.py` / `lesson9_only_imu.py` | lesson9 变体 | 数据通路实验 |
| `lesson12_px4_ego_tracker.py` | lesson12 | B样条→PX4 offboard 跟踪器 |
| `lesson13_px4_ego_tracker.py` | lesson13 | 速度版跟踪器 |
| `offboard.py` / `px4_test.py` / `msg_px4_fmu_out_vehicle_status.py` / `rk3588_yolo.py` | 实验/工具 | — |

**launch 文件**：`src/hw_insight/launch/lessonN.launch.py`（副本+注释在 `origins/launch/`）
**RViz 配置**：`src/hw_insight/rviz/lessonN.rviz`
**EGO-Planner launch（真正的参数文件）**：`src/hw_insight/ego_planner/lesson12/precision.launch.py`、`lesson13/speed.launch.py`（副本 `origins/ego_planner/`）

### 其他相关包
| 包 | 位置 | 用在 |
|---|---|---|
| `airsim_ros_pkgs`（含 `ytjy_airsim_fast`、`ytjy_airsim_lesson12` 自研节点） | `src/airsim_ros_pkgs/`（C++） | 4/8/9/10/11/12/13 |
| `vins`（VINS-Fusion） | `src/vins/`，配置 `src/vins/config/ytjy/lessonN_extrinsics.yaml` | 7/8/9 |
| OpenVINS | `src/hw_insight/openvins/lesson10/estimator_config.yaml` | 10 |
| ORB-SLAM3 | `src/orb_slam3/`（算法）+ `src/hw_insight/orb_slam3/lesson11/stereo-inertial.yaml`（配置） | 11 |
| `ego_planner`（规划器本体，C++） | `src/hw_insight/ego_planner/` | 12/13 |
| `hw_interface` | 自定义消息 `HWSimpleKeyboardInfo` 等 | 3+ |

## 4. 课程唯一独立脚本
`/home/hw/sitl/lesson2/hw-avoidance-simple.py`（副本+注释 `origins/hw-avoidance-simple.py`）—— 唯一不用 ROS2 的课，MAVSDK 直连 `udp://:14540`。

## 5. 数据流一张图

```
Windows 宿主: UE/AirSim 场景
      │ RPC
      ▼
airsim 节点 (airism_node / ytjy_airsim_fast / ytjy_airsim_lesson12)
      │ 图像/深度/IMU/GPS/真值里程计
      ▼
感知: depth_image_proc+octomap(4) │ YOLO(6) │ VINS(7,8,9) │ OpenVINS(10) │ ORB-SLAM3(11)
      │
      ▼
规划: EGO-Planner (12/13)
      │ B样条轨迹
      ▼
跟踪: lesson12/13_px4_ego_tracker ──┐
控制: move_velocity / keyboard ─────┤ MicroXRCEAgent(8888) ↔ PX4 uXRCE client
MAVSDK(2) ── udp://:14540 ──────────┘ PX4 SITL (none_iris / ytjy_fpv2)
```
