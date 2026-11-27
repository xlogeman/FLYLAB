# 双边联排手册（Windows BeimingSim ↔ WSL flylab）

## 网络拓扑与端口表

| 端口 | 用途 | 对端 | 所属侧 |
|---|---|---|---|
| TCP 4560 | HIL 物理仿真链路（PX4 连这里） | UnrealEditor | Windows 监听 |
| TCP 41451 | AirSim RPC API（传感器节点、地面站、云台、飞行控制） | airsim_sensor_* / flylab_gcs | Windows 监听 |
| UDP 14580→14540 | PX4 onboard 实例 | **被 AirSim 1Hz 心跳抢占，MAVROS 永远连不上，勿用** | WSL |
| UDP 14582→14542 | **MAVROS 专属实例** | MAVROS `fcu_url:=udp://:14542@` | WSL |
| UDP 18570→14550 | GCS 实例 | QGC 等 | WSL |
| UDP 8888 | uXRCE-DDS | MicroXRCEAgent | WSL |

连接方向：WSL → Windows（4560/41451 主动连出）；Windows `ControlIp` → WSL（UE 以 1Hz 向该地址发心跳，抢占 onboard 实例）。

**雷达数据链路（2026-11-27 起）**：UE 端 Lidar1 已切自研碰撞射线雷达 SensorType 12（RaycastLidar，Windows 端 UE 插件实现，浮点精确碰撞射线，无渲染深度精度问题）。WSL 侧 `airsim_lidar` 节点优先调 RPC `getRaycastLidarData`（服务端无此方法/传感器时自动回退 `getGPULidarData`）；`/airsim/lidar/points_raw` 话题格式不变（x,y,z NED + intensity,ring,time）。RaycastLidar 无段级渲染时刻，逐点时间退化为快照模型。若 UE 日志/回退警告出现，先确认 Windows 端 settings.json Lidar1 SensorType=12 且 UE 已重启。

**ControlIp 双通道机制**（`MavLinkMultirotorApi.hpp`）：PX4 模式下 HIL 链路（TCP 4560）只传 HIL_SENSOR 物理数据，其余全部指令走 `ControlIp` 指向的第二条独立 UDP 控制通道（settings 每载具段，**默认 127.0.0.1 仅同机有效**）。跨机时 ControlIp 必须填 PX4 所在机器（WSL）可达 IP，否则出现"HIL 有数据但收不到指令"的半连接假象。

## 故障 → 责任侧速查

| 现象 | 最可能原因 | 责任侧 / 动作 |
|---|---|---|
| PX4 起来后一直等连接、无 HIL 数据 | UE 没启动 / 4560 不通 / PX4_SIM_HOST_ADDR 错 | WSL：`nc -zv $PX4_SIM_HOST_ADDR 4560`；Windows：确认 UE 在编辑器中运行 |
| 传感器节点 / 地面站连不上 RPC | 41451 不通，或 WSL IP 变了但 UE 端 ControlIp 未更新 | **WSL：先跑 `bash scripts/sync-airsim-ip.sh`**；Windows：核对 `Documents\AirSim\settings.json` 的 ControlIp |
| MAVROS 永远 connecting | 用了 14580，被 AirSim 心跳抢占 | WSL：改用专属实例 `fcu_url:=udp://:14542@`（PX4 侧 14582↔14542） |
| 悬停漂移 | PhysicsParams.Mass 改了但 MPC_THR_HOVER 未重算 | 两侧成对改：按 `MPC_THR_HOVER = (1/推重比 − 0.2)/0.8` 重算，写入 PX4 airframe，重启 UE |
| 深度/图像话题正常但点云异常或帧率 ~1.7Hz | CPU 雷达 `LimitPoints` 没设 false，被 5000 点截断 | Windows：settings.json 雷达段 `LimitPoints: false`（CPU 雷达 SensorType 6；GPU 雷达 SensorType 8 用 `configs/fast_lio/settings.json`） |
| 点云去畸变拖影 / FAST-LIO 反复 reset | WSL 时钟漂移 + timesyncd 步进纠偏导致时间戳跳变 | WSL：确认 chrony slew 模式生效，`chronyc tracking` 的 System time 应为毫秒级 |
| `ros2 topic hz` min 为负 / max 秒级 | min 负 = 时钟被步进；max 飙高 = 发布端真断流 | 分别对应时钟问题 / Windows 侧 UE 卡顿或断流 |
| 相机话题存在但画面黑/错 | UE 端该相机未在 settings.json 定义，或 IMU/相机模式不匹配 | Windows：对照 WSL 端 `configs/settings.json` 模板检查对应 Sensor 段 |
| 换机型后物理参数不对 / 增益怪异 | Tab 切换用的 `settings_<机型>.json` 不存在或与 BP 蓝图名不匹配，**静默回退**默认 settings.json | Windows：检查 `Documents\AirSim\settings_<机型>.json` 存在且与 `Content/UAV/BP_<机型>` 命名一致；蓝图须为 AFlyingPawn 子类 |
| 改了 AirSim 插件代码不生效、行为原样复现 | 改的是 `Plugins/AirSim/Source/AirLib/`（预编译静态库，UBT 不重编） | Windows：走 README 的 MSBuild 三步重编流程（Rebuild AirLib → 删 AirSim 中间产物 → 重编 UE） |

## 两侧 AI 的动作清单

### Windows 端（BeimingSim 侧）能做
- 检查/修改 `Documents\AirSim\settings.json` 及机型档 `settings_<机型>.json`：`ControlIp`（填 WSL 当前 IP，默认 127.0.0.1 仅同机）、载具段 `PhysicsParams`、各 Sensor 定义、LiDAR `SensorType` 与 `LimitPoints`；改完重启 UE（游戏内 O 键有可视化配置面板）
- 确认 UE 已打开项目且主场景 `TestMap` 在跑（唯一仿真地图）；查 UE 输出日志中的 AirSim 心跳/连接报错
- 键位排查：Tab 机型热切换菜单、F9 自由视角、O 配置面板；反复切机型变卡用 `Tools/monitor_perf.py` 采样
- 改 `Plugins/AirSim/` 或 `Source/BeimingSim/`；涉及共轴物理逻辑在 `RotorActuator.hpp` / `MultiRotorParams.hpp` / `MavLinkMultirotorApi.hpp`（注意 normalizeRotorControls 的 `0.8u+0.2` 变换与 HIL 通道 2/3 舵机指令）；改 AirLib 源码必须走重编流程
- 无编辑器环境时可用打包版 `BeimingSim_Build/Windows/启动仿真-窗口模式.bat`（注意打包版强制烘焙了 Plugins/AirSim/Content 与 UAV 目录）

### WSL 端（flylab 侧）能做
- `bash scripts/sync-airsim-ip.sh` — WSL IP 变化后一键更新（最高频修复动作）
- `nc -zv $PX4_SIM_HOST_ADDR 4560` / `nc -zv $PX4_SIM_HOST_ADDR 41451` — 验证 HIL / RPC 连通
- `bash scripts/run-px4.sh <机型> --clean` — 换机型 / 清参数缓存；PX4 控制台看 HIL 状态
- `ros2 topic hz/echo` 检查各传感器话题；`MicroXRCEAgent udp4 -p 8888` 与 MAVROS 14542 实例起停
- 改 ROS2 节点 / airframe / 链路配置后 `colcon build --parallel-workers 2`

### 向对方索要的关键信息
- WSL→Windows 方向问：UE 是否运行？`ControlIp` 当前值？相关 Sensor 段内容？UE 日志关键报错？
- Windows→WSL 方向问：WSL 当前 IP？PX4 是否起、控制台输出？`nc` 连通结果？`ros2 topic hz` 数据？

## 双边沟通格式约定

向另一侧 AI 描述问题时按此结构给全，减少来回：

1. **现象**：本侧看到的具体输出（日志关键行、`ros2 topic` 数据、UE 报错），不猜结论
2. **已验证**：本侧已确认正常的检查项（如 "4560 nc 通"、"PX4 在跑"）
3. **请对方确认**：具体的值或动作（如 "请贴出 settings.json 的 ControlIp 和 LiDAR 段"）
4. **时间线**：问题出现前本侧做过的改动（尤其 IP 变化、配置修改、代码更新）
