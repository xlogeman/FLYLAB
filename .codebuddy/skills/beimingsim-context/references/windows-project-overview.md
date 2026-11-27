# BeimingSim 项目全貌

> 路径：`D:\A_Code\MyProject\BeimingSim`（Windows 端）。UE 5.8 + Cosys-AirSim 5.8 插件（v3.5.0 本地补丁版），是 FLYLAB 分布式无人机仿真系统的 Windows/UE 侧。详细文档见项目 `README.md`。

## 目录结构

| 目录 | 说明 |
|---|---|
| `BeimingSim.uproject` | 项目入口，EngineAssociation 5.8，单 Runtime 模块 `BeimingSim`，额外启用 ModelingToolsEditorMode |
| `Source/BeimingSim/` | 自研 C++ 模块（见下） |
| `Plugins/AirSim/` | 基于 Cosys-AirSim 修改的插件，**源码已入库，勿用原版覆盖**；补丁注释标 "BEIMING patch" |
| `Content/` | 资产（TestMap、UAV 机型、ArUco 标记、环境资产），git 不入库需单独同步 |
| `Config/` | DefaultEngine/DefaultGame 等 ini |
| `Tools/monitor_perf.py` | 性能监控脚本：按秒采样 UE 进程 CPU/GPU/内存到 CSV（依赖 psutil），专为排查"反复切机型后越来越卡"设计 |
| `BeimingSim_Build/Windows/` | 可分发打包版（BeimingSim.exe + `启动仿真-窗口模式.bat`） |
| `Binaries/Intermediate/DerivedDataCache/Saved` | 编译与运行中间产物 |

## 自研 C++ 模块（Source/BeimingSim/）

| 目录 | 内容 |
|---|---|
| `Core/` | `BeimingSimGameMode`（继承 AAirSimGameMode，替换 PlayerController 为显示光标版；BeginPlay 自动生成机型热切换器与配置管理器）+ `BeimingSimPlayerController`（F9 自由视角切换） |
| `Camera/` | `FreeFlyCameraPawn`：编辑器视口风格自由相机，右键拖视角、WASD/QE/Space 移动、Shift 加速、滚轮调速、穿墙无碰撞；与 F9 载具视角互切 |
| `Drone/` | `DroneModelSwitcher` + `DroneSwitchWidget`：**Tab 机型热切换**——递归扫描 `/Game/UAV` 下所有 `AFlyingPawn` 子类蓝图（`*_C` 生成类，打包版可用），显示名去 `BP_` 前缀、字母序排序，菜单第 0 项固定 AirSim 默认机型；同类别"外观换装"（SimMode 重建使配置生效），跨类别（人物/车辆）走 Possession，Pawn 常驻不销毁；含 3D 预览缩略图 |
| `Character/` | `BeimingCharacter`：第三人称行走人物，传统 BindKey 输入（不依赖 Enhanced Input），默认 SKM_Quinn 外观 |
| `Walker/` | `WalkerNPC` 巡逻 NPC：两点往返 / 多点 Waypoints 顺序 / 随机游走三模式，配 `WalkerAIController` 与 `WalkerWaypoint`；有 NavMesh 自动绕障，无则直线移动 |
| `Config/` | `SimConfigManager` + `SimConfigWidget`（**O 键唤出**）：游戏内 settings.json 可视化编辑面板（根设置、Wind/OriginGeopoint/TimeOfDay、各载具/相机/传感器条目 SensorType 1-11），取代了被移除的原版 F10 天气菜单 |

## 地图与机型

- **地图**：`Content/Maps/TestMap` 是唯一主仿真场景（DefaultEngine.ini 的 GameDefaultMap 与 EditorStartupMap 都指向它）；`Lvl_ThirdPerson` 是模板演示关。全局 GameMode 固定为 `BeimingSimGameMode`。
- **机型**（Content/UAV/，每架一个 BP + 机身/桨网格）：`BP_X050`（共轴双桨）、`BP_X100`（共轴）、`BP_X300`（共轴）、`BP_X450`（四桨）。新增机型零代码：建 `BP_<机型>`（AFlyingPawn 子类）入扫描目录即可；**蓝图资产名必须与 settings 文件名一致**（`BP_X050` ↔ `settings_X050.json`），否则热切换静默回退默认四旋翼参数。

## AirSim 插件 BEIMING 补丁分布（改插件前先查这里）

补丁约 20+ 文件 100+ 处，主题归纳：

| 文件 | 补丁主题 |
|---|---|
| `SimHUD` | 运行时重建 SimMode（机型切换用）：切生效 settings 文件、防重入延迟重建、起飞位姿记录/恢复、CVar SetByConsole |
| `SimModeBase` | 重建时的世界级清理：按 `kAirSimBeaconTag` 销毁旧信标/世界灯、RPC server 异步安全关停、移除加载屏 |
| `PawnSimApi` | 相机跨轮复用（稳定名 `BEIMING_Cam_<载具>_<键>`）、Enabled=false 相机隐藏、灯光记录销毁 |
| `PIPCamera` / `AirBlueprintLib` | 相机重建防重复追加；输入绑定 append 防重复（重建 SimMode 时） |
| `Weather/WeatherLib` | 移除 F10 天气菜单（改由 O 键配置面板接管） |
| `RpcLibServerBase` / `ApiServerBase` | RPC server 异步安全关停 |
| `UnrealGPULidarSensor` / `WorldSimApi` | LidarCamera 析构同步销毁、同名 Actor 加后缀；相机查找带校验不再 fatal |
| `Annotation/AnnotationComponent` | UE 5.8 Nanite `FSkinnedSceneProxy` 虚函数 LNK2001 兼容补丁 |
| `MultiRotorParams.hpp`（AirLib） | **PhysicsParams 扩展**：RotorCount 支持 2（共轴）/4/6/8，`RotorZTop/Bottom` 上下桨高度（NED 必须负值），`MaxTiltAngle` 倾转角，`TiltFilterTc` 低通（默认 0.05s） |
| `RotorActuator.hpp` | 共轴反扭矩固定沿堆叠轴（防交叉耦合）；推力与油门线性 |
| `MavLinkMultirotorApi.hpp` | `normalizeRotorControls` 保留 PX4 变换 `sim_ctrl = 0.8u + 0.2`（因此 `MPC_THR_HOVER = (1/推重比 − 0.2)/0.8`，Z 环增益按 `1/r = (0.8+0.2/hover)/0.8` 补偿）；HIL_ACTUATOR_CONTROLS 解码保留原始通道 2/3 作倾转舵机指令 |

## ArUco 与注记

`Content/ArUco/` 是材质形态的基准标记（单点码 0/1、5×5_231_213、fractal 二/三级、grid_9x9），直接贴场景物体；插件 `Annotation/` 提供按名字/层级自动注记，`FiducialBeacon` 加载 fiducial 信标网格。

## 构建与打包要点

- `Config/DefaultGame.ini` 强制烘焙 `../Plugins/AirSim/Content` 与 `UAV` 目录——运行时按名字/软引用加载，UBT 打包器发现不了，漏掉会导致打包版缺机型/缺材质。
- 渲染：Lumen 动态 GI、虚拟阴影、关闭静态光照与硬件光追、Substrate 材质、DX12/SM6。
- 打包分发版在 `BeimingSim_Build/Windows/`（含中文启动 bat）。
