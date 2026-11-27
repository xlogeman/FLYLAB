# BeimingSim Windows 端排查手册

## settings.json 体系

| 文件 | 位置 | 何时生效 |
|---|---|---|
| `settings.json` | `Documents\AirSim\settings.json` | 默认配置，启动时加载 |
| `settings_<机型>.json` | 同目录，如 `settings_X050.json` | Tab 切换到对应机型时优先使用，SimMode 重建生效 |
| 模板来源 | WSL 端 `~/flylab/ros2_ws/src/flylab_flight/configs/` | 复制过来后重启 UE |

加载顺序（SimHUD.cpp）：命令行 `-settings=` → 可执行文件目录 → 启动目录 → `Documents\AirSim\`。游戏内还有 **O 键**可视化配置面板（SimConfigWidget），改完仍需重启 UE 才对 AirSim RPC 侧生效。

**高频坑**：切了机型但物理参数不对 → 检查 `settings_<机型>.json` 是否存在且名字与 BP 蓝图显示名严格一致（`BP_X050` ↔ `settings_X050.json`）；不一致会静默回退默认 `settings.json` 的四旋翼参数。

## ControlIp 双通道机制（跨机对接必读）

PX4（MAVLink 固件）模式下 AirSim 与飞控有**两条独立链路**：

1. **HIL 链路**：TCP 4560，只传 HIL_SENSOR 物理数据
2. **控制通道**：`ControlIp` + `ControlPortLocal`（settings 每载具段，默认 127.0.0.1），独立 UDP，承载除 HIL 外的全部指令

PX4 在 WSL 端时 **ControlIp 必须填 WSL 可达 IP**（支持 `local/localhost` 别名与 `remote` 特殊值）；WSL IP 变化 → UE 端心跳发往旧地址 → WSL 侧传感器 RPC 与链路异常，修复动作在 WSL 侧跑 `scripts/sync-airsim-ip.sh`。验证：UE 日志看心跳是否对端可达；WSL 侧 `nc -zv $PX4_SIM_HOST_ADDR 41451`（RPC）、4560（HIL）。

## AirLib.lib 重编流程（最大坑）

`Plugins/AirSim/Source/AirLib/` 编译为**预编译静态库**（`lib/x64/Debug|Release/AirLib.lib`），UE 的 UBT 不会重编它。症状：改了 AirLib 源码、VS 里编 BeimingSim 显示 `Target is up to date, 0 actions`，行为原样复现。

修改 AirLib 源码/头文件后必须按序执行（PowerShell）：

```powershell
# 1. 重编 AirLib（改头文件时必须 /t:Rebuild 全量）
& "D:\VisualStudio\MSBuild\Current\Bin\MSBuild.exe" `
    "Plugins\AirSim\Source\AirLib\AirLib.vcxproj" `
    /t:Rebuild /p:Configuration=Release /p:Platform=x64 /m

# 2. 删 AirSim 模块中间产物（防新旧混链接）
Remove-Item -Recurse -Force Intermediate\Build\Win64\UnrealEditor\*\Development\AirSim

# 3. 正常编译 UE
dotnet "D:\UE_5.8\Engine\Binaries\DotNET\UnrealBuildTool\UnrealBuildTool.dll" `
    BeimingSimEditor Win64 Development -Project="$PWD\BeimingSim.uproject" -WaitMutex
```

- Debug Target 用 Debug lib，其余用 Release lib；怀疑增量失灵就核对 `lib\x64\Release\AirLib.lib` 修改时间或直接 Rebuild
- 只改 `Source/AirSim/`（UE 模块部分）或 `Source/BeimingSim/` 无需此流程

## 常用键位

| 键 | 功能 | 实现位置 |
|---|---|---|
| Tab | 机型热切换侧滑菜单（ESC 收起） | `DroneModelSwitcher.cpp` BindTabKey（未就绪 1s 重试） |
| F9 | 载具视角 ↔ 自由飞行相机 | Controller 级 + FreeFlyCameraPawn 双处绑定 |
| O | 游戏内 settings 配置面板 | SimConfigWidget（取代原 F10 天气菜单） |

## 排查清单（Windows 端 AI 动作）

1. **连接类问题**：UE 是否运行且 TestMap 在跑？UE 输出日志里 AirSim 心跳/RPC 报错行？`Documents\AirSim\settings.json` 的 ControlIp 当前值？
2. **传感器/图像异常**：对照 WSL 端模板检查对应 Sensor 段（相机 Enabled、LiDAR SensorType 6/8 与 `LimitPoints: false`）；相机在 settings 里 Enabled:false 会被补丁隐藏不注册
3. **物理异常**：`PhysicsParams` 段与 WSL 端 `configs/vehicles/<机型>.json` 是否一致；改过 Mass 是否同步了 PX4 `MPC_THR_HOVER`
4. **机型切换异常**：settings_<机型>.json 命名匹配？BP 蓝图是否 `AFlyingPawn` 子类且在 `/Game/UAV` 下？反复切换变卡 → `Tools/monitor_perf.py` 采样分析
5. **改了插件代码不生效**：是否改在 AirLib（走重编流程）？AirLib.lib 时间戳？
