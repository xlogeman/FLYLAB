---
name: beimingsim-context
description: BeimingSim（Windows 端 D:\A_Code\MyProject\BeimingSim，UE 5.8 + Cosys-AirSim 无人机仿真项目）的项目背景介绍。当任务涉及修改该 UE 工程/其 AirSim 插件、排查 UE 侧仿真问题（相机、机型切换、settings.json、ControlIp、天气/传感器面板）、或与 WSL 端 flylab 平台联排时使用。本 skill 由 WSL 端深度调研后代写，Windows 端 AI 直接安装即可获得项目认知，无需自行撰写。
---

# BeimingSim 项目背景（UE 5.8 + Cosys-AirSim 仿真端）

## 项目定位

BeimingSim 是 FLYLAB 分布式无人机仿真系统的 **Windows/UE 端**：物理仿真、传感器渲染、RPC API（TCP 41451）与 PX4 HIL 链路（TCP 4560）都在这里。与 WSL 端 `~/flylab`（PX4 SITL + ROS2）通过两条链路耦合，两侧代码库不共用 git。完整文档见项目内 `README.md`（机型参数推导与配对流程写得很全），本 skill 是其 AI 向摘要与补充。

## 详细资料（按需加载）

- `references/windows-project-overview.md` — 项目全貌：目录结构、自研 C++ 模块（Tab 机型热切换/F9 自由视角/行人 NPC/游戏内配置面板）、地图与机型清单、AirSim 插件 BEIMING 补丁分布、ArUco 支持、打包要点。
- `references/windows-debugging.md` — Windows 端排查手册：settings.json 生效路径与机型切换联动、ControlIp 双通道机制、AirLib.lib 预编译库重编流程（最大坑）、常用键位、性能监控脚本。

## 使用规则

1. **先读本 skill 再动手**：涉及 AirSim 插件源码修改时，先查补丁分布表确认改动点是否已有本地补丁——大量补丁围绕"可重建 SimMode"，任何新改动必须兼容 SimMode 重建（Actor 清理、输入绑定防重复、RPC server 安全关停）。
2. **AirLib.lib 是预编译静态库**：改 `Plugins/AirSim/Source/AirLib/` 源码必须走 README 的 MSBuild 重编流程（三步：MSBuild Rebuild → 删 AirSim 中间产物 → 重编 UE），否则改动看似生效实则旧代码；只改 `Source/AirSim/`、`Source/BeimingSim/` 无需此流程。
3. **settings 双文件机制**：Tab 切换机型时优先用 `Documents\AirSim\settings_<机型>.json`（按显示名匹配），不存在则回退默认 `settings.json`；蓝图资产名必须与 settings 文件名一致，否则静默回退默认参数。改完配置重启 UE 生效。
4. **跨机对接**：ControlIp 是 HIL 之外的第二条独立 UDP 控制通道，PX4 在 WSL 端时必须填 WSL 可达 IP（默认 127.0.0.1 仅同机有效）；WSL IP 变化由 WSL 侧 `scripts/sync-airsim-ip.sh` 同步。
5. **机型配对是两侧成对的**：新增机型 = 本端 settings_<机型>.json + BP 蓝图（Content/UAV/）↔ WSL 端 vehicles 档案 + PX4 airframe（联动 `MPC_THR_HOVER`、增益缩放），公式与避坑清单见本端 README §新增共轴双旋翼机型指南。
6. **与 WSL 端联排**：网络拓扑、端口表、故障→责任侧速查见 WSL 端 `flylab-airsim-context` skill 的 `references/joint-debugging.md`（两侧共用一份认知）。

## 维护规则

skill 是文档快照，过期比缺失更糟（AI 会拿旧事实自信排查）。项目演进时按触发条件同步更新：

- **触发更新的改动**（事实变更）：新增/删除机型或地图、键位与自研模块调整、AirSim 插件补丁增删（改了"可重建 SimMode"相关逻辑尤其要记）、settings 机制/ControlIp 行为变化、AirLib 重编流程变化。调参、日常 bugfix、场景摆设不需要动 skill。
- **更新落点**：结构/模块/机型/地图/补丁 → `windows-project-overview.md`；排查路径/键位/构建流程 → `windows-debugging.md`；跨端铁律 → 本文件使用规则。
- **双边同步**：涉及 WSL 端或双边约定（端口、机型配对、ControlIp、settings 模板）时，同步更新 flylab 仓库 `flylab-airsim-context/references/joint-debugging.md`——联排手册以该仓库为单一事实来源，本端副本跟随。
- **方式**：先读现有内容做增量修改，不整篇重写；更新完成后向用户确认要点是否准确。
