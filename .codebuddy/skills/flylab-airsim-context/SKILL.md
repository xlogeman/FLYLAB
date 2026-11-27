---
name: flylab-airsim-context
description: FLYLAB 无人机仿真平台的跨端项目背景介绍。当任务涉及 WSL 端 ~/flylab（PX4 + Cosys-AirSim SITL 仿真平台）与 Windows 端 D:\A_Code\MyProject\BeimingSim（UE 仿真项目）两侧联合排查、AirSim↔PX4 连接问题、settings.json / PhysicsParams / ControlIp 配置、传感器或链路不通等问题时使用。两侧 AI 各自安装此 skill 后共享同一份项目认知。
---

# FLYLAB 跨端项目背景（WSL flylab ↔ Windows BeimingSim）

## 这个系统是什么

一套分布式无人机仿真实验系统，分跑在两台环境上：

- **Windows 端**：`D:\A_Code\MyProject\BeimingSim` — UE 仿真项目（Cosys-AirSim 5.8 插件位于 `BeimingSim/Plugins/AirSim/`），负责物理仿真、传感器渲染、RPC API 服务。AI 在这一侧改 UE 工程 / settings.json / AirSim 插件。
- **WSL 端**：`~/flylab`（Ubuntu 22.04 + ROS2 Humble）— PX4 SITL v1.15.2、ROS2 工作区 `ros2_ws`（感知建图、VIO、避障规划、offboard 控制、PyQt5 地面站）、自研机型与启动脚本。AI 在这一侧改 ROS2 节点 / PX4 airframe / 链路配置。

两侧代码库**不共用 git**，靠网络链路耦合：WSL 端 PX4 通过 TCP 4560 连 UE 的 HIL 物理链路，传感器节点与地面站通过 TCP 41451 调 AirSim RPC。任何一侧的改动都可能让另一侧"看起来坏了"——排查时先明确问题属于哪一侧、接口在哪一侧。

## 详细资料（按需加载）

- `references/wsl-flylab-overview.md` — WSL 端 flylab 项目全貌：目录结构、三大自研包、功能链路、节点分层、机型库。
- `references/joint-debugging.md` — 双边联排手册：网络拓扑与端口表、故障→责任侧速查、两侧 AI 各自能做的检查与修复动作、需要向对方索要的信息。
- Windows 端（BeimingSim）的详细项目认知在姊妹 skill `beimingsim-context`（其 `references/windows-project-overview.md` 与 `windows-debugging.md`），本 skill 的联排手册与它互补共用。

## 使用规则

1. **先定侧再动手**：遇到"连不上 / 无数据 / 飞行异常"，先查 `references/joint-debugging.md` 的故障速查表，确定是 Windows 侧（UE 未启动、settings.json 配错、ControlIp 失效）还是 WSL 侧（IP 变化未同步、端口被抢占、PX4 参数失配），再给出动作。
2. **配置的单一事实来源**：AirSim 场景配置的模板在 WSL 端 `ros2_ws/src/flylab_flight/configs/`（含 `settings.json` 与各链路 json），生效位置在 Windows 端 `Documents\AirSim\settings.json`（复制后需重启 UE）。改配置时提醒用户"模板改完要复制到 Windows 端生效"。
3. **WSL IP 会变**：WSL 重启后 IP 变化会导致 UE 端 `ControlIp` 失效，WSL 侧有 `bash scripts/sync-airsim-ip.sh` 自动更新；这是最高频的联排故障。
4. **参数联动**：settings.json 载具段 `PhysicsParams`（质量/桨/转速）改了之后，PX4 侧 `MPC_THR_HOVER` 必须按公式重算，否则悬停漂移；两侧改动要成对。
5. **双边沟通格式**：向另一侧 AI 描述问题时，给出——本侧现象（日志关键行 / `ros2 topic` 或 UE 输出）、本侧已验证的检查项、需要对方确认的具体值（如 UE 是否在编辑器运行、ControlIp 当前值、settings.json 相关段）。

## 维护规则

skill 是文档快照，过期比缺失更糟（AI 会拿旧事实自信排查）。项目演进时按触发条件同步更新：

- **触发更新的改动**（事实变更）：新增/修改机型（airframe + vehicles 档案 + BP 蓝图）、端口/链路拓扑调整、新增功能链路或 launch、配置路径变化、新踩出的坑及其修复约定、节点分层结构调整。调参、日常 bugfix、算法实验不需要动 skill。
- **更新落点**：项目结构/链路/机型 → `wsl-flylab-overview.md`；端口/故障/约定 → `joint-debugging.md`；跨端铁律 → 本文件使用规则。
- **双边同步**：`joint-debugging.md` 与 Windows 端 `beimingsim-context` 是共认知文档——改了任一侧内容，提醒用户把改动同步到另一侧副本（以本仓库为单一事实来源）。
- **方式**：先读现有内容做增量修改，不整篇重写；更新完成后向用户确认要点是否准确。
