# ytjy-sitl 课程体系 · 本质拆解工作区

> 目的：不用 `ytjy-sitl-lessonN` 一键启动，逐条理解每个命令到底干了什么。
> 用法：`bare/` 里每个脚本 = 对应课程剥掉所有封装后的真实命令；`origins/` 是被调用实体的原件副本。

## 一、一键命令到底发生了什么（三层洋葱）

```
第1层  ytjy-sitl-lessonN                 ← ~/.bashrc 的 alias
        │
        ├─ ytjy-sitl-init-env            ← 清场: pkill 全家 + 删 PX4 参数缓存
        │    (bare/00-init-env.sh)
        │
        └─ terminator --layout=...       ← 只是"开一个多窗格终端", 无任何魔法
             │
第2层    每个窗格执行一条 bash 命令        ← terminator/config 里 command= 字段
             │
第3层    make px4_sitl_default none_iris  ← 真正干活的实体
         MicroXRCEAgent udp4 -p 8888
         ros2 launch hw_insight lessonN.launch.py
         ros2 run <pkg> <node> ...
```

**关键认知：terminator/alias 只是排版工具。** 全部"课程内容"就是十来条命令的组合，
全部已还原到 `bare/lessonN.sh`（可独立运行，或逐条拆开在多个终端里手动敲）。

## 二、反复出现的 6 个基础设施（先把这 6 个吃透）

| # | 实体 | 本质 | 出现于 |
|---|---|---|---|
| 1 | `make px4_sitl_default none_iris` | 编译 PX4 + 以内置机型 none_iris 启动 SITL（不接外部仿真器，PX4 自带动力学） | 1~11 |
| 2 | `MicroXRCEAgent udp4 -p 8888` | uXRCE-DDS Agent：把 PX4 uORB ↔ ROS2 DDS 话题互译（`/fmu/out/...`、`/fmu/in/...`） | 3/5/6/8/9/12/13 |
| 3 | `ros2 run airsim_ros_pkgs ...` | 从 Windows UE 里的 AirSim 拉传感器数据发到 ROS2（标准版/自研快速版/lesson12 专版三种） | 3~13 |
| 4 | `./killpx4.sh; ./lesson5.sh` | 多实例 PX4：`px4 -d -i N` 起第 N 架，SDK 端口 14540+(N-1) 自动错开 | 5 |
| 5 | `ros2 run hw_insight keyboard_velocity` | 键盘 → NED 速度消息（作者自己写的节点） | 3/5/6/12/13 |
| 6 | 机型文件 airframes | `10016_none_iris` 默认机型；`70002_ytjy_fpv2` 自研高速机型（参数全链路整定） | 12/13 |

## 三、13 课本质拆解速查表

| 课 | 新增东西（相对前面的课） | 一句话 |
|---|---|---|
| 1 | PX4 SITL 最小系统 | `make px4_sitl_default none_iris`，就这一条 |
| 2 | MAVSDK 直连（无 ROS2！） | python 脚本连 `udp://:14540`，任务飞行+测距避障（爬升策略） |
| 3 | DDS 桥 + 第一个自研 ROS2 节点 | PX4↔ROS2 打通，move_velocity + keyboard_velocity |
| 4 | 感知建图流水线 | 深度图 →(depth_image_proc)→ 点云 →(octomap_server)→ 占据地图 |
| 5 | 多机 | `px4 -d -i 1..4` 多实例 + multi_move_velocity 按实例分流 |
| 5-qgc | 地面站视角 | 只看不动手：多机 + QGC 连 1455x 端口 |
| 6 | 视觉目标检测 | YOLOv8 检测框 → lesson6_track 转速度指令追踪目标 |
| 7 | VINS-Fusion 单目+IMU + GPS 融合 | vins(绿) + loop_fusion(红) + global_fusion(白) 三轨迹对比 |
| 8 | VINS-Fusion 双目+IMU | 自研 airsim 快速通路起步；vins 独立窗格跑 lesson8 外参配置 |
| 9 | 自研 C++ 快速数据通路 | `ytjy_airsim_fast` 替换标准 airsim_node |
| 10 | OpenVINS | 滤波路线(MSCKF) vs 前面优化路线(VINS)；`ros2 topic hz` 体检数据链路 |
| 11 | ORB-SLAM3 双目惯性 | 词袋字典 + stereo-inertial 配置 + 话题重映射对接 AirSim |
| 12 | EGO-Planner + 自研机型 FPV2 | 深度图+真值里程计→栅格图→B样条轨迹→tracker→PX4 offboard |
| 13 | 提速版 EGO | 同 12，speed.launch.py 高速参数 + FPV2 机型动力匹配 |

## 四、课程知识主线（学习顺序建议）

```
飞控基础     1: PX4 SITL        → 飞控是什么、参数/机型/MAVLink口
外部控制     2: MAVSDK          → 不依赖 ROS 怎么控飞机
ROS2 通道    3: uXRCE-DDS       → PX4 和 ROS2 两个世界怎么握手
感知         4: 深度图→octomap  → 相机数据怎么变地图
多机         5: PX4 多实例      → 端口/话题命名空间隔离
视觉检测     6: YOLO            → 像素→目标→指令
状态估计     7~11: VINS→OpenVINS→ORB-SLAM3 → 三大主流 VIO 方案横向对比
规划控制     12/13: EGO-Planner → 地图→轨迹→跟踪→PX4, 闭环完成
```

## 五、目录说明

```
fucker/
├── README.md            ← 本文件（总拆解）
├── source-map.md        ← 原始系统中源码/配置的路径地图
├── bare/                ← 每课裸命令脚本（无别名无terminator，逐条=terminator一个窗格）
│   ├── 00-init-env.sh   ← 清场动作（含风险注释，学习时建议手动跑）
│   └── lesson1~13.sh    ← 真实命令 + 本质注释 + launch 内部展开
├── code/                ← 【代码归档】全部源码+配置自包含（4.0G），无需再回原系统翻找
│   │   详见 code/README.md（命令→文件映射总表）、REPRODUCE.md（复现清单）
│   ├── hw_insight/      ← 课程主包全量：节点源码+launch+rviz+YOLO权重
│   ├── ego_planner/     ← EGO-Planner 算法源码（planner/ + uav_simulator/）
│   ├── airsim_ros_pkgs/ ← AirSim 节点源码（含 3 个自研 C++ 节点）
│   ├── airsim_interfaces/, hw_interface/, px4_msgs/  ← 消息定义
│   ├── orb_slam3/       ← ORB-SLAM3 的 ROS2 wrapper（852M，含词表副本）
│   ├── ORB_SLAM3_FIXED/ ← ORB-SLAM3 完整库工程（libORB_SLAM3.so 源头）
│   ├── Pangolin/, Sophus/ ← ORB-SLAM3 编译依赖库
│   ├── openvins/        ← OpenVINS 全量源码
│   ├── vins/            ← VINS-Fusion 全量源码
│   ├── px4.1.15.2/      ← PX4 v1.15.2 全量源码（ROMFS/boards/Tools/src…）
│   ├── vins-config/, px4/  ← 配置快照 / PX4 启动脚本（快速入口）
│   └── shell/           ← bashrc 别名原件 + terminator 布局原件
├── REPRODUCE.md         ← 【复现清单】环境/依赖/构建顺序/Windows端资源，必读
└── origins/             ← 底层实体原件副本（改这里不影响原系统）
    ├── killpx4.sh, lesson5-multi-px4.sh      ← 多机 PX4 脚本
    ├── hw-avoidance-simple.py                ← lesson2 MAVSDK 避障（版权: 个人学习免费）
    ├── launch/lesson*.launch.py              ← 各课 ROS2 launch
    ├── ego_planner/lesson12-precision.launch.py, lesson13-speed.launch.py
    └── airframes/10016_none_iris, 70001_ytjy_fpv1, 70002_ytjy_fpv2
```

> code/ 仅排除了非源码内容：PX4 的 build/ 编译产物（1.3G）与 .git 版本历史（505M）、
> __pycache__；apt/pip 安装的标准件（depth_image_proc/octomap_server/topic_tools/
> rviz2/MicroXRCEAgent/mavsdk）本身无本地源码。

## 六、动手路径建议

1. 先读 `bare/lesson1.sh`，手敲跑通 PX4 SITL，`commander takeoff` 起飞一次。
2. 读 `origins/airframes/10016_none_iris` 和 `70002_ytjy_fpv2`（有逐行中文注释），理解机型=一串 `param set`。
3. `bare/lesson3.sh` 手动起 PX4+Agent，`ros2 topic list` 看 `/fmu/out/...`，亲手 echo 一条再发一条 offboard 指令。
4. 读 `origins/hw-avoidance-simple.py`（211 行，最短闭环），理解 14540 直连。
5. 按 4→6→7→10→11→12 顺序读 `origins/launch/` 各 launch.py（内含作者中文注释，是最好的教材）。
6. 最后精读 `origins/ego_planner/lesson12-precision.launch.py`——EGO 全部调参思路都在里面。

## 七、坑与提醒

- `ytjy-clean-all` 的 `pkill -9 -f "px4|dds|lesson|ros|vins|airsim|python"` 会误杀无关进程，学习期建议精确 kill（见 `bare/00-init-env.sh` 注释）。
- lesson3~13 依赖 Windows 宿主机的 UE/AirSim 端（`.bashrc` 里 `PX4_SIM_HOST_ADDR` 指向网关），先开 UE 再跑。
- lesson12 全局地图 260x260x7m 是静态内存数组，WSL 内存至少 11G，否则 EGO 直接挂。
- PX4 参数缓存 `parameters*` 不删的话，改了机型参数也不会生效——这就是每课先 `ytjy-px4-reset` 的原因。
