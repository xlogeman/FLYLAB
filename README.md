# fucker/ —— ytjy-sitl 课程复现工作区（已扁平化，自包含）

> 原始 `code/` 归档与 `bare/`、`origins/` 已删除，本目录即全部。
> 归档时代的原始说明保留在 `docs/README-archive.md`、`docs/source-map.md`。

```
fucker/
├── env.sh            ← 环境总入口（每个新终端先 source /root/fucker/env.sh）
├── README.md         ← 本文件
├── REPRODUCE.md      ← 复现清单（实测踩坑记录 + 新机器操作步骤）
├── ros2_ws/          ← colcon 工作空间（38 源码包全绿，install/ 45 包）
│   ├── src/          ← 课程 10 包 + mavros/libmavconn/mavros_msgs（源码 2.15.0）
│   └── build/ install/ log/
├── px4.1.15.2/       ← PX4 v1.15.2 全量（含编译产物，可直接启动）
├── ORB_SLAM3_FIXED/  ← ORB-SLAM3 库（含 lib/*.so 与 Thirdparty 编译产物）
├── Cosys-AirSim/     ← AirSim 本体（UE 端插件同源 fork，airsim_ros_pkgs 编译依赖）
├── mavros/           ← mavros 源码原件（已复制进 ros2_ws/src，此为留档）
├── scripts/
│   ├── bare/         ← 每课裸命令脚本（含一键启动 fucker-run-px4.sh）
│   ├── origins/      ← launch 原件、多机脚本、MAVSDK 避障、airframes
│   ├── px4/          ← PX4 快速启动入口
│   └── gcs-relay.py  ← QGC 双向 UDP 中继（QGC 必备）
├── configs/
│   ├── vins-config/  ← VINS 课程配置
│   └── shell/        ← 原作者 bashrc / terminator 布局（参考用）
└── docs/             ← 归档时代说明（README-archive / source-map，历史参考）
```

## src/ 包清单

| 包 | 说明 |
|---|---|
| hw_insight | 课程主包：节点+launch+rviz+YOLO 权重（.pt 已随包） |
| hw_interface / airsim_ros_pkgs / airsim_interfaces / px4_msgs | 消息与桥接，lesson3 起的基础设施 |
| planner / uav_simulator | EGO-Planner（lesson12/13） |
| openvins | OpenVINS（ov_core/ov_init/ov_msckf 等 5 包） |
| vins | VINS-Fusion（4 子包） |
| orb_slam3 | ORB-SLAM3 wrapper；`CMakeModules/FindORB_SLAM3.cmake` 指向 `/root/fucker/ORB_SLAM3_FIXED` |
| mavros / libmavconn / mavros_msgs | 源码编译 2.15.0（官方源无 C++ 二进制，见 REPRODUCE.md） |

## 常用命令

```bash
# 0. 环境总入口（每个新终端）
source /root/fucker/env.sh

# 1. 启动 PX4（先开 Windows 端 UE/Cosys-AirSim 场景）
bash /root/fucker/scripts/bare/fucker-run-px4.sh          # --clean 清参数缓存

# 2. DDS Agent（lesson3 起，另开终端）
MicroXRCEAgent udp4 -p 8888

# 3. 按课验证
ros2 launch hw_insight lesson3.launch.py
ros2 run hw_insight keyboard_velocity

# 4. QGC 地面站（Windows 端）：先起中继再开 QGC
nohup python3 /root/fucker/scripts/gcs-relay.py > /root/fucker/gcs-relay.log 2>&1 &

# 5. 重新编译（改了 src/ 之后）
cd /root/fucker/ros2_ws && colcon build --parallel-workers 2 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release    # ⚠ 并行度别调高，会打爆 WSL 内存
```

## 关键适配点（动过的地方，详见 REPRODUCE.md）

- `orb_slam3/CMakeModules/FindORB_SLAM3.cmake` → 指向本目录 `ORB_SLAM3_FIXED`
- `env.sh` LD_LIBRARY_PATH → `ORB_SLAM3_FIXED/{lib,Thirdparty/DBoW2/lib,Thirdparty/g2o/lib}`
- PX4 build 缓存内绝对路径已改为 `/root/fucker/px4.1.15.2`
- `airsim_ros_pkgs/CMakeLists.txt` 的 `AIRSIM_ROOT` → `/root/fucker/Cosys-AirSim`
- 课程脚本内原机器路径（/home/hw/...）已全部改写为本目录
