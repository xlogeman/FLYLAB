# REPRODUCE.md —— 复现清单（2026-09-21 实操修订版）

> 首轮复现已在 根目录完成，**全部关键路径已验证**。本文 = 实测踩坑记录 + 新机器操作清单。
> 工作区 `work/` 已自包含，`code/` 归档仅作对照，lesson3 验证通过后可删。

---

## 一、最终目录布局（全部实体复制，无软链依赖）

```
fucker/
├── env.sh            ← 环境总入口（ROS2 + ORB_SLAM3 库路径 + PX4_SIM_HOST_ADDR + 软件渲染）
├── ros2_ws/          ← colcon 工作空间，38 源码包全绿，install/ 共 45 包
│   └── src/          ← 课程 10 包 + mavros/libmavconn/mavros_msgs（源码 2.15.0）
├── px4.1.15.2/       ← PX4 全量（含编译产物，build 缓存路径已修正到本机）
├── ORB_SLAM3_FIXED/  ← ORB-SLAM3 库（含 lib/*.so 与 Thirdparty 编译产物）
├── Cosys-AirSim/     ← AirSim 本体（归档缺失部分，用 UE 端同源 fork 补齐）
├── mavros/           ← mavros 源码（ros2 分支 2.15.0，已复制进 ros2_ws/src）
├── scripts/          ← bare/（课程脚本+run-px4.sh）+ origins/（launch/airframes）
├── configs/          ← vins-config + shell（原作者 bashrc，仅参考）
└── docs/             ← 原 README / source-map
```

## 二、apt 依赖（照装）

```bash
# ROS2 官方源（arch 过滤必须与 dpkg --print-architecture 一致，否则整条源被静默跳过）
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt install ros-humble-desktop python3-colcon-common-extensions \
    ros-humble-depth-image-proc ros-humble-octomap-server ros-humble-octomap-rviz-plugins \
    ros-humble-topic-tools ros-humble-tf2-tools ros-humble-cv-bridge \
    ros-humble-image-transport ros-humble-rviz2 ros-humble-geometry2
```

pip 包见原清单（PX4 编译链一组 + 课程节点一组，版本照抄不变）。

**⚠ mavros 不能 apt 装**：官方源长期只发布了 `ros-humble-mavros-msgs`，C++ 本体
`mavros`/`libmavconn` 的 jammy amd64 构建一直缺失（索引里根本没有，不是配置问题）。
解决：源码编译进工作空间（已完成），额外依赖：

```bash
sudo apt install ros-humble-mavlink ros-humble-eigen-stl-containers \
    ros-humble-ament-cmake-google-benchmark libasio-dev libgeographic-dev geographiclib-tools
# mavros 运行时数据集（装完必须执行）：
sudo bash ros2_ws/install/mavros/lib/mavros/install_geographiclib_datasets.sh
```

## 三、归档缺元数据导致的四个坑（新机器照改）

1. **PX4 无 .git**（归档排除了 .git）→ Makefile 第 38 行直接报错：
   ```bash
   git init && git add -A && git commit -m init && git tag -f v1.15.2
   ```
2. **子模块无 .git** → px4_git.cmake 的构建目标依赖 `<子模块>/.git`：
   ```bash
   git config -f .gitmodules --get-regexp '^submodule\..*\.path$' | awk '{print $2}' | \
     while read p; do (cd "$p" && git init -q && git add -A && git commit -qm init); done
   ```
   同时 NuttX 仓库要补 tag（版本头脚本用）：`git tag -f nuttx-10.3.0`
3. **全部 .sh 丢失执行位** → `find . -name "*.sh" -exec chmod +x {} +`
4. **build/ 缓存含绝对路径** → 目录移动/复制后 sed 替换 `旧路径 → 新路径`（cmake/ninja 全部重算，编译产物无需重编）

## 四、编译坑

| 坑 | 修法 |
|---|---|
| Pangolin `colour.h` 报 `numeric_limits is not a member of std` | 补 `#include <limits>`（新版 GCC 不再间接引入），已改 |
| Pangolin/Sophus/ORB_SLAM3 的旧 build 目录带原机路径 | `rm -rf build` 重配 |
| **colcon 默认并行打爆 WSL 内存 → VM 直接崩溃断连**（实测两次） | 必须 `colcon build --parallel-workers 2` + `export MAKEFLAGS=-j2` |
| orbslam3 wrapper 硬编码 `/home/hw/tmp/ORB_SLAM3_FIXED`（无条件 set()，-D 无法覆盖） | `src/orb_slam3/CMakeModules/FindORB_SLAM3.cmake` 第 8 行已改为本机路径 |

## 五、AirSim 本体（归档漏项 → Cosys 补位）

- 归档只有 `ros2/` 子树，**AirSim 仓库本体（AirLib/MavLinkCom/cmake/external）没进归档**，原审计"100% 覆盖"漏了这块。
- UE 端用 Cosys-AirSim 插件 → ROS 侧 AirLib 用同源 fork（RPC 协议严格配对）。
- `airsim_ros_pkgs/CMakeLists.txt` 已改：`AIRSIM_ROOT=/root/fucker/Cosys-AirSim`，rpclib 目录 2.3.1（内容 v2.3.0，等价）。
- Cosys zip 不含子模块，已补：`external/rpclib/rpclib-2.3.1`（rpclib v2.3.0）+ `AirLib/deps/eigen3 → /usr/include/eigen3` 软链。
- 编译实测通过（2min46s，-j2）。

## 六、Windows 宿主端（唯一归档外资源）

| 项 | 要点 |
|---|---|
| UE + Cosys-AirSim 场景 | 已有 |
| settings.json | 放 `C:\Users\<user>\Documents\AirSim\`；**ControlIp/UdpIp 填 WSL 的 IP**（`hostname -I`，重启会变，试 localhost 透传）；LockStep+UseTcp+TcpPort 4560 保持 |
| 防火墙 | 放行 4560/TCP、41451/TCP、14540+/UDP |
| QGC | lesson5-qgc 用，连 1455x 端口 |
| 多机（lesson5/12） | Vehicles 里加 PX4-2/3/4，TcpPort/ControlPortLocal 递增 |

## 七、启动流程（每课）

```bash
source /root/fucker/env.sh          # 每个新终端
bash scripts/bare/fucker-run-px4.sh        # --clean 清参数缓存（改参数后必用）
MicroXRCEAgent udp4 -p 8888              # lesson3 起，另开终端
# 先开 UE 场景再起 PX4；看到 "Ready for takeoff!" 即就绪
# lesson12 需 WSL 内存 ≥11G
```

## 八、删除 code/ 前的验证清单

- [x] PX4 启动成功（已验证：路径修正后 make 正常）
- [x] colcon build 全绿（38 包）
- [x] MAVROS + 数据集
- [ ] lesson3 实测（DDS Agent + ros2 topic list 看到 /fmu/out/...）← 删 code/ 前最后一关

## 九、原机已弃用项

- `python3-colcon` 无此包名，用 `python3-colcon-common-extensions`
- 原作者 bashrc/alias 体系 → 由 `env.sh` 取代
- `PX4_SIM_HOST_ADDR` 指向 WSL 网关（env.sh 自动取默认路由）
