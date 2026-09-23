#!/usr/bin/env bash
# ============================================================================
# 本质还原: ytjy-sitl-lesson12
# 原封装: alias ... ='ytjy-sitl-init-env; terminator --layout=ytjy-sitl-lesson12'
# terminator 布局 7 窗格:
#   左列: [Keyboard][AirSim] / [PX4 EGO TRACKER] / [RVIZ][DDS]
#   右列: [FPV2 机型 PX4][EGO Planner]
#
# 课题: EGO-Planner 局部路径规划 + 自研 FPV2 机型 (高速穿越)
# ⚠ 需要先在 Windows 宿主机启动 UE/AirSim 仿真环境
# ============================================================================
set -e
source /opt/ros/humble/setup.bash
source /root/fucker/ros2_ws/install/local_setup.bash

# ---- 窗格 [Keyboard] -------------------------------------------------------
ros2 run hw_insight keyboard_velocity

# ---- 窗格 [AirSim]: 专门为 lesson12 写的取数节点 (深度图+真值里程计) --------
cd /root/fucker/ros2_ws
ros2 run airsim_ros_pkgs ytjy_airsim_lesson12
#   发出 EGO 需要的两路数据:
#     /airsim/depth_cam/image_raw   # 深度图 (感知障碍物)
#     /airsim/true_odom/data        # 真值里程计 (代替 SLAM, 课程先简化)

# ---- 窗格 [PX4 EGO TRACKER] ------------------------------------------------
ros2 run hw_insight lesson12_px4_ego_tracker
#   本质: 订阅 EGO 规划出的 B 样条轨迹 → 采样成位置/速度设定点 →
#         通过 uXRCE-DDS 写进 PX4 的 offboard 设定点 (飞机跟着轨迹飞)
#   源码: src/hw_insight/hw_insight/lesson12_px4_ego_tracker.py

# ---- 窗格 [RVIZ] -----------------------------------------------------------
ros2 run rviz2 rviz2 -d /root/fucker/ros2_ws/src/hw_insight/rviz/lesson12.rviz

# ---- 窗格 [DDS] ------------------------------------------------------------
MicroXRCEAgent udp4 -p 8888

# ---- 窗格 [FPV2 机型 PX4]: 原别名 ytjy-sitl-fpv2 (和 lesson1 机型不同!) -----
cd /root/fucker/px4.1.15.2
make px4_sitl_default ytjy_fpv2
#   本质: make 第二参数 = 机型 ytjy_fpv2
#     → 加载 ROMFS/px4fmu_common/init.d-posix/airframes/70002_ytjy_fpv2
#       (副本+逐行中文注释: ../origins/airframes/70002_ytjy_fpv2)
#   内容: 按 AirSim 物理模型标定的四轴几何(电机位置/转向/力矩系数) +
#         推重比 10:1 的高速整定(位置环/速度环/姿态环/角速度环全链路 PID)
#   这课用它是为了跟上 EGO 规划的高动态轨迹 (最大速度/加速度远大于默认 iris)

# ---- 窗格 [EGO Planner] ----------------------------------------------------
ros2 launch hw_insight lesson12.launch.py
#   launch 副本: ../origins/launch/lesson12.launch.py
#   它只是壳, include 了 ego_planner/lesson12/precision.launch.py
#   (副本+逐行中文注释: ../origins/ego_planner/lesson12-precision.launch.py)
#   真正启动: ros2 run ego_planner ego_planner_node (EGO-Planner 规划器本体)
#   数据流:
#     /airsim/true_odom/data + /airsim/depth_cam/image_raw
#       → 栅格地图(0.1m, raycasting 概率更新, 障碍膨胀 0.65m)
#       → RViz 点 2D Nav Goal → /goal_pose_3d (flight_type=1 目标点模式)
#       → EGO: A* 初始路径 → B样条优化(平滑/碰撞/可行性加权) →
#         /drone_0_planning/bspline → tracker → PX4 offboard
#   关键参数(课程重点, 见 precision.launch.py 注释):
#     map_size 260x260x7m, 局部更新范围 22x22x7m, 最大速度 3m/s,
#     WSL 至少分配 11G 内存 (全局静态数组)
