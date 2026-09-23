#!/usr/bin/env bash
# ============================================================================
# 本质还原: ytjy-sitl-lesson13  (lesson12 的提速版)
# 原封装: alias ... ='ytjy-sitl-init-env; terminator --layout=ytjy-sitl-lesson13'
# terminator 布局与 lesson12 相同的 7 窗格, 只是换了 lesson13 的三个文件
# ============================================================================
set -e
source /opt/ros/humble/setup.bash
source /root/fucker/ros2_ws/install/local_setup.bash

# ---- 窗格 [Keyboard] -------------------------------------------------------
ros2 run hw_insight keyboard_velocity

# ---- 窗格 [AirSim] (与 lesson12 同一个节点) --------------------------------
cd /root/fucker/ros2_ws
ros2 run airsim_ros_pkgs ytjy_airsim_lesson12

# ---- 窗格 [PX4 EGO TRACKER]: 换成 lesson13 版本 ----------------------------
ros2 run hw_insight lesson13_px4_ego_tracker

# ---- 窗格 [RVIZ]: 配置换成 lesson13.rviz ----------------------------------
ros2 run rviz2 rviz2 -d /root/fucker/ros2_ws/src/hw_insight/rviz/lesson13.rviz

# ---- 窗格 [DDS] ------------------------------------------------------------
MicroXRCEAgent udp4 -p 8888

# ---- 窗格 [FPV2 机型 PX4] (同 lesson12) ------------------------------------
cd /root/fucker/px4.1.15.2 && make px4_sitl_default ytjy_fpv2

# ---- 窗格 [EGO Planner]: 换成 speed.launch.py ------------------------------
ros2 launch hw_insight lesson13.launch.py
#   include ego_planner/lesson13/speed.launch.py
#   (副本: ../origins/ego_planner/lesson13-speed.launch.py)
#
# 与 lesson12 的本质差别 = "精度版" vs "速度版" 的 EGO 参数对比:
#   lesson12 precision:  速度 3 m/s, 保守, 轨迹平滑, 适合精密飞行
#   lesson13 speed:      速度更高(见该 launch 内 manager_max_vel 等参数),
#                        搭配 lesson13_px4_ego_tracker 与 FPV2 机型的高速整定
# 学习方式: 对比两个 launch.py 的参数 diff, 理解"机型动力能力↔规划参数"的匹配
