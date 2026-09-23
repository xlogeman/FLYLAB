#!/usr/bin/env bash
# ============================================================================
# 本质还原: ytjy-sitl-lesson5-qgc  (多机课的 QGC 地面站视角)
# 原封装: alias ... ='ytjy-sitl-init-env; terminator --layout=ytjy-sitl-lesson5-qgc'
# terminator 布局 3 窗格: [Lesson5 qgc(sleep 5)] | [SITL PX4 多机][DDS AGENT]
# ============================================================================
set -e
source /opt/ros/humble/setup.bash
source /root/fucker/ros2_ws/install/local_setup.bash

# ---- 窗格 [Lesson5 qgc]: 原别名 ytjy-sitl-run5-qgc -------------------------
# alias ytjy-sitl-run5-qgc='cd /root/fucker/ros2_ws; ros2 launch hw_insight lesson5-qgc.launch.py'
cd /root/fucker/ros2_ws
ros2 launch hw_insight lesson5-qgc.launch.py
#   launch 副本: ../origins/launch/lesson5-qgc.launch.py
#   内部: airsim_node + rviz2(lesson5.rviz)
#   ⚠ 4 个 multi_move_velocity 节点全部被注释掉了 —— 这课重点是"看", 不是"控"

# ---- 窗格 [SITL PX4] 多机 (同 lesson5-keyboard) ----------------------------
cd /root/fucker/px4.1.15.2/ytjy && ./killpx4.sh && ./lesson5.sh

# ---- 窗格 [DDS AGENT] ------------------------------------------------------
MicroXRCEAgent udp4 -p 8888

# 本质: PX4 实例的 14550+ 系列端口自动广播 MAVLink, QGC(装在 Windows 宿主)
#       监听这些端口即可同时看到 4 架飞机。lesson5.sh 里没显式转发, 靠 WSL 端口
#       映射/镜像网络让宿主机 QGC 收到。
