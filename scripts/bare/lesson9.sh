#!/usr/bin/env bash
# ============================================================================
# 本质还原: ytjy-sitl-lesson9
# 原封装: alias ... ='ytjy-sitl-init-env; terminator --layout=ytjy-sitl-lesson9'
# terminator 布局 5 窗格:
#   [AirSim C(sleep 2)][DDS AGENT][switch(sleep 5)] | [SITL PX4][VINS FUSION(sleep 5)]
#
# 课题: 自研快速数据通路 (ytjy_airsim_fast) + VINS-Fusion (lesson9 外参配置)
# ⚠ 需要先在 Windows 宿主机启动 UE/AirSim 仿真环境
# ============================================================================
set -e
source /opt/ros/humble/setup.bash
source /root/fucker/ros2_ws/install/local_setup.bash

# ---- 窗格 [AirSim C]: 换用自研节点, 不再用标准 airsim_node ------------------
cd /root/fucker/ros2_ws
ros2 run airsim_ros_pkgs ytjy_airsim_fast
#   本质: 作者自己写的 C++ 版 AirSim 取数节点, 跳过标准节点的冗余封装,
#         更快地把 左/右目图像 + IMU + GPS + 真值里程计 发到 ROS2 话题
#         (源码在 airsim_ros_pkgs 包里, executable=ytjy_airsim_fast)

# ---- 窗格 [DDS AGENT] ------------------------------------------------------
MicroXRCEAgent udp4 -p 8888

# ---- 窗格 [switch]: 原别名 ytjy-sitl-run9-rviz ----------------------------
# alias ytjy-sitl-run9-rviz='cd /root/fucker/ros2_ws; ros2 launch hw_insight lesson9.launch.py'
cd /root/fucker/ros2_ws
ros2 launch hw_insight lesson9.launch.py
#   launch 副本+注释: ../origins/launch/lesson9.launch.py
#   实际只启用: static TF world→map + rviz2(lesson8.rviz) + lesson8_vins_switch
#   (vins/loop/global_fusion/airsim_fast 在 launch 内被注释, 数据链路在别的窗格)

# ---- 窗格 [SITL PX4] -------------------------------------------------------
cd /root/fucker/px4.1.15.2 && make px4_sitl_default none_iris

# ---- 窗格 [VINS FUSION] ----------------------------------------------------
ros2 run vins vins_node \
  /root/fucker/ros2_ws/src/vins/config/ytjy/lesson9_extrinsics.yaml \
  --ros-args -p config_file:=/root/fucker/ros2_ws/src/vins/config/ytjy/lesson9_extrinsics.yaml \
             -r __node:=vins_estimator -r __ns:=/vins_estimator
#   与 lesson8 的差别: 配置换成 lesson9_extrinsics.yaml, 对接 ytjy_airsim_fast
#   发布的话题(真值里程计 /airsim/true_odom/data 等命名)是 lesson12 EGO 的输入
