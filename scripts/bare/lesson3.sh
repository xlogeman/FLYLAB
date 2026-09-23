#!/usr/bin/env bash
# ============================================================================
# 本质还原: ytjy-sitl-lesson3
# 原封装: alias ... ='ytjy-sitl-init-env; terminator --layout=ytjy-sitl-lesson3'
# terminator 布局 4 窗格: [SITL PX4][DDS AGENT] | [Lesson3(sleep 5)][keyboard control]
#
# 课题: PX4 ↔ ROS2 打通 (uXRCE-DDS 桥) + ROS2 速度控制 + 键盘控制
# ============================================================================
set -e
source /opt/ros/humble/setup.bash
source /root/fucker/ros2_ws/install/local_setup.bash

# ---- 窗格 [SITL PX4] -------------------------------------------------------
cd /root/fucker/px4.1.15.2 && make px4_sitl_default none_iris

# ---- 窗格 [DDS AGENT]: 原别名 ytjy-sitl-dds -------------------------------
# alias ytjy-sitl-dds='MicroXRCEAgent udp4 -p 8888'
# 本质: PX4 内部跑着 uxrce_dds_client, 通过 UDP 8888 找 Agent;
#       Agent 把 PX4 的 uORB 话题 ↔ ROS2 DDS 话题 双向翻译:
#         /fmu/out/...  = PX4 → ROS2 (传感器/状态)
#         /fmu/in/...   = ROS2 → PX4 (offboard 控制指令)
MicroXRCEAgent udp4 -p 8888

# ---- 窗格 [Lesson3]: 原别名 ytjy-sitl-run3 (原布局延迟 5s 等 PX4/DDS 就绪) --
# alias ytjy-sitl-run3='cd /root/fucker/ros2_ws; ros2 launch hw_insight lesson3.launch.py'
cd /root/fucker/ros2_ws
ros2 launch hw_insight lesson3.launch.py
#   launch 文件副本+注释: ../origins/launch/lesson3.launch.py
#   内部等效于逐条:
#     include airsim_ros_pkgs/launch/airsim_node.launch.py   # 连 UE/AirSim 端
#     ros2 run hw_insight  move_velocity                     # 速度控制节点(offboard)
#     rviz2 -d .../rviz/depth_cloud.rviz                     # 深度点云可视化
#     rviz2 -d .../rviz/image_lidar.rviz                     # 图像+虚拟激光可视化

# ---- 窗格 [keyboard control]: 原别名 ytjy-sitl-keyboard-velocity -----------
ros2 run hw_insight keyboard_velocity
#   源码: /root/fucker/ros2_ws/src/hw_insight/hw_insight/keyboard_velocity.py
#   本质: 终端原始模式读按键 → 维护 NED 速度分量(步进 X=5 Y=5 Z=2 Yaw=2, 单位 cm/s)
#         → 发布消息(经 move_velocity 或直发 /fmu/in/... 使无人机机动)
