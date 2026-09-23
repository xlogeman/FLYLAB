#!/usr/bin/env bash
# ============================================================================
# 本质还原: ytjy-sitl-lesson11
# 原封装: alias ... ='ytjy-sitl-init-env; terminator --layout=ytjy-sitl-lesson11'
# terminator 布局 5 窗格:
#   [AirSim(sleep 2)][Camera FPS(sleep 5)][RVIZ(sleep 5)] | [SITL PX4][ORB SLAM3(sleep 5)]
#
# 课题: ORB-SLAM3 双目+惯性 (特征点法, 对比 VINS/OpenVINS)
# ⚠ 需要先在 Windows 宿主机启动 UE/AirSim 仿真环境
# ============================================================================
set -e
source /opt/ros/humble/setup.bash
source /root/fucker/ros2_ws/install/local_setup.bash

# ---- 窗格 [AirSim]: 又换一个取数节点, 按 ORB-SLAM3 需要的话题名发数据 -------
cd /root/fucker/ros2_ws
ros2 run airsim_ros_pkgs ytjy_airsim_ros_topic

# ---- 窗格 [Camera FPS] -----------------------------------------------------
ros2 topic hz /airsim/left_cam/image_raw -w 100

# ---- 窗格 [RVIZ] -----------------------------------------------------------
ros2 run rviz2 rviz2 -d /root/fucker/ros2_ws/src/hw_insight/rviz/lesson11.rviz

# ---- 窗格 [SITL PX4] -------------------------------------------------------
cd /root/fucker/px4.1.15.2 && make px4_sitl_default none_iris

# ---- 窗格 [ORB SLAM3] ------------------------------------------------------
ros2 run orbslam3 stereo-inertial \
  /root/fucker/ros2_ws/src/orb_slam3/vocabulary/ORBvoc.txt \
  /root/fucker/ros2_ws/src/hw_insight/orb_slam3/lesson11/stereo-inertial.yaml \
  false \
  --ros-args \
    -r /camera/left:=/airsim/left_cam/image_raw \
    -r /camera/right:=/airsim/right_cam/image_raw \
    -r /imu:=/airsim/imu_ros2/data
#   本质: ORB-SLAM3 的 stereo-inertial 模式(第3个参数 false=不做回环可视化? 见课程):
#     参数1: ORB 词袋字典文件 (特征匹配用)
#     参数2: 双目内参+双目外参+IMU噪声配置 yaml
#     重映射: 把 ORB-SLAM3 默认的 /camera/left /camera/right /imu 话题
#             接到 AirSim 自研节点发布的真实话题上
#   源码/配置: src/orb_slam3(算法本体) + src/hw_insight/orb_slam3/lesson11/(配置)
