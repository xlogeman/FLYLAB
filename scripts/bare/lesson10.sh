#!/usr/bin/env bash
# ============================================================================
# 本质还原: ytjy-sitl-lesson10
# 原封装: alias ... ='ytjy-sitl-init-env; terminator --layout=ytjy-sitl-lesson10'
# terminator 布局 5 窗格:
#   [AirSim(sleep 2)][Camera FPS(sleep 5)][RVIZ(sleep 5)] | [SITL PX4][OPEN VINS(sleep 5)]
#
# 课题: OpenVINS 滤波式视觉惯性里程计 (对比 lesson8/9 的优化式 VINS)
# ⚠ 需要先在 Windows 宿主机启动 UE/AirSim 仿真环境
# ============================================================================
set -e
source /opt/ros/humble/setup.bash
source /root/fucker/ros2_ws/install/local_setup.bash

# ---- 窗格 [AirSim] ---------------------------------------------------------
cd /root/fucker/ros2_ws
ros2 run airsim_ros_pkgs ytjy_airsim_fast

# ---- 窗格 [Camera FPS]: 数据链路体检 --------------------------------------
ros2 topic hz /airsim/left_cam/image_raw -w 100
#   本质: 统计左目图像实际帧率 (窗口 100 帧)。SLAM 对帧率敏感, 先确认数据源 OK

# ---- 窗格 [RVIZ]: 原别名 ytjy-sitl-run9-rviz 名字有误导, 实际跑 lesson10 ----
# 注意 terminator 布局里 command 写的是 'ros2 launch hw_insight lesson10.launch.py'
ros2 launch hw_insight lesson10.launch.py
#   launch 副本: ../origins/launch/lesson10.launch.py
#   内部: 只有 rviz2 -d .../rviz/lesson10.rviz (ytjy_airsim_fast 被注释)
#   可视化内容: OpenVINS 的位姿/特征点/轨迹

# ---- 窗格 [SITL PX4] -------------------------------------------------------
cd /root/fucker/px4.1.15.2 && make px4_sitl_default none_iris

# ---- 窗格 [OPEN VINS] ------------------------------------------------------
ros2 run ov_msckf run_subscribe_msckf \
  --ros-args -p config_path:=/root/fucker/ros2_ws/src/hw_insight/openvins/lesson10/estimator_config.yaml
#   本质: OpenVINS = MSCKF(多状态约束卡尔曼滤波) 视觉惯性里程计, 纯滤波,
#         与 VINS-Fusion(滑动窗口非线性优化) 是两条技术路线; 订阅图像+IMU,
#         配置在 openvins/lesson10/estimator_config.yaml
