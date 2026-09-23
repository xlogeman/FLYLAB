#!/usr/bin/env bash
# ============================================================================
# 本质还原: ytjy-sitl-lesson7
# 原封装: alias ... ='ytjy-sitl-init-env; terminator --layout=ytjy-sitl-lesson7'
# terminator 布局 2 栏: [SITL PX4] | [Lesson7 (sleep 5)]
#
# 课题: VINS-Fusion 单目+IMU 视觉惯性里程计 + GPS 全局融合 (三色轨迹对比)
# ============================================================================
set -e
source /opt/ros/humble/setup.bash
source /root/fucker/ros2_ws/install/local_setup.bash

# ---- 窗格 [SITL PX4] -------------------------------------------------------
cd /root/fucker/px4.1.15.2 && make px4_sitl_default none_iris

# ---- 窗格 [Lesson7]: 原别名 ytjy-sitl-run7 --------------------------------
cd /root/fucker/ros2_ws
ros2 launch hw_insight lesson7.launch.py
#   launch 副本+注释: ../origins/launch/lesson7.launch.py
#   内部等效于逐条:
#     include airsim_ros_pkgs/launch/airsim_node.launch.py
#
#     ros2 run topic_tools throttle messages \
#          /airsim_node/PX4/gps/Gps 5.0 /airsim_node/PX4/gps/gps_throttled
#       本质: AirSim 的 GPS 话题是 100Hz, 真机 GPS 只有 1~10Hz;
#             降频到 5Hz 模拟真实 GPS, 再喂给 global_fusion
#
#     ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world map
#       本质: RViz 需要一棵完整 TF 树, 手工补 world→map 静态变换
#
#     rviz2 -d .../rviz/lesson7.rviz
#
#     ros2 run vins vins_node /root/fucker/ros2_ws/src/vins/config/AIRSIM/euroc_mono_imu_config.yaml
#       本质: VINS-Fusion 单目+IMU 紧耦合优化 → 绿色轨迹 (纯视觉惯性, 会漂移)
#
#     ros2 run loop_fusion loop_fusion_node <同一配置>
#       本质: 回环检测修正累计漂移 → 红色轨迹
#
#     ros2 run global_fusion global_fusion_node  (gps_type:=0, /gps←gps_throttled)
#       本质: 把(降频)GPS 和 VINS 局部里程计做图优化融合 → 白色轨迹 (全局一致)
#
# 学习主线: 看三条轨迹怎么分叉/贴合 → 理解 视觉漂移 / 回环 / GPS 全局约束
