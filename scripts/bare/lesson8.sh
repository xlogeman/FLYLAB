#!/usr/bin/env bash
# ============================================================================
# 本质还原: ytjy-sitl-lesson8
# 原封装: alias ... ='ytjy-sitl-init-env; terminator --layout=ytjy-sitl-lesson8'
# terminator 布局 5 窗格:
#   [AirSim(sleep 3)][DDS AGENT][switch(sleep 12)] | [SITL PX4][VINS FUSION(sleep 12)]
#
# 课题: VINS-Fusion 双目+IMU (用 lesson8_vins_switch 做配置切换)
# ⚠ 需要先在 Windows 宿主机启动 UE/AirSim 仿真环境
# ============================================================================
set -e
source /opt/ros/humble/setup.bash
source /root/fucker/ros2_ws/install/local_setup.bash

# ---- 窗格 [AirSim]: 从这课起不再用 launch, 直接 run 节点 (延迟 3s) ----------
cd /root/fucker/ros2_ws
ros2 launch airsim_ros_pkgs airsim_node.launch.py
#   本质: 标准版 airsim_node, 从 UE 端拉取 图像/IMU/GPS/深度 等传感器话题

# ---- 窗格 [DDS AGENT] ------------------------------------------------------
MicroXRCEAgent udp4 -p 8888

# ---- 窗格 [switch]: 原别名 ytjy-sitl-run8 (延迟 12s 等 AirSim 数据就绪) -----
# alias ytjy-sitl-run8='cd /root/fucker/ros2_ws; ros2 launch hw_insight lesson8.launch.py'
cd /root/fucker/ros2_ws
ros2 launch hw_insight lesson8.launch.py
#   launch 副本+注释: ../origins/launch/lesson8.launch.py
#   ⚠ 注意: launch 里 vins_estimator/loop_fusion/global_fusion/airsim_node
#     全部被注释掉了, 实际只启动 3 个:
#     - static TF world_enu→world
#     - rviz2 -d .../rviz/lesson8.rviz
#     - ros2 run hw_insight lesson8_vins_switch
#       本质: 运行时动态"切换" VINS 的数据源/配置的胶水节点 (课程演示用)
#   真正的 VINS 由下面窗格单独跑:
cd /root/fucker/px4.1.15.2 && make px4_sitl_default none_iris    # 窗格 [SITL PX4]

# ---- 窗格 [VINS FUSION] (延迟 12s): ----------------------------------------
ros2 run vins vins_node \
  /root/fucker/ros2_ws/src/vins/config/ytjy/lesson8_extrinsics.yaml \
  --ros-args -p config_file:=/root/fucker/ros2_ws/src/vins/config/ytjy/lesson8_extrinsics.yaml \
             -r __node:=vins_estimator -r __ns:=/vins_estimator
#   本质: VINS-Fusion 双目+IMU 紧耦合里程计;
#         lesson8_extrinsics.yaml 里配了双目相机外参 + 与 AirSim 话题的对接
#         (图像话题/IMU 话题/频率/外参矩阵都在该 yaml 中, 值得逐行读)
