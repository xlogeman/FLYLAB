#!/usr/bin/env bash
# ============================================================================
# 本质还原: ytjy-sitl-lesson4
# 原封装: alias ... ='ytjy-sitl-init-env; terminator --layout=ytjy-sitl-lesson4'
# terminator 布局 2 栏: [SITL PX4] | [Lesson4 (sleep 5)]
#
# 课题: 深度相机 → 点云 → octomap 占据栅格地图 (感知建图流水线)
# ============================================================================
set -e
source /opt/ros/humble/setup.bash
source /root/fucker/ros2_ws/install/local_setup.bash

# ---- 窗格 [SITL PX4] -------------------------------------------------------
cd /root/fucker/px4.1.15.2 && make px4_sitl_default none_iris

# ---- 窗格 [Lesson4]: 原别名 ytjy-sitl-run4 --------------------------------
cd /root/fucker/ros2_ws
ros2 launch hw_insight lesson4.launch.py
#   launch 文件副本+注释: ../origins/launch/lesson4.launch.py
#   内部等效于逐条:
#     include airsim_ros_pkgs/launch/airsim_node.launch.py
#       → airsim_node: 连接 UE 里的 AirSim 插件, 发布图像/IMU/GPS/里程计等话题
#
#     ros2 run depth_image_proc point_cloud_xyz_node  (重映射)
#       image_rect ← /airsim_node/PX4/CameraDepth/DepthPlanar   # AirSim 深度图
#       camera_info ← /airsim_node/PX4/CameraDepth/camera_info  # 相机内参
#       points     → /depth_camera/pointers                     # 输出点云
#       本质: 用内参把每个像素的深度值反投影成 3D 点 (针孔模型)
#
#     ros2 run octomap_server octomap_server_node  (参数+重映射)
#       cloud_in ← /depth_camera/pointers,  frame_id=world_ned, 分辨率 0.25m
#       本质: 点云 → 概率占据栅格(八叉树), 逐年融合多帧观测, 滤除噪点
#
#     rviz2 -d .../rviz/lesson4.rviz   # 显示地图
#
# 同目录还有变体 launch (更细的学习素材):
#   lesson4_depth_to_cloud.launch.py / lesson4_cloud_to_map.launch.py
#   lesson4_color.launch.py / lesson4_dynamic.launch.py
