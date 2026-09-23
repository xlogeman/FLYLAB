#!/usr/bin/env bash
# ============================================================================
# 本质还原: ytjy-sitl-lesson6
# 原封装: alias ... ='ytjy-sitl-init-env; terminator --layout=ytjy-sitl-lesson6'
# terminator 布局 4 窗格: [SITL PX4][DDS AGENT] | [Lesson6(sleep 5)][keyboard]
#
# 课题: YOLO 目标检测 + 检测结果驱动的目标追踪
# ============================================================================
set -e
source /opt/ros/humble/setup.bash
source /root/fucker/ros2_ws/install/local_setup.bash

# ---- 窗格 [SITL PX4] -------------------------------------------------------
cd /root/fucker/px4.1.15.2 && make px4_sitl_default none_iris

# ---- 窗格 [DDS AGENT] ------------------------------------------------------
MicroXRCEAgent udp4 -p 8888

# ---- 窗格 [Lesson6]: 原别名 ytjy-sitl-run6 --------------------------------
cd /root/fucker/ros2_ws
ros2 launch hw_insight lesson6.launch.py
#   launch 副本+注释: ../origins/launch/lesson6.launch.py
#   内部等效于逐条:
#     ros2 run hw_insight lesson6_yolo
#       参数: yolo_model=yolov8n.pt, camera_name=CameraDepth1
#       本质: 订阅 AirSim 深度/图像话题 → ultralytics YOLOv8 推理 → 发布检测结果
#       (launch 里还有 CameraDepth2/3 两个 yolo 实例, 被注释了)
#     include airsim_ros_pkgs/launch/airsim_node.launch.py
#     rviz2 -d .../rviz/lesson6.rviz
#     ros2 run hw_insight lesson6_track
#       本质: 订阅 YOLO 检测框 → 计算目标在画面中的偏差 → 转成速度指令让飞机追踪目标
# 源码: .../src/hw_insight/hw_insight/lesson6_yolo.py  lesson6_track.py
# (变体: lesson6_noyolo.launch.py = 不跑 YOLO 只跑追踪)

# ---- 窗格 [keyboard] -------------------------------------------------------
ros2 run hw_insight keyboard_velocity
