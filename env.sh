#!/usr/bin/env bash
# ============================================================================
# 环境总入口 —— 编译和运行前都先 source 本文件
#   source /root/fucker/env.sh
# (work/ 自包含, 不再依赖 code/ 原始归档)
# ============================================================================

# ROS2 Humble
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash

# colcon 工作空间 (build 之后自动生效)
_ws=/root/fucker/ros2_ws
if [ -f "$_ws/install/setup.bash" ]; then
    source "$_ws/install/setup.bash"
fi

# ORB_SLAM3 运行时库 (orbslam3 wrapper 找 libORB_SLAM3.so / libDBoW2 / libg2o)
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:\
/root/fucker/ORB_SLAM3_FIXED/lib:\
/root/fucker/ORB_SLAM3_FIXED/Thirdparty/DBoW2/lib:\
/root/fucker/ORB_SLAM3_FIXED/Thirdparty/g2o/lib

# AirSim 所在的 Windows 宿主机 (WSL 默认网关)
export PX4_SIM_HOST_ADDR=$(ip route | grep default | awk '{print $3}')

# rviz/ORB 走软件渲染 (WSL 无独显穿透)
export LIBGL_ALWAYS_SOFTWARE=1
export DISPLAY=${DISPLAY:-:0}
