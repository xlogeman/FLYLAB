#!/usr/bin/env bash
# ============================================================================
# 本质还原: ytjy-sitl-lesson1
# 原封装: alias ytjy-sitl-lesson1='ytjy-sitl-init-env; terminator --layout=ytjy-sitl-lesson1'
# terminator 布局(terminator/config): 单窗格 [SITL PX4]
#
# 课题: PX4 SITL 最小系统 —— 不接任何外部仿真器, 纯 PX4 内置动力学仿真
# ============================================================================
set -e
source /opt/ros/humble/setup.bash

# ---- 窗格 [SITL PX4] 的真实命令 (~/.bashrc: ytjy-sitl-px4) ------------------
cd /root/fucker/px4.1.15.2
make px4_sitl_default none_iris
#
# 展开本质:
#   "make px4_sitl_default"  → 编译出 build/px4_sitl_default/bin/px4 可执行文件
#   "none_iris"              → make 的第二个参数 = 机型目标:
#     - 设置 PX4_SIM_MODEL=none_iris 后直接启动 px4
#     - 加载机型脚本 ROMFS/px4fmu_common/init.d-posix/airframes/10016_none_iris
#       (副本: ../origins/airframes/10016_none_iris)
#
# 等效手动写法(更能看清本质):
#   make px4_sitl_default                      # 先编译
#   PX4_SIM_MODEL=none_iris ./build/px4_sitl_default/bin/px4
#
# 启动后自带的 MAVLink 接口(后面课程全靠它):
#   udp://:14540  → 供 SDK/MAVROS/GCS 连接 (offsdk, QGC 等)
#   udp://:14550  → 供地面站
#   串口  /dev/ttyACM0(虚拟) → 对应 uXRCE-DDS 客户端通道 (lesson3 起的 DDS 桥)
