#!/usr/bin/env bash
# ============================================================================
# 本质还原: ytjy-sitl-lesson2
# 原封装: alias ... ='ytjy-sitl-init-env; terminator --layout=ytjy-sitl-lesson2'
# terminator 布局: 左[MAVSDK AUTO AVOIDANCE] | 右[SITL PX4] (右窗格 sleep 5)
#
# 课题: MAVSDK(Python) 任务飞行 + 简单避障
# ============================================================================
set -e

# ---- 窗格 [SITL PX4] (先启动, 等 5 秒是为了让 PX4 就绪) --------------------
source /opt/ros/humble/setup.bash
cd /root/fucker/px4.1.15.2
make px4_sitl_default none_iris

# ---- 窗格 [MAVSDK AUTO AVOIDANCE]: 原别名 ytjy-sitl-run2 ------------------
# alias ytjy-sitl-run2='cd /root/fucker/ros2_ws; /root/fucker/scripts/origins/hw-avoidance-simple.py'
# 注意: 这课完全没用 ROS2! 脚本用 MAVSDK 直连 PX4 的 MAVLink 口
python3 /root/fucker/scripts/origins/hw-avoidance-simple.py
#   副本+逐行注释: ../origins/hw-avoidance-simple.py
#
# 脚本本质 (见源码):
#   1. drone.connect("udp://:14540")   → PX4 SITL 默认在 14540 等连接
#   2. monitor_distance()              → 订阅 distance_sensor (5 路虚拟测距: 前/左45/右45/上45/下45)
#   3. check_leave_home()              → 判断是否远离 home (home 附近不避障)
#   4. obstacle_avoidance() 主循环:
#        最小距离 < 7m → mission.pause_mission() 暂停任务
#        → offboard 速度控制: 先倒退 0.5s, 再以 z=-1.7 (NED 下为负=向上) 爬升
#        距离 > 15m → 停止爬升, offboard.stop(), mission.start_mission() 恢复任务
