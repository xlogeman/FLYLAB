#!/usr/bin/env bash
# ============================================================================
# 本质还原: ytjy-sitl-lesson5-keyboard   (多机课, 注意 PX4 启动方式变了)
# 原封装: alias ... ='ytjy-sitl-init-env; terminator --layout=ytjy-sitl-lesson5-keyboard'
# terminator 布局 4 窗格: [SITL PX4 多机][DDS AGENT] | [Lesson5 keyboard(sleep 5)][keyboard]
#
# 课题: PX4 多实例仿真 (1 台机 4 个实例) + 多机 ROS2 控制
# ============================================================================
set -e
source /opt/ros/humble/setup.bash
source /root/fucker/ros2_ws/install/local_setup.bash

# ---- 窗格 [SITL PX4]: 原别名 ytjy-sitl-px4-multi (和前面课程的单机启动不同!) --
# alias ytjy-sitl-px4-multi='cd /root/fucker/px4.1.15.2/ytjy; ./killpx4.sh; ./lesson5.sh'
cd /root/fucker/px4.1.15.2/ytjy
./killpx4.sh        # 内容就一行: pkill -x px4  (副本: ../origins/killpx4.sh)
./lesson5.sh        # 副本: ../origins/lesson5-multi-px4.sh
#
# lesson5.sh 的本质(逐行):
#   export PX4_SIM_MODEL=none_iris
#   ../build/px4_sitl_default/bin/px4 -d -i 1 &   # 实例1 后台
#   ../build/px4_sitl_default/bin/px4 -d -i 2 &   # 实例2 后台
#   ../build/px4_sitl_default/bin/px4 -d -i 3 &   # 实例3 后台
#   ../build/px4_sitl_default/bin/px4 -d -i 4     # 实例4 前台(挡住窗格)
#
# 关键点: -i N 指定实例号 → 端口自动错开:
#   实例 N 的 SDK 端口 = 14540 + (N-1)  → 14540/14541/14542/14543
#   (这就是为什么 ROS2/MAVSDK 侧要用 px4_instance 参数区分连哪台)

# ---- 窗格 [DDS AGENT] ------------------------------------------------------
MicroXRCEAgent udp4 -p 8888      # 一个 Agent 可以同时桥接 4 个 PX4 实例

# ---- 窗格 [Lesson5 keyboard]: 原别名 ytjy-sitl-run5-keyboard ---------------
cd /root/fucker/ros2_ws
ros2 launch hw_insight lesson5-keyboard.launch.py
#   launch 副本+注释: ../origins/launch/lesson5-keyboard.launch.py
#   内部等效于逐条:
#     include airsim_ros_pkgs/launch/airsim_node.launch.py
#     ros2 run hw_insight multi_move_velocity  -p px4_instance:=1   # 控制实例1
#     ros2 run hw_insight multi_move_velocity  -p px4_instance:=2   # 控制实例2
#     ... (共 5 个, 实例 1~5)
#     rviz2 -d .../rviz/lesson5.rviz
#   multi_move_velocity 本质 = move_velocity 的多机版: 按实例号选择不同的
#   PX4 话题命名空间, 把速度指令发给指定的那架飞机

# ---- 窗格 [keyboard] -------------------------------------------------------
ros2 run hw_insight keyboard_velocity
