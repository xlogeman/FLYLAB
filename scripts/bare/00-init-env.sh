#!/usr/bin/env bash
# ============================================================================
# 本质还原: ytjy-sitl-init-env  (= 每课启动前的"清场"动作)
# 原封装 (~/.bashrc:127):
#     alias ytjy-sitl-init-env='ytjy-clean-all; ytjy-px4-reset'
#     alias ytjy-clean-all='pkill -9 -f "px4|dds|lesson|ros|vins|airsim|python"'
#     alias ytjy-px4-reset='rm /root/fucker/px4.1.15.2/build/px4_sitl_default/rootfs/parameters*'
# ----------------------------------------------------------------------------
# 目的:
#   1) 杀掉上一轮仿真残留的所有进程 (PX4 / DDS Agent / ROS 节点 / VINS / AirSim / python)
#   2) 删除 PX4 的参数缓存文件 → 下次启动 PX4 时全部飞参恢复 airframe 默认值
#      (PX4 SITL 启动时把当前参数写到 rootfs/parameters*, 存在则不再按机型重新加载)
#
# ⚠ 学习提示: pkill 的匹配词太宽 (含 python/ros/lesson), 会误杀无关进程。
#   学习期间建议改用精确杀法:
#     pkill -x px4            # 只杀 PX4
#     pkill -f MicroXRCEAgent # 只杀 DDS Agent
# ============================================================================
set -x
pkill -9 -f "px4|dds|lesson|ros|vins|airsim|python" || true
rm -f /root/fucker/px4.1.15.2/build/px4_sitl_default/rootfs/parameters*
