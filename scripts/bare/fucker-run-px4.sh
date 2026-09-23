#!/usr/bin/env bash
# ============================================================================
# 一键启动 PX4 SITL (机型: 10016_none_iris)
# 用法:
#   ./run-px4.sh            正常启动
#   ./run-px4.sh --clean    先删参数缓存再启动 (等价课程习惯的 ytjy-px4-reset)
# 说明: make 已编译过时是秒级直接启动; 需要先开 Windows 端 UE/AirSim
# ============================================================================
set -e

PX4_DIR="/root/fucker/px4.1.15.2"

# ROS2 环境 (lesson1.sh 同款, 对 PX4 本身无害)
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash

# --clean: 删 PX4 参数缓存 (parameters*.bson), 否则上次改的机型参数/状态会残留
if [ "$1" = "--clean" ]; then
    echo "[run-px4] 清理 PX4 参数缓存..."
    rm -f "$PX4_DIR/parameters.bson" "$PX4_DIR/parameters_backup.bson"
fi

# 清理残留锁文件: px4 被 SIGKILL/WSL 崩溃后 /tmp/px4_lock-N 不会自动消失,
# 会导致 "PX4 server already running" 假报错。仅在确认无 px4 存活时才删。
if ! pgrep -f "bin/px4" > /dev/null; then
    rm -f /tmp/px4_lock-*
fi

# git 元数据自检 (幂等): 主仓库 .git / 子模块 / NuttX tag 任一缺失自动补齐,
# 全部完整时秒退。归档瘦身剔除了 git 历史, 首次运行或迁移目录后会触发修复
bash "$(dirname "$0")/../px4-restore-git.sh"

# Windows 网关 IP = WSL 默认路由 (AirSim 在那台机器上监听 TCP 4560)
export PX4_SIM_HOST_ADDR=$(ip route | grep default | awk '{print $3}')
echo "[run-px4] PX4_SIM_HOST_ADDR=$PX4_SIM_HOST_ADDR"

cd "$PX4_DIR"
make px4_sitl_default none_iris
