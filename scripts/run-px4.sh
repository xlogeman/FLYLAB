#!/usr/bin/env bash
# ============================================================================
# 一键启动 PX4 SITL
# 用法:
#   ./run-px4.sh                        正常启动 (默认机型 none_iris)
#   ./run-px4.sh none_ytjy_x450         指定机型启动
#   ./run-px4.sh --clean                先删参数缓存再启动
#   ./run-px4.sh none_ytjy_x450 --clean 指定机型 + 清参数缓存
# 说明: make 已编译过时是秒级直接启动; 需要先开 Windows 端 UE/AirSim
# ============================================================================
set -e

PX4_DIR="/root/flylab/px4.1.15.2"

# 解析参数: 机型名 或 --clean (可任意顺序)
MODEL="none_iris"
CLEAN=""
for arg in "$@"; do
    case "$arg" in
        --clean) CLEAN="--clean" ;;
        *) MODEL="$arg" ;;
    esac
done

# ROS2 环境 (lesson1.sh 同款, 对 PX4 本身无害)
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash

# --clean: 删 PX4 参数缓存 (parameters*.bson), 否则上次改的机型参数/状态会残留
if [ "$CLEAN" = "--clean" ]; then
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
bash "$(dirname "$0")/px4-restore-git.sh"

# AirSim 地址 (PX4 SITL 经此连 TCP 4560 HIL): 已显式导出则尊重 (env.sh
# 按网络模式设置过); 未导出时按模式推导 —— mirrored (WSL 共享主机 IP,
# localhost 即 Windows 主机) 用 127.0.0.1; NAT (WSL IP 在 172.16/12) 下
# 默认网关即 Windows 主机。注意不能无条件取网关: mirrored 下网关是
# 路由器, PX4 会把 HIL 连到路由器上导致 AirSim 收不到载具
if [ -z "$PX4_SIM_HOST_ADDR" ]; then
    WSL_IP=$(hostname -I | awk '{print $1}')
    case "$WSL_IP" in
        172.1[6-9].*|172.2*|172.3[01].*)   # NAT 网段 (172.16/12)
            export PX4_SIM_HOST_ADDR=$(ip route | grep default | awk '{print $3}') ;;
        *)                                  # mirrored: WSL IP = 主机 IP
            export PX4_SIM_HOST_ADDR=127.0.0.1 ;;
    esac
fi
echo "[run-px4] PX4_SIM_HOST_ADDR=$PX4_SIM_HOST_ADDR"

cd "$PX4_DIR"
make px4_sitl_default "$MODEL"
