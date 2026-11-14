#!/usr/bin/env bash
# ============================================================================
# 同步所有 AirSim settings.json 的 ControlIp (自适应网络模式 + 直写 Windows 端)
#
# 背景: settings.json 是 UE (Windows) 读的, 其中 ControlIp 指 PX4 所在的
#       WSL 地址, UE 每 1Hz 向 ControlIp:ControlPortRemote 发心跳 (MAVLink
#       控制通道)。ControlIp 过期 → 解锁/起飞等 armDisarm 类 RPC 全部
#       "Request timed out" (RPC 本身可达, 但 AirSim 无法把指令转给 PX4)。
#
# 网络模式自适应:
#   NAT      (WSL IP 在 172.16/12) → ControlIp = WSL IP (WSL 重启常变!)
#   mirrored (WSL 共享主机 IP)     → ControlIp = 127.0.0.1 (localhost 恒定,
#                                    WSL 与 Windows 共享端口空间, 直达 PX4)
#
# 同步范围: 仓库内全部 settings.json + Windows 端生效文件
#           (/mnt/c/Users/*/Documents/AirSim/settings.json) + 机型模板
#           (settings_*.json, 换机型复制成 settings.json 时不同步会把
#           过期 IP 带回来) —— 后两者必须直写, 靠手动复制必忘。
#
# 输出约定: env.sh 每次 source 调用, IP 未变时静默; 只有实际改写了文件
#           才输出一行 (ControlIp 只在 UE 启动时读取, 改完需重启 UE)。
#
# 用法: 开机启动 UE 前跑一次 (env.sh 已自动调用)
#   bash /root/flylab/scripts/sync-airsim-ip.sh
# ============================================================================
set -e

IP=$(hostname -I | awk '{print $1}')
if [ -z "$IP" ]; then
    echo "[sync-airsim-ip] 错误: 未获取到 WSL IP (hostname -I 为空)" >&2
    exit 1
fi

case "$IP" in
    172.1[6-9].*|172.2*|172.3[01].*)   # NAT 网段 (172.16/12)
        CONTROL_IP="$IP"; MODE="NAT" ;;
    *)                                  # mirrored: WSL IP = 主机 IP
        CONTROL_IP="127.0.0.1"; MODE="mirrored" ;;
esac

FILES=(
    /root/flylab/ros2_ws/src/flylab_flight/configs/settings.json
    /root/flylab/ros2_ws/src/flylab_flight/configs/*/settings.json
    /mnt/c/Users/*/Documents/AirSim/settings.json
    /mnt/c/Users/*/Documents/AirSim/settings_*.json
)
n=0
for f in "${FILES[@]}"; do
    [ -f "$f" ] || continue
    grep -q '"ControlIp"' "$f" || continue
    # 已是目标值直接跳过 (以此判断"实际变化", 避免每次 source 都输出)
    grep -q "\"ControlIp\": \"$CONTROL_IP\"" "$f" && continue
    if sed -i "s/\"ControlIp\": \"[^\"]*\"/\"ControlIp\": \"$CONTROL_IP\"/" "$f"; then
        n=$((n + 1))
    else
        echo "[sync-airsim-ip] 写入失败 (权限?): $f" >&2
    fi
done

if [ "$n" -gt 0 ]; then
    echo "[sync-airsim-ip] ControlIp 变更 → $CONTROL_IP ($MODE, $n 个文件) —— 须重启 UE 生效"
fi
