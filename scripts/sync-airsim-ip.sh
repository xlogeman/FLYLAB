#!/usr/bin/env bash
# ============================================================================
# 同步所有 AirSim settings.json 的 ControlIp 为当前 WSL IP
#
# 背景: settings.json 是 UE (Windows) 读的, 其中 ControlIp 指 PX4 所在的
#       WSL 地址, UE 每 1Hz 向 ControlIp:ControlPortRemote 发心跳。
#       WSL 重启后 eth0 IP 常变, 硬编码的旧 IP 会让控制通道静默断掉
#       (UE 端无报错, PX4 侧表现为收到 TCP HIL 数据但无 UDP 心跳)。
#
# 用法: 每次开机启动 UE 前跑一次
#   bash /root/flylab/scripts/sync-airsim-ip.sh
# ============================================================================
set -e

IP=$(hostname -I | awk '{print $1}')
if [ -z "$IP" ]; then
    echo "[sync-airsim-ip] 错误: 未获取到 WSL IP (hostname -I 为空)" >&2
    exit 1
fi
echo "[sync-airsim-ip] 当前 WSL IP: $IP"

for f in /root/flylab/ros2_ws/src/flylab/configs/settings.json \
         /root/flylab/ros2_ws/src/flylab/configs/*/settings.json; do
    [ -f "$f" ] || continue
    if grep -q '"ControlIp"' "$f"; then
        sed -i "s/\"ControlIp\": \"[^\"]*\"/\"ControlIp\": \"$IP\"/" "$f"
        echo "  updated: $f"
    else
        echo "  skip (无 ControlIp): $f"
    fi
done

echo "[sync-airsim-ip] 完成。记得把改动后的文件复制到 Windows 端 Documents\\AirSim\\settings.json"
