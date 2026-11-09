#!/usr/bin/env bash
# ============================================================================
# 清理 Fast DDS 遗留共享内存段 (fastrtps_*)
#
# 背景: Fast DDS 的 SHM 传输在同机进程间零拷贝传数据 (env.sh 经
# fastrtps_shm.xml 启用, 30Hz 图像转发必需), 但节点进程被强杀 (kill/崩溃/
# 地面站杀子进程) 时不会清理已创建的段, 日积月累把 /dev/shm 吃满 —— 满了
# 之后新节点报 "Failed to create segment ... No such file or directory",
# 全部回退 UDP, 某些版本还会连 UDP 一起出问题。
#
# 安全性: 逐段 fuser 检测占用, 仍被存活进程使用的段 (正在跑的 GCS/节点)
# 一律跳过, 只删真正无主的遗留段 —— 任何时机运行都不会打断通信。
#
# 用法: 独立运行, 或被 env.sh 在 source 时自动调用 (静默, 只在有清理时输出)
#   bash /root/flylab/scripts/clean-fastrtps-shm.sh
# ============================================================================

shopt -s nullglob
removed=0
total=0
freed_bytes=0

for f in /dev/shm/fastrtps_*; do
    total=$((total + 1))
    # fuser -s 静默探测: 退出码 0 = 有进程占用, 非 0 = 无主
    if fuser -s "$f" 2>/dev/null; then
        continue
    fi
    size=$(stat -c %s "$f" 2>/dev/null || echo 0)
    rm -f -- "$f" && {
        removed=$((removed + 1))
        freed_bytes=$((freed_bytes + size))
    }
done

if [ "$removed" -gt 0 ]; then
    printf '[clean-fastrtps-shm] 清理遗留共享内存段 %d/%d 个, 释放 %d MB (/dev/shm 剩余 %s)\n' \
        "$removed" "$total" "$((freed_bytes / 1024 / 1024))" \
        "$(df -h /dev/shm --output=avail | tail -1 | tr -d ' ')"
fi
