#!/usr/bin/env python3
"""全局配置 —— 路径、端口与窗口常量。"""

import os

from ament_index_python.packages import get_package_share_directory

# ---------- flylab_vision 参数配置目录 (ArUco 检测/降落等, 改 yaml 即调参) ----------
VISION_CONFIG_DIR = os.path.join(get_package_share_directory('flylab_vision'), 'configs')
ARUCO_PARAMS_FILE = os.path.join(VISION_CONFIG_DIR, 'aruco.yaml')

# ---------- 地面站窗口大小 ----------
# 初始窗口大小 = 屏幕可用区域 × 比例（仅初始值，窗口可拖拽调整）
WINDOW_SIZE_RATIO = (0.63, 0.75)   # (宽, 高)

# ---------- AirSim RPC (相机视图) ----------
AIRSIM_RPC_PORT = 41451   # 与 settings.json 的 ApiServerPort 一致

# ---------- PX4 SITL / ULG 路径 ----------
# 经 scripts/run-px4.sh 启动，target 与 PX4_SYS_AUTOSTART 严格对应（见 README §6，
# 机型注册表见 registry.PX4_MODELS）
PX4_SCRIPT = '/root/flylab/scripts/run-px4.sh'
# PX4 SITL 日志根目录 (root/日期/时刻.ulg), run-px4.sh 默认 build rootfs 下;
# 实机 SD 卡日志用窗口内"打开文件..."手动选
ULG_LOG_ROOT = '/root/flylab/px4.1.15.2/build/px4_sitl_default/rootfs/log'
