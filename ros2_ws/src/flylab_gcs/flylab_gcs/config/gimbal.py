#!/usr/bin/env python3
"""云台与航向跟随的控制律参数 (AirSim 相机视角旋转, 经 simSetCameraPose)。"""

# ---------- 云台 ----------
GIMBAL_JOY_RATE_DEG_S = 60.0   # 摇杆满偏转动角速度 (°/s)

# 飞行中姿态跟随频率 (Hz)：按当前机体姿态重下发云台角，防止按键时积累姿态差
# 一次性归零；同时也是 follow 模式的重下发频率。越高抗晃动越好，但 RPC 与
# 渲染开销增大。0 = 关闭（仅地面静态调试）
GIMBAL_HOLD_HZ = 15.0

# 回中动画角速度 (°/s)：平滑转回机头正前方，方向键可打断
GIMBAL_SLEW_RATE_DEG_S = 90.0

# 云台限位 (度)。俯仰相对 settings.json 安装角（各相机安装 Pitch 已为 0），
# -90/+90 即垂直向下 ~ 垂直向上。安装角由 simGetCameraInfo 运行时实测，
# 改安装角只需同步调整限位，云台/反控摆位/降落方向映射自动适配
GIMBAL_YAW_LIMIT_DEG = 45.0      # 偏航 ±45°
GIMBAL_PITCH_MIN_DEG = -90.0     # 俯仰下限 (垂直向下)
GIMBAL_PITCH_MAX_DEG = 90.0      # 俯仰上限 (垂直向上)

# ---------- 航向跟随 (云台偏航带动机头转向视线方向) ----------
# 双环: 外环在地面站 —— 误差 = 反卷后的云台偏航 (视线世界方位 − 航向),
# P 律转成偏航角速率并入遥控量 angular.z (走手动遥控 offboard 通道);
# 内环是 PX4 偏航角速率闭环 (teleop_node sendVelocityFRDYawRate)。
# 相机侧由抓取线程反卷: 航向转动多少, 存储的云台偏航就回卷多少,
# 视线钉在世界系, 航向追上视线时云台偏航自然回中。
# 前提: 通信链路已开 + 手动遥控接管中 (云台面板"跟随"按钮随之解锁)。
GIMBAL_FOLLOW_HZ = 25.0                # 外环频率 (Hz, 界面定时器)
GIMBAL_FOLLOW_KP = 1.2                 # P 增益 (1/s): 满速率对应误差 ≈ 速率/KP
GIMBAL_FOLLOW_DEADBAND_DEG = 2.0       # 死区: 悬停姿态噪声不触发无限微转
GIMBAL_FOLLOW_FRESH_SEC = 0.5          # 航向测量新鲜度上限 (过期按误差 0 停转)
GIMBAL_FOLLOW_USER_YAW_DEADBAND = 0.05 # 遥控偏航杆死区 (归一化): 超过视为用户在转机头
GIMBAL_FOLLOW_USER_HOLD_SEC = 0.3      # 用户转机头判定窗口: 锚点随实测航向平移 (相机偏航临时刚性)

# 偏航角速率上限: 航向跟随外环与 teleop_node 共用同一上限 —— 修改时两处
# 自动同步 (teleop 启动命令经 commands.TELEOP_CMD 引用本常量下发)
TELEOP_YAW_RATE_DEG_S = 45.0
GIMBAL_FOLLOW_YAW_RATE_DEG_S = TELEOP_YAW_RATE_DEG_S

# ---------- 数字变焦 (显示层中心裁切, 不改 UE 相机 FOV) ----------
# 在抓取层实现（裁切 1/z 后放回原分辨率），图传/转发/算法标注同一像素系。
# 云台面板竖滑杆 / 滚轮步进 / 双击复位
DISPLAY_ZOOM_MIN = 1.0      # 变焦下限 (原图)
DISPLAY_ZOOM_MAX = 4.0      # 变焦上限 (倍)
DISPLAY_ZOOM_STEP = 0.1     # 滚轮每档步进 (倍)

# ---------- 视场角 (AirSim 相机, 经 simSetCameraFov 实时调整) ----------
# 所有 UE 相机共用一个值，切相机时对齐。默认值仅用于界面显示，用户动过滑杆
# 才下发并持久化（与云台角同一状态文件），重启/重连自动恢复。
# 注意: FOV 变化会影响按 FOV 估算内参的算法（如 ArUco 降落）
GIMBAL_FOV_MIN_DEG = 30.0
GIMBAL_FOV_MAX_DEG = 120.0
GIMBAL_FOV_DEFAULT_DEG = 90.0
