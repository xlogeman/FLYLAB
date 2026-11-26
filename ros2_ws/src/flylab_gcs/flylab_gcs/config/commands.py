#!/usr/bin/env python3
"""进程启动命令与指令字符串 (ros2 run/launch、AirSim 飞控命令、反控
模式枚举) —— 工具栏各面板经 ManagedProcess 拉起/回收的命令都在这里。"""

from .gimbal import TELEOP_YAW_RATE_DEG_S
from .paths import ARUCO_PARAMS_FILE

# ---------- ArUco 降落 (主窗口独立按钮) ----------
# 点击拉起 aruco_landing_node（接管 offboard → 视觉对齐调速下降 → 切 PX4
# AUTO.LAND），再点发 landing_stop 取消。与算法下拉框互不影响。
# 参数来自 flylab_vision/configs/aruco.yaml（--params-file 携带, -p 可临时覆盖）。
# 前提: 通信链路在跑 + 已解锁升空 + 运行 enables_landing 的算法
LANDING_CMD = ['ros2', 'run', 'flylab_vision', 'aruco_landing_node',
               '--ros-args', '--params-file', ARUCO_PARAMS_FILE]
LANDING_START_CMD = 'landing_start'
LANDING_STOP_CMD = 'landing_stop'

# ---------- 视觉反控降落模式 (算法注册表 landing_modes 声明) ----------
# 反控形态随算法: 声明了多个 landing_modes 的算法 (码检测) 在"接管控制→
# 视觉反控"子菜单显示模式子项, 未声明的不显示模式子项:
#   图像空间 = 现有逻辑 (cx/cy/面积图像伺服, 不依赖 3D 位姿)
#   3D 姿态  = 以码的实际距离 (solvePnP 位姿, 经云台姿态旋到机体系) 米级闭环
#              降落 —— 前提是位姿尺度可信 (码边长准确或已标定)
# 模式经 -p landing_mode:=<mode> 传给 aruco_landing_node。其他算法将来可在
# ALGORITHMS 注册各自的反控形态。
LANDING_MODE_IMAGE = 'image'
LANDING_MODE_POSE3D = 'pose3d'

# ---------- 目标追踪 (画面框选即追踪) ----------
# 流程: 菜单启动追踪节点 (yolo_track/ortrack) → 画面拖出矩形松开即发 ROI
# (latched) + track_begin 自动锁定 (yolo: 框选区检测裁块出 ReID 外观向量,
# 逐帧余弦相似度续跟; ortrack: 直接裁 128 模板逐帧匹配), 追踪中重新框选
# 即换目标; 停止靠工具栏"停止算法" (无独立取消指令)。ROI 进入追踪即消费。
TRACK_BEGIN_CMD = 'track_begin'
# 目标追踪反控 (接管控制菜单 → 追踪节点, 与 ArUco 降落反控同一机制):
# 追踪节点接管 offboard 拿到飞机控制权 (悬停跟拍 + 机头跟随), 云台伺服
# 让目标保持在画面中央; 再点菜单项或停止算法即交还控制权
TRACK_TAKEOVER_START_CMD = 'track_takeover_start'
TRACK_TAKEOVER_STOP_CMD = 'track_takeover_stop'
# 目标追踪反控模式 (菜单子项; 随 track_takeover_start 以模式后缀下发
# 节点, 如 "track_takeover_start dive_curve", 同 landing_mode 的传参约定)。
# 悬停跟拍/曲线/直线打击/机体跟随(水平) 为现役功能, 直线跟随为预留接口
TRACK_TAKEOVER_GIMBAL = 'gimbal'          # 悬停跟拍 (现役)
TRACK_TAKEOVER_DIVE = 'dive_curve'        # 曲线打击: 两相制导定速撞击 (现役)
TRACK_TAKEOVER_STRAIGHT = 'dive_straight'  # 直线打击: 全程沿视线不分相 (现役)
TRACK_TAKEOVER_BODYFOLLOW = 'bodyfollow_horiz'  # 机体跟随-水平: 保持水平距离跟随 (现役)
TRACK_TAKEOVER_BODYFOLLOW_SLANT = 'bodyfollow_slant'  # 机体跟随-直线: 保持斜距跟随 (预留)

# ---------- 手动遥控 (双摇杆 → offboard 速度, 执行节点 flylab_vision/teleop_node) ----------
TELEOP_START_CMD = 'teleop_start'
TELEOP_STOP_CMD = 'teleop_stop'
TELEOP_CMD = ['ros2', 'run', 'flylab_vision', 'teleop_node',
              '--ros-args',
              '-p', 'v_xy:=10.0',          # 平移速度上限 m/s (默认5.0; 飞控按 MPC_XY_VEL_MAX 二次夹紧)
              '-p', 'v_z:=3.0',            # 升降速度上限 m/s (默认2.0; 飞控按 MPC_Z_VEL_MAX_UP/DN 二次夹紧)
              '-p', f'yaw_rate_deg:={TELEOP_YAW_RATE_DEG_S}']  # 与 gimbal.TELEOP_YAW_RATE_DEG_S 同源

# ---------- 通信链路 (地面站一键拉起) ----------
# uXRCE-DDS: PX4 ↔ ROS2 直连桥, UDP 8888, 供 tracker:=uxrce / aruco_landing 链路
UXRCE_CMD = ['MicroXRCEAgent', 'udp4', '-p', '8888']
# MAVROS: 14542 专属实例 (14580 会被 AirSim 抢占, 详见 README)
MAVROS_CMD = ['ros2', 'run', 'mavros', 'mavros_node', '--ros-args',
              '-p', 'fcu_url:=udp://:14542@', '-p', 'tgt_system:=1',
              '-p', 'tgt_component:=1']

# ---------- 飞行控制 (AirSim API 经 MAVLink 控制通道直发 PX4, 与 offboard tracker 独立) ----------
VEHICLE_CMDS = [
    {'name': '解锁 (Arm)',     'cmd': 'arm'},
    {'name': '起飞 (Takeoff)', 'cmd': 'takeoff'},
    {'name': '降落 (Land)',    'cmd': 'land'},
    {'name': '悬停 (Hover)',   'cmd': 'hover'},
    {'name': '返航 (GoHome)',  'cmd': 'gohome'},
    {'name': '上锁 (Disarm)',  'cmd': 'disarm'},
]
