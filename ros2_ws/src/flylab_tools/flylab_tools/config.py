#!/usr/bin/env python3
"""flylab_tools 全局配置 —— 改这里，不改代码。"""

# ---------- AirSim RPC (相机视图) ----------
AIRSIM_RPC_PORT = 41451   # 与 settings.json 的 ApiServerPort 一致

# ---------- ROS 通信 ----------
# 视觉功能选项卡显示的图像话题 (airsim_camera 等节点发布)
CAMERA_TOPICS = ['/airsim/front_cam/image_raw']

# 算法启停指令话题：按钮按下/抬起时发布 std_msgs/String，算法节点自行订阅执行
CMD_TOPIC = '/flylab/vision_cmd'

# 框选目标话题：画面上拖拽框选确认后发布 std_msgs/Int32MultiArray [x,y,w,h] (原图坐标)
BBOX_TOPIC = '/target/bbox'

# 模型输出叠加话题：模型节点每帧往这里发 std_msgs/String(JSON 标注列表)，
# 界面自动叠加到画面上。schema 见 overlay.py 文档。
ANNOTATION_TOPIC = '/flylab/annotations'

# ---------- 云台 (AirSim 相机视角旋转, 经 simSetCameraPose) ----------
GIMBAL_STEP_DEG = 1.0     # 每次按键步进角度 (按钮支持按住连发)

# 回中动画角速度 (度/秒): 回中不再是瞬跳, 而是以该速度平滑转回机头正前方,
# 按方向键可随时打断。越小越柔, 90 ≈ 实机云台回中的手感
GIMBAL_SLEW_RATE_DEG_S = 90.0

# 云台限位 (度), 模拟真实二轴云台的机械限位。
# 俯仰是相对 settings.json 安装角的: front/left/right_cam 自带 -36.87° 下俯
# 安装角, 绝对俯仰 = 相对俯仰 + 安装角。min/max 取 -53.13/+126.87 使绝对
# 俯仰恰好限定在 垂直向下(-90°) ~ 垂直向上(+90°), 到垂直即停。
# 注意 depth_cam 无安装角, 同一限位下可越过垂直各多转 36.87°
GIMBAL_YAW_LIMIT_DEG = 45.0      # 偏航限位 ±45°
GIMBAL_PITCH_MIN_DEG = -53.13    # 俯仰下限 (绝对 -90°, 垂直向下)
GIMBAL_PITCH_MAX_DEG = 126.87    # 俯仰上限 (绝对 +90°, 垂直向上)

# ---------- 工具按钮 (可切换: 按下=启动, 再按=停止) ----------
# name=按钮文字, start/stop=对应发布的指令
TOOLS = [
    {'name': '目标检测', 'start': 'detect_start', 'stop': 'detect_stop'},
    {'name': '目标追踪', 'start': 'track_start', 'stop': 'track_stop'},
]

# 相机画面是否启用拖拽框选 (发给 BBOX_TOPIC)
SELECT_TARGET = True
