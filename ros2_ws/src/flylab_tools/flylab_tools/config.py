#!/usr/bin/env python3
"""flylab_tools 全局配置 —— 改这里，不改代码。"""

# ---------- AirSim RPC (相机视图) ----------
AIRSIM_RPC_PORT = 41451   # 与 settings.json 的 ApiServerPort 一致

# ---------- ROS 通信 ----------
# 视觉功能选项卡显示的图像话题 (airsim_camera 等节点发布)
CAMERA_TOPICS = ['/airsim/front_cam/image_raw']

# ---------- 显示画面转发 (算法节点的图像输入) ----------
# 地面站把"当前显示的帧"发布到该话题 —— 无论是 AirSim 相机、本机摄像头
# 还是视频文件源, 算法节点订阅它即可处理用户正在看的画面 (深度伪彩帧除外)。
# 分辨率保持原图, 保证标注叠加坐标对齐。
DISPLAY_IMAGE_TOPIC = '/flylab/display/image_raw'
DISPLAY_IMAGE_HZ = 30      # 转发频率上限 (0 = 关闭转发)

# 算法启停指令话题：按钮按下/抬起时发布 std_msgs/String，算法节点自行订阅执行
CMD_TOPIC = '/flylab/vision_cmd'

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

# ---------- 算法选择 (下拉框 + 运行/取消按钮) ----------
# name=下拉框显示文字, start/stop=运行/取消时发布的指令。
# cmd=算法节点启动命令 (可选): 填了则地面站点"运行"时自动拉起进程、点"取消"
# 时自动回收; 不填则假定节点已在外部运行, 只发启停指令。
# 界面交互对所有算法统一: 新算法只要遵循"收到 start 指令开始、收到 stop
# 指令停止"的约定, 在这里加一项即可出现在下拉框里。
ALGORITHMS = [
    {'name': 'ArUco检测',
     'cmd': ['ros2', 'run', 'flylab_vision', 'aruco_node'],
     'start': 'aruco_start', 'stop': 'aruco_stop'},
    {'name': '目标检测', 'start': 'detect_start', 'stop': 'detect_stop'},
    {'name': '目标追踪', 'start': 'track_start', 'stop': 'track_stop'},
]
