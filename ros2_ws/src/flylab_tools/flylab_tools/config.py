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
GIMBAL_STEP_DEG = 5.0     # 每次按键步进角度 (按钮支持按住连发)

# ---------- 工具按钮 (可切换: 按下=启动, 再按=停止) ----------
# name=按钮文字, start/stop=对应发布的指令
TOOLS = [
    {'name': '目标检测', 'start': 'detect_start', 'stop': 'detect_stop'},
    {'name': '目标追踪', 'start': 'track_start', 'stop': 'track_stop'},
]

# 相机画面是否启用拖拽框选 (发给 BBOX_TOPIC)
SELECT_TARGET = True
