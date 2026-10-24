#!/usr/bin/env python3
"""flylab_gcs 全局配置 —— 改这里，不改代码。"""

# ---------- 地面站窗口大小 ----------
# 初始窗口大小 = 屏幕可用区域 × 比例 (画面随窗口等比缩放;
# 窗口无固定尺寸, 启动后可直接拖拽边缘调整, 此处只影响初始大小)
WINDOW_SIZE_RATIO = (0.4, 0.4)   # (宽, 高) 相对屏幕可用区域的比例

# ---------- AirSim RPC (相机视图) ----------
AIRSIM_RPC_PORT = 41451   # 与 settings.json 的 ApiServerPort 一致

# ---------- ROS 通信 ----------
# ---------- 显示画面转发 (算法节点的图像输入) ----------
# 地面站把"当前显示的帧"发布到该话题 —— 无论是 AirSim 相机、本机摄像头
# 还是视频文件源, 算法节点订阅它即可处理用户正在看的画面 (深度伪彩帧除外)。
# 分辨率保持原图, 保证标注叠加坐标对齐。
DISPLAY_IMAGE_TOPIC = '/flylab/display/image_raw'
DISPLAY_IMAGE_HZ = 30
# 大图转发 QoS: True=RELIABLE (不丢帧, 但写端可能阻塞等 ACK, 吞吐上限
# 受 ACK 往返限制); False=BEST_EFFORT (无 ACK 等待、吞吐满速, 大图 UDP
# 分片下可能丢帧)。转发已在独立线程, RELIABLE 阻塞不再拖累界面; 若实测
# 转发端稳定低于源帧率 (ros2 topic hz /flylab/display/image_raw), 改 False
DISPLAY_IMAGE_RELIABLE = False

# 算法启停指令话题：按钮按下/抬起时发布 std_msgs/String，算法节点自行订阅执行
CMD_TOPIC = '/flylab/vision_cmd'

# 模型输出叠加话题：模型节点每帧往这里发 std_msgs/String(JSON 标注列表)，
# 界面自动叠加到画面上。schema 见 overlay.py 文档。
ANNOTATION_TOPIC = '/flylab/annotations'

# ---------- 相机姿态 (ArUco 降落云台泛化) ----------
# 当前显示相机相对机体的姿态四元数 (geometry_msgs/Quaternion, x/y/z/w),
# 语义: 相机 NED 体约定 (x 视轴, y 图右, z 图下) 相对机体 FRD 的旋转。
# 地面站以 10Hz 发布; aruco_landing_node 据此把图像误差方向旋到机体系,
# 适配任意云台形态 (刚性安装 / 世界系自稳 / yaw 随动, 俯仰不限于垂直向下)。
# 非 AirSim 源 (本机摄像头/视频文件) 不发布 —— 节点超时后自动回退
# "垂直向下 + 图像上边朝机头" 刚性假设 (flip 参数)。
GIMBAL_STATE_TOPIC = '/flylab/gimbal_state'

# ---------- 云台 (AirSim 相机视角旋转, 经 simSetCameraPose) ----------
GIMBAL_STEP_DEG = 1.0     # 每次按键步进角度 (按钮支持按住连发)

# 飞行中姿态跟随频率 (Hz)。相机是刚性挂载, 云台角只在下发瞬间按当时机体
# 姿态换算; 飞行中机体姿态一变, 记忆的云台角就与实际视角脱钩, 下次按键会
# 把积累的姿态差一次性归零 —— 表现为方向键不再纯上下/左右。除按键立即下发
# 外, 以该频率按当前机体姿态重下发保持基准新鲜; 0 = 关闭 (仅地面静态调试)
GIMBAL_HOLD_HZ = 10.0

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

# ---------- ArUco 降落 (主窗口独立按钮) ----------
# 完整降落链路: 点击按钮拉起 aruco_landing_node (三态状态机:
# 接管 offboard → 视觉对齐调速下降 → 满足条件切 PX4 AUTO.LAND),
# 再次点击发 landing_stop 取消并交还控制权。
# 与算法下拉框互不影响: ArUco检测可同时运行 (两路独立检测互不干扰)。
# 使用前提: MicroXRCEAgent 在跑 + 飞机已解锁升空 + 画面为垂直向下视角看到码。
LANDING_CMD = ['ros2', 'run', 'flylab_vision', 'aruco_landing_node']
LANDING_START_CMD = 'landing_start'
LANDING_STOP_CMD = 'landing_stop'
# 降落节点的状态回传话题 (tracking/landing/landed/idle),
# 地面站据此复位按钮并给成功/中止反馈
LANDING_STATUS_TOPIC = '/flylab/landing_status'

# ---------- 通信链路 (地面站一键拉起, 省开终端) ----------
# uXRCE-DDS: PX4 ↔ ROS2 直连桥, UDP 8888。供 tracker:=uxrce / aruco_landing
# (link:=uxrce) 链路使用
UXRCE_CMD = ['MicroXRCEAgent', 'udp4', '-p', '8888']
# MAVROS: 经典 MAVLink 桥, 用 14542 专属实例 (14580 会被 AirSim 抢占, 详见 README)
MAVROS_CMD = ['ros2', 'run', 'mavros', 'mavros_node', '--ros-args',
              '-p', 'fcu_url:=udp://:14542@', '-p', 'tgt_system:=1',
              '-p', 'tgt_component:=1']

# ---------- 算法选择 (下拉框 + 运行/取消按钮) ----------
# name=下拉框显示文字, start/stop=运行/取消时发布的指令。
# cmd=算法节点启动命令 (可选): 填了则地面站点"运行"时自动拉起进程、点"取消"
# 时自动回收; 不填则假定节点已在外部运行, 只发启停指令。
# enables_landing=True 的算法是 ArUco 降落的前置条件 (该算法运行中且云台
# 转到垂直向下, 主窗口的 "ArUco降落" 按钮才可点)。
# 界面交互对所有算法统一: 新算法只要遵循"收到 start 指令开始、收到 stop
# 指令停止"的约定, 在这里加一项即可出现在下拉框里。
ALGORITHMS = [
    {'name': 'ArUco检测',
     'cmd': ['ros2', 'run', 'flylab_vision', 'aruco_node'],
     'start': 'aruco_start', 'stop': 'aruco_stop',
     'enables_landing': True},
    {'name': '目标检测',
     'cmd': ['ros2', 'run', 'flylab_vision', 'yolo_detector_node',
             '--ros-args', '-p', 'device:=cuda'],   # 无 CUDA 环境会自动回退 CPU
     'start': 'detect_start', 'stop': 'detect_stop'},
    {'name': '图像分类',
     'cmd': ['ros2', 'run', 'flylab_vision', 'yolo_cls_node',
             '--ros-args', '-p', 'device:=cuda'],   # 无 CUDA 环境会自动回退 CPU
     'start': 'cls_start', 'stop': 'cls_stop'},
    {'name': '深度估计',
     'cmd': ['ros2', 'run', 'flylab_vision', 'yolo_depth_node',
             '--ros-args', '-p', 'device:=cuda'],   # 无 CUDA 环境会自动回退 CPU
     'start': 'depth_start', 'stop': 'depth_stop'},
    {'name': '旋转框检测',
     'cmd': ['ros2', 'run', 'flylab_vision', 'yolo_obb_node',
             '--ros-args', '-p', 'device:=cuda'],   # 无 CUDA 环境会自动回退 CPU
     'start': 'obb_start', 'stop': 'obb_stop'},
    {'name': '实例分割',
     'cmd': ['ros2', 'run', 'flylab_vision', 'yolo_segment_node',
             '--ros-args', '-p', 'device:=cuda'],   # 无 CUDA 环境会自动回退 CPU
     'start': 'seg_start', 'stop': 'seg_stop'},
    {'name': '语义分割',
     'cmd': ['ros2', 'run', 'flylab_vision', 'yolo_semantic_node',
             '--ros-args', '-p', 'device:=cuda'],   # 无 CUDA 环境会自动回退 CPU
     'start': 'semantic_start', 'stop': 'semantic_stop'},
    {'name': '姿态估计',
     'cmd': ['ros2', 'run', 'flylab_vision', 'yolo_pose_node',
             '--ros-args', '-p', 'device:=cuda'],   # 无 CUDA 环境会自动回退 CPU
     'start': 'pose_start', 'stop': 'pose_stop'},
    {'name': '目标追踪', 'start': 'track_start', 'stop': 'track_stop'},
]
