#!/usr/bin/env python3
"""flylab_gcs 全局配置 —— 改这里，不改代码。"""

import os

from ament_index_python.packages import get_package_share_directory

# ---------- flylab_vision 参数配置目录 (ArUco 检测/降落等, 改 yaml 即调参) ----------
VISION_CONFIG_DIR = os.path.join(get_package_share_directory('flylab_vision'), 'configs')
ARUCO_PARAMS_FILE = os.path.join(VISION_CONFIG_DIR, 'aruco.yaml')

# ---------- 地面站窗口大小 ----------
# 初始窗口大小 = 屏幕可用区域 × 比例（仅初始值，窗口可拖拽调整）
WINDOW_SIZE_RATIO = (0.6, 0.74)   # (宽, 高)

# ---------- AirSim RPC (相机视图) ----------
AIRSIM_RPC_PORT = 41451   # 与 settings.json 的 ApiServerPort 一致

# ---------- ROS 通信 ----------
# ---------- 显示画面转发 (算法节点的图像输入) ----------
# 当前显示帧发布到此话题，算法节点订阅即处理用户正在看的画面（深度伪彩帧除外）。
# 分辨率保持原图，保证标注坐标对齐。
DISPLAY_IMAGE_TOPIC = '/flylab/display/image_raw'
DISPLAY_IMAGE_HZ = 30
# 转发 QoS: True=RELIABLE 不丢帧但写端可能等 ACK；False=BEST_EFFORT 吞吐满速可能丢帧。
# 实测转发稳定低于源帧率时改 False
DISPLAY_IMAGE_RELIABLE = False
# Scene 图传走 PNG 压缩（线上体积约 1/3，净赚帧率）；False = 始终原始传输
DISPLAY_SCENE_COMPRESS = False

# 算法启停指令：按钮按下/抬起时发布 std_msgs/String
CMD_TOPIC = '/flylab/vision_cmd'

# 模型输出叠加：算法节点每帧发 std_msgs/String(JSON)，界面叠加到画面，schema 见 overlay.py
ANNOTATION_TOPIC = '/flylab/annotations'

# ---------- 相机姿态 (ArUco 降落云台泛化) ----------
# 当前显示相机相对机体的姿态四元数 (x/y/z/w)，相机 NED 体约定相对机体 FRD。
# 地面站 10Hz 发布；aruco_landing_node 据此把图像误差旋到机体系，适配任意云台形态。
# 非 AirSim 源不发布 —— 节点超时后回退"垂直向下 + 图像上边朝机头"刚性假设。
GIMBAL_STATE_TOPIC = '/flylab/gimbal_state'

# ---------- 云台 (AirSim 相机视角旋转, 经 simSetCameraPose) ----------
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

# 视场角同步话题 (std_msgs/Float64, latched): 用户拖动视场滑杆即发布,
# ArUco 等按 FOV 估算内参的算法节点订阅后实时更新; latched 让后启动的
# 节点一接入即拿到当前值。未调整过 (保持 settings.json 原值) 不发布。
CAMERA_FOV_TOPIC = '/flylab/camera_fov'

# ---------- ArUco 降落 (主窗口独立按钮) ----------
# 点击拉起 aruco_landing_node（接管 offboard → 视觉对齐调速下降 → 切 PX4
# AUTO.LAND），再点发 landing_stop 取消。与算法下拉框互不影响。
# 参数来自 flylab_vision/configs/aruco.yaml（--params-file 携带, -p 可临时覆盖）。
# 前提: 通信链路在跑 + 已解锁升空 + 运行 enables_landing 的算法
LANDING_CMD = ['ros2', 'run', 'flylab_vision', 'aruco_landing_node',
               '--ros-args', '--params-file', ARUCO_PARAMS_FILE]
LANDING_START_CMD = 'landing_start'
LANDING_STOP_CMD = 'landing_stop'
# 降落节点状态回传话题 (tracking/landing/landed/idle)，地面站据此复位按钮
LANDING_STATUS_TOPIC = '/flylab/landing_status'
# 算法信息面板数据话题 (std_msgs/String JSON): 各算法节点按 src 分组发布
# 自定义 schema, 右栏"算法信息"面板按运行状态动态显示 (见 algo_output_panel.py)
ARUCO_INFO_TOPIC = '/flylab/aruco_info'

# ---------- ArUco 码边长运行时调节 (右栏"算法信息"面板滑杆) ----------
# 场景里放了多种尺寸的码时, 拖动滑杆运行中同步 aruco_node /
# aruco_landing_node 的 marker_length (影响位姿解算尺度 → 面板的
# "距离/实际宽度"显示; 检测与降落控制不受影响)。latched 话题: 后启动的
# 节点一接入即拿到当前值; 未拖动过不发布 (与视场角同步同一约定),
# 节点用 aruco.yaml 的 marker_length。默认值应与该 yaml 保持一致。
ARUCO_MARKER_LENGTH_TOPIC = '/flylab/aruco_marker_length'
ARUCO_MARKER_LENGTH_MIN = 0.1       # 米 (滑杆下限, 与步进对齐保证档位落在整数)
ARUCO_MARKER_LENGTH_MAX = 2.00      # 米 (滑杆上限)
ARUCO_MARKER_LENGTH_STEP = 0.1      # 米 (滑杆步进, 拖动按此跳档)
ARUCO_MARKER_LENGTH_DEFAULT = 0.8   # 米 (与 aruco.yaml 默认一致)

# ---------- PX4 SITL (地面站一键启停) ----------
# 经 scripts/run-px4.sh 启动，target 与 PX4_SYS_AUTOSTART 严格对应（见 README §6）
PX4_SCRIPT = '/root/flylab/scripts/run-px4.sh'
PX4_MODELS = [
    {'name': '10016_none_iris',       'target': 'none_iris'},
    {'name': '70003_fucker_3120g_tw23', 'target': 'none_fucker_3120g_tw23'},
    {'name': '70004_fucker_2000g_tw15', 'target': 'none_fucker_2000g_tw15'},
]

# ---------- AirSim 导出项目 (PX4 SITL 启动时联动拉起) ----------
# 机型 → 专属 exe 映射（键 = PX4_MODELS 的 name），未命中回退默认；None = 不拉起
AIRSIM_APP_DEFAULT = \
    '/mnt/d/A_Code/MyProject/BeimingSim/BeimingSim_Build/Windows/BeimingSim.exe'
AIRSIM_APP_FOR_MODEL = {
    # '10016_none_iris': '/mnt/d/.../另一个导出项目/Windows/xxx.exe',
}


def airsim_app_for(model_name):
    """机型对应的 AirSim 导出项目 exe (未配置用默认, None=不拉起)。"""
    return AIRSIM_APP_FOR_MODEL.get(model_name, AIRSIM_APP_DEFAULT)

# ---------- 手动遥控 (双摇杆 → offboard 速度, 执行节点 flylab_vision/teleop_node) ----------
# 界面发归一化遥控量 (-1..1, Twist): linear.x=前后 linear.y=左右
# linear.z=升降(升+) angular.z=偏航(左转+)；teleop_node 换算成机体系 FRD 速度
TELEOP_CMD_TOPIC = '/flylab/teleop_cmd'
TELEOP_STATUS_TOPIC = '/flylab/teleop_status'
TELEOP_START_CMD = 'teleop_start'
TELEOP_STOP_CMD = 'teleop_stop'
TELEOP_CMD = ['ros2', 'run', 'flylab_vision', 'teleop_node']

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

# ---------- 视觉算法选择 (下拉框 + 运行/取消按钮) ----------
# name=显示文字, start/stop=启停指令；cmd 填了则自动拉起/回收进程，不填则假定
# 节点已在外部运行。enables_landing=True 是 ArUco 降落的前置条件。
# 新算法遵循 start/stop 指令约定后在这里加一项即可。
ALGORITHMS = [
    {'name': 'ArUco检测',
     'cmd': ['ros2', 'run', 'flylab_vision', 'aruco_node',
             '--ros-args', '--params-file', ARUCO_PARAMS_FILE],
     'start': 'aruco_start', 'stop': 'aruco_stop',
     'enables_landing': True,
     'panel': 'detect'},   # 右栏算法信息面板分组 (algo_output_panel.SECTIONS)
    {'name': '目标检测',
     'cmd': ['ros2', 'run', 'flylab_vision', 'yolo_detector_node',
             '--ros-args', '-p', 'device:=cuda'],
     'start': 'detect_start', 'stop': 'detect_stop'},
    {'name': '图像分类',
     'cmd': ['ros2', 'run', 'flylab_vision', 'yolo_cls_node',
             '--ros-args', '-p', 'device:=cuda'],
     'start': 'cls_start', 'stop': 'cls_stop'},
    {'name': '深度估计',
     'cmd': ['ros2', 'run', 'flylab_vision', 'yolo_depth_node',
             '--ros-args', '-p', 'device:=cuda'],
     'start': 'depth_start', 'stop': 'depth_stop'},
    {'name': '旋转框检测',
     'cmd': ['ros2', 'run', 'flylab_vision', 'yolo_obb_node',
             '--ros-args', '-p', 'device:=cuda'],
     'start': 'obb_start', 'stop': 'obb_stop'},
    {'name': '实例分割',
     'cmd': ['ros2', 'run', 'flylab_vision', 'yolo_segment_node',
             '--ros-args', '-p', 'device:=cuda'],
     'start': 'seg_start', 'stop': 'seg_stop'},
    {'name': '语义分割',
     'cmd': ['ros2', 'run', 'flylab_vision', 'yolo_semantic_node',
             '--ros-args', '-p', 'device:=cuda'],
     'start': 'semantic_start', 'stop': 'semantic_stop'},
    {'name': '姿态估计',
     'cmd': ['ros2', 'run', 'flylab_vision', 'yolo_pose_node',
             '--ros-args', '-p', 'device:=cuda'],
     'start': 'pose_start', 'stop': 'pose_stop'},
    {'name': '目标追踪', 'start': 'track_start', 'stop': 'track_stop'},
]

# ---------- 飞行链路选择 (flylab_flight 一键 launch, 下拉框 + 运行/取消) ----------
# 点选即启动，再点停止，选其他停旧换新。链路输出直通终端。
# 避障/跟踪类链路依赖通信链路先开启（need_link）。
FLIGHT_LINKS = [
    {'name': '感知建图',
     'cmd': ['ros2', 'launch', 'flylab_flight', 'depth_mapping.launch.py']},
    {'name': 'EGO 避障',
     # mavros:=false: MAVROS 由工具栏统一管理, 避免双 MAVROS 抢 14542
     'cmd': ['ros2', 'launch', 'flylab_flight', 'fly_ego.launch.py', 'mavros:=false'],
     'need_link': True},
    {'name': 'SUPER 避障',
     'cmd': ['ros2', 'launch', 'flylab_flight', 'fly_super.launch.py', 'mavros:=false'],
     'need_link': True},
    {'name': 'OpenVINS',
     'cmd': ['ros2', 'launch', 'flylab_flight', 'openvins_msckf.launch.py']},
    {'name': 'VINS 单目',
     'cmd': ['ros2', 'launch', 'flylab_flight', 'vins_mono.launch.py']},
    {'name': 'VINS 双目',
     'cmd': ['ros2', 'launch', 'flylab_flight', 'vins_stereo.launch.py']},
    {'name': 'ORB-SLAM3',
     'cmd': ['ros2', 'launch', 'flylab_flight', 'orb_slam3.launch.py']},
    {'name': 'FAST-LIO',
     'cmd': ['ros2', 'launch', 'flylab_flight', 'fast_lio.launch.py']},
]
