#!/usr/bin/env python3
"""flylab_gcs 全局配置 —— 改这里，不改代码。"""

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

# ---------- ROS 通信 ----------
# ---------- 显示画面转发 (算法节点的图像输入) ----------
# 当前显示帧发布到此话题，算法节点订阅即处理用户正在看的画面（深度伪彩帧除外）。
# 分辨率保持原图，保证标注坐标对齐。
DISPLAY_IMAGE_TOPIC = '/flylab/display/image_raw'
# 转发 QoS: True=RELIABLE 不丢帧但写端可能等 ACK；False=BEST_EFFORT 吞吐满速可能丢帧。
# 实测转发稳定低于源帧率时改 False
DISPLAY_IMAGE_RELIABLE = False
# Scene 图传是否走 PNG 压缩（线上体积约 1/3）。实测关闭：UE 服务端的
# PNG 编码是单线程且比省下的传输更贵（640x480 下 RPC 反而多 ~30ms），
# 640x480 原始传输 ~900KB 对 WSL 链路无压力
DISPLAY_SCENE_COMPRESS = False

# ---------- 深度估计悬停读数 ----------
# yolo_depth_node 把降采样深度图 (16UC1, 毫米) 发到此话题, 界面订阅后
# 鼠标悬停画面即显示该处深度气泡 (数据随算法启停自然出现/过期)
DEPTH_MAP_TOPIC = '/flylab/depth_map'
DEPTH_MAP_FRESH_SEC = 0.5   # 超过此间隔无新深度图视为节点已停, 隐藏读数

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
GIMBAL_FOLLOW_YAW_RATE_DEG_S = 45.0    # 偏航角速率上限 (须 = teleop yaw_rate_deg)
GIMBAL_FOLLOW_FRESH_SEC = 0.5          # 航向测量新鲜度上限 (过期按误差 0 停转)
GIMBAL_FOLLOW_USER_YAW_DEADBAND = 0.05 # 遥控偏航杆死区 (归一化): 超过视为用户在转机头
GIMBAL_FOLLOW_USER_HOLD_SEC = 0.3      # 用户转机头判定窗口: 锚点随实测航向平移 (相机偏航临时刚性)

# ---------- 航向跟随的算法通道 (视觉反控算法控制云台 + 机头跟随) ----------
# offboard 单写者原则: 反控节点独占 setpoint 流时, 跟随指令必须由算法
# 节点自己注入 (sendVelocityFRDYawRate), GCS 不再经 teleop 发偏航量。
# 地面站只提供师资与云台控制权, 三个话题 (aruco_landing 已接入):
#   gimbal_cmd    算法 → 云台 [yaw_rate, pitch_rate] (°/s), 兼作 follow 心跳
#   gimbal_follow 算法请求相机侧反卷 (世界锁定 + 误差收敛回中), std_msgs/Bool
#   gimbal_yaw_err 地面站广播反卷后的偏航误差 (度, 右+; NaN=未跟随),
#                  latched —— 算法订阅后 yaw_rate = KP × err 即完成接入
GIMBAL_CMD_TOPIC = '/flylab/gimbal_cmd'
GIMBAL_FOLLOW_REQ_TOPIC = '/flylab/gimbal_follow'
GIMBAL_YAW_ERR_TOPIC = '/flylab/gimbal_yaw_err'
GIMBAL_FOLLOW_ALGO_TIMEOUT_SEC = 1.0   # 算法 follow 心跳超时 (gimbal_cmd 断流自动退)

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
# 自定义 schema, 右栏"算法信息"面板按运行状态动态显示 (见 panel_sections.py)
ALGO_INFO_TOPIC = '/flylab/algo_info'

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

# ---------- ArUco 码边长运行时调节 (右栏"算法信息"面板滑杆) ----------
# 场景里放了多种尺寸的码时, 拖动滑杆运行中同步 aruco_node /
# aruco_landing_node 的 marker_length (影响位姿解算尺度 → 面板的
# "距离/实际宽度"显示; 检测与降落控制不受影响)。latched 话题: 后启动的
# 节点一接入即拿到当前值; 未拖动过不发布 (与视场角同步同一约定),
# 节点用 aruco.yaml 的 marker_length。默认值应与该 yaml 保持一致。
ARUCO_MARKER_LENGTH_TOPIC = '/flylab/aruco_marker_length'
ARUCO_MARKER_LENGTH_MIN = 0.1       # 米 (滑杆下限, 与步进对齐保证档位落在整数)
ARUCO_MARKER_LENGTH_MAX = 2.00      # 米 (滑杆上限)
ARUCO_MARKER_LENGTH_STEP = 0.05     # 米 (滑杆步进, 拖动按此跳档)
ARUCO_MARKER_LENGTH_DEFAULT = 0.8   # 米 (与 aruco.yaml 默认一致)

# ---------- ArUco 字典运行时切换 (右栏"算法信息"面板下拉框) ----------
# 场景换了码的字典 (如 4x4 换 6x6) 时, 下拉选择运行中同步 aruco_node /
# aruco_landing_node 的 dict_name (下一帧检测生效)。latched 话题: 后启动的
# 节点一接入即拿到当前值; 未选择过不发布 (与码边长同步同一约定), 节点用
# aruco.yaml 的 dict_name。默认值应与该 yaml 保持一致。
# 可选名字 = OpenCV 预定义字典 (与 flylab_vision ArucoDetector 的
# dict_table 一致, 未知名字节点拒绝并保持原字典)。
ARUCO_DICT_TOPIC = '/flylab/aruco_dict'
ARUCO_DICTS = [
    'DICT_4X4_50', 'DICT_4X4_100', 'DICT_4X4_250', 'DICT_4X4_1000',
    'DICT_5X5_50', 'DICT_5X5_100', 'DICT_5X5_250', 'DICT_5X5_1000',
    'DICT_6X6_50', 'DICT_6X6_100', 'DICT_6X6_250', 'DICT_6X6_1000',
    'DICT_7X7_50', 'DICT_7X7_100', 'DICT_7X7_250', 'DICT_7X7_1000',
    'DICT_ARUCO_ORIGINAL',
    'DICT_APRILTAG_16h5', 'DICT_APRILTAG_25h9',
    'DICT_APRILTAG_36h10', 'DICT_APRILTAG_36h11',
]
ARUCO_DICT_DEFAULT = 'DICT_4X4_1000'   # 与 aruco.yaml 默认一致

# ---------- fractal 分形配置运行时切换 (右栏"算法信息"面板下拉框) ----------
# 分形码 = 大码内嵌套逐层缩小的小码 (UCO aruco 3.1.12), 下拉运行中同步
# fractal_node 的 fractal_config (重建检测器, 下一帧检测生效)。latched 话题:
# 后启动的节点一接入即拿到当前值; 未选择过不发布 (与 ArUco 字典同步同一
# 约定), 节点用 aruco.yaml 的 fractal_config。默认值应与该 yaml 保持一致。
FRACTAL_CONFIG_TOPIC = '/flylab/fractal_config'
FRACTAL_CONFIGS = [
    'FRACTAL_2L_6', 'FRACTAL_3L_6', 'FRACTAL_4L_6', 'FRACTAL_5L_6',
]
FRACTAL_CONFIG_DEFAULT = 'FRACTAL_5L_6'   # 与 aruco.yaml 默认一致

# ---------- 人脸下游任务切换 (右栏"算法信息"面板下拉框) ----------
# SCRFD 检出人脸后, FaceXFormer 对置信度前 N 张脸 (张数经 FACE_TOPK_TOPIC
# 同步, 0 = 全部) 拼批跑选定任务; 下拉切换经该话题同步 face_detect_node
# (下一帧生效), 未知任务名节点拒绝并保持原任务。键 = 发给节点的任务名
# (节点 facex_task_id 表), 值 = 界面显示名。
# 'none' = 只做人脸检测不跑 FaceXFormer (节点的默认状态)。
FACE_TASK_TOPIC = '/flylab/face_task'
FACE_TASKS = {
    'none': '无任务 (仅检测)',
    'parsing': '人像分割',
    'landmarks': '68 关键点',
    'headpose': '头部姿态',
    'attributes': '人脸属性',
    'age_gender_race': '年龄性别',
    'visibility': '点可见性',
}
FACE_TASK_DEFAULT = 'none'   # 与 face_detect_node 的 facex_task 默认一致

# 人脸任务张数: FaceXFormer 批量跑置信度前 N 张脸, 0 = 全部检出脸
# (模型动态 batch, 脸多时拼批摊薄推理开销); 键 = 发给节点的数值字符串,
# 值 = 显示名。默认 1 与 face_detect_node 的 facex_topk 一致。
FACE_TOPK_TOPIC = '/flylab/face_topk'
FACE_TOPKS = {
    '1': '前 1 张',
    '2': '前 2 张',
    '3': '前 3 张',
    '5': '前 5 张',
    '0': '全部',
}
FACE_TOPK_DEFAULT = '1'

# ---------- 降落航向偏置同步 ----------
# aruco_landing_node 机头相对码 0° 方向 (OpenCV 角点 0→1, 码图案 x 轴) 的
# 落点朝向偏置 (度, 右正)。latched 话题: 后启动的节点一接入即拿到当前值;
# 未动过不发布 (与码边长同步同一约定), 节点用 aruco.yaml 的 yaw_offset_deg。
# 运行中修改会解锁航向、按新朝向重新对齐。
LANDING_YAW_OFFSET_TOPIC = '/flylab/landing_yaw_offset'
LANDING_YAW_OFFSET_MIN = 0         # 度 (下限; 左转用大值表示, 如左 90° = 270)
LANDING_YAW_OFFSET_MAX = 360       # 度 (上限)
LANDING_YAW_OFFSET_STEP = 5        # 度 (上下箭头单步)
LANDING_YAW_OFFSET_DEFAULT = 0     # 度 (初始显示值, 与 aruco.yaml 的 yaw_offset_deg 一致)

# ---------- 目标追踪 (画面框选即追踪) ----------
# 流程: 菜单启动追踪节点 (yolo_track/ortrack) → 画面拖出矩形松开即发 ROI
# (latched) + track_begin 自动锁定 (yolo: 框选区检测裁块出 ReID 外观向量,
# 逐帧余弦相似度续跟; ortrack: 直接裁 128 模板逐帧匹配), 追踪中重新框选
# 即换目标; 停止靠工具栏"停止算法" (无独立取消指令)。ROI 进入追踪即消费。
TRACK_ROI_TOPIC = '/flylab/track_roi'
TRACK_BEGIN_CMD = 'track_begin'
# 目标追踪反控 (接管控制菜单 → 追踪节点, 与 ArUco 降落反控同一机制):
# 追踪节点接管 offboard 拿到飞机控制权 (悬停跟拍 + 机头跟随), 云台伺服
# 让目标保持在画面中央; 再点菜单项或停止算法即交还控制权
TRACK_TAKEOVER_START_CMD = 'track_takeover_start'
TRACK_TAKEOVER_STOP_CMD = 'track_takeover_stop'
# 目标追踪反控模式 (菜单三态; 随 track_takeover_start 以模式后缀下发
# 节点, 如 "track_takeover_start dive", 同 landing_mode 的传参约定)。
# 悬停跟拍/曲线/直线打击为现役功能, 机体跟随为预留接口 (节点侧行为待实现)
TRACK_TAKEOVER_GIMBAL = 'gimbal'          # 悬停跟拍 (现役)
TRACK_TAKEOVER_DIVE = 'curve_dive'        # 曲线打击: 两相制导定速撞击 (现役)
TRACK_TAKEOVER_STRAIGHT = 'straight_dive'  # 直线打击: 全程沿视线不分相 (现役)
TRACK_TAKEOVER_BODYFOLLOW = 'bodyfollow'  # 机体跟随: 预留
# 反控节点状态回传 (engaged=接管成功 / idle=已交还 / failed=接管失败),
# 地面站据此联动菜单项 (同降落/遥控状态回传模式)
TRACK_STATUS_TOPIC = '/flylab/track_status'
# 目标追踪反控机头跟随开关 (云台面板"跟随"按钮在反控中复用为该开关):
# latched Bool 发给追踪节点, True=机头跟随视线方向 (默认) / False=机头
# 保持航向, 仅云台伺服转 (目标仍居中, 云台偏航 ±45° 限位内)
TRACK_FOLLOW_TOPIC = '/flylab/track_follow'
# 目标追踪俯视角度同步 (算法信息"俯视角度"输入框): latched Float64 发给
# 追踪节点 (度, 向下为正)。反控接管中设置后, 节点高度环控制飞机升降,
# 使云台以该俯视角注视目标 (云台伺服保持目标居中, 飞机升降直到俯角
# 收敛); 未动过不发布 = 不启用高度环 (与朝向偏置同步同一约定)
TRACK_PITCH_TOPIC = '/flylab/track_pitch'
TRACK_PITCH_MIN = 5        # 度 (下限)
TRACK_PITCH_MAX = 85       # 度 (上限; 云台俯仰限位 ±90 内)
TRACK_PITCH_STEP = 1       # 度 (上下箭头单步)
TRACK_PITCH_DEFAULT = 30   # 度 (初始显示值)

# ---------- PX4 SITL (地面站一键启停) ----------
# 经 scripts/run-px4.sh 启动，target 与 PX4_SYS_AUTOSTART 严格对应（见 README §6）
PX4_SCRIPT = '/root/flylab/scripts/run-px4.sh'
PX4_MODELS = [
    {'name': '10016_none_iris',       'target': 'none_iris'},
    {'name': '70003_fucker_coax_x050',  'target': 'none_fucker_coax_x050'},
    {'name': '70004_fucker_coax_x100',  'target': 'none_fucker_coax_x100'},
    {'name': '70005_fucker_coax_x300',  'target': 'none_fucker_coax_x300'},
    {'name': '70006_fucker_x450',       'target': 'none_fucker_x450'},
]

# ---------- ULG 飞行日志分析 (工具栏"日志分析"独立窗口, 见 ulg_window.py) ----------
# PX4 SITL 日志根目录 (root/日期/时刻.ulg), run-px4.sh 默认 build rootfs 下;
# 实机 SD 卡日志用窗口内"打开文件..."手动选
ULG_LOG_ROOT = '/root/flylab/px4.1.15.2/build/px4_sitl_default/rootfs/log'

# ---------- 手动遥控 (双摇杆 → offboard 速度, 执行节点 flylab_vision/teleop_node) ----------
# 界面发归一化遥控量 (-1..1, Twist): linear.x=前后 linear.y=左右
# linear.z=升降(升+) angular.z=偏航(左转+)；teleop_node 换算成机体系 FRD 速度
TELEOP_CMD_TOPIC = '/flylab/teleop_cmd'
TELEOP_STATUS_TOPIC = '/flylab/teleop_status'
TELEOP_START_CMD = 'teleop_start'
TELEOP_STOP_CMD = 'teleop_stop'
TELEOP_CMD = ['ros2', 'run', 'flylab_vision', 'teleop_node',
              '--ros-args',
              '-p', 'v_xy:=10.0',          # 平移速度上限 m/s (默认5.0; 飞控按 MPC_XY_VEL_MAX 二次夹紧)
              '-p', 'v_z:=3.0',            # 升降速度上限 m/s (默认2.0; 飞控按 MPC_Z_VEL_MAX_UP/DN 二次夹紧)
              '-p', 'yaw_rate_deg:=45.0']  # 须 = GIMBAL_FOLLOW_YAW_RATE_DEG_S

# ---------- 游戏手柄 (双摇杆驱动界面虚拟摇杆, 走同一条遥控下发链路) ----------
# pygame/SDL 读取; 左手柄杆=高度/偏航, 右手柄杆=平移, 与界面双摇杆同布局。
# 量只注入画面 (VirtualJoystick.set_external → changed), 下发路径与鼠标
# 拖动完全一致, 界面上能看到操控力度。WSL 下手柄需 usbipd 直通进系统:
#   usbipd attach --busid <busid> --wsl <发行版>
GAMEPAD_POLL_HZ = 50        # 读取频率 (Hz)
GAMEPAD_DEADZONE = 0.08     # 死区: 内视为回中, 外线性重标定到全行程
# 轴号按 Linux xpad 驱动原始轴序 (SDL 不做重映射): 左摇杆 0/1, 左扳机 2,
# 右摇杆 3/4, 右扳机 5 —— 注意与 SDL Xbox 标准布局不同, 扳机平时恒 -1!
# 其他手柄用 `python3 /root/flylab/scripts/test-gamepad.py` 看轴号后在此覆盖
GAMEPAD_AXES = {'lx': 0, 'ly': 1, 'rx': 3, 'ry': 4}
# 摇杆物理上推是负轴值, 统一翻转成 VirtualJoystick 约定 (前/右为正)
GAMEPAD_INVERT = {'lx': 1.0, 'ly': -1.0, 'rx': 1.0, 'ry': -1.0}

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

# ---------- 视觉算法选择 (下拉菜单: 一级=任务分组, 二级=具体算法/模型) ----------
# 结构: [{'name': 分组名, 'items': [算法条目...]}, ...] —— 同一任务类别下
# 挂多个模型/算法变体 (不同权重、不同节点实现), 菜单按分组展开子菜单。
# 算法条目字段 (原有一级条目语义不变):
#   name            显示文字 (运行时按钮 "视觉算法: <name>" 同步)
#   cmd             启动命令 (填了自动拉起/回收进程, 不填假定节点已在外部运行)
#   start/stop      启停指令 (std_msgs/String)
#   enables_landing True = ArUco 降落 (视觉反控) 的前置条件
#   landing_modes   反控模式子菜单 [(mode, 显示名)...]
#   panel           右栏算法信息面板分组 (panel_sections.SECTIONS)
# 加模型变体: yolo_*_node 支持 -p model:=<权重路径> (默认取 share 下的
# yolo26s* 系列), 复制一个条目改 name/model 即可, 例如:
#   {'name': '自训检测模型',
#    'cmd': ['ros2', 'run', 'flylab_vision', 'yolo_detector_node',
#            '--ros-args', '-p', 'device:=cuda',
#            '-p', 'model:=<你的 .onnx 路径>'],
#    'start': 'detect_start', 'stop': 'detect_stop'}
# 新算法遵循 start/stop 指令约定后加一个条目即可; 无 'items' 键的旧式
# 平铺条目也兼容, 直接显示为一级菜单项。
ALGORITHMS = [
    {'name': '基础码检测', 'items': [
        {'name': 'ArUco检测',
         'cmd': ['ros2', 'run', 'flylab_vision', 'aruco_node',
                 '--ros-args', '--params-file', ARUCO_PARAMS_FILE],
         'start': 'aruco_start', 'stop': 'aruco_stop',
         'enables_landing': True,
         # 反控模式子项: 声明了多个 landing_modes 的算法在"接管控制→视觉反控"
         # 子菜单显示模式子项 (ArUco: 图像空间/3D 姿态); 未声明的反控算法
         # 不显示模式子项 (enables_takeover 算法显示追踪三态, 其余隐藏子菜单)
         'landing_modes': [('image', '图像空间'), ('pose3d', '3D 姿态')],
         'panel': 'aruco'},   # 右栏算法信息面板分组 (panel_sections.SECTIONS)
        {'name': 'fractal检测',
         'cmd': ['ros2', 'run', 'flylab_vision', 'fractal_node',
                 '--ros-args', '--params-file', ARUCO_PARAMS_FILE],
         'start': 'fractal_start', 'stop': 'fractal_stop',
         'panel': 'fractal',
         'enables_landing': True,
         'landing_modes': [('image', '图像空间'), ('pose3d', '3D 姿态')],
         # 降落节点检测器类型: 启动反控时经 -p detector:= 传给
         # aruco_landing_node (aruco 条目不声明, 默认 aruco)
         'landing_detector': 'fractal'},
    ]},
    {'name': '目标检测', 'items': [
        {'name': 'yolo26s',
         'cmd': ['ros2', 'run', 'flylab_vision', 'yolo_detector_node',
                 '--ros-args', '-p', 'device:=cuda'],
         # device 须用 cuda: CPU 推理 236~250ms/帧 (attention 结构跑不动),
         # cuda 17~25ms; 整链输出 ≈ 抓取帧率, 瓶颈在抓取侧而非推理侧
         'start': 'detect_start', 'stop': 'detect_stop',
         'panel': 'detect'},   # 右栏算法信息面板: 检出数 + 前 3 目标摘要 (SECTIONS['detect'])
        # 变体在此往下加 (见上方"加模型变体"示例)
    ]},
    {'name': '图像分类', 'items': [
        {'name': 'yolo26s-cls',
         'cmd': ['ros2', 'run', 'flylab_vision', 'yolo_cls_node',
                 '--ros-args', '-p', 'device:=cuda'],
         'start': 'cls_start', 'stop': 'cls_stop',
         'panel': 'cls'},   # 右栏算法信息面板: top-3 类别摘要 (SECTIONS['cls'])
    ]},
    {'name': '深度估计', 'items': [
        {'name': 'yolo26s-depth',
         'cmd': ['ros2', 'run', 'flylab_vision', 'yolo_depth_node',
                 '--ros-args', '-p', 'device:=cuda'],
         'start': 'depth_start', 'stop': 'depth_stop',
         'panel': 'depth'},   # 右栏算法信息面板: 伪彩深度图 + 摘要 (SECTIONS['depth'])
    ]},
    {'name': '旋转框检测', 'items': [
        {'name': 'yolo26s-obb',
         'cmd': ['ros2', 'run', 'flylab_vision', 'yolo_obb_node',
                 '--ros-args', '-p', 'device:=cuda'],
         'start': 'obb_start', 'stop': 'obb_stop',
         'panel': 'obb'},   # 右栏算法信息面板: 检出数 + 前 3 旋转框摘要 (SECTIONS['obb'])
    ]},
    {'name': '实例分割', 'items': [
        {'name': 'yolo26s-seg',
         'cmd': ['ros2', 'run', 'flylab_vision', 'yolo_segment_node',
                 '--ros-args', '-p', 'device:=cuda'],
         'start': 'seg_start', 'stop': 'seg_stop',
         'panel': 'seg'},   # 右栏算法信息面板: 实例数 + 主目标摘要 (SECTIONS['seg'])
    ]},
    {'name': '语义分割', 'items': [
        {'name': 'yolo26s-sem',
         'cmd': ['ros2', 'run', 'flylab_vision', 'yolo_semantic_node',
                 '--ros-args', '-p', 'device:=cuda'],
         'start': 'semantic_start', 'stop': 'semantic_stop',
         'panel': 'semantic'},   # 右栏算法信息面板: 配色类别图 + 图例 (SECTIONS['semantic'])
    ]},
    {'name': '姿态估计', 'items': [
        {'name': 'yolo26s-pose',
         'cmd': ['ros2', 'run', 'flylab_vision', 'yolo_pose_node',
                 '--ros-args', '-p', 'device:=cuda'],
         'start': 'pose_start', 'stop': 'pose_stop',
         'panel': 'pose'},   # 右栏算法信息面板: 人数 + 主目标摘要 (SECTIONS['pose'])
    ]},
    {'name': '手部姿态', 'items': [
        # WiLoR (CVPR25): 3D 手部定位+重建, 两级 ONNX 管线。模型在包内
        # models/hand/ (detector.onnx + wilor_fp16.onnx 单文件), 换模型用
        # -p detector_model/wilor_model 覆盖
        {'name': 'WiLoR',
         'cmd': ['ros2', 'run', 'flylab_vision', 'wilor_node',
                 '--ros-args', '-p', 'device:=cuda'],
         'start': 'wilor_start', 'stop': 'wilor_stop',
         'panel': 'wilor'},   # 右栏算法信息面板: 手数 + 左右手摘要 (SECTIONS['wilor'])
    ]},
    {'name': '人脸检测', 'items': [
        # SCRFD-10G 人脸检测 (insightface, facexformer 前置检测器同款):
        # 框 + 5 关键点标注, 模型在包内 models/face/, 换模型用 -p model:=覆盖
        {'name': 'SCRFD',
         'cmd': ['ros2', 'run', 'flylab_vision', 'face_detect_node',
                 '--ros-args', '-p', 'device:=cuda'],
         'start': 'face_start', 'stop': 'face_stop',
         'panel': 'face'},   # 右栏算法信息面板: 人脸数 + 前 3 张脸摘要 (SECTIONS['face'])
    ]},
    {'name': '目标追踪', 'items': [
        {'name': 'yolo26s-track',
         'cmd': ['ros2', 'run', 'flylab_vision', 'yolo_track_node',
                 '--ros-args', '-p', 'device:=cuda'],
         'start': 'track_start', 'stop': 'track_stop',
         # enables_takeover: 支持目标追踪反控 (接管控制→视觉反控子菜单的
         # 追踪三态: 悬停跟拍/俯冲打击/机体跟随, 接管 offboard 悬停跟拍
         # + 云台伺服保持目标居中)
         'enables_takeover': True,
         'panel': 'track'},   # 右栏面板: 追踪状态卡片 (框选在画面完成)
        # ORTrack (CVPR 2025): 模板匹配式单目标追踪, 无需检测候选,
        # 框选即锁定; 与 yolo26s-track 共用右栏追踪面板与框选/开始按钮
        {'name': 'ORTrack',
         'cmd': ['ros2', 'run', 'flylab_vision', 'ortrack_node',
                 '--ros-args', '-p', 'device:=cuda'],
         'start': 'track_start', 'stop': 'track_stop',
         'enables_takeover': True,
         'panel': 'track'},
    ]},
]


def iter_algorithms():
    """展平算法注册表 (分组 -> 算法条目), 供按 name 查找/遍历。
    兼容旧式平铺条目: 无 'items' 键的项视为算法条目本身。"""
    for group in ALGORITHMS:
        if 'items' in group:
            yield from group['items']
        else:
            yield group

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
