#!/usr/bin/env python3
"""算法信息面板的调节控件值域与枚举注册表 (latched 同步参数的界面侧
取值, 话题见 topics.py, 控件生成见 latched_sync.LATCHED_PARAMS)。
默认值应与 flylab_vision/configs/aruco.yaml 或节点 ROS 参数默认保持一致。"""

# ---------- ArUco 码边长 (滑杆) ----------
ARUCO_MARKER_LENGTH_MIN = 0.1       # 米 (滑杆下限, 与步进对齐保证档位落在整数)
ARUCO_MARKER_LENGTH_MAX = 2.00      # 米 (滑杆上限)
ARUCO_MARKER_LENGTH_STEP = 0.05     # 米 (滑杆步进, 拖动按此跳档)
ARUCO_MARKER_LENGTH_DEFAULT = 0.8   # 米 (与 aruco.yaml 默认一致)

# ---------- ArUco 字典 (下拉框) ----------
# 可选名字 = OpenCV 预定义字典 (与 flylab_vision ArucoDetector 的
# dict_table 一致, 未知名字节点拒绝并保持原字典)。
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

# ---------- fractal 分形配置 (下拉框) ----------
FRACTAL_CONFIGS = [
    'FRACTAL_2L_6', 'FRACTAL_3L_6', 'FRACTAL_4L_6', 'FRACTAL_5L_6',
]
FRACTAL_CONFIG_DEFAULT = 'FRACTAL_5L_6'   # 与 aruco.yaml 默认一致

# ---------- 人脸下游任务 (下拉框) ----------
# 键 = 发给节点的任务名 (节点 facex_task_id 表), 值 = 界面显示名。
# 'none' = 只做人脸检测不跑 FaceXFormer (节点的默认状态)。
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

# ---------- 人脸任务张数 (下拉框) ----------
# FaceXFormer 批量跑置信度前 N 张脸, 0 = 全部检出脸 (模型动态 batch,
# 脸多时拼批摊薄推理开销); 键 = 发给节点的数值字符串, 值 = 显示名。
# 默认 1 与 face_detect_node 的 facex_topk 一致。
FACE_TOPKS = {
    '1': '前 1 张',
    '2': '前 2 张',
    '3': '前 3 张',
    '5': '前 5 张',
    '0': '全部',
}
FACE_TOPK_DEFAULT = '1'

# ---------- 降落航向偏置 (数字框) ----------
# aruco_landing_node 机头相对码 0° 方向 (OpenCV 角点 0→1, 码图案 x 轴) 的
# 落点朝向偏置 (度, 右正)。运行中修改会解锁航向、按新朝向重新对齐。
LANDING_YAW_OFFSET_MIN = 0         # 度 (下限; 左转用大值表示, 如左 90° = 270)
LANDING_YAW_OFFSET_MAX = 360       # 度 (上限)
LANDING_YAW_OFFSET_STEP = 5        # 度 (上下箭头单步)
LANDING_YAW_OFFSET_DEFAULT = 0     # 度 (初始显示值, 与 aruco.yaml 的 yaw_offset_deg 一致)

# ---------- 目标追踪俯视角度 (数字框) ----------
TRACK_PITCH_MIN = 5        # 度 (下限)
TRACK_PITCH_MAX = 85       # 度 (上限; 云台俯仰限位 ±90 内)
TRACK_PITCH_STEP = 1       # 度 (上下箭头单步)
TRACK_PITCH_DEFAULT = 30   # 度 (初始显示值)

# ---------- 目标追踪跟随距离 (数字框, 机体跟随-水平) ----------
TRACK_DIST_MIN = 3.0       # 米 (下限; 太近反控风险高)
TRACK_DIST_MAX = 100.0     # 米 (上限)
TRACK_DIST_STEP = 0.1      # 米 (上下箭头单步)
TRACK_DIST_DEFAULT = 10.0  # 米 (初始显示值, 与 track_takeover 参数默认一致)

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
