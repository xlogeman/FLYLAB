#!/usr/bin/env python3
"""算法/飞行链路/机型注册表 —— 新增算法、链路、机型变体改这里。"""

from .paths import ARUCO_PARAMS_FILE

# ---------- PX4 SITL 机型 (工具栏一键启停, target 与 PX4_SYS_AUTOSTART 对应) ----------
PX4_MODELS = [
    {'name': '10016_none_iris',       'target': 'none_iris'},
    {'name': '70003_fucker_coax_x050',  'target': 'none_fucker_coax_x050'},
    {'name': '70004_fucker_coax_x100',  'target': 'none_fucker_coax_x100'},
    {'name': '70005_fucker_coax_x300',  'target': 'none_fucker_coax_x300'},
    {'name': '70006_fucker_x450',       'target': 'none_fucker_x450'},
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
         # marker_landing_node (aruco 条目不声明, 默认 aruco)
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
