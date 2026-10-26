#!/usr/bin/env python3
"""flylab_gcs 全局配置 —— 改这里，不改代码。"""

# ---------- 地面站窗口大小 ----------
# 初始窗口大小 = 屏幕可用区域 × 比例 (画面随窗口等比缩放;
# 窗口无固定尺寸, 启动后可直接拖拽边缘调整, 此处只影响初始大小)。
# 高度取大些: 画面下方留出日志框 (免切终端看日志)
WINDOW_SIZE_RATIO = (0.5, 0.55)   # (宽, 高) 相对屏幕可用区域的比例

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
# Scene 图传走 PNG 压缩传输 (线上体积约原始 BGR 的 1/3, UE 编码 + 本端
# 解码均为 C 实现, 净赚帧率)。注意 PNG 无损不糊; True 时若原始回退路径
# 触发, 会自动做 RGB→BGR 通道翻转保持色彩一致。False = 始终原始传输
DISPLAY_SCENE_COMPRESS = False

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
GIMBAL_JOY_RATE_DEG_S = 60.0   # 云台摇杆满偏时的转动角速度 (°/s), 偏转量线性缩放

# 飞行中姿态跟随频率 (Hz)。相机是刚性挂载, 云台角只在下发瞬间按当时机体
# 姿态换算; 飞行中机体姿态一变, 记忆的云台角就与实际视角脱钩, 下次按键会
# 把积累的姿态差一次性归零 —— 表现为方向键不再纯上下/左右。除按键立即下发
# 外, 以该频率按当前机体姿态重下发保持基准新鲜; 0 = 关闭 (仅地面静态调试)。
# 该频率同时是云台稳定 (follow) 模式的重下发频率: 越高抗机体晃动越好,
# 但每拍有 2 次 RPC (kinematics + 当前相机 simSetCameraPose, 首次同步/
# 切换相机时再多几拍), 且 UE 端搬相机 actor 挤占渲染线程拖慢图传。
# 15Hz 补偿延迟 ~66ms, 人眼无感; 再高收益递减、图传帧率代价明显
GIMBAL_HOLD_HZ = 15.0

# 回中动画角速度 (度/秒): 回中不再是瞬跳, 而是以该速度平滑转回机头正前方,
# 按方向键可随时打断。越小越柔, 90 ≈ 实机云台回中的手感
GIMBAL_SLEW_RATE_DEG_S = 90.0

# 云台限位 (度), 模拟真实二轴云台的机械限位。
# 俯仰是相对 settings.json 安装角的: 各相机安装 Pitch 已改 0 (水平),
# 相对俯仰 = 绝对俯仰, min/max 取 -90/+90 使绝对俯仰恰好限定在
# 垂直向下(-90°) ~ 垂直向上(+90°), 到垂直即停。
# (安装角运行时由 simGetCameraInfo 实测, 以后再改 settings.json 安装角,
# 只需同步调整这里的限位, 云台/反控摆位/降落方向映射均自动适配)
GIMBAL_YAW_LIMIT_DEG = 45.0      # 偏航限位 ±45°
GIMBAL_PITCH_MIN_DEG = -90.0     # 俯仰下限 (垂直向下)
GIMBAL_PITCH_MAX_DEG = 90.0      # 俯仰上限 (垂直向上)

# ---------- 视场角 (AirSim 相机, 经 simSetCameraFov 实时调整) ----------
# 云台面板"视场"滑杆的调节范围与默认值。所有 UE 相机共用同一个值: 下发到
# 当前显示的相机, 其余相机切到时对齐, 保证各相机视场一致。默认值只用于
# 界面显示 —— 用户从未动过滑杆时不下发, 保持 settings.json 里各相机的
# 原始 FOV; 一旦调整过即持久化 (与云台角同一个状态文件), 重启/重连/
# UE 重 play 后自动恢复。注意: FOV 变化会改变图传画面几何 —— 未提供
# 标定文件、按 FOV 估算内参的算法 (如 ArUco 降落) 会受影响, 飞行中
# 调整需留意
GIMBAL_FOV_MIN_DEG = 30.0
GIMBAL_FOV_MAX_DEG = 120.0
GIMBAL_FOV_DEFAULT_DEG = 90.0

# ---------- ArUco 降落 (主窗口独立按钮) ----------
# 完整降落链路: 点击按钮拉起 aruco_landing_node (三态状态机:
# 接管 offboard → 视觉对齐调速下降 → 满足条件切 PX4 AUTO.LAND),
# 再次点击发 landing_stop 取消并交还控制权。
# 与算法下拉框互不影响: ArUco检测可同时运行 (两路独立检测互不干扰)。
# 使用前提: 通信链路在跑 + 飞机已解锁升空 + 运行 enables_landing 的算法
# (点击按钮时自动把云台俯仰转到垂直向下)。
LANDING_CMD = ['ros2', 'run', 'flylab_vision', 'aruco_landing_node']
LANDING_START_CMD = 'landing_start'
LANDING_STOP_CMD = 'landing_stop'
# 降落节点的状态回传话题 (tracking/landing/landed/idle),
# 地面站据此复位按钮并给成功/中止反馈
LANDING_STATUS_TOPIC = '/flylab/landing_status'

# ---------- PX4 SITL (地面站一键启停, 省开终端) ----------
# 经 scripts/run-px4.sh 启动 (自动补 git 元数据/清残留锁/设 PX4_SIM_HOST_ADDR),
# target 是 make px4_sitl_default 的机型目标名, 与 PX4_SYS_AUTOSTART 严格
# 对应 (见 README §6 机型库)。新增机型: 先按 README §6 加好 PX4 侧文件
PX4_SCRIPT = '/root/flylab/scripts/run-px4.sh'
PX4_MODELS = [
    {'name': '10016_none_iris',       'target': 'none_iris'},
    {'name': '70003_fucker_3120g_tw23', 'target': 'none_fucker_3120g_tw23'},
    {'name': '70004_fucker_2000g_tw15', 'target': 'none_fucker_2000g_tw15'},
]

# ---------- AirSim 导出项目 (PX4 SITL 启动时联动拉起) ----------
# 工具栏 PX4 SITL 选机型时, 除启动 PX4 外还拉起 Windows 端导出的 AirSim
# 项目 exe (WSL 经 interop 直启 Windows GUI 进程, 停止/崩溃/退出一并回收)。
# 机型 → 专属 exe 的映射 (键 = config.PX4_MODELS 的 name); 未命中的机型
# 回退 AIRSIM_APP_DEFAULT。后续导出新项目后, 在映射里按机型加一条即可;
# 置 None 则该机型不拉起项目窗口。
AIRSIM_APP_DEFAULT = \
    '/mnt/d/A_Code/MyProject/BeimingSim/BeimingSim_Build/Windows/BeimingSim.exe'
AIRSIM_APP_FOR_MODEL = {
    # '10016_none_iris': '/mnt/d/.../另一个导出项目/Windows/xxx.exe',
}


def airsim_app_for(model_name):
    """机型对应的 AirSim 导出项目 exe (未配置的机型用默认, None=不拉起)。"""
    return AIRSIM_APP_FOR_MODEL.get(model_name, AIRSIM_APP_DEFAULT)

# ---------- 手动遥控 (双摇杆 → offboard 速度, 执行节点 flylab_vision/teleop_node) ----------
# 界面发归一化遥控量 (-1..1, geometry_msgs/Twist): linear.x=前后 linear.y=左右
# linear.z=升降(升+) angular.z=偏航(左转+); teleop_node 按速度上限换算成
# 机体系 FRD 速度 + NED 偏航角速率
TELEOP_CMD_TOPIC = '/flylab/teleop_cmd'
TELEOP_STATUS_TOPIC = '/flylab/teleop_status'
TELEOP_START_CMD = 'teleop_start'
TELEOP_STOP_CMD = 'teleop_stop'
TELEOP_CMD = ['ros2', 'run', 'flylab_vision', 'teleop_node']

# ---------- 通信链路 (地面站一键拉起, 省开终端) ----------
# uXRCE-DDS: PX4 ↔ ROS2 直连桥, UDP 8888。供 tracker:=uxrce / aruco_landing
# (link:=uxrce) 链路使用
UXRCE_CMD = ['MicroXRCEAgent', 'udp4', '-p', '8888']
# MAVROS: 经典 MAVLink 桥, 用 14542 专属实例 (14580 会被 AirSim 抢占, 详见 README)
MAVROS_CMD = ['ros2', 'run', 'mavros', 'mavros_node', '--ros-args',
              '-p', 'fcu_url:=udp://:14542@', '-p', 'tgt_system:=1',
              '-p', 'tgt_component:=1']

# ---------- 飞行控制 (AirSim API 经 RPC 直发 PX4) ----------
# 工具栏"飞行控制"按钮的菜单项, cmd 对应 vehicle_control.VehicleController._exec
# 的命令 key。命令经 AirSim 的 MAVLink 控制通道下发 PX4, 与 offboard
# tracker (mavros/uxrce) 相互独立; 结果看 QGC / mavros state
VEHICLE_CMDS = [
    {'name': '解锁 (Arm)',     'cmd': 'arm'},
    {'name': '起飞 (Takeoff)', 'cmd': 'takeoff'},
    {'name': '降落 (Land)',    'cmd': 'land'},
    {'name': '悬停 (Hover)',   'cmd': 'hover'},
    {'name': '返航 (GoHome)',  'cmd': 'gohome'},
    {'name': '上锁 (Disarm)',  'cmd': 'disarm'},
]

# ---------- 视觉算法选择 (下拉框 + 运行/取消按钮) ----------
# name=下拉框显示文字, start/stop=运行/取消时发布的指令。
# cmd=算法节点启动命令 (可选): 填了则地面站点"运行"时自动拉起进程、点"取消"
# 时自动回收; 不填则假定节点已在外部运行, 只发启停指令。
# enables_landing=True 的算法是 ArUco 降落的前置条件
# 界面交互对所有算法统一: 新算法只要遵循"收到 start 指令开始、收到 stop
# 指令停止"的约定, 在这里加一项即可出现在下拉框里。
ALGORITHMS = [
    {'name': 'ArUco检测',
     'cmd': ['ros2', 'run', 'flylab_vision', 'aruco_node'],
     'start': 'aruco_start', 'stop': 'aruco_stop',
     'enables_landing': True},
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
# name=下拉框显示文字, cmd=ros2 launch 命令 (整链路: 传感器/桥接/算法/跟踪器)。
# 交互与视觉算法按钮一致: 点选即启动, 再点运行中的即停止, 选其他停旧换新;
# 运行中按钮点亮 + 文字跟随。链路输出直通终端 (含 RViz 等长驻进程, 输出量大)。
# 注意: 避障/跟踪类链路依赖通信链路 (MAVROS/uXRCE) 在跑, 先在工具栏开启。
FLIGHT_LINKS = [
    {'name': '感知建图',
     'cmd': ['ros2', 'launch', 'flylab_flight', 'depth_mapping.launch.py']},
    {'name': 'EGO 避障',
     # mavros:=false: 不随 launch 自启动 MAVROS 桥, 通信链路统一由
     # 工具栏"通信链路"按钮管理 (避免双 MAVROS 抢 14542 / 状态脱节)
     # need_link=True: 需先在工具栏开启通信链路, 菜单项才可点 (跟踪器依赖桥)
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
