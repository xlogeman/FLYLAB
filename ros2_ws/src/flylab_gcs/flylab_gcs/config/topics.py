#!/usr/bin/env python3
"""ROS 话题名与通信参数 (QoS 开关/新鲜度/超时) —— 地面站与视觉节点的
全部话题在此集中定义, 节点侧 (flylab_vision) 与此处一一对应。"""

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
# 地面站 10Hz 发布；marker_landing_node 据此把图像误差旋到机体系，适配任意云台形态。
# 非 AirSim 源不发布 —— 节点超时后回退"垂直向下 + 图像上边朝机头"刚性假设。
GIMBAL_STATE_TOPIC = '/flylab/gimbal_state'

# ---------- 航向跟随的算法通道 (视觉反控算法控制云台 + 机头跟随) ----------
# offboard 单写者原则: 反控节点独占 setpoint 流时, 跟随指令必须由算法
# 节点自己注入 (sendVelocityFRDYawRate), GCS 不再经 teleop 发偏航量。
# 地面站只提供师资与云台控制权, 三个话题 (marker_landing 已接入):
#   gimbal_cmd    算法 → 云台 [yaw_rate, pitch_rate] (°/s), 兼作 follow 心跳
#   gimbal_follow 算法请求相机侧反卷 (世界锁定 + 误差收敛回中), std_msgs/Bool
#   gimbal_yaw_err 地面站广播反卷后的偏航误差 (度, 右+; NaN=未跟随),
#                  latched —— 算法订阅后 yaw_rate = KP × err 即完成接入
GIMBAL_CMD_TOPIC = '/flylab/gimbal_cmd'
GIMBAL_FOLLOW_REQ_TOPIC = '/flylab/gimbal_follow'
GIMBAL_YAW_ERR_TOPIC = '/flylab/gimbal_yaw_err'
GIMBAL_FOLLOW_ALGO_TIMEOUT_SEC = 1.0   # 算法 follow 心跳超时 (gimbal_cmd 断流自动退)

# 视场角同步话题 (std_msgs/Float64, latched): 用户拖动视场滑杆即发布,
# ArUco 等按 FOV 估算内参的算法节点订阅后实时更新; latched 让后启动的
# 节点一接入即拿到当前值。未调整过 (保持 settings.json 原值) 不发布。
CAMERA_FOV_TOPIC = '/flylab/camera_fov'

# ---------- latched 运行时参数同步话题 (界面值域见 ui_ranges, 控件声明
# 见 latched_sync.LATCHED_PARAMS; 共同约定: 后启动的节点一接入即拿到
# 当前值, 界面 "未动过不发布") ----------
# ArUco 码边长: 运行中同步 aruco_node / marker_landing_node 的
# marker_length (位姿解算尺度)
ARUCO_MARKER_LENGTH_TOPIC = '/flylab/aruco_marker_length'
# ArUco 字典: 运行中同步 dict_name (下一帧检测生效)
ARUCO_DICT_TOPIC = '/flylab/aruco_dict'
# fractal 分形配置: 运行中同步 fractal_node 的 fractal_config (重建检测器)
FRACTAL_CONFIG_TOPIC = '/flylab/fractal_config'
# 人脸下游任务: 下拉切换经该话题同步 face_detect_node (下一帧生效)
FACE_TASK_TOPIC = '/flylab/face_task'
# 人脸任务张数: FaceXFormer 批量跑置信度前 N 张脸 (0 = 全部)
FACE_TOPK_TOPIC = '/flylab/face_topk'
# 降落航向偏置: 运行中修改 marker_landing_node 解锁航向按新朝向重新对齐
LANDING_YAW_OFFSET_TOPIC = '/flylab/landing_yaw_offset'

# ---------- ArUco 降落 ----------
# 降落节点状态回传话题 (tracking/landing/landed/idle)，地面站据此复位按钮
LANDING_STATUS_TOPIC = '/flylab/landing_status'
# 算法信息面板数据话题 (std_msgs/String JSON): 各算法节点按 src 分组发布
# 自定义 schema, 右栏"算法信息"面板按运行状态动态显示 (见 panel_sections.py)
ALGO_INFO_TOPIC = '/flylab/algo_info'

# ---------- 目标追踪 ----------
# (流程与 latched 参数话题见 latched_sync.LATCHED_PARAMS 声明表)
TRACK_ROI_TOPIC = '/flylab/track_roi'
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
# 目标追踪跟随距离同步 (机体跟随-水平, 算法信息"跟随距离"输入框): latched
# Float64 发给追踪节点 (米)。机体跟随时距离环控制飞机水平收放保持该水平
# 距离; 未动过不发布 = 节点用参数默认值 (bodyfollow_dist_m, 同俯视角度约定)
TRACK_DIST_TOPIC = '/flylab/track_dist'

# ---------- 手动遥控 ----------
# 界面发归一化遥控量 (-1..1, Twist): linear.x=前后 linear.y=左右
# linear.z=升降(升+) angular.z=偏航(左转+)；teleop_node 换算成机体系 FRD 速度
TELEOP_CMD_TOPIC = '/flylab/teleop_cmd'
TELEOP_STATUS_TOPIC = '/flylab/teleop_status'
