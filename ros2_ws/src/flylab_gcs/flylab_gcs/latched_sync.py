#!/usr/bin/env python3
"""latched 参数同步管道 —— 声明式注册表, 加一个同步参数只改这里。

地面站 → 视觉节点的大量运行时参数 (码边长/字典/分形配置/人脸任务/追踪
俯角与距离...) 走同一条 latched 管道: reliable + transient_local QoS,
后启动的节点一接入即拿到当前值; 界面控件遵守 "未动过不发布" 约定 (节点
用各自 yaml/ROS 参数默认值)。声明表之前的实现里, 每加一个参数要穿 4 个
文件: config (话题+值域) → ros_node (发布器+publish 方法) →
algo_output_panel (控件工厂+显隐矩阵) → main_window (注入回调); 现在全部
收敛到本文件的 LATCHED_PARAMS:

  ros_node          LatchedPublisher 按表统一创建发布器, publish_param()
  AlgoOutputPanel   按条目的 ui 字段自动生成顶部调节控件 + 数据驱动显隐
  main_window       LatchedKeepalive 承载 "变化即发布 + 周期重发兜底"
  节点侧            不变 (仍按各自话题订阅, 见 config 各话题注释)

一条声明 = 一个 LatchedParam:
  key/topic/msg/encode   发布侧: encode 缺省按 msg 类型取 float/str/bool
  kind                   面板控件类型: 'slider'/'spin'/'spin_f'/'combo';
                         None = 无面板控件 (FOV 由云台滑杆+轮询节流驱动、
                         机头跟随开关由云台面板按钮驱动), 只走发布管道
  lo/hi/step/default     控件值域与初始显示值 (话题/值域常量仍定义在
                         config, 这里只引用 —— 改值域去 config)
  choices                combo 候选: list (候选即发布值) 或 dict
                         (键=发布值, 值=显示名, 如人脸任务)
  groups                 显隐归属 [(分组 key, landing_kind|None)...]:
                         None = 任意降落检测器类型都算命中; landing 分组
                         的归属靠 kind 区分 (字典属于 aruco 降落, 分形
                         配置属于 fractal 降落), 与原显隐矩阵逐条对应
  auto_sync              True = 未动过时显示值持续跟随云台俯角 (仅
                         追踪俯视角度, 约定见原 _sync_track_pitch)
  stretch/decimals/suffix/width/tooltip   控件布局与文案

新增参数步骤: ① config 加话题与值域常量 ② 本表加一条 LatchedParam
③ 节点侧订阅该话题 —— 界面/发布侧零改动。
"""
import time

from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Bool, Float64, String

from . import config

# latched 约定统一 QoS: reliable + transient_local, KeepLast(1) 深度
LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST, depth=1)


class LatchedParam:
    """一条 latched 同步参数的完整声明 (发布侧 + 可选面板控件)。"""

    def __init__(self, key, topic, msg, encode=None, *, kind=None,
                 title='', tooltip='', choices=None,
                 lo=None, hi=None, step=None, default=None,
                 decimals=0, suffix='', width=None, stretch=1,
                 groups=(), auto_sync=False):
        self.key = key
        self.topic = topic
        self.msg = msg
        if encode is None:
            encode = {Float64: float, String: str, Bool: bool}[msg]
        self.encode = encode
        self.kind = kind
        self.title = title
        self.tooltip = tooltip
        self.choices = choices
        self.lo = lo
        self.hi = hi
        self.step = step
        self.default = default
        self.decimals = decimals
        self.suffix = suffix
        self.width = width
        self.stretch = stretch
        self.groups = groups
        self.auto_sync = auto_sync

    def visible_for(self, keys, landing_kind):
        """激活分组 keys + 降落检测器类型下, 本参数控件是否可见。"""
        for gkey, gkind in self.groups:
            if gkey in keys and (gkind is None or gkind == landing_kind):
                return True
        return False


# ---------- 声明表 (顺序 = 面板顶部调节行的排列顺序) ----------

LATCHED_PARAMS = [
    # ArUco 码边长: 场景含多种尺寸码时运行中同步 aruco_node /
    # aruco_landing_node 的 marker_length (位姿解算尺度); 未拖动过
    # 不发布, 节点用 aruco.yaml 的值
    LatchedParam(
        'marker_length', config.ARUCO_MARKER_LENGTH_TOPIC, Float64,
        kind='slider', title='ArUco 码边长 (m)',
        tooltip='场景含多种尺寸码时拖动同步算法节点; 先把码摆到已知距离,\n'
                '调到面板"实际宽度"≈码的真实边长, 此时"码位置"\n'
                '的 z 分量 (末位) 即准确距离',
        lo=config.ARUCO_MARKER_LENGTH_MIN, hi=config.ARUCO_MARKER_LENGTH_MAX,
        step=config.ARUCO_MARKER_LENGTH_STEP,
        default=config.ARUCO_MARKER_LENGTH_DEFAULT,
        decimals=2, stretch=2,
        groups=[('aruco', None), ('fractal', None),
                ('landing', 'aruco'), ('landing', 'fractal')]),
    # 降落朝向偏置: 机头相对码 0° 方向的落点朝向 (度, 右正), 运行中
    # 修改 aruco_landing_node 解锁航向按新朝向重新对齐; 未动过不发布
    LatchedParam(
        'landing_yaw_offset', config.LANDING_YAW_OFFSET_TOPIC, Float64,
        kind='spin', title='朝向偏置 (°)',
        tooltip='视觉反控降落时机头相对码 0° 方向 (码角点 0→1, 即码图案 x 轴)\n'
                '的落点朝向, 右转为正; 0 = 机头对齐码 0° 方向。\n'
                '运行中修改立即生效: 飞机会悬停转到新朝向再继续下降',
        lo=config.LANDING_YAW_OFFSET_MIN, hi=config.LANDING_YAW_OFFSET_MAX,
        step=config.LANDING_YAW_OFFSET_STEP,
        default=config.LANDING_YAW_OFFSET_DEFAULT,
        suffix='°', width=78,
        groups=[('landing', None), ('fractal', None)]),
    # ArUco 字典: 场景换码字典时下拉运行中同步 dict_name (下一帧生效);
    # 未选择过不发布, 节点用 aruco.yaml 的值
    LatchedParam(
        'aruco_dict', config.ARUCO_DICT_TOPIC, String,
        kind='combo', title='ArUco 字典',
        tooltip='场景换了码的字典 (如 4x4 换 6x6) 时下拉切换, 运行中同步\n'
                '检测/降落节点, 下一帧检测生效; 降落中切换会重新选择目标',
        choices=config.ARUCO_DICTS, default=config.ARUCO_DICT_DEFAULT,
        groups=[('aruco', None), ('landing', 'aruco')]),
    # 人脸下游任务: FaceXFormer 对检出人脸跑的分析任务 (下一帧生效);
    # 键 = 发给节点的任务名 (节点 facex_task_id 表), 值 = 显示名
    LatchedParam(
        'face_task', config.FACE_TASK_TOPIC, String,
        kind='combo', title='人脸任务',
        tooltip='FaceXFormer 对检出人脸跑的下游分析任务 (张数见"人脸数量"),\n'
                '下拉运行中切换, 下一帧生效:\n'
                '人像分割/68 关键点/头部姿态/人脸属性/年龄性别/点可见性',
        choices=config.FACE_TASKS, default=config.FACE_TASK_DEFAULT,
        groups=[('face', None)]),
    # 人脸任务张数: 批量跑置信度前 N 张脸 (0 = 全部, 动态 batch);
    # 键 = 发给节点的数值字符串, 值 = 显示名
    LatchedParam(
        'face_topk', config.FACE_TOPK_TOPIC, String,
        kind='combo', title='人脸数量',
        tooltip='FaceXFormer 批量跑任务的人脸张数 (按置信度排序):\n'
                '前 1/2/3/5 张或全部检出脸; 脸多时拼批一次推理,\n'
                '下拉运行中切换, 下一帧生效',
        choices=config.FACE_TOPKS, default=config.FACE_TOPK_DEFAULT,
        groups=[('face', None)]),
    # fractal 分形配置: 嵌套层数 (2L~5L), 下拉运行中同步 fractal_node
    # 重建检测器 (下一帧生效); 未选择过不发布, 节点用 aruco.yaml 的值
    LatchedParam(
        'fractal_config', config.FRACTAL_CONFIG_TOPIC, String,
        kind='combo', title='fractal 配置',
        tooltip='分形码嵌套层数 (2L~5L, 数字越大层数越多, 远距/遮挡越鲁棒,\n'
                '内层码越小), 场景换了码时下拉切换, 运行中同步 fractal_node,\n'
                '下一帧检测生效',
        choices=config.FRACTAL_CONFIGS, default=config.FRACTAL_CONFIG_DEFAULT,
        groups=[('fractal', None), ('landing', 'fractal')]),
    # 目标追踪俯视角度 (度, 向下正): 反控接管中追踪节点高度环据此控制
    # 飞机升降使云台以该俯角注视目标; 未动过不发布 = 不启用高度环。
    # auto_sync: 未动过时显示值持续跟随云台当前俯角 (用户一动即停)
    LatchedParam(
        'track_pitch', config.TRACK_PITCH_TOPIC, Float64,
        kind='spin', title='俯视角度 (°)',
        tooltip='反控接管中云台注视目标的俯视角 (向下为正, 水平=0):\n'
                '设置后飞机自动升降, 直到以该俯视角注视目标;\n'
                '未设置时反控保持原地悬停。运行中修改立即生效',
        lo=config.TRACK_PITCH_MIN, hi=config.TRACK_PITCH_MAX,
        step=config.TRACK_PITCH_STEP, default=config.TRACK_PITCH_DEFAULT,
        suffix='°', width=78, auto_sync=True,
        groups=[('track', None)]),
    # 目标追踪跟随距离 (米): 机体跟随-水平时距离环保持的水平距离;
    # 未动过不发布, 节点用参数默认值 bodyfollow_dist_m
    LatchedParam(
        'track_dist', config.TRACK_DIST_TOPIC, Float64,
        kind='spin_f', title='跟随距离 (m)',
        tooltip='机体跟随 (水平跟随) 时飞机保持与目标的水平距离 (米):\n'
                '设置后飞机自动前进/后退, 直到距离收敛到该值;\n'
                '未设置时用节点默认值 10m。运行中修改立即生效',
        lo=config.TRACK_DIST_MIN, hi=config.TRACK_DIST_MAX,
        step=config.TRACK_DIST_STEP, default=config.TRACK_DIST_DEFAULT,
        decimals=1, suffix='m', width=86,
        groups=[('track', None)]),
    # ---------- 无面板控件的纯发布参数 ----------
    # 视场角: 用户调整过 FOV 时变化即发布 (滑杆在云台面板, 由主窗口
    # 轮询节流 + LatchedKeepalive 周期重发兜底), 算法节点同步估算内参
    LatchedParam('camera_fov', config.CAMERA_FOV_TOPIC, Float64),
    # 目标追踪反控机头跟随开关: True=机头跟随视线 / False=机头保持
    # 航向仅转云台; 由云台面板"跟随"按钮在反控中点击下发
    LatchedParam('track_follow', config.TRACK_FOLLOW_TOPIC, Bool),
]


class LatchedPublisher:
    """按 LATCHED_PARAMS 统一创建 latched 发布器 (原 ros_node 十余段
    QoSProfile 样板的收敛点)。publish(key, value) 按声明编码并发布。"""

    def __init__(self, node, params=None):
        if params is None:
            params = LATCHED_PARAMS
        self._params = {p.key: p for p in params}
        self.pubs = {p.key: node.create_publisher(p.msg, p.topic, LATCHED_QOS)
                     for p in params}

    def publish(self, key, value):
        """按声明编码 value 并发布到对应话题 (key 须在声明表内)。"""
        param = self._params[key]
        msg = param.msg()
        msg.data = param.encode(value)
        self.pubs[key].publish(msg)


class LatchedKeepalive:
    """latched 发布策略: 值变化即发布 + 每 resend_sec 周期重发兜底。
    latched 只在订阅匹配瞬间投递一次, 任何漏收 (DDS 发现前发布/订阅端
    短暂掉线) 都会让节点停在旧值, 周期重发保证最终一致。供轮询节拍
    调用 sync(publish, value); value=None 由调用方自行跳过。"""

    def __init__(self, resend_sec=2.0):
        self._resend = resend_sec
        self._last = None
        self._t = 0.0

    def sync(self, publish, value):
        now = time.monotonic()
        if value != self._last or now - self._t >= self._resend:
            self._last = value
            self._t = now
            publish(value)
