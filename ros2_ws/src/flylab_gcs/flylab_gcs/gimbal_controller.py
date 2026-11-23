#!/usr/bin/env python3
"""软件云台控制器 —— 云台/视场角/航向跟随的状态机 + 独立 RPC 线程。

与视频抓取 (camera_grabber.Grabber) 解耦: 抓取线程只拉图, 本模块经独立
RPC 连接下发相机姿态 (simSetCameraPose) 与视场角 (simSetCameraFov) ——
机体运动时的姿态跟随重下发不占抓取往返, 若与拉图串行, 每拍多 2 次 RPC
( kinematics + setCameraPose, 后者还触发 UE 重渲染) 会把图传帧率拉低一半。

世界系稳定云台: 把视线方位/仰角钉在世界系 (经 simSetCameraPose), 机体
转动/倾斜由周期重下发 (GIMBAL_HOLD_HZ) 补偿, 悬停抖动由指纹容差
(APPLY_TOL_DEG) 拦截; 云台稳定关闭即刚性挂载 (FPV 手感)。

航向跟随 (云台偏航带动机头) 双来源: 'ui' = 界面按钮 (指令走 teleop
offboard 通道, GCS P 律); 'algo' = 反控算法经 gimbal_follow 请求 (算法
是 offboard 唯一写者, 偏航指令由算法注入, GCS 只做反卷与误差广播)。
相机侧反卷与来源无关; 算法来源靠 gimbal_cmd 心跳超时自动退出。

支撑模块 (纯函数):
  quat_math.py  四元数数学工具
  gimbal.py     云台限位/安装基准采集
"""
import math
import threading
import time

try:
    import cosysairsim as airsim
except ImportError:
    import sys
    sys.path.insert(0, '/root/flylab/Cosys-AirSim/PythonClient')
    import cosysairsim as airsim

from . import config
from .gimbal import (APPLY_TOL_DEG, MountNotReady, clamp_gimbal,
                     capture_mount)
from .quat_math import (gimbal_pitch_quat, gimbal_yaw_quat, quat_conj,
                        quat_mul, quat_rot_vec, quat_yaw_el)

VEHICLE = 'PX4'
# settings.json 里配置过的全部相机: simSetCameraPose 按机体系下发, 首次
# 同步时逐个下发一次 (供 UE 重玩检测), 切到某相机时指纹含相机名必失配
DEFAULT_CAMERAS = ['left_cam', 'front_cam', 'right_cam', 'depth_cam']
# 云台线程循环节拍: 摇杆步进/回中动画的推进频率 (下发频率仍由
# GIMBAL_HOLD_HZ 节流; RPC 与拉图分离, 姿态跟随重下发不占抓取往返)
GIMBAL_LOOP_HZ = 50


def _wrap180(deg):
    """角度归一化到 [-180, 180)。"""
    d = math.fmod(deg + 180.0, 360.0)
    if d < 0:
        d += 360.0
    return d - 180.0


class GimbalController(threading.Thread):
    """云台/FOV/航向跟随控制器 (后台线程, 独立 RPC 连接)。

    由 Grabber 创建并驱动: 源切换经 on_camera_changed 通知 (相机镜像 +
    置 dirty), 连接状态经 set_connected 通知, 连接纪元 (0->1) 经
    on_connected 触发云台复核与 FOV 重采。界面经 Grabber 的委托方法
    访问, 不直接持有本类。"""

    def __init__(self, log, host, port, camera):
        super().__init__(daemon=True, name='gimbal-rpc')
        self.log = log
        self.host, self.port = host, int(port)
        self.lock = threading.Lock()
        # 当前相机镜像 (Grabber 经 on_camera_changed 同步): 本模块只读
        self._camera = camera
        self._airsim_src = True   # 当前源是否 AirSim 相机 (有云台/FOV)
        self._connected = False   # AirSim RPC 连通镜像 (抓取线程探针更新)
        # 软件云台: 全局角度 (yaw, pitch), 作用于所有 AirSim 相机。
        # 无持久化: 每次连接 AirSim 从回中开始, 安装基准重采 (见连接处)
        self._gimbal = (0.0, 0.0)
        self._gimbal_mount = {}    # 相机安装位姿 (连接后按需重采)
        self._gimbal_last = {}     # 本会话各相机最近一次下发姿态
        self._fov = None           # 全局视场角 (度): 所有相机共用
        # (None=尚未采纳; 连接后查询相机真实 FOV 采纳, 见 _adopt_camera_fov)
        self._fov_dirty = False       # 视场角待下发/待采纳标记
        self._fov_adopt_t = 0.0       # 最近一次 FOV 采纳尝试时刻 (失败重试退避)
        self._gimbal_anim = None   # 回中动画目标 (yaw, pitch), None=无动画
        self._anim_t_prev = 0.0
        # 摇杆连续转动: 当前角速度 (°/s) 与上次帧时刻 (None=未在转动)
        self._gimbal_rate = (0.0, 0.0)
        self._rate_t_prev = None
        self._gimbal_dirty = False
        # 云台稳定 (follow 模式): 云台角按 GIMBAL_HOLD_HZ 世界系持续重下发
        # (俯仰/横滚不随机体晃动, 偏航跟随机头); 关闭为刚性挂载 (FPV 手感)。
        # UE 原生 Gimbal.Stabilization 与 simSetCameraPose 互斥, 故在地面站侧实现
        self._gimbal_stab = True
        # 上次成功下发的 (机体姿态, 云台角, 相机) 指纹: 悬停抖动在容差内
        # 则跳过下发 RPC, 省下的串行等待直接转化为图传帧率
        self._gimbal_apply_key = None
        # 会话内限位拒绝标记 (方向 -> bool), 按键被限位卡住时标红, 不持久化
        self._gimbal_blocked = {'yaw+': False, 'yaw-': False,
                                'pitch+': False, 'pitch-': False}
        self._gimbal_apply_t = 0.0   # 上次云台下发时刻 (姿态跟随节流用)
        # 航向跟随 (云台偏航带动机头): _follow 开启后偏航语义从"跟随机头"
        # 翻转为"世界系锚定" —— 偏航输入写世界方位锚点 _az_target, 抓取
        # 每拍把存储 yaw 反卷回 wrap(锚点 − 航向), 机头转动时 yaw 收敛回 0;
        # _veh_yaw_t 为最近一次机体航向测量时刻 (外环新鲜度依据)
        self._follow = False
        self._follow_src = 'ui'      # 跟随来源: 'ui'(teleop) / 'algo'(反控算法)
        self._algo_cmd_t = None      # 最近一次算法云台指令时刻 (心跳判定)
        self._az_target = None       # 视线世界方位锚点 (度, 北偏东右+), None=待快照
        self._veh_yaw_t = None       # 航向测量时刻 (monotonic, 秒)
        self._veh_yaw_prev = None    # 上拍航向 (用户转机头时按增量平移锚点用)
        self._user_yaw_t = None      # 最近一次遥控偏航杆活动时刻 (None=未在打杆)
        self.running = True

    # ---------- 状态通知 (Grabber 驱动) ----------

    def on_camera_changed(self, name, airsim_src):
        """源切换通知: 更新相机镜像并置双 dirty (切源后视角/视场已过时,
        下一拍立即补下发)。与原实现同语义: 帧提交前的视角状态先于
        新源画面就绪。"""
        with self.lock:
            self._camera = name
            self._airsim_src = airsim_src
            self._gimbal_dirty = True
            self._fov_dirty = True

    def set_connected(self, ok):
        """AirSim 连通镜像 (抓取线程探针结果): 云台线程据此等待建连。"""
        with self.lock:
            self._connected = bool(ok)

    def on_connected(self):
        """连接纪元 (连接数 0->1) 通知: 云台状态复核 + FOV 重采。
        云台状态复核: 保留基准/角度, 复位跟随 + 限位标红, 重下发恢复视角
        (重采会污染安装基准, 见 _reset_gimbal)。FOV 置 dirty 触发
        _apply_fov —— _fov 恒为 None (无记忆), 走查询相机真实 FOV 采纳
        路径, 算法节点启动即同步。"""
        self._reset_gimbal()
        with self.lock:
            self._fov_dirty = True

    def stop(self):
        self.running = False

    # ---------- 软件云台 (界面线程调用, 角度由云台线程经 RPC 写入相机) ----------

    def gimbal_set_rate(self, yaw_rate, pitch_rate):
        """设置云台连续转动角速度 (°/s, 摇杆控制), 每帧按限位夹紧步进,
        回零即停; 限位卡住的方向记入 _gimbal_blocked 供标红。
        计时仅在停止→转动的沿上重置: 算法伺服每帧流式重发速率 (30Hz),
        若每次都重置 _rate_t_prev, _rate_tick 当帧只计时不动, 下一帧才
        步进 —— 动一帧停一帧等效 15Hz 阶梯 (视觉一顿一顿); 非零→非零
        保持计时则每帧匀速步进, 与摇杆手感一致。"""
        with self.lock:
            if not self._airsim_src:
                return
            if yaw_rate == 0.0 and pitch_rate == 0.0:
                self._gimbal_rate = (0.0, 0.0)
                return
            started = self._gimbal_rate == (0.0, 0.0)
            self._gimbal_rate = (yaw_rate, pitch_rate)
            self._gimbal_anim = None   # 摇杆打断回中/摆位动画
            if started:
                self._rate_t_prev = None   # 仅启动沿重新计时

    def gimbal_reset(self):
        """云台回中: 启动平滑回中动画 (非瞬跳), 角速度见
        config.GIMBAL_SLEW_RATE_DEG_S; 摇杆可打断。
        航向跟随中偏航归反卷所有 (yaw 恒为锚点−航向), 回中只收俯仰。"""
        with self.lock:
            self._gimbal_blocked = {k: False for k in self._gimbal_blocked}
            if not self._airsim_src:
                self._gimbal = (0.0, 0.0)
                return
            yaw_tgt = self._gimbal[0] if self._follow else 0.0
            self._gimbal_anim = (yaw_tgt, 0.0)
            self._anim_t_prev = time.monotonic()

    def gimbal_pitch_bottom(self):
        """视觉反控前自动摆位: 仅俯仰平滑转到垂直向下限位, 偏航保持
        当前值不动 (用户手动调好的视角不被打断); 复用回中动画机制
        匀速转动。"""
        with self.lock:
            if not self._airsim_src:
                return
            self._gimbal_blocked = {k: False for k in self._gimbal_blocked}
            self._gimbal_anim = (self._gimbal[0],
                                 config.GIMBAL_PITCH_MIN_DEG)
            self._anim_t_prev = time.monotonic()

    def get_gimbal(self):
        """当前云台角度 (yaw, pitch)；非 AirSim 源返回 None。"""
        with self.lock:
            if not self._airsim_src:
                return None
            return self._gimbal

    def get_gimbal_blocked(self):
        """会话内被限位卡住的方向 (按键不再动作), 界面标红用;
        非 AirSim 源恒为全 False。"""
        with self.lock:
            return dict(self._gimbal_blocked)

    def get_gimbal_stab(self):
        """云台稳定 (follow 模式) 是否开启 (界面开关初始态)。"""
        with self.lock:
            return self._gimbal_stab

    def set_gimbal_stab(self, on):
        """开关云台稳定: 开启立即补一拍下发 (视角马上稳定);
        关闭即停在当前姿态转为刚性跟随。"""
        with self.lock:
            if self._gimbal_stab == on:
                return
            self._gimbal_stab = on
            self._gimbal_dirty = True

    # ---------- 航向跟随 (云台偏航带动机头) ----------

    def set_follow(self, on, source='ui'):
        """开关航向跟随。开启: 打断回中动画 (其偏航目标与反卷冲突),
        方位锚点置 None 待 _follow_tick 首拍快照 (航向+当前偏航, 视角不跳);
        关闭: 偏航语义无声切回"跟随机头", 视角连续不跳变。
        来源归属不同时互不越权: UI 只能退自己的, 反之亦然。"""
        with self.lock:
            if not self._airsim_src:
                return
            if on:
                if self._follow and self._follow_src == source:
                    return
                self._follow = True
                self._follow_src = source
                self._az_target = None   # 来源切换重快照 (值不变, 仅换归属)
                self._veh_yaw_prev = None
                self._gimbal_anim = None
            else:
                if not self._follow or self._follow_src != source:
                    return
                self._follow = False
                self._az_target = None
            self._gimbal_dirty = True

    def clear_ui_follow(self):
        """交还遥控/异常复位时调用: 仅退出 UI 来源的跟随, 算法来源的
        跟随不受遥控生命周期影响 (由算法自己退或心跳超时退)。"""
        self.set_follow(False, source='ui')

    def note_user_yaw(self, active):
        """遥控偏航杆活动标记 (外环定时器 25Hz 调用): 跟随开启时用户
        打杆转机头, 锚点随实测航向增量平移 (相机偏航临时刚性, 画面跟转
        不拉回), 机制见 _follow_tick。"""
        with self.lock:
            self._user_yaw_t = time.monotonic() if active else None

    def set_algo_follow(self, on):
        """反控算法经 /flylab/gimbal_follow 请求/取消跟随 (ros_node 转发)。"""
        self.set_follow(on, source='algo')

    def note_algo_cmd(self, yaw_rate, pitch_rate):
        """算法云台指令入口 (ros_node 转发, 复用摇杆速率通道):
        [yaw_rate, pitch_rate] (°/s), 同时作为算法 follow 的心跳
        (断流超时自动退跟随, 见 _follow_timeout_check)。"""
        with self.lock:
            self._algo_cmd_t = time.monotonic()
        self.gimbal_set_rate(yaw_rate, pitch_rate)

    def get_follow(self):
        """航向跟随是否开启 (界面按钮状态同步依据)。"""
        with self.lock:
            return self._follow

    def get_follow_error(self):
        """航向跟随误差 (度, 右+ = 视线在机头右侧): 真实视线方位 − 航向。
        取最近一次实际下发的相机机体系姿态 (_gimbal_last) 的视轴方位角,
        而非存储云台角的模型值 —— 安装基准若被污染 (重采把已转动
        的相机位姿采进去), 模型值与真实视线差一个基准偏移, 跟随会把机头
        收敛到偏移后的航向 (图像与面板都显示对准、机头却不对准码朝向)。
        视轴 (+x) 水平方位角在俯仰全域 (上仰/平视/下俯) 都等于视线方位;
        不能用"屏幕上方"向量 —— 其方位角只在下俯 (pitch<0) 时等于视线
        方位, 平视时投影为零 (误差恒 0 机头不转), 上仰时翻转 180° (机头
        转向视线反方向)。仅近垂直 (±90°, 视轴水平投影退化) 时退回上方
        向量 (垂直向下时图像上方即偏航方向, 垂直摆位语义不变)。
        未跟随 / 航向测量过期 / 相机姿态缺失 返回 None, 外环按 0 处理
        (停止转向, 遥控量保持悬停)。"""
        with self.lock:
            if not self._follow or self._veh_yaw_t is None:
                return None
            if time.monotonic() - self._veh_yaw_t \
                    > config.GIMBAL_FOLLOW_FRESH_SEC:
                return None
            q = self._gimbal_last.get(self._camera)
            if q is None:
                return self._gimbal[0]   # 尚未下发过 (理论少见): 退回模型值
            v = quat_rot_vec(q, airsim.Vector3r(1.0, 0.0, 0.0))
            if math.hypot(v.x_val, v.y_val) > 0.2:
                return math.degrees(math.atan2(v.y_val, v.x_val))
            up = quat_rot_vec(q, airsim.Vector3r(0.0, 0.0, -1.0))
            return math.degrees(math.atan2(up.y_val, up.x_val))

    def _follow_timeout_check(self):
        """算法来源跟随的心跳超时 (仅云台线程调用): gimbal_cmd 断流
        (算法崩溃/停发) 超时后自动退跟随, 偏航语义切回跟随机头
        (视角连续), 并上屏告警。"""
        with self.lock:
            if not (self._follow and self._follow_src == 'algo'):
                return
            if self._algo_cmd_t is None or time.monotonic() - self._algo_cmd_t \
                    <= config.GIMBAL_FOLLOW_ALGO_TIMEOUT_SEC:
                return
            self._follow = False
            self._az_target = None
            self._gimbal_dirty = True
        self.log('航向跟随: 算法云台指令断流超时 '
                 f'(>{config.GIMBAL_FOLLOW_ALGO_TIMEOUT_SEC:.0f}s), 已自动退出')

    # ---------- 视场角 (界面线程调用, 由云台线程经 RPC 写入相机) ----------

    def set_fov(self, deg):
        """设置全局视场角 (度): 记忆 + 置 dirty, 下一拍下发到当前相机。
        滑杆高频调用, 节流靠抓取帧率 (每拍最多一次 RPC)。"""
        with self.lock:
            if not self._airsim_src:
                return
            deg = max(config.GIMBAL_FOV_MIN_DEG,
                      min(config.GIMBAL_FOV_MAX_DEG, float(deg)))
            if self._fov == deg:
                return
            self._fov = deg
            self._fov_dirty = True

    def get_fov(self):
        """全局视场角 (度); 未调整过返回默认 (界面滑杆显示用)。
        本机/视频文件源返回 None (界面置灰依据)。"""
        with self.lock:
            if not self._airsim_src:
                return None
            return self._fov if self._fov is not None \
                else config.GIMBAL_FOV_DEFAULT_DEG

    def get_fov_applied(self):
        """实际生效的全局视场角 (度); 用户从未调整过返回 None
        (视场角同步发布依据: 不拿界面默认值冒充相机真实 FOV)。
        本机/视频文件源返回 None。"""
        with self.lock:
            if not self._airsim_src:
                return None
            return self._fov

    def get_cam_orientation(self):
        """当前显示相机的机体系姿态四元数 (x, y, z, w)；非 AirSim 源返回 None。

        语义: 相机 NED 体约定 (x 视轴, y 图右, z 图下) 相对机体 FRD 的旋转,
        取最近一次实际下发的机体相对姿态, 云台未动过时回退安装基准。
        供 aruco_landing 做图像方向 -> 机体系的泛化映射。"""
        with self.lock:
            if not self._airsim_src:
                return None
            q = self._gimbal_last.get(self._camera)
            if q is None:
                mount = self._gimbal_mount.get(self._camera)
                if mount is None:
                    return None
                q = mount[1]
            return (q.x_val, q.y_val, q.z_val, q.w_val)

    # ---------- 内部状态机 (仅云台线程调用) ----------

    def _rate_tick(self):
        """摇杆连续转动: 每帧按角速度步进 (限位夹紧并标红)。
        航向跟随中偏航输入改写世界方位锚点 (yaw 由 _follow_tick 反卷
        生成, 限位也由其按"最大超前角"收口), 俯仰照旧。"""
        ry, rp = self._gimbal_rate
        if ry == 0.0 and rp == 0.0:
            self._rate_t_prev = None
            return
        now = time.monotonic()
        if self._rate_t_prev is None:
            self._rate_t_prev = now
            return
        dt = min(0.1, now - self._rate_t_prev)
        self._rate_t_prev = now
        yaw, pitch = self._gimbal
        if self._follow:
            self._az_target = _wrap180(self._az_target + ry * dt)
            npitch = clamp_gimbal(yaw, pitch + rp * dt)[1]
            if rp > 0:
                self._gimbal_blocked['pitch+'] = npitch == pitch
                if npitch != pitch:
                    self._gimbal_blocked['pitch-'] = False
            elif rp < 0:
                self._gimbal_blocked['pitch-'] = npitch == pitch
                if npitch != pitch:
                    self._gimbal_blocked['pitch+'] = False
            if npitch != pitch:
                self._gimbal = (yaw, npitch)
                self._gimbal_dirty = True
            return
        nyaw, npitch = clamp_gimbal(yaw + ry * dt, pitch + rp * dt)
        # 限位拒绝标记: 该方向被夹住 => 标红; 有实际动作 => 清对向红
        if ry > 0:
            self._gimbal_blocked['yaw+'] = nyaw == yaw
            if nyaw != yaw:
                self._gimbal_blocked['yaw-'] = False
        elif ry < 0:
            self._gimbal_blocked['yaw-'] = nyaw == yaw
            if nyaw != yaw:
                self._gimbal_blocked['yaw+'] = False
        if rp > 0:
            self._gimbal_blocked['pitch+'] = npitch == pitch
            if npitch != pitch:
                self._gimbal_blocked['pitch-'] = False
        elif rp < 0:
            self._gimbal_blocked['pitch-'] = npitch == pitch
            if npitch != pitch:
                self._gimbal_blocked['pitch+'] = False
        if (nyaw, npitch) != (yaw, pitch):
            self._gimbal = (nyaw, npitch)
            self._gimbal_dirty = True   # RPC 下发由节流周期负责

    def _anim_tick(self):
        """回中动画推进一步 (仅云台线程调用, 须持锁): 以恒定角速度朝目标
        (0,0) 推进, 时间驱动不受帧率影响; 到位后停用动画并持久化。"""
        if self._gimbal_anim is None:
            return
        ty, tp = self._gimbal_anim
        yaw, pitch = self._gimbal
        dy, dp = ty - yaw, tp - pitch
        dist = math.hypot(dy, dp)
        now = time.monotonic()
        dt = min(0.1, now - self._anim_t_prev)
        self._anim_t_prev = now
        max_d = config.GIMBAL_SLEW_RATE_DEG_S * dt
        if dist <= max_d:
            self._gimbal = (ty, tp)
            self._gimbal_anim = None
        else:
            s = max_d / dist
            self._gimbal = clamp_gimbal(yaw + dy * s, pitch + dp * s)
        self._gimbal_dirty = True

    def _follow_tick(self, client):
        """航向跟随的相机侧半环 (仅云台线程, 每拍调用): 测量机体航向并
        缓存; 跟随中把存储偏航反卷回世界方位锚点 (yaw = wrap(锚点 − 航向))
        —— 视线钉在世界系, 航向转动时 yaw 自然收敛回 0, 机头追上视线即停。
        限位在此按"最大超前角"收口 (锚点同步回拉), 限位标红方向随之更新。
        返回机体姿态四元数 (供 _apply_gimbal 复用, 省一次 RPC);
        测量失败返回 None。"""
        try:
            q_b = client.simGetGroundTruthKinematics(VEHICLE).orientation
        except Exception:
            return None
        yaw_b, _ = quat_yaw_el(q_b)
        with self.lock:
            self._veh_yaw_t = time.monotonic()
            if not self._follow:
                return q_b
            if self._az_target is None:
                # 进入跟随后的首次测量: 快照锚点 = 航向 + 当前偏航 (不跳变)
                self._az_target = _wrap180(yaw_b + self._gimbal[0])
            elif (self._user_yaw_t is not None
                  and time.monotonic() - self._user_yaw_t
                  <= config.GIMBAL_FOLLOW_USER_HOLD_SEC
                  and self._veh_yaw_prev is not None):
                # 用户打偏航杆转机头: 锚点随实测航向增量平移 —— 相机偏航
                # 变刚性 (画面跟转), 松杆后锚点停在新视线方向; 误差两侧
                # 同移保持不变, 外环不再把机头拉回。增量取 wrap 差 (防
                # 359→0 跳变); 打杆期间外环也让位 (teleop_panel)
                self._az_target = _wrap180(
                    self._az_target
                    + _wrap180(yaw_b - self._veh_yaw_prev))
            self._veh_yaw_prev = yaw_b
            yaw = _wrap180(self._az_target - yaw_b)
            limit = config.GIMBAL_YAW_LIMIT_DEG
            if abs(yaw) > limit:
                # 超前角到限位: yaw 夹住, 锚点回拉随航向 (持续追到顶
                # = 机头以最大角速率追视线, 视线方位不再前伸), 标红该向
                self._gimbal_blocked['yaw+' if yaw > 0 else 'yaw-'] = True
                self._gimbal_blocked['yaw-' if yaw > 0 else 'yaw+'] = False
                yaw = limit if yaw > 0 else -limit
                self._az_target = _wrap180(yaw_b + yaw)
            else:
                self._gimbal_blocked['yaw+'] = False
                self._gimbal_blocked['yaw-'] = False
            if yaw != self._gimbal[0]:
                self._gimbal = (yaw, self._gimbal[1])
                self._gimbal_dirty = True
        return q_b

    def _reset_gimbal(self):
        """连接建立时云台状态复核 (重连/重 play): 复位跟随/动画/限位标红,
        置 dirty 触发重下发。安装基准、云台角与下发记录保留 —— RPC 断连
        不会移动 UE 端相机, 保留的状态仍与实际一致, 重下发即恢复原视角
        (无跳变, 降落中云台姿态也不会中断); 若按旧逻辑清基准重采, 会把
        重连前已被转动的相机位姿污染进安装基准 —— 错位视觉上无感, 却让
        跟随收敛到偏移的航向。UE 重 play 场景: 相机自己回到安装位, 保留
        的基准仍然正确, 重下发同样一致。航向跟随一并退出 (重连后航向/
        锚点均失效, 界面按钮随状态同步弹起)。"""
        with self.lock:
            self._gimbal_anim = None
            self._gimbal_rate = (0.0, 0.0)
            self._gimbal_dirty = True
            self._gimbal_blocked = {k: False
                                    for k in self._gimbal_blocked}
            self._follow = False
            self._follow_src = 'ui'
            self._algo_cmd_t = None
            self._az_target = None
            self._veh_yaw_t = None
            self._veh_yaw_prev = None
            self._user_yaw_t = None

    def _apply_gimbal(self, client, yaw_deg, pitch_deg, cam_cur, q_b=None):
        """把云台角度经 simSetCameraPose 同步写到 AirSim 相机。
        q_b: 预取的机体姿态四元数 (航向跟随每拍已测, 复用省一次 RPC);
        None 则自行获取。

        只持续下发当前相机; 其余相机仅在首次同步时下发一次 (供 UE 重玩
        检测), 切到它时指纹含相机名必失配, 下一拍立即补发。飞行中每拍
        只有 1 次 kinematics + 1 次 simSetCameraPose, 且走云台专用 RPC
        连接, 不阻塞拉图。

        simSetCameraPose 按"机体相对位姿"解释输入, 若把云台角直接叠加在
        机体系安装姿态上, 偏航绕的是机体 z 轴 (斜轴), 连续步进会沿斜锥面
        进动、视角甩偏。改为世界系稳定云台: 目标视线用 (方位角, 仰角) 在
        世界系直接重建 —— 方位 = 机体前向方位 + 云台偏航 + 安装偏航,
        仰角 = 安装仰角 + 云台俯仰, roll=0。视线钉在世界系, 方向键与机体
        姿态完全无关。"""
        try:
            if q_b is None:
                q_b = client.simGetGroundTruthKinematics(VEHICLE).orientation
            # 指纹 = (机体姿态, 云台角, 当前相机), 姿态按四元数夹角容差判定
            # (|dot| = cos(θ/2)): 悬停抖动在容差内跳过下发, 真实机动照常补偿
            last = self._gimbal_apply_key
            if last is not None:
                lq, lyaw, lpitch, lcam = last
                dot = abs(q_b.x_val * lq.x_val + q_b.y_val * lq.y_val
                          + q_b.z_val * lq.z_val + q_b.w_val * lq.w_val)
                if (lcam == cam_cur and lyaw == yaw_deg and lpitch == pitch_deg
                        and dot >= math.cos(math.radians(APPLY_TOL_DEG) / 2.0)):
                    return
            applied = {}
            for cam in DEFAULT_CAMERAS:
                if cam != cam_cur and cam in self._gimbal_last:
                    continue   # 非当前相机且已同步过: 切到它时才会再下发
                if cam not in self._gimbal_mount:
                    try:
                        self._gimbal_mount[cam] = capture_mount(client, cam)
                    except MountNotReady:
                        # 基准未就绪 (刚连上时机体弹跳/运动学未就绪): 整批
                        # 跳过本拍, 靠稳定模式的周期重下发下一拍重采 ——
                        # 污染的基准会把相机错位到机身外, 云台一动拍到机体
                        return
                    except Exception as e:
                        self._gimbal_mount[cam] = None   # 该相机不可用, 跳过
                        self.log(f'云台: 获取 {cam} 安装位姿失败, 跳过: {e}')
                        continue
                mount = self._gimbal_mount[cam]
                if mount is None:
                    continue
                pos, ori = mount
                # 世界系重建目标姿态 (方位/仰角, roll=0), 再反算回机体系下发
                yaw_b, _ = quat_yaw_el(q_b)
                yaw_m, el_m = quat_yaw_el(ori)
                q_des_world = quat_mul(
                    gimbal_yaw_quat(yaw_b + yaw_deg + yaw_m),
                    gimbal_pitch_quat(el_m + pitch_deg))
                q_cmd = quat_mul(quat_conj(q_b), q_des_world)
                try:
                    client.simSetCameraPose(cam, airsim.Pose(pos, q_cmd), VEHICLE)
                    applied[cam] = q_cmd
                except Exception as e:
                    self.log(f'云台: {cam} 下发失败: {e}')
            if applied:
                # 下发记录仅在成功后更新 (供切相机后的补发判定);
                # 指纹同步更新 (悬停抖动容差内据此跳过下发)
                with self.lock:
                    self._gimbal_last = applied
                    self._gimbal_apply_key = (q_b, yaw_deg, pitch_deg, cam_cur)
        except Exception as e:
            # 连接级失败上抛给云台线程: 复位连接退避 1s 重连 (不在
            # 节流频率下反复失败刷屏)
            self.log(f'云台控制失败: {e}')
            raise
        finally:
            self._gimbal_dirty = False

    def _apply_fov(self, client, cam):
        """把全局视场角经 simSetCameraFov 写入当前相机 (走云台专用 RPC)。
        只在 dirty 时对齐一次, 不逐帧重发; 从未调过 (None) 则查询相机
        真实 FOV 采纳为当前值 (见 _adopt_camera_fov), 不改动相机。"""
        with self.lock:
            fov = self._fov
            adopt = fov is None
        if adopt:
            # 采纳失败自动重试 (5s 退避): 此前查询前就清 dirty, 一次失败
            # 整个会话不再采纳/发布 FOV 话题, 算法节点只能用 yaml 默认值,
            # 距离解算全程偏差 (表现为"拖一次视场滑杆才准")
            if time.monotonic() - self._fov_adopt_t < 5.0:
                return
            self._fov_adopt_t = time.monotonic()
            self._adopt_camera_fov(client, cam)
            return
        try:
            client.simSetCameraFov(cam, fov, VEHICLE)
            self._fov_dirty = False
        except Exception as e:
            self.log(f'视场角下发失败: {e}')
            raise   # _fov_dirty 保持 True, 云台线程重连后自动重试

    def _adopt_camera_fov(self, client, cam):
        """从未调整过 FOV 时查询当前相机真实 FOV (settings.json 配置值)
        采纳为当前值: 只读不改相机, 不落盘 (下次连接按当时真实值重取,
        不拿旧值覆盖 settings.json 的改动)。采纳后 get_fov_applied 有值,
        视场角话题即发布, ArUco 等算法节点启动即同步, 无需先拖一次滑杆;
        界面滑杆也直接显示真实 FOV。"""
        try:
            info = client.simGetCameraInfo(cam, VEHICLE)
            fov = float(info['fov'] if isinstance(info, dict) else info.fov)
        except Exception as e:
            self.log(f'视场角查询失败 ({cam}): {e}')
            return
        if not 0.0 < fov <= 180.0:
            return
        with self.lock:
            if self._fov is None:
                self._fov = fov
        self.log(f'视场角: 采纳相机 {cam} 当前 FOV {fov:.0f}° (同步算法节点)')

    # ---------- 后台线程 ----------

    def run(self):
        """云台/FOV/航向跟随后台线程 (独立 RPC 连接, rationale 见模块 docstring)。

        每 GIMBAL_LOOP_HZ 一拍: 心跳超时检查 → 摇杆步进/回中动画 →
        航向跟随反卷 → 周期重下发云台 → FOV 对齐。下发频率仍由
        GIMBAL_HOLD_HZ 节流, 悬停抖动由 _apply_gimbal 指纹容差拦截。"""
        client = None
        while self.running:
            try:
                if not self._connected:
                    # 抓取线程的探针确认连通后才建连接, 避免与探针重复
                    time.sleep(0.2)
                    continue
                with self.lock:
                    cam = self._camera
                    airsim_src = self._airsim_src
                if not airsim_src:
                    time.sleep(0.05)
                    continue
                if client is None:
                    client = airsim.VehicleClient(
                        ip=self.host, port=self.port, timeout_value=3.0)
                t_start = time.monotonic()
                self._follow_timeout_check()
                now = time.monotonic()
                with self.lock:
                    self._rate_tick()    # 摇杆连续转动: 每拍按角速度推进
                    self._anim_tick()    # 回中动画: 每拍推进并置 dirty
                    gim_dirty, gim = self._gimbal_dirty, self._gimbal
                    stab = self._gimbal_stab
                    follow = self._follow
                # 云台角只在下发瞬间按当时机体姿态换算, 机体姿态一变记忆
                # 角就与实际视角脱钩, 故除 dirty 立即下发外还周期重下发;
                # 稳定模式下居中角 (0,0) 也持续重下发 (视角保持水平),
                # 非稳定模式居中=刚性挂载 (省 RPC)
                apply_due = gim_dirty or (
                    config.GIMBAL_HOLD_HZ > 0
                    and (stab or gim != (0.0, 0.0))
                    and now - self._gimbal_apply_t
                    >= 1.0 / config.GIMBAL_HOLD_HZ)
                qb = None
                if follow and (apply_due
                               or now - (self._veh_yaw_t or 0.0) >= 0.2):
                    # 航向跟随: 测航向 + 反卷云台偏航 (RPC 在锁外);
                    # 反卷改写 gim 后重读, 本拍下发即用最新角
                    qb = self._follow_tick(client)
                    with self.lock:
                        gim_dirty, gim = self._gimbal_dirty, self._gimbal
                    apply_due = apply_due or gim_dirty
                if apply_due:
                    self._apply_gimbal(client, *gim, cam, q_b=qb)
                    self._gimbal_apply_t = now
                # 视场角: dirty 时按当前相机记忆值对齐一次
                if self._fov_dirty:
                    self._apply_fov(client, cam)
                time.sleep(max(0.0, 1.0 / GIMBAL_LOOP_HZ
                               - (time.monotonic() - t_start)))
            except Exception as e:
                self.log(f'云台线程: 连接中断: {e}，1 秒后重连...')
                client = None
                time.sleep(1.0)
