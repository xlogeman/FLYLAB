#!/usr/bin/env python3
"""飞行控制 —— AirSim 飞控命令的独立 RPC 会话线程。

解锁/起飞/降落/悬停/返航/上锁, 经 AirSim API → MAVLink 控制通道 (14580)
下发 PX4, 与 offboard tracker (mavros/uxrce) 相互独立。

为什么独立于 camera_grabber: 飞控与相机抓取是两个功能域。抓图线程有
自己的重连节奏 (UE 未 play 时探针循环) 和拉流负载, 飞控命令不应寄生
其中 —— 连通状态、命令执行成败在这里自治, 抓图卡顿/切源/重连互不影响。

线程模型: 守护线程 0.5s 一拍 —— ping 维护连通状态 (界面按钮门控依据,
状态迁移才打日志不刷屏) + 处理命令队列。客户端短超时 (2s), ping 不通
快速失败, 绝不长阻塞。takeoff/land/hover/gohome 为异步指令: 下发即
返回不等待完成, 实际动作由 PX4 执行 (结果看 QGC / mavros state)。

注意: 飞控方法 (takeoff/land/...) 在 MultirotorClient 上, 抓图用的
VehicleClient 没有 (armDisarm 两边都有) —— 本模块统一用 MultirotorClient。
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

VEHICLE = 'PX4'

_LABELS = {'arm': '解锁', 'disarm': '上锁', 'takeoff': '起飞',
           'land': '降落', 'hover': '悬停', 'gohome': '返航'}


class VehicleController(threading.Thread):
    """飞控命令执行线程: 命令队列 + 连通状态维护。"""

    def __init__(self, log, host, port):
        super().__init__(daemon=True)
        self.log = log          # tee logger (.info/.error, 自动分流界面日志框)
        self.host, self.port = host, int(port)
        self._lock = threading.Lock()
        self._cmd = None        # 待执行命令 key (界面 request 置入)
        self._connected = False
        self._flying = False    # 已解锁且在空中 (视觉反控门控依据)
        self._fly_streak = 0    # 连续报"空中"的拍数 (过滤上电瞬态误报)
        # QGC 式遥测 (0.5s 心跳顺带采样 kinematics, 飞行中积分行程/时长):
        # alt=相对起飞点高度 hs=水平速度 vs=垂直速度 hdg=航向 dist=累计行程
        # ftime=飞行时间(秒) flying=是否在空中
        self._tele = {'alt': 0.0, 'hs': 0.0, 'vs': 0.0, 'hdg': 0.0,
                      'dist': 0.0, 'ftime': 0.0, 'flying': False}
        self._tele_t = 0.0        # 上次遥测采样时刻 (monotonic, 0=未采样)
        self._tele_prev = None    # 上次采样水平位置 (x, y)
        self._z_ground = 0.0      # 起飞点 z (NED), 高度基准
        self._track = []          # 航迹 [(north, east), ...] 米, 飞行中每拍记录
        self._home = None         # 起飞点水平位置 (north, east)
        self._client = None     # 持久 RPC 客户端 (ping 失败时重建)
        self.running = True

    # ---------- 界面线程调用 ----------

    def is_connected(self):
        """RPC 是否连通 (界面按钮门控依据)。"""
        with self._lock:
            return self._connected

    def is_flying(self):
        """已解锁且在空中 (视觉反控门控依据); 未连通/查询失败为 False。"""
        with self._lock:
            return self._flying

    def get_telemetry(self):
        """最近一次采样遥测 (dict, 界面 33ms 轮询读, 采样随 0.5s 心跳)。"""
        with self._lock:
            return dict(self._tele)

    def get_track(self):
        """航迹 (地图控件用): (轨迹点列表拷贝, 起飞点 or None)。"""
        with self._lock:
            return list(self._track), self._home

    def request(self, key):
        """请求执行飞控命令, 置入队列立即返回 (执行在本线程)。"""
        with self._lock:
            self._cmd = key

    def stop(self):
        self.running = False

    # ---------- 线程体 ----------

    def run(self):
        while self.running:
            t0 = time.monotonic()
            with self._lock:
                cmd, self._cmd = self._cmd, None
            self._ping()
            if cmd is not None:
                self._exec(cmd)
            time.sleep(max(0.0, 0.5 - (time.monotonic() - t0)))

    def _ping(self):
        """维持 RPC 连接并更新连通状态 (状态迁移才打日志, 不刷屏)。"""
        try:
            if self._client is None:
                self._client = airsim.MultirotorClient(
                    ip=self.host, port=self.port, timeout_value=2.0)
            self._client.ping()
        except Exception:
            self._client = None
            with self._lock:
                was, self._connected = self._connected, False
            if was:
                self.log.warning('飞行控制: AirSim 连接断开')
            return
        with self._lock:
            was, self._connected = self._connected, True
        if not was:
            self.log.info(f'飞行控制: 已连接 AirSim {self.host}:{self.port}')
        self._poll_flying()

    def _poll_flying(self):
        """随心跳低频查询飞行状态 (离地, 供视觉反控门控) + 采集遥测六值。
        注: 不用 st.rc.armed —— PX4 SITL 经 MAVLink 通道解锁时 RCData.armed
        不可靠 (不走模拟遥控器通道, 恒为 False)。改用 LandedState 判离地:
        Landed=在地面, Takeoff/Landing/Flying=离空。状态迁移打一条日志;
        单次查询失败保持上次状态 (RPC 抖动不产生假迁移), 下个心跳恢复。
        遥测: 高度=相对起飞点 (起飞瞬间记 z 基准), 垂直速度 NED 上为正,
        航向 0-360°; 飞行中按采样间隔积分水平行程与飞行时长 (0.5s 采样,
        行程为折线近似, 略短于实际轨迹), 起飞瞬间清零按次累计, 落地保留;
        高度/速度落地后显示 0, 航向始终跟随。"""
        try:
            st = self._client.getMultirotorState()
            # ready 标识 PX4 固件链路已建立 (is_ready_ 连接成功才置位)。
            # PX4 没跑/启动中时 landed_state 是默认值 (landed=false → 误报
            # Flying 且永远不会有 EXTENDED_SYS_STATE 纠正), 必须 False 才可信
            raw_flying = (st.landed_state != airsim.LandedState.Landed
                          and bool(st.ready))
        except Exception:
            return   # 单次查询失败不动状态不打日志 (避免抖动出假迁移), 下个心跳恢复
        # PX4 上电瞬态: landed 默认 false, AirSim 在收到首条 EXTENDED_SYS_STATE
        # 前把地面飞机误报为 Flying (~1~2s) —— 报"空中"需连续两拍确认才采信,
        # 报"地面"立即采信 (落地宁早勿晚), 瞬态单拍即被吞掉
        if raw_flying:
            self._fly_streak += 1
            flying = self.is_flying() or self._fly_streak >= 2
        else:
            self._fly_streak = 0
            flying = False
        kin = st.kinematics_estimated
        pos, vel, q = kin.position, kin.linear_velocity, kin.orientation
        # 物理异常/仿真毛刺时 AirSim 可能回 NaN/Inf 坐标 —— 整拍丢弃:
        # NaN 进航迹/遥测会沿 渲染→paintEvent 抛异常→PyQt abort 的链路
        # 带崩整个界面 (曾致 GUI 静默退出、托管进程无人回收)
        if not (math.isfinite(pos.x_val) and math.isfinite(pos.y_val)
                and math.isfinite(pos.z_val)
                and math.isfinite(vel.x_val) and math.isfinite(vel.y_val)
                and math.isfinite(vel.z_val)):
            self.log.warning('飞行控制: 丢弃一拍异常遥测 (坐标非有限值, 仿真物理异常?)')
            return
        now = time.monotonic()
        with self._lock:
            was, self._flying = self._flying, flying
        if was != flying:
            self.log.info(
                f'飞行控制: 飞行状态 -> {"空中 (反控可用)" if flying else "地面"}')
        if flying and not was:   # 起飞瞬间: 记高度/起飞点基准, 行程/时长/航迹清零
            self._z_ground = pos.z_val
            self._home = (pos.x_val, pos.y_val)
            self._track = []
            self._tele_t = now
            self._tele_prev = (pos.x_val, pos.y_val)
        tele = dict(self._tele)
        tele['flying'] = flying
        if flying:
            dt = max(0.0, now - self._tele_t) if self._tele_t else 0.0
            if self._tele_prev is not None and dt > 0:
                tele['dist'] = self._tele['dist'] + math.hypot(
                    pos.x_val - self._tele_prev[0],
                    pos.y_val - self._tele_prev[1])
                tele['ftime'] = self._tele['ftime'] + dt
            self._tele_prev = (pos.x_val, pos.y_val)
            self._tele_t = now
            tele['alt'] = max(0.0, self._z_ground - pos.z_val)   # NED z 下为正
            tele['hs'] = math.hypot(vel.x_val, vel.y_val)
            tele['vs'] = -vel.z_val                              # 上为正
        else:
            self._tele_prev = None
            self._tele_t = 0.0
            tele['alt'] = tele['hs'] = tele['vs'] = 0.0
        # 航迹记录 (NED x=北 y=东), 供地图控件绘制航线: 地面连通也记录
        # (地图即刻显示所在点, 不用等起飞), 起飞瞬间清零重新累计飞行航迹
        if not self._track or self._track[-1] != (pos.x_val, pos.y_val):
            self._track.append((pos.x_val, pos.y_val))
            if len(self._track) > 2000:
                del self._track[:500]   # 超上限丢最旧一批 (省得每拍裁剪)
        tele['hdg'] = (math.degrees(math.atan2(
            2.0 * (q.w_val * q.z_val + q.x_val * q.y_val),
            1.0 - 2.0 * (q.y_val * q.y_val + q.z_val * q.z_val))) + 360.0) % 360.0
        with self._lock:
            self._tele.update(tele)

    def _exec(self, key):
        """执行命令: 短超时客户端 + ping 不通快速失败, 绝不长阻塞。"""
        label = _LABELS.get(key, key)
        try:
            if not self.is_connected():
                raise RuntimeError('AirSim RPC 未连接 (UE 未 play?)')
            mc = self._client
            if key == 'arm':
                mc.armDisarm(True, VEHICLE)
            elif key == 'disarm':
                mc.armDisarm(False, VEHICLE)
            elif key == 'takeoff':
                mc.takeoffAsync(10, VEHICLE)
            elif key == 'land':
                mc.landAsync(60, VEHICLE)
            elif key == 'hover':
                mc.hoverAsync(VEHICLE)
            elif key == 'gohome':
                # 注: 签名是 goHomeAsync(timeout_sec, vehicle_name) —— 第一参
                # 是超时秒数, 之前误传 vehicle_name 导致 RPC 失败无声无息
                mc.goHomeAsync(60, VEHICLE)
            else:
                self.log.error(f'飞行控制: 未知命令 {key}')
                return
            self.log.info(f'飞行控制 [{label}]: 已下发')
        except Exception as e:
            self.log.error(f'飞行控制 [{label}] 失败: {e}')
