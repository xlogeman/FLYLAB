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
        """随心跳低频查询飞行状态 (离地), 供视觉反控门控。
        注: 不用 st.rc.armed —— PX4 SITL 经 MAVLink 通道解锁时 RCData.armed
        不可靠 (不走模拟遥控器通道, 恒为 False)。改用 LandedState 判离地:
        Landed=在地面, Takeoff/Landing/Flying=离空。状态迁移打一条日志
        (排查门控问题), 查询失败视为不在地面流程 (下次心跳恢复)。"""
        try:
            st = self._client.getMultirotorState()
            flying = st.landed_state != airsim.LandedState.Landed
        except Exception:
            return   # 单次查询失败不动状态不打日志 (避免抖动出假迁移), 下个心跳恢复
        with self._lock:
            was, self._flying = self._flying, flying
        if was != flying:
            self.log.info(
                f'飞行控制: 飞行状态 -> {"空中 (反控可用)" if flying else "地面"}')

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
                mc.goHomeAsync(VEHICLE)
            else:
                self.log.error(f'飞行控制: 未知命令 {key}')
                return
            self.log.info(f'飞行控制 [{label}]: 已下发')
        except Exception as e:
            self.log.error(f'飞行控制 [{label}] 失败: {e}')
