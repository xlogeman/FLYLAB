#!/usr/bin/env python3
"""AirSim 相机抓取器 —— 直连 RPC 拉图 + 本机 V4L2 摄像头, 与界面解耦。

被 camera_view (独立调试) 和 ground_station 共用。只负责视频源抓取与
帧提交 (多 worker 并发 RPC、本机/文件源、数字变焦裁切、转发挂钩);
软件云台/视场角/航向跟随的状态机与独立 RPC 线程在 gimbal_controller.py,
本类持有其实例并把公有接口原样委托出去 (界面与主窗口无感)。

支撑模块:
  gimbal_controller.py 云台控制器 (状态机 + 独立 RPC 线程, 世界系稳定)
  quat_math.py         四元数数学工具 (纯函数)
  gimbal.py            云台限位/状态持久化/安装基准采集 (纯函数)
  camera_sources.py    本机摄像头/视频文件设备 IO (colorize_depth 也在此)
"""
import threading
import time

import numpy as np

try:
    import cosysairsim as airsim
except ImportError:
    import sys
    sys.path.insert(0, '/root/flylab/Cosys-AirSim/PythonClient')
    import cosysairsim as airsim

from . import config
from .camera_sources import LocalFileReader, cv2
# DEFAULT_CAMERAS 在 gimbal_controller (云台逐相机下发/首拍同步用),
# 此处再导出供 camera_widget 源菜单等引用方使用
from .gimbal_controller import (DEFAULT_CAMERAS, VEHICLE, GimbalController)

GRAB_HZ = 60        # AirSim 拉取频率上限 (实际帧率 = min(该值, 1/RPC往返延迟))
# AirSim 拉图流水线 worker 数: 每个 worker 独立 RPC 连接并发拉图。
# 单连接每帧串行等完整 RPC 往返 (实测 55~75ms -> 13~18fps), 帧率 =
# 1000/RPC; 多连接让"上一帧回传/解码"与"下一帧服务端出图"重叠, 服务端
# rpclib 多线程处理并发请求 (云台独立连接已验证可行), 有效帧率提升 ~N 倍。
# 2 worker 实测 RPC 79ms -> 源头 25.7fps 封顶, 算法端输出仅 ~17fps;
# 提到 3 worker (~37fps 源头) 对冲成对帧坍缩损耗
AIRSIM_FETCH_THREADS = 3
LOCAL_HZ = 30       # 本机摄像头拉取频率上限

DEFAULT_CAMERA = 'front_cam'   # 启动默认源 (AirSim)
LOCAL_PREFIX = 'local:'   # 本机摄像头在下拉框里的条目前缀, 如 local:/dev/video0
FILE_PREFIX = 'file:'     # 视频文件源前缀, 如 file:/path/to/demo.mp4 (循环播放)


def _is_airsim_src(name):
    """源名 -> 是否 AirSim 相机 (本机摄像头/视频文件无云台与 FOV)。"""
    return not name.startswith((LOCAL_PREFIX, FILE_PREFIX))


def _to_np(data, dtype):
    """AirSim msgpack 返回的图像数据可能是 bytes(bin 编码)或 list(array 编码)，
    统一转成 numpy 数组。"""
    if isinstance(data, np.ndarray):
        return data
    if isinstance(data, list):
        return np.asarray(data, dtype=dtype)
    return np.frombuffer(data, dtype=dtype)


class Grabber(threading.Thread):
    """后台线程：以 GRAB_HZ/LOCAL_HZ 持续拉取当前相机的最新一帧。
    云台/FOV/航向跟随方法为 gimbal_controller.GimbalController 的委托
    (对外接口与拆分前一致)。"""

    def __init__(self, log, host, port):
        super().__init__(daemon=True)
        self.log = log
        self.host, self.port = host, int(port)
        self.lock = threading.Lock()
        self.camera = DEFAULT_CAMERA
        self.frame = None      # (np.ndarray, is_depth)
        self._gen = 0          # 切源代数: 阻塞读取期间切源, 旧源的帧作废
        self.fps = 0.0
        self.n, self.t_prev = 0, time.time()
        # 场景帧提交后回调 (O(1) 引用, 由 main_window 挂到转发器): 转发
        # 与 GUI 轮询解耦 —— 多 worker 成对回包时 33ms 轮询采样会漏采近
        # 一半 (实测提交 ~11/s vs 抓取 ~21/s), 见 _commit 内说明
        self.forward_hook = None
        # 多 worker 拉图: 连接纪元计数 (0->1 触发重连初始化)
        self._conn_gate = threading.Lock()
        self._conn_count = 0          # 已连接的 worker 数
        self._conn_last_log = 0.0     # 上次"等待 AirSim"日志时刻 (10s 节流)
        # 连接纪元 (0->1) 会话重置钩子: 全体 worker 断过再连上视为新会话
        # (切机型/UE 重启), 界面层据此清空旧飞机残留状态。由 camera_widget
        # 挂到 Qt 信号 (抓取线程只调 emit, 跨线程投递到 GUI 线程执行)
        self.session_hook = None
        # 本机摄像头/视频文件设备 IO (见 camera_sources.LocalFileReader;
        # 设备仅在抓取线程访问, 播放控制标志位由界面线程设置)
        self.reader = LocalFileReader(log)
        # 软件云台/FOV/航向跟随: 状态机 + 独立 RPC 线程 (见 gimbal_controller)
        self.gimbal = GimbalController(log, host, port, DEFAULT_CAMERA)
        # 数字变焦 (抓取层裁切, 图传/转发/标注同一像素系);
        # _raw 留未变焦原帧, 档位/平移变化时重提交。
        # _pan 为裁切窗口中心相对图像中心的偏移 (原始帧像素, 画面跟手:
        # 窗口向拖动反方向移动), 裁切时夹在图内
        self._zoom = 1.0
        self._pan = (0.0, 0.0)
        self._raw = None
        # PNG 压缩传输 (config.DISPLAY_SCENE_COMPRESS); 服务端不支持时回退原始传输
        self._scene_compress = config.DISPLAY_SCENE_COMPRESS and cv2 is not None
        self.running = True
        self._airsim_ok = False     # AirSim RPC 是否已连通 (云台可用性依据)

    def set_camera(self, name):
        with self.lock:
            self.camera = name
            self._gen += 1      # 旧源读取阻塞期间的帧提交时被代数拒绝
            self.frame = None   # 丢弃旧相机画面，避免切换时闪现错图
        self.gimbal.on_camera_changed(name, _is_airsim_src(name))

    def stop(self):
        self.running = False
        self.gimbal.stop()

    def release_local(self):
        """释放本机摄像头设备 (须在抓取线程 join 之后调用)。"""
        self.reader.release()

    def get(self):
        with self.lock:
            return self.frame, self.camera, self.fps

    # ---------- 视频文件播放控制 (界面线程调用, 标志位由抓取线程处理) ----------

    @property
    def file_paused(self):
        return self.reader.paused

    def set_file_paused(self, paused):
        self.reader.paused = paused

    def stop_file(self):
        """停止播放: 回到第 0 帧并暂停。"""
        self.reader.stop_flag = True

    def set_file_position(self, frac):
        """跳转到相对位置 (0..1)，暂停中也可用。"""
        self.reader.seek = max(0.0, min(1.0, frac))

    @property
    def file_speed(self):
        return self.reader.speed

    def set_file_speed(self, speed):
        """设置播放倍速 (抓取线程按 原始帧间隔/倍速 重排帧节拍)。"""
        self.reader.speed = max(0.1, speed)

    def get_progress(self):
        """(当前帧, 总帧数, 是否暂停, 文件fps)；非文件源返回 None。"""
        with self.lock:
            if self.camera.startswith(FILE_PREFIX):
                return self.reader.progress()
        return None

    # ---------- 软件云台/视场角/航向跟随 (委托 GimbalController) ----------

    def gimbal_set_rate(self, yaw_rate, pitch_rate):
        """云台连续转动角速度 (°/s, 摇杆控制), 见 GimbalController。"""
        self.gimbal.gimbal_set_rate(yaw_rate, pitch_rate)

    def gimbal_reset(self):
        """云台回中 (平滑动画), 见 GimbalController。"""
        self.gimbal.gimbal_reset()

    def gimbal_pitch_bottom(self):
        """视觉反控前自动摆位 (俯仰转垂直向下), 见 GimbalController。"""
        self.gimbal.gimbal_pitch_bottom()

    def airsim_connected(self):
        """AirSim RPC 是否已连通 (断线/未 play 时为 False)。"""
        with self.lock:
            return self._airsim_ok

    def get_gimbal(self):
        """当前云台角度 (yaw, pitch)；非 AirSim 源返回 None。"""
        return self.gimbal.get_gimbal()

    def get_gimbal_down_deg(self):
        """当前云台俯角 (度, 向下正: 水平=0 垂直下=-pitch 90)；非 AirSim
        源返回 None。俯视角度输入框初值同步用 (云台 pitch+ 为抬头)。"""
        g = self.get_gimbal()
        return None if g is None else -g[1]

    def get_gimbal_blocked(self):
        """会话内被限位卡住的方向 (界面标红用)。"""
        return self.gimbal.get_gimbal_blocked()

    def get_gimbal_stab(self):
        """云台稳定 (follow 模式) 是否开启。"""
        return self.gimbal.get_gimbal_stab()

    def set_gimbal_stab(self, on):
        """开关云台稳定 (世界系重下发 vs 刚性挂载)。"""
        self.gimbal.set_gimbal_stab(on)

    def set_follow(self, on, source='ui'):
        """开关航向跟随 (云台偏航带动机头)。"""
        self.gimbal.set_follow(on, source=source)

    def clear_ui_follow(self):
        """仅退出 UI 来源的跟随 (交还遥控/异常复位)。"""
        self.gimbal.clear_ui_follow()

    def note_user_yaw(self, active):
        """遥控偏航杆活动标记 (外环定时器上报, 见 GimbalController)。"""
        self.gimbal.note_user_yaw(active)

    def set_algo_follow(self, on):
        """反控算法经 /flylab/gimbal_follow 请求/取消跟随。"""
        self.gimbal.set_algo_follow(on)

    def note_algo_cmd(self, yaw_rate, pitch_rate):
        """算法云台指令入口 (兼算法 follow 心跳)。"""
        self.gimbal.note_algo_cmd(yaw_rate, pitch_rate)

    def get_follow(self):
        """航向跟随是否开启 (界面按钮状态同步依据)。"""
        return self.gimbal.get_follow()

    def get_follow_error(self):
        """航向跟随误差 (度, 右+); 未跟随/数据过期返回 None。"""
        return self.gimbal.get_follow_error()

    def set_fov(self, deg):
        """设置全局视场角 (度), 下一拍经 RPC 下发。"""
        self.gimbal.set_fov(deg)

    def get_fov(self):
        """全局视场角 (度); 未调整过返回默认, 本机/文件源返回 None。"""
        return self.gimbal.get_fov()

    def get_fov_applied(self):
        """实际生效 FOV; 用户从未调整过返回 None (同步发布依据)。"""
        return self.gimbal.get_fov_applied()

    def get_cam_orientation(self):
        """当前相机的机体系姿态四元数; 非 AirSim 源返回 None。"""
        return self.gimbal.get_cam_orientation()

    # ---------- 数字变焦 (界面滚轮/双击/滑杆, 裁切由抓取线程在 _commit 做) ----------

    def set_zoom(self, factor):
        """设置数字变焦倍数 (1.0=原图, 上限 config.DISPLAY_ZOOM_MAX):
        立即用缓存的未变焦原帧重提交一帧, 暂停中的文件源也能即时看到;
        之后每帧按该倍数裁切。缩回 1.0 时平移复位。"""
        with self.lock:
            factor = max(1.0, min(config.DISPLAY_ZOOM_MAX, float(factor)))
            if self._zoom == factor:
                return
            self._zoom = factor
            if factor <= 1.0:
                self._pan = (0.0, 0.0)   # 回到全图, 平移无意义
            pan = self._pan
            raw, gen = self._raw, self._gen
        if raw is None:
            return
        arr, is_depth = raw
        if factor > 1.0:
            arr = self._zoom_crop(arr, factor, pan)
        with self.lock:
            if gen == self._gen:
                self.frame = (arr, is_depth)

    def get_zoom(self):
        with self.lock:
            return self._zoom

    def set_pan_delta(self, dx, dy):
        """变焦拖动平移: dx/dy 为显示画面坐标位移 (原图像素, 画面跟手)。
        显示帧是裁切内容放回原分辨率, 折算回原始裁切域需除以倍数;
        画面跟手 = 裁切窗口反向移动。平移后立即重提交一帧 (暂停的
        文件源同样即时看到); 未变焦时忽略。"""
        with self.lock:
            z = self._zoom
            if z <= 1.0:
                return
            px, py = self._pan
            self._pan = (px - dx / z, py - dy / z)
            pan = self._pan
            raw, gen = self._raw, self._gen
        if raw is None:
            return
        arr, is_depth = raw
        arr = self._zoom_crop(arr, z, pan)
        with self.lock:
            if gen == self._gen:
                self.frame = (arr, is_depth)

    @staticmethod
    def _zoom_crop(arr, z, pan):
        """按倍数 z 与窗口偏移 pan 裁切 1/z 区域并放回原分辨率
        (图传/转发/标注坐标不变)。pan 为窗口中心相对图像中心的原始
        像素偏移, 裁切时夹在图内 —— 缩放档位变化后无需另存钳制。"""
        h, w = arr.shape[:2]
        cw, ch = max(1, int(w / z)), max(1, int(h / z))
        mx, my = (w - cw) / 2.0, (h - ch) / 2.0   # 窗口中心可动半径
        px = max(-mx, min(mx, pan[0]))
        py = max(-my, min(my, pan[1]))
        x0 = int(round(w / 2.0 + px - cw / 2.0))
        y0 = int(round(h / 2.0 + py - ch / 2.0))
        x0, y0 = max(0, x0), max(0, y0)
        crop = np.ascontiguousarray(arr[y0:y0 + ch, x0:x0 + cw])
        if cv2 is None:          # 无 OpenCV 只裁不缩, 显示端等比放大兜底
            return crop
        return cv2.resize(crop, (w, h), interpolation=cv2.INTER_LINEAR)

    # ---------- 帧提交与抓取线程 ----------

    def _commit(self, arr, is_depth, gen):
        """写入最新帧并统计 fps。切源代数不符 (读取阻塞期间切了源) 时
        丢弃, 否则旧源最后一帧会在切源后回填。
        注: 多 worker 并发拉图不做"发出时刻"防乱序拒绝 —— 服务端串行
        处理 RPC, 后发请求的响应内容是处理时刻拷贝的最新渲染帧 (比先发
        的上一帧更新), 按发出时刻判旧会误杀近一半帧 (实测 28fps 拉回
        15fps 提交); 响应内容与其到达顺序基本一致, 直接按到达顺序提交。"""
        with self.lock:
            if gen != self._gen:
                return
            zoom = self._zoom
            pan = self._pan
            self._raw = (arr, is_depth)   # 未变焦原帧 (变焦档/平移变化时重提交)
        if zoom > 1.0:
            arr = self._zoom_crop(arr, zoom, pan)   # 裁切放锁外, 不阻塞 get()
        with self.lock:
            if gen != self._gen:
                return
            self.frame = (arr, is_depth)
        self.n += 1
        # 场景帧 (非深度) 提交后直接触发转发: 帧内容与界面显示一致 (变焦
        # 已裁切)。暂停的视频源无新提交即无转发, 语义与 GUI 轮询采样等价;
        # 算法节点未订阅时转发器自会跳过 publish
        if not is_depth and arr.ndim == 3:
            hook = self.forward_hook
            if hook is not None:
                hook(arr)
        now = time.time()
        if now - self.t_prev >= 1.0:
            self.fps = self.n / (now - self.t_prev)
            self.n, self.t_prev = 0, now

    def run(self):
        # 云台/FOV/航向跟随 RPC 走独立线程 (独立连接, rationale 见
        # gimbal_controller 模块 docstring)
        self.gimbal.start()
        # AirSim 拉图流水线: N 个 worker 各持独立 RPC 连接并发拉图 (见
        # AIRSIM_FETCH_THREADS 注释); 本机/文件源仍由本线程处理
        for i in range(AIRSIM_FETCH_THREADS):
            threading.Thread(target=self._airsim_fetch_loop, args=(i,),
                             daemon=True, name=f'airsim-fetch-{i}').start()
        while self.running:
            t_start = time.monotonic()
            with self.lock:
                cam = self.camera
                gen = self._gen
            if cam.startswith(LOCAL_PREFIX):    # 本机摄像头: 不经 AirSim
                arr = self.reader.read_local(cam[len(LOCAL_PREFIX):])
                if arr is not None:
                    self._commit(arr, False, gen)
                time.sleep(max(0.0, 1.0 / LOCAL_HZ - (time.monotonic() - t_start)))
                continue
            if cam.startswith(FILE_PREFIX):     # 视频文件: 按文件帧率循环播放
                arr = self.reader.read_file(cam[len(FILE_PREFIX):])
                if arr is not None:
                    self._commit(arr, False, gen)
                time.sleep(max(0.0, self.reader.interval - (time.monotonic() - t_start)))
                continue
            self.reader.close()                 # 切回 AirSim 相机时释放本机设备
            # AirSim 源的连接与拉取在 _airsim_fetch_* 线程, 这里只等待
            time.sleep(0.05)

    def _airsim_connect(self):
        """独立连接 (每 worker 一个)。探测必须用短超时探针: VehicleClient
        默认 RPC 超时 3600s, UE 未 play 时 ping 会把线程长期阻塞。
        返回 VehicleClient; 未连上返回 None (内部已节流重试)。
        连接纪元 (连接数 0->1) 执行一次云台复核/FOV 重采, N 个 worker
        先后连上只触发一次。"""
        try:
            probe = airsim.VehicleClient(
                ip=self.host, port=self.port, timeout_value=0.5)
            probe.ping()
        except Exception as e:
            # 未连接状态持续重试: 节流到每 10 秒一条, 不刷屏
            if time.monotonic() - self._conn_last_log > 10.0:
                self.log(f'等待 AirSim {self.host}:{self.port}: {e}')
                self._conn_last_log = time.monotonic()
            with self.lock:
                self._airsim_ok = False
            self.gimbal.set_connected(False)
            time.sleep(1.0)
            return None
        # 正式连接带 3s 超时: 默认 3600s 下 UE 卡顿会把拉图线程
        # 冻结成图传假死, 超时抛异常走重连
        client = airsim.VehicleClient(
            ip=self.host, port=self.port, timeout_value=3.0)
        with self._conn_gate:
            first = self._conn_count == 0
            self._conn_count += 1
        with self.lock:
            self._airsim_ok = True
        self.gimbal.set_connected(True)
        self.log(f'已连接 AirSim {self.host}:{self.port}')
        if first:
            # 云台状态复核 + FOV 重采 (云台侧回中重下发, 见
            # GimbalController.on_connected)
            self.gimbal.on_connected()
            # 通知界面层会话重置 (清航迹/遥测/标注/追踪等旧飞机残留)
            hook = self.session_hook
            if hook is not None:
                try:
                    hook()
                except Exception as e:
                    self.log(f'会话重置通知失败: {e}')
        return client

    def _airsim_fetch_loop(self, worker_id):
        """AirSim 拉图 worker: 独立 RPC 连接并发拉图 (见 AIRSIM_FETCH_THREADS)。
        循环: 读当前相机 -> simGetImages -> 解码 -> 提交 (带拍摄时刻,
        过期帧丢弃防乱序回包旧帧覆盖新帧)。"""
        client = None
        while self.running:
            try:
                with self.lock:
                    cam = self.camera
                    gen = self._gen   # 本轮帧的切源代数, 提交时校验
                if not _is_airsim_src(cam):
                    time.sleep(0.1)   # 本机/文件源由 run() 线程处理
                    continue
                if client is None:
                    client = self._airsim_connect()
                    if client is None:
                        continue
                t_start = time.monotonic()
                is_depth = 'depth' in cam.lower()
                # 云台/FOV/跟随 RPC 全部在 GimbalController 线程处理 (独立
                # 连接), 拉图循环只剩 simGetImages
                if is_depth:
                    # 深度必须用 DepthPlanar: Scene+float 读回会触发 D3D12
                    # 断言并返回 NaN/Inf
                    req = airsim.ImageRequest(
                        cam, airsim.ImageType.DepthPlanar, True, False)
                else:
                    # PNG 压缩传输 (体积 1/3, 约定见 config.DISPLAY_SCENE_COMPRESS)
                    req = airsim.ImageRequest(
                        cam, airsim.ImageType.Scene, False, self._scene_compress)
                resp = client.simGetImages([req], VEHICLE)

                arr = self._decode_response(resp, is_depth)
                if arr is not None:
                    self._commit(arr, is_depth, gen)
                else:
                    time.sleep(0.05)
                # 自适应节流: 扣除 RPC 往返后补足剩余间隔
                time.sleep(max(0.0, 1.0 / GRAB_HZ - (time.monotonic() - t_start)))
            except Exception as e:
                self.log(f'拉图连接中断 (worker {worker_id}): {e}，1 秒后重连...')
                client = None
                with self._conn_gate:
                    self._conn_count = max(0, self._conn_count - 1)
                time.sleep(1.0)

    def _decode_response(self, resp, is_depth):
        """simGetImages 响应 -> 帧数组 (深度 float32 米 / BGR uint8), 无效返回 None。"""
        if not resp or resp[0].width <= 0:
            return None
        r = resp[0]
        if is_depth:
            return _to_np(
                r.image_data_float, np.float32).reshape(r.height, r.width)
        # BGR (compress=False) 或 PNG (compress=True)
        data = _to_np(r.image_data_uint8, np.uint8)
        arr = None
        if self._scene_compress:
            arr = cv2.imdecode(data, cv2.IMREAD_COLOR)
            if arr is None:
                # 服务端没按 PNG 返回 (版本差异/异常数据):
                # 本会话回退原始传输, 下一拍走原始解码路径
                self._scene_compress = False
                self.log('图传: 服务端未按 PNG 返回, 回退原始传输')
        if arr is None and not self._scene_compress:
            # 服务端原始字节是 RGB 序, 显示约定 BGR —— 翻转通道
            arr = np.ascontiguousarray(
                data.reshape(r.height, r.width, 3)[..., ::-1])
        return arr
