#!/usr/bin/env python3
"""游戏手柄输入线程 (pygame/SDL)。

读手柄双摇杆 → 死区/夹紧/轴翻转 → Qt 信号投递 GUI 线程, 由控制面板注入
VirtualJoystick.set_external (画面旋钮联动, 下发走与鼠标拖动同一条链路)。

- 轴布局按 SDL Xbox 布局默认 (左 0/1 右 2/3), 其他手柄在 config.GAMEPAD_AXES
  覆盖 (查轴号: python3 -m pygame.examples.joystick_gui)
- 摇杆物理上推是负轴值, 统一翻转为 VirtualJoystick 约定 (前/左为正) 再发
- 热插拔: 事件驱动 (JOYDEVICEADDED/REMOVED), 断开即发零保悬停;
  初始化失败只报一次并让线程退出, 不拖垮地面站

WSL 下手柄需 usbipd 直通进系统 (与摄像头同一套流程):
  usbipd attach --busid <busid> --wsl <发行版>
直通后 SDL 自动看到 /dev/input/js*。
"""
import atexit
import math
import os
import threading
import time

# 无显示环境也能 init SDL (只需 joystick 子系统)
os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')

from PyQt5.QtCore import QObject, pyqtSignal

from . import config


class GamepadReader(QObject):
    """手柄读取线程。start/stop 由控制面板按"手柄控制"开关管理,
    信号自动排队投递 GUI 线程 (emit 来自工作线程安全)。"""

    stick = pyqtSignal(float, float, float, float)   # (lx, ly, rx, ry) 前/左为正
    plugged = pyqtSignal(bool, str)                  # (连接状态, 手柄名; 断开为空串)
    failed = pyqtSignal(str)                         # 初始化失败 (线程即将退出)

    def __init__(self):
        super().__init__()
        self._thread = None
        self._stop = threading.Event()
        # 进程退出前先干净停线程 (join 等 pygame.quit 走完), 否则守护线程
        # 会在 pygame 调用中途被强杀, 每次退出刷一段线程异常回溯
        atexit.register(self.stop)

    # ---------- 生命周期 ----------

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name='gamepad-reader', daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        t = self._thread
        if t is not None and t is not threading.current_thread():
            t.join(timeout=2.0)
        self._thread = None

    def is_running(self):
        return self._thread is not None and self._thread.is_alive()

    # ---------- 读取循环 ----------

    def _run(self):
        try:
            import pygame
            pygame.init()
            pad = self._pick_first_pad()
            self.plugged.emit(pad is not None,
                              pad.get_name() if pad is not None else '')
            while not self._stop.is_set():
                # 事件泵: 热插拔通知从这里出来 (get 即 pump)
                for ev in pygame.event.get():
                    if ev.type == pygame.JOYDEVICEADDED:
                        pad = pygame.joystick.Joystick(ev.device_index)
                        pad.init()
                        self.plugged.emit(True, pad.get_name())
                    elif ev.type == pygame.JOYDEVICEREMOVED:
                        pad = None
                        # 断开保悬停: 双摇杆回中
                        self.stick.emit(0.0, 0.0, 0.0, 0.0)
                        self.plugged.emit(False, '')
                if pad is not None:
                    self.stick.emit(
                        self._axis(pad, 'lx'), self._axis(pad, 'ly'),
                        self._axis(pad, 'rx'), self._axis(pad, 'ry'))
                time.sleep(1.0 / config.GAMEPAD_POLL_HZ)
        except Exception as e:   # pygame 缺失 / 无 SDL 输入权限等
            # 进程退出期 (SDL 已拆/解释器收尾) 的连锁异常: 静默返回, 此刻
            # 发 Qt 信号本身也可能抛, 不能让它变成未捕获回溯刷屏
            if self._stop.is_set():
                return
            try:
                self.failed.emit(f'手柄线程异常: {e}')
            except Exception:
                pass
        finally:
            try:
                import pygame
                pygame.quit()
            except Exception:
                pass
            try:
                self.plugged.emit(False, '')
            except Exception:
                pass

    def _pick_first_pad(self):
        """取第一个已连接手柄 (取第 0 个, 多手柄场景罕见, 不做管理)。"""
        import pygame
        for i in range(pygame.joystick.get_count()):
            pad = pygame.joystick.Joystick(i)
            pad.init()
            return pad
        return None

    def _axis(self, pad, key):
        """读单轴: 越界轴号兜零 → 翻转为前/左正 → 死区内外线性重标定。"""
        idx = config.GAMEPAD_AXES[key]
        if idx >= pad.get_numaxes():
            return 0.0
        v = pad.get_axis(idx) * config.GAMEPAD_INVERT[key]
        dz = config.GAMEPAD_DEADZONE
        if abs(v) <= dz:
            return 0.0
        # 死区边缘归零 → 全行程线性拉伸, 避免过死区即跳变
        return (v - math.copysign(dz, v)) / (1.0 - dz)
