#!/usr/bin/env python3
"""托管子进程 —— 地面站拉起的所有外部进程的统一生命周期管理。

覆盖四类进程: PX4 SITL (make→px4 进程树) / 通信链路 (uXRCE-DDS Agent、
MAVROS) / 算法节点 / 降落节点。统一解决三件重复的事:
1. 独立进程组启动 (start_new_session=True): 停止时 killpg 整棵树回收,
   避免 ros2 run / make → px4 留孤儿 (残留 px4 占住 /tmp/px4_lock-N)
2. 停止: SIGTERM 整组 → 超时 SIGKILL
3. 输出接管 (可选): 逐行读 stdout/stderr, 剥 ANSI 转义后回调
   (PX4 SITL 用, 经 Qt 信号队列上界面日志框; 其余进程日志直通终端)

注意: 方法在 GUI 线程调用, stop() 最长阻塞 timeout 秒 (SIGTERM 后进程
通常立即退出, 实际无感)。
"""
import os
import re
import signal
import subprocess
import threading

# 子进程输出里的 ANSI 转义序列 (颜色/光标控制), 上屏前剥掉
_ANSI_RE = re.compile(
    r'\x1b\[[0-9;?]*[0-9A-Za-z]|\x1b\][^\x07\x1b]*(?:\x07|\x1b\\)')


class ManagedProcess:
    """一个被地面站托管的外部进程。"""

    def __init__(self, log, name, on_line=None):
        """log: 带 .info/.error 的日志器 (TeeLogger, 自动分流界面日志框);
        name: 日志显示名; on_line: 输出行回调 (仅 capture_output=True 时)。"""
        self.log = log
        self.name = name
        self.on_line = on_line
        self.proc = None

    def start(self, cmd, capture_output=False):
        """拉起进程 (已在运行则忽略并返回 True)。
        启动失败 (OSError / 可执行文件不在 PATH) 返回 False 并记日志。"""
        if self.is_running():
            return True
        self.log.info(f'启动 {self.name}: {" ".join(cmd)}')
        try:
            self.proc = subprocess.Popen(
                cmd, start_new_session=True,
                stdout=subprocess.PIPE if capture_output else None,
                stderr=subprocess.STDOUT if capture_output else None,
                text=True, bufsize=1, errors='replace')
        except (OSError, FileNotFoundError) as e:
            self.log.error(f'{self.name} 启动失败: {e}')
            self.proc = None
            return False
        if capture_output:
            threading.Thread(target=self._pump, daemon=True).start()
        return True

    def _pump(self):
        """逐行读子进程输出 → 回调 (进程退出/被杀时管道关闭, 循环自然结束)。"""
        for line in self.proc.stdout:
            text = _ANSI_RE.sub('', line.rstrip())
            if text and self.on_line is not None:
                self.on_line(text)

    def is_running(self):
        return self.proc is not None and self.proc.poll() is None

    def exited(self):
        """已启动且已退出的退出码; 运行中/未启动返回 None (存活轮询用)。"""
        if self.proc is None or self.proc.poll() is None:
            return None
        return self.proc.returncode

    def clear(self):
        """丢弃进程句柄 (崩溃复位后允许重新 start)。"""
        self.proc = None

    def stop(self, timeout=2.0):
        """SIGTERM 整个进程组, 超时再 SIGKILL。"""
        proc, self.proc = self.proc, None
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except (ProcessLookupError, PermissionError, OSError):
            proc.terminate()
        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError, OSError):
                proc.kill()

    def terminate(self):
        """轻量停止: 仅 SIGTERM 不等待 (需要延迟回收/异步回收的场景)。"""
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()
