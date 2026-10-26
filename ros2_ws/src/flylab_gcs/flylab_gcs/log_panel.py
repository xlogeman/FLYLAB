#!/usr/bin/env python3
"""画面下方日志框 —— 终端日志分流上屏, 免切终端。

数据来源: rclpy 日志 (TeeLogger 分流) 与 PX4 进程输出, 经 Qt 信号队列
投递 append_line。

抗日志风暴: 行先入内存缓冲 (O(1) 不触发渲染), 定时器批量上屏并限速;
缓冲满丢最旧并计数, 恢复后补一条折叠提示。无论上游刷多快, GUI 线程
渲染节奏恒定。maximumBlockCount 限行数防内存膨胀。
"""
import html
import time

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QPlainTextEdit


class LogPanel(QPlainTextEdit):
    _COLORS = {'WARNING': '#b8860b', 'ERROR': '#b03030', 'FATAL': '#b03030',
               'PX4': '#4a6a8a'}
    _FLUSH_MS = 200        # 批量上屏周期
    _MAX_PENDING = 400     # 缓冲上限 (超出丢最旧, 计数待折叠提示)
    _MAX_PER_FLUSH = 80    # 每次刷新最多上屏行数 (渲染限速)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setReadOnly(True)
        self.setMaximumBlockCount(800)
        self.setMinimumHeight(150)
        self.setStyleSheet(
            'QPlainTextEdit { font-family: monospace; font-size: 12px; '
            'background: #fafafa; border: 1px solid #ccc; }')
        self._buf = []      # 待上屏行 (level, text)
        self._dropped = 0   # 风暴期间被丢弃的行数
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._flush)
        self._timer.start(self._FLUSH_MS)

    def append_line(self, level, text):
        """入缓冲 (槽, 经信号队列调用恒在 GUI 线程; O(1) 不触发渲染)。"""
        if len(self._buf) >= self._MAX_PENDING:
            self._dropped += 1
            return
        self._buf.append((level, text))

    def _flush(self):
        """批量上屏: 每周期最多 _MAX_PER_FLUSH 行, 一次 appendHtml 一行,
        滚动条只在批次末尾归位一次 (风暴时渲染量恒定, 不随上游速率变化)。"""
        if not self._buf:
            return
        if self._dropped:
            self._render('INFO', f'… (日志过密, 已折叠 {self._dropped} 条)')
            self._dropped = 0
        for level, text in self._buf[:self._MAX_PER_FLUSH]:
            self._render(level, text)
        del self._buf[:self._MAX_PER_FLUSH]

    def _render(self, level, text):
        color = self._COLORS.get(level, '#333')
        ts = time.strftime('%H:%M:%S')
        self.appendHtml(
            f'<span style="color:#999">{ts}</span> '
            f'<b style="color:{color}">{html.escape(level)}</b> '
            f'{html.escape(text)}')
        sb = self.verticalScrollBar()
        sb.setValue(sb.maximum())
