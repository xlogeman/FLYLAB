#!/usr/bin/env python3
"""手动遥控拨杆控件 —— 虚拟摇杆 + 回弹拨杆。

VirtualJoystick: 2D 摇杆 (圆盘内拖动, 松手回中)。信号 changed(x, y) 给出
归一化偏移 (-1..1): x 右正, y 前正 (Qt 的 y 轴向下已在内部翻转)。
SnapSlider: 回弹拨杆 (竖直 QSlider, 上=正), 拖动实时 valueChanged, 松手回中。

均由 main_window 的手动遥控面板使用, 各轴组合成归一化 Twist 发给
teleop_node (力度=偏离中位的距离)。
"""
import math

from PyQt5.QtCore import QPointF, pyqtSignal, Qt
from PyQt5.QtGui import QColor, QPainter, QPen
from PyQt5.QtWidgets import QSlider, QWidget


class VirtualJoystick(QWidget):
    """2D 虚拟摇杆: 拖动方向 = 平移方向, 偏离圆心的距离 = 力度 (0..1)。"""

    changed = pyqtSignal(float, float)   # (x, y) 归一化: x 右正, y 前正
    RADIUS = 52

    def __init__(self, parent=None):
        super().__init__(parent)
        self._knob = QPointF(0.0, 0.0)   # 旋钮相对圆心的偏移 (像素)
        self.setFixedSize(self.RADIUS * 2 + 14, self.RADIUS * 2 + 14)

    def mousePressEvent(self, ev):
        self._drag(ev.pos())

    def mouseMoveEvent(self, ev):
        self._drag(ev.pos())

    def mouseReleaseEvent(self, ev):
        self.reset()

    def _drag(self, pos):
        center = QPointF(self.width() / 2, self.height() / 2)
        off = pos - center
        length = math.hypot(off.x(), off.y())
        limit = float(self.RADIUS)
        if length > limit:
            off *= limit / length
        if self._knob != off:
            self._knob = off
            self.update()
            # Qt 的 y 轴向下: 界面向上拖 = y 负 → 输出 y 正 (前)
            self.changed.emit(off.x() / limit, -off.y() / limit)

    def reset(self):
        """回中并发出零值 (松开/交还控制权时调用)。"""
        if self._knob != QPointF(0.0, 0.0):
            self._knob = QPointF(0.0, 0.0)
            self.update()
        self.changed.emit(0.0, 0.0)

    def paintEvent(self, ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        center = QPointF(self.width() / 2, self.height() / 2)
        p.setPen(QPen(QColor('#bbb'), 2))
        p.drawEllipse(center, self.RADIUS, self.RADIUS)
        p.setPen(QPen(QColor('#ddd'), 1))
        p.drawLine(QPointF(center.x() - self.RADIUS, center.y()),
                   QPointF(center.x() + self.RADIUS, center.y()))
        p.drawLine(QPointF(center.x(), center.y() - self.RADIUS),
                   QPointF(center.x(), center.y() + self.RADIUS))
        engaged = self._knob != QPointF(0.0, 0.0)
        p.setBrush(QColor('#2d8a3e') if engaged else QColor('#999'))
        p.setPen(Qt.NoPen)
        p.drawEllipse(center + self._knob, 15, 15)


class SnapSlider(QSlider):
    """回弹拨杆: 竖直, 上=正 下=负, 中位 0; 拖动实时发 valueChanged, 松手回中。"""

    def __init__(self, parent=None):
        super().__init__(Qt.Vertical, parent)
        self.setRange(-100, 100)
        self.setValue(0)
        self.setTickPosition(QSlider.TicksBothSides)
        self.setTickInterval(50)
        self.setFixedHeight(110)

    def mouseReleaseEvent(self, ev):
        super().mouseReleaseEvent(ev)
        self.setValue(0)     # 回中 (触发 valueChanged → 界面发零)
