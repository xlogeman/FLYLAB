#!/usr/bin/env python3
"""QGC 式罗盘控件 —— 航向指示。

外圈刻度盘 (北/东/南/西 + 度数刻度) 随航向反向旋转, 中央机头标记恒指
上方, 中部数字显示当前航向 (0-360°)。数据与遥测条同源 (VehicleController
0.5s 心跳采样), 采样间隔内用低通平滑追赶, 转动不跳变。
"""
from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import QSizePolicy, QWidget


class Compass(QWidget):
    """圆形罗盘: set_heading 喂入 0-360°, 内部 33ms 低通平滑显示。
    尺寸自适应: 表盘取宽高中较小者居中绘制, 可与旁边控件等高。"""

    CARDINALS = ((0, '北'), (90, '东'), (180, '南'), (270, '西'))

    def __init__(self, parent=None):
        super().__init__(parent)
        self._hdg = 0.0      # 平滑显示角
        self._target = 0.0   # 目标航向
        self.setMinimumSize(110, 110)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._ease)
        self._timer.start(33)

    def set_heading(self, deg):
        self._target = deg % 360.0

    @staticmethod
    def _wrap(d):
        """角度差归到 (-180, 180] (最短路径追赶)。"""
        return (d + 180.0) % 360.0 - 180.0

    def _ease(self):
        d = self._wrap(self._target - self._hdg)
        if abs(d) < 0.05:
            if self._hdg != self._target:
                self._hdg = self._target
                self.update()
            return
        self._hdg = (self._hdg + d * 0.25) % 360.0
        self.update()

    def paintEvent(self, ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        cx, cy = self.width() / 2, self.height() / 2
        r = min(self.width(), self.height()) / 2 - 6

        # 表盘底
        p.setBrush(QColor('#fafafa'))
        p.setPen(QPen(QColor('#bbb'), 1.5))
        p.drawEllipse(QPointF(cx, cy), r, r)

        # 刻度盘: 随航向反向旋转 (航向 90° = 东, 盘上"东"转到正上方)
        p.save()
        p.translate(cx, cy)
        p.rotate(-self._hdg)
        for a in range(0, 360, 15):
            p.save()
            p.rotate(a)
            major, mid = a % 90 == 0, a % 45 == 0
            ln = 9 if major else (6 if mid else 3)
            p.setPen(QPen(QColor('#333') if major else QColor('#aaa'),
                          2 if major else 1))
            p.drawLine(QPointF(0, -r + 2), QPointF(0, -r + 2 + ln))
            p.restore()
        # 方位字随盘转、字面保持水平
        for a, ch in self.CARDINALS:
            p.save()
            p.rotate(a)
            p.translate(0, -(r - 15))
            p.rotate(-(a - self._hdg))
            f = QFont()
            f.setPixelSize(max(11, int(r * 0.2)))
            f.setBold(a == 0)
            p.setFont(f)
            p.setPen(QColor('#d9534f') if a == 0 else QColor('#444'))
            p.drawText(QRectF(-12, -9, 24, 18), Qt.AlignCenter, ch)
            p.restore()
        p.restore()

        # 中央机头标记 (恒指上方)
        p.setBrush(QColor('#2d8a3e'))
        p.setPen(Qt.NoPen)
        p.drawPolygon(QPolygonF([
            QPointF(cx, cy - r * 0.30),
            QPointF(cx - 6, cy + 2),
            QPointF(cx + 6, cy + 2)]))

        # 航向数字 (机头标记下方)
        f = QFont()
        f.setPixelSize(max(12, int(r * 0.26)))
        f.setBold(True)
        p.setFont(f)
        p.setPen(QColor('#222'))
        p.drawText(QRectF(cx - r / 2, cy + 4, r, r * 0.4),
                   Qt.AlignHCenter | Qt.AlignTop,
                   f'{int(round(self._hdg)) % 360:03d}°')

        # 外沿当前航向指针 (固定在表盘正上方的小三角, 指向读数)
        p.setBrush(QColor('#555'))
        p.drawPolygon(QPolygonF([
            QPointF(cx, cy - r - 5),
            QPointF(cx - 4, cy - r + 3),
            QPointF(cx + 4, cy - r + 3)]))
        p.end()
