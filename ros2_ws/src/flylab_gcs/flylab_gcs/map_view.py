#!/usr/bin/env python3
"""飞行地图 —— 俯视航线视图 (北在上, 米制网格)。

数据与遥测条同源: VehicleController 0.5s 心跳采样 AirSim kinematics
(NED 米, 相对出生点), 飞行中记录轨迹点。显示: 轨迹折线 + 当前位置
机头箭头 (按航向) + 起飞点 H 标记 + 米制网格。

视野自适应: 自动框住全部轨迹留边距 (跨度下限 20m, 避免刚起飞时
过度放大), 网格步长取 1/2/5×10^n 米, 北固定朝上。
"""
import math

from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QBrush, QColor, QFont, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import QSizePolicy, QWidget


class MapView(QWidget):
    """俯视航线地图: update_state 喂入轨迹 (数据 0.5s 一拍), 直接重绘。"""

    MIN_SPAN_M = 20.0        # 视野最小跨度 (米), 刚起飞不满屏狂放
    TRAIL = QColor('#1a73e8')
    ARROW = QColor('#2d8a3e')

    def __init__(self, parent=None):
        super().__init__(parent)
        self._track = []     # [(north, east), ...] 米 (NED, 相对出生点)
        self._home = None    # 起飞点 (north, east)
        self._hdg = 0.0
        self.setMinimumHeight(200)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def update_state(self, track, home, hdg):
        # 只收有限值坐标: NaN/Inf 进入轨迹会让 paintEvent 抛异常,
        # PyQt 对绘制回调里的未捕获异常直接 abort 整个进程
        self._track = [p for p in track
                       if len(p) == 2 and math.isfinite(p[0]) and math.isfinite(p[1])]
        self._home = home if (home is not None
                              and math.isfinite(home[0])
                              and math.isfinite(home[1])) else None
        self._hdg = hdg if math.isfinite(hdg) else 0.0
        self.update()

    def paintEvent(self, ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, QColor('#f2f4f7'))

        pts = self._track or ([self._home] if self._home else [])
        if not pts:
            p.setPen(QColor('#999'))
            p.drawText(QRectF(0, 0, w, h), Qt.AlignCenter, '待飞行数据...')
            return

        # 视野: 框住全部轨迹留 12% 边距, 跨度下限防放大过度
        ns = [n for n, e in pts]
        es = [e for n, e in pts]
        pad = 0.12
        span_n = max(ns) - min(ns)
        span_e = max(es) - min(es)
        span_n = max(span_n, self.MIN_SPAN_M)
        span_e = max(span_e, self.MIN_SPAN_M)
        cn, ce = (max(ns) + min(ns)) / 2, (max(es) + min(es)) / 2
        span_n *= 1 + 2 * pad
        span_e *= 1 + 2 * pad
        scale = min(w / span_e, h / span_n)   # 屏幕像素/米 (x=东, y=-北)

        def to_screen(n, e):
            return QPointF(w / 2 + (e - ce) * scale,
                           h / 2 - (n - cn) * scale)

        # 米制网格: 步长取 1/2/5×10^n, 使格距在 60~150px 之间
        step = 10.0 ** (int(math.log10(max(span_e, 1e-6))) - 1)
        for m in (5, 2, 1):
            if span_e * scale / (step * m) <= 8:
                step *= m
                break
        p.setPen(QPen(QColor('#dde1e6'), 1))
        n0 = math.floor((cn - span_n / 2) / step) * step
        e0 = math.floor((ce - span_e / 2) / step) * step
        k = 0
        while (n0 + k * step) <= cn + span_n / 2:
            y = h / 2 - (n0 + k * step - cn) * scale
            p.drawLine(QPointF(0, y), QPointF(w, y))
            k += 1
        k = 0
        while (e0 + k * step) <= ce + span_e / 2:
            x = w / 2 + (e0 + k * step - ce) * scale
            p.drawLine(QPointF(x, 0), QPointF(x, h))
            k += 1
        p.setPen(QColor('#aab2bb'))
        f = QFont()
        f.setPixelSize(10)
        p.setFont(f)
        p.drawText(QRectF(6, 4, 120, 14), Qt.AlignLeft, f'□ {step:g} m')

        # 轨迹折线
        if len(self._track) >= 2:
            p.setPen(QPen(self.TRAIL, 2))
            p.drawPolyline(QPolygonF([to_screen(n, e)
                                      for n, e in self._track]))

        # 起飞点 H 标记
        if self._home is not None:
            hp = to_screen(*self._home)
            p.setBrush(QColor('#666'))
            p.setPen(Qt.NoPen)
            p.drawRect(QRectF(hp.x() - 4, hp.y() - 4, 8, 8))
            p.setPen(QColor('white'))
            p.drawText(QRectF(hp.x() - 4, hp.y() - 4, 8, 8),
                       Qt.AlignCenter, 'H')

        # 当前位置机头箭头 (按航向, 0°=北=上, 顺时针)
        cur = to_screen(*pts[-1])
        p.save()
        p.translate(cur)
        p.rotate(self._hdg)
        p.setBrush(QBrush(self.ARROW))
        p.setPen(Qt.NoPen)
        p.drawPolygon(QPolygonF([
            QPointF(0, -9), QPointF(-6, 7), QPointF(0, 4), QPointF(6, 7)]))
        p.restore()
        p.end()
