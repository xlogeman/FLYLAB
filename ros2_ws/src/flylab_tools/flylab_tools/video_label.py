#!/usr/bin/env python3
"""通用视频显示控件 —— 只管"把一帧 BGR 图画出来 + 鼠标框选"，不关心数据来源。

数据来源 (AirSim RPC / ROS 话题 / 算法结果) 由各功能模块决定，
把处理好的 BGR numpy 数组喂给 show_frame() 即可。
"""
import numpy as np
from PyQt5.QtCore import QPoint, QPointF, QRect, QRectF, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QImage, QPen, QPixmap, QPainter, QPolygonF
from PyQt5.QtWidgets import QLabel


class VideoLabel(QLabel):
    """视频画面 + 可选的拖拽框选交互。

    框选流程：鼠标左键按下开始 -> 画面自动冻结 -> 拖出矩形 ->
    回车或双击确认 (emit bbox_selected) / Esc 取消 (emit selection_cancelled)。
    """

    bbox_selected = pyqtSignal(int, int, int, int)   # 原图坐标 x, y, w, h
    selection_cancelled = pyqtSignal()

    def __init__(self, text='等待图像...'):
        super().__init__(text)
        self.setMinimumSize(640, 480)
        self.setAlignment(Qt.AlignCenter)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setMouseTracking(True)

        self._pixmap = None       # 最新一帧的原始 QPixmap
        self._img_size = (0, 0)
        self._select_enabled = False
        self._frozen = False      # 框选期间冻结画面
        self._sel_start = None    # QPoint (widget 坐标)
        self._sel_cur = None
        self._annotations = []    # 模型输出标注 (overlay.py schema)
        self._hud = None          # 画面左上角 HUD 文字 (如帧率)

    # ---------- 帧显示 ----------

    def show_frame(self, arr):
        """显示一帧 BGR uint8 numpy 数组。框选冻结期间丢弃新帧。"""
        if self._frozen or arr is None:
            return
        h, w = arr.shape[:2]
        img = QImage(arr.data, w, h, w * 3, QImage.Format_BGR888).copy()
        self._pixmap = QPixmap.fromImage(img)
        self._img_size = (w, h)
        self._redraw()

    # ---------- 框选 ----------

    def set_select_enabled(self, on):
        self._select_enabled = on
        if not on:
            self._cancel_selection()

    def _cancel_selection(self):
        if self._frozen:
            self._frozen = False
            self.selection_cancelled.emit()
        self._sel_start = self._sel_cur = None
        self._redraw()

    def _confirm_selection(self):
        if self._sel_start is None or self._sel_cur is None:
            return
        p1 = self._to_image(self._sel_start)
        p2 = self._to_image(self._sel_cur)
        x0, x1 = sorted((p1.x(), p2.x()))
        y0, y1 = sorted((p1.y(), p2.y()))
        x0, x1 = max(0, x0), min(self._img_size[0] - 1, x1)
        y0, y1 = max(0, y0), min(self._img_size[1] - 1, y1)
        self._frozen = False
        self._sel_start = self._sel_cur = None
        self._redraw()
        self.bbox_selected.emit(x0, y0, x1 - x0 + 1, y1 - y0 + 1)

    def _to_image(self, pos):
        """widget 坐标 -> 原图坐标 (画面 KeepAspectRatio 居中缩放)。"""
        w, h = self._img_size
        if w == 0 or h == 0:
            return QPoint(0, 0)
        scale = min(self.width() / w, self.height() / h)
        dx = (self.width() - w * scale) / 2
        dy = (self.height() - h * scale) / 2
        return QPoint(int((pos.x() - dx) / scale), int((pos.y() - dy) / scale))

    # ---------- 事件 ----------

    def mousePressEvent(self, ev):
        if (self._select_enabled and self._pixmap is not None
                and ev.button() == Qt.LeftButton):
            self._frozen = True
            self._sel_start = self._sel_cur = ev.pos()
            self._redraw()
        super().mousePressEvent(ev)

    def mouseMoveEvent(self, ev):
        if self._sel_start is not None:
            self._sel_cur = ev.pos()
            self._redraw()
        super().mouseMoveEvent(ev)

    def mouseDoubleClickEvent(self, ev):
        if self._sel_start is not None:
            self._confirm_selection()
            return
        super().mouseDoubleClickEvent(ev)

    def keyPressEvent(self, ev):
        if ev.key() == Qt.Key_Return and self._sel_start is not None:
            self._confirm_selection()
        elif ev.key() == Qt.Key_Escape:
            self._cancel_selection()
        else:
            super().keyPressEvent(ev)

    def resizeEvent(self, ev):
        self._redraw()
        super().resizeEvent(ev)

    # ---------- 绘制 ----------

    def set_hud(self, text):
        """设置左上角 HUD 文字 (如 '58.3 fps')，None 隐藏。"""
        if text == self._hud:
            return
        self._hud = text
        self._redraw()

    def _redraw(self):
        if self._pixmap is None:
            return
        pm = self._pixmap
        if self._sel_start is not None and self._sel_cur is not None:
            pm = pm.copy()
            painter = QPainter(pm)
            painter.setPen(QPen(Qt.green, 2))
            painter.drawRect(QRect(self._sel_start, self._sel_cur).normalized())
            painter.end()
        scaled = pm.scaled(
            self.size(), Qt.KeepAspectRatio, Qt.FastTransformation)
        painter = QPainter(scaled)
        if self._annotations:
            self._draw_annotations(painter)
        if self._hud:
            self._draw_hud(painter)
        painter.end()
        self.setPixmap(scaled)

    def _draw_hud(self, painter):
        """左上角 HUD 文字, 半透明黑底保证任何画面下都可读。"""
        fm = painter.fontMetrics()
        pad = 4
        w = fm.horizontalAdvance(self._hud) + 2 * pad
        h = fm.height() + 2 * pad
        painter.fillRect(QRect(4, 4, w, h), QColor(0, 0, 0, 140))
        painter.setPen(QPen(QColor(255, 255, 255)))
        painter.drawText(4 + pad, 4 + pad + fm.ascent(), self._hud)

    # ---------- 模型输出叠加 ----------

    def set_annotations(self, annos):
        """替换当前叠加的模型标注 (overlay.py schema)。内容没变不重绘。"""
        if annos == self._annotations:
            return
        self._annotations = annos
        self._redraw()

    def _draw_annotations(self, painter):
        """把原图坐标的标注画到已缩放的显示 pixmap 上。"""
        w, h = self._img_size
        if w == 0 or h == 0:
            return
        scale = min(self.width() / w, self.height() / h)
        dx = (self.width() - w * scale) / 2
        dy = (self.height() - h * scale) / 2
        lw = max(1, round(2 * scale))
        for a in self._annotations:
            c = QColor(*a.get('color', [0, 255, 0]))
            label = a.get('label', '')
            if a['type'] == 'bbox':
                x, y, bw, bh = a['box']
                r = QRectF(dx + x * scale, dy + y * scale,
                           bw * scale, bh * scale)
                painter.setPen(QPen(c, lw))
                painter.drawRect(r)
                if label:
                    painter.setPen(QPen(c))
                    painter.drawText(
                        int(r.left()), int(r.top()) - 4, label)
            elif a['type'] == 'point':
                px, py = a['point']
                painter.setPen(QPen(c, lw))
                painter.drawEllipse(
                    QPointF(dx + px * scale, dy + py * scale), lw * 2, lw * 2)
                if label:
                    painter.setPen(QPen(c))
                    painter.drawText(
                        int(dx + px * scale) + 6, int(dy + py * scale) - 4,
                        label)
            elif a['type'] == 'polyline':
                pts = a['points']
                if len(pts) < 2:
                    continue
                qpts = [QPointF(dx + px * scale, dy + py * scale)
                        for px, py in pts]
                painter.setPen(QPen(c, lw))
                painter.drawPolyline(QPolygonF(qpts))
                if a.get('close'):
                    painter.drawLine(qpts[-1], qpts[0])
            elif a['type'] == 'text':
                tx, ty = a['pos']
                painter.setPen(QPen(c))
                painter.drawText(int(dx + tx * scale), int(dy + ty * scale),
                                 a.get('text', ''))
