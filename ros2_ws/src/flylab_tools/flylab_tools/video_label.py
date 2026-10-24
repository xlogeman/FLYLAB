#!/usr/bin/env python3
"""通用视频显示控件 —— 只管"把一帧 BGR 图画出来 + 鼠标框选"，不关心数据来源。

数据来源 (AirSim RPC / ROS 话题 / 算法结果) 由各功能模块决定，
把处理好的 BGR numpy 数组喂给 show_frame() 即可。
"""
import base64

import cv2
import numpy as np
from PyQt5.QtCore import QPoint, QPointF, QRect, QRectF, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QImage, QPen, QPixmap, QPainter, QPolygonF
from PyQt5.QtWidgets import QLabel


class VideoLabel(QLabel):
    """视频画面 + 可选的拖拽框选交互 (休眠接口, 默认关闭)。

    地面站当前没有算法消费目标框, 框选整体停用 —— 点击/拖动画面无任何反应。
    将来接入目标选取类算法时, 重新启用只需两行:
        video.set_select_enabled(True)          # 按下左键冻结画面, 拖出矩形
        video.bbox_selected.connect(handler)    # 回调收到原图坐标 x, y, w, h
    确认: 回车或双击; 取消: Esc。"""

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
        self._hud = None          # 画面左上角 HUD 文字 (如图像帧率)
        self._hud_algo = None     # HUD 第二行 (算法输出帧率)

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

    def set_algo_hud(self, text):
        """设置 HUD 第二行文字 (如 '算法 15.0 fps')，None 隐藏。"""
        if text == self._hud_algo:
            return
        self._hud_algo = text
        self._redraw()

    def _redraw(self):
        if self._pixmap is None:
            return
        # 先缩放, 之后所有绘制都在缩放后 pixmap 的坐标系里进行:
        # pixmap 会按 AlignCenter 居中显示, 尺寸 = 原图 * scale (KeepAspectRatio)
        scaled = self._pixmap.scaled(
            self.size(), Qt.KeepAspectRatio, Qt.FastTransformation)
        painter = QPainter(scaled)
        if self._sel_start is not None and self._sel_cur is not None:
            # 选择框: widget 坐标 -> pixmap 坐标 (减去居中黑边偏移)
            ox = (self.width() - scaled.width()) // 2
            oy = (self.height() - scaled.height()) // 2
            painter.setPen(QPen(Qt.green, 2))
            painter.drawRect(QRect(self._sel_start, self._sel_cur)
                             .normalized().translated(-ox, -oy))
        if self._annotations:
            self._draw_annotations(painter, scaled)
        if self._hud or self._hud_algo:
            self._draw_hud(painter)
        painter.end()
        self.setPixmap(scaled)

    def _draw_hud(self, painter):
        """左上角 HUD 文字 (可多行: 图像帧率 + 算法输出帧率),
        半透明黑底保证任何画面下都可读。"""
        lines = [t for t in (self._hud, self._hud_algo) if t]
        if not lines:
            return
        fm = painter.fontMetrics()
        pad = 4
        w = max(fm.horizontalAdvance(t) for t in lines) + 2 * pad
        h = len(lines) * fm.height() + 2 * pad
        painter.fillRect(QRect(4, 4, w, h), QColor(0, 0, 0, 140))
        painter.setPen(QPen(QColor(255, 255, 255)))
        y = 4 + pad + fm.ascent()
        for t in lines:
            painter.drawText(4 + pad, y, t)
            y += fm.height()

    # ---------- 模型输出叠加 ----------

    def set_annotations(self, annos):
        """替换当前叠加的模型标注 (overlay.py schema)。内容没变不重绘。"""
        if annos == self._annotations:
            return
        self._annotations = annos
        self._redraw()

    def _decode_image_anno(self, anno):
        """image 图元: base64 -> QImage。同一份 data 只解码一次 (缓存 1 条),
        每帧更新的稠密输出 (深度图) 与窗口重绘都走缓存, 不重复解码。
        支持带透明通道的 PNG (4 通道, 语义分割叠加用: 无效像素透明不遮挡
        原画面) 与普通 JPEG (3 通道, 深度伪彩画中画), 按解码结果自动区分。"""
        data = anno.get('data', '')
        if not data:
            return None
        cached_data, cached_img = getattr(self, '_img_anno_cache', ('', None))
        if data == cached_data:
            return cached_img
        try:
            buf = np.frombuffer(base64.b64decode(data), np.uint8)
            mat = cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)
            if mat is None:
                return None
            if mat.ndim == 3 and mat.shape[2] == 4:
                # BGRA -> RGBA (Qt Format_RGBA8888 逐字节对应), 保留透明度
                rgba = cv2.cvtColor(mat, cv2.COLOR_BGRA2RGBA)
                h, w = rgba.shape[:2]
                img = QImage(rgba.data, w, h, w * 4,
                             QImage.Format_RGBA8888).copy()
            else:
                bgr = mat if (mat.ndim == 3 and mat.shape[2] == 3) \
                    else cv2.cvtColor(mat, cv2.COLOR_GRAY2BGR)
                h, w = bgr.shape[:2]
                img = QImage(bgr.data, w, h, w * 3, QImage.Format_BGR888).copy()
        except Exception:
            return None
        self._img_anno_cache = (data, img)
        return img

    def _draw_annotations(self, painter, scaled):
        """把原图坐标的标注画到缩放后的显示 pixmap 上
        (pixmap 即缩放后的图像本身, 无需再加居中偏移)。"""
        w, h = self._img_size
        if w == 0 or h == 0:
            return
        scale = scaled.width() / w
        lw = max(1, round(1.5 * scale))            # point/polyline/text 线宽
        lw_box = max(2, round(2.5 * scale))        # bbox 目标框: 加粗一档更醒目
        for a in self._annotations:
            c = QColor(*a.get('color', [0, 255, 0]))
            label = a.get('label', '')
            if a['type'] == 'bbox':
                x, y, bw, bh = a['box']
                r = QRectF(x * scale, y * scale,
                           bw * scale, bh * scale)
                painter.setPen(QPen(c, lw_box))
                painter.drawRect(r)
                if label:
                    # 类别名+置信度放框内左上角 (半透明黑底, 任何画面可读);
                    # 框太矮放不下文字时贴底部内缘, 横向越界时向左收
                    fm = painter.fontMetrics()
                    tw = fm.horizontalAdvance(label)
                    pad = 2
                    tx = int(r.left()) + pad
                    ty = int(r.top()) + fm.ascent() + pad
                    if bh * scale < fm.height() + 2 * pad:
                        ty = int(r.bottom()) - pad
                    if tx + tw + 2 * pad > scaled.width():
                        tx = scaled.width() - tw - 2 * pad
                    painter.fillRect(
                        QRect(tx - pad, ty - fm.ascent() - pad,
                              tw + 2 * pad, fm.height() + 2 * pad),
                        QColor(0, 0, 0, 150))
                    painter.setPen(QPen(c))
                    painter.drawText(tx, ty, label)
            elif a['type'] == 'point':
                px, py = a['point']
                painter.setPen(QPen(c, lw))
                painter.drawEllipse(
                    QPointF(px * scale, py * scale), lw * 2, lw * 2)
                if label:
                    painter.setPen(QPen(c))
                    painter.drawText(
                        int(px * scale) + 6, int(py * scale) - 4, label)
            elif a['type'] == 'polyline':
                pts = a['points']
                if len(pts) < 2:
                    continue
                qpts = [QPointF(px * scale, py * scale) for px, py in pts]
                painter.setPen(QPen(c, lw))
                if a.get('close') and a.get('fill'):
                    # 闭合多边形填充为半透明色块 (实例分割掩码), 轮廓仍描边。
                    # save/restore 防止 brush 泄漏到后续图元
                    painter.save()
                    fc = QColor(c)
                    fc.setAlphaF(a.get('fill_alpha', 0.45))
                    painter.setBrush(fc)
                    painter.drawPolygon(QPolygonF(qpts))
                    painter.restore()
                else:
                    painter.drawPolyline(QPolygonF(qpts))
                    if a.get('close'):
                        painter.drawLine(qpts[-1], qpts[0])
            elif a['type'] == 'image':
                img = self._decode_image_anno(a)
                if img is None:
                    continue
                iw, ih = img.width(), img.height()
                # 绘制宽度: w 为原图像素, 省略按图像原生尺寸
                dw = a.get('w') or iw
                dh = ih * dw / iw
                x, y = a['pos']
                painter.save()
                alpha = a.get('alpha', 1.0)
                if alpha < 1.0:
                    painter.setOpacity(alpha)
                painter.drawImage(
                    QRectF(x * scale, y * scale, dw * scale, dh * scale), img)
                painter.restore()
            elif a['type'] == 'text':
                tx, ty = a['pos']
                text = a.get('text', '')
                if not text:
                    continue
                painter.save()
                f = painter.font()
                f.setBold(True)
                if a.get('px'):
                    # 标注指定文字像素高 (原图像素): 乘显示缩放后 setPixelSize,
                    # 字号与码框在屏幕上等比 (窗口缩放不失比例)
                    f.setPixelSize(max(4, int(a['px'] * scale)))
                else:
                    # 缺省: 基准字号 + 偏移 (a['size'], 缺省 +1), 空拍画面里醒目
                    f.setPointSize(f.pointSize() + a.get('size', 1))
                painter.setFont(f)
                fm = painter.fontMetrics()
                tw = fm.horizontalAdvance(text)
                x = int(tx * scale)
                y = int(ty * scale)
                # align=right 时 pos.x 为文字右边缘 (绘制端量宽, C++ 侧
                # 不知道字体宽度, 只能发右锚点)
                if a.get('align') == 'right':
                    x -= tw
                # 半透明黑底条: 任何画面下都有足够对比度 (同 HUD 处理)
                pad = 3
                painter.setPen(Qt.NoPen)
                painter.setBrush(QColor(0, 0, 0, 150))
                painter.drawRect(x - pad, y - fm.ascent() - pad,
                                 tw + 2 * pad, fm.height() + 2 * pad)
                painter.setPen(QPen(c))
                painter.drawText(x, y, text)
                painter.restore()
