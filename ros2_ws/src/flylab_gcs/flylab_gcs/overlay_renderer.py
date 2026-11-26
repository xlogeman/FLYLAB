#!/usr/bin/env python3
"""overlay 标注图元渲染器 —— 把原图坐标的标注画到缩放后的显示 pixmap 上。

从 video_label.py 抽出 (原 _draw_annotations/_decode_image_anno, 约占该
文件 1/4): 五种图元 (bbox/point/polyline/image/text, schema 见 overlay.py)
的绘制规则与布局微调 (label 贴底/越界左收、align=right、字号等比) 集中
在此, VideoLabel 只管帧缓存/缩放缓存/框选/手势, 两者经 draw() 一条接口
衔接 —— 渲染规则改动不再碰显示控件。

同一渲染器实例内 image 图元做单条解码缓存 (33ms 轮询 vs 节点 10Hz 图片
更新不同步, 同一份 base64 反复重绘时只解码一次)。
"""
import base64

import cv2
import numpy as np
from PyQt5.QtCore import QPointF, QRect, QRectF, Qt
from PyQt5.QtGui import QColor, QImage, QPen, QPolygonF


class OverlayRenderer:
    """无状态渲染器 (仅 image 解码缓存): draw(painter, annotations,
    img_size, scaled) 在 painter 当前坐标系画出全部标注。scaled 为缩放后
    的显示 pixmap 宽度 (pixmap 即缩放后的图像本身, 无需再加居中偏移)。"""

    def __init__(self):
        self._img_anno_cache = ('', None)   # (base64, QImage) 单条缓存

    def draw(self, painter, annotations, img_size, scaled_w):
        """画全部标注: img_size = 原图 (w, h), 坐标换算 scale = scaled_w / w。"""
        w, h = img_size
        if w == 0 or h == 0:
            return
        scale = scaled_w / w
        lw = max(1, round(1.5 * scale))            # point/polyline/text 线宽
        lw_box = max(2, round(2.5 * scale))        # bbox 目标框: 加粗一档更醒目
        for a in annotations:
            c = QColor(*a.get('color', [0, 255, 0]))
            label = a.get('label', '')
            if a['type'] == 'bbox':
                self._draw_bbox(painter, a, c, label, scale, lw_box, scaled_w)
            elif a['type'] == 'point':
                self._draw_point(painter, a, c, label, scale, lw)
            elif a['type'] == 'polyline':
                self._draw_polyline(painter, a, c, scale, lw)
            elif a['type'] == 'image':
                self._draw_image(painter, a, scale)
            elif a['type'] == 'text':
                self._draw_text(painter, a, c, scale)

    # ---------- 各图元 ----------

    def _draw_bbox(self, painter, a, c, label, scale, lw_box, scaled_w):
        x, y, bw, bh = a['box']
        r = QRectF(x * scale, y * scale, bw * scale, bh * scale)
        painter.setPen(QPen(c, lw_box))
        painter.drawRect(r)
        if not label:
            return
        # 类别名+置信度放框内左上角 (半透明黑底); 框太矮贴底部,
        # 横向越界向左收
        fm = painter.fontMetrics()
        tw = fm.horizontalAdvance(label)
        pad = 2
        tx = int(r.left()) + pad
        ty = int(r.top()) + fm.ascent() + pad
        if bh * scale < fm.height() + 2 * pad:
            ty = int(r.bottom()) - pad
        if tx + tw + 2 * pad > scaled_w:
            tx = scaled_w - tw - 2 * pad
        painter.fillRect(
            QRect(tx - pad, ty - fm.ascent() - pad,
                  tw + 2 * pad, fm.height() + 2 * pad),
            QColor(0, 0, 0, 150))
        painter.setPen(QPen(c))
        painter.drawText(tx, ty, label)

    def _draw_point(self, painter, a, c, label, scale, lw):
        px, py = a['point']
        painter.setPen(QPen(c, lw))
        painter.drawEllipse(
            QPointF(px * scale, py * scale), lw * 2, lw * 2)
        if label:
            painter.setPen(QPen(c))
            painter.drawText(
                int(px * scale) + 6, int(py * scale) - 4, label)

    def _draw_polyline(self, painter, a, c, scale, lw):
        pts = a['points']
        if len(pts) < 2:
            return
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

    def _draw_image(self, painter, a, scale):
        img = self._decode_image_anno(a)
        if img is None:
            return
        iw, ih = img.width(), img.height()
        # 绘制宽度: w 为原图像素, 省略按图像原生尺寸;
        # 高度: 显式 h 优先 (目标区域与图像纵横比不一致时), 否则按纵横比
        dw = a.get('w') or iw
        dh = a.get('h') or ih * dw / iw
        x, y = a['pos']
        painter.save()
        alpha = a.get('alpha', 1.0)
        if alpha < 1.0:
            painter.setOpacity(alpha)
        painter.drawImage(
            QRectF(x * scale, y * scale, dw * scale, dh * scale), img)
        painter.restore()

    def _draw_text(self, painter, a, c, scale):
        tx, ty = a['pos']
        text = a.get('text', '')
        if not text:
            return
        painter.save()
        f = painter.font()
        f.setBold(True)
        if a.get('px'):
            # 指定像素高 (原图像素) 乘缩放, 字号与码框屏幕等比
            f.setPixelSize(max(4, int(a['px'] * scale)))
        else:
            # 缺省: 基准字号 + 偏移
            f.setPointSize(f.pointSize() + a.get('size', 1))
        painter.setFont(f)
        fm = painter.fontMetrics()
        tw = fm.horizontalAdvance(text)
        x = int(tx * scale)
        y = int(ty * scale)
        # align=right 时 pos.x 为文字右边缘 (C++ 侧不知字体宽度)
        if a.get('align') == 'right':
            x -= tw
        # 半透明黑底条保证对比度
        pad = 3
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(0, 0, 0, 150))
        painter.drawRect(x - pad, y - fm.ascent() - pad,
                         tw + 2 * pad, fm.height() + 2 * pad)
        painter.setPen(QPen(c))
        painter.drawText(x, y, text)
        painter.restore()

    # ---------- image 图元解码 ----------

    def _decode_image_anno(self, anno):
        """image 图元: base64 -> QImage, 同一份 data 只解码一次 (缓存 1 条)。
        支持带透明通道的 PNG (语义分割叠加) 与普通 JPEG (深度伪彩画中画)。"""
        data = anno.get('data', '')
        if not data:
            return None
        cached_data, cached_img = self._img_anno_cache
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
