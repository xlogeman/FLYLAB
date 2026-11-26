#!/usr/bin/env python3
"""通用视频显示控件 —— 只管"把一帧 BGR 图画出来 + 鼠标框选"，不关心数据来源。

数据来源 (AirSim RPC / ROS 话题 / 算法结果) 由各功能模块决定，
把处理好的 BGR numpy 数组喂给 show_frame() 即可。
模型标注叠加 (overlay.py schema) 的图元渲染在 overlay_renderer.py,
本控件经 set_annotations 喂入、_redraw 时调 OverlayRenderer.draw()。
"""
from PyQt5.QtCore import QPoint, QRect, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QImage, QPen, QPixmap, QPainter
from PyQt5.QtWidgets import QLabel, QToolTip

from .overlay_renderer import OverlayRenderer


class VideoLabel(QLabel):
    """视频画面 + 可选的拖拽框选交互 (默认关闭)。

    目标追踪消费目标框: 主窗口接 bbox_selected 发 ROI, 追踪期间经
    set_select_enabled(True/False) 开关框选。
    确认: 松手 (拖出 ≥6px 矩形) 或回车/双击; 取消: Esc 或松手时框太小
    (只取消画框, 不退出框选模式)。"""

    bbox_selected = pyqtSignal(int, int, int, int)   # 原图坐标 x, y, w, h
    zoom_step = pyqtSignal(int)    # 数字变焦: +1 放大 / -1 缩小 (滚轮)
    zoom_reset = pyqtSignal()      # 数字变焦复位 (双击画面)
    pan_moved = pyqtSignal(int, int)   # 变焦拖动平移: 显示坐标位移 (px)

    def __init__(self, text='等待图像...'):
        super().__init__(text)
        # 最小尺寸给小些: 窗口拉小时左栏能跟着收缩, 不被下限顶爆
        self.setMinimumSize(320, 240)
        self.setAlignment(Qt.AlignCenter)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setMouseTracking(True)

        self._pixmap = None       # 最新一帧的原始 QPixmap
        self._scaled = None       # _pixmap 按 widget 尺寸缩放后的缓存 (见 _redraw)
        self._scaled_src = None   # 缓存对应的源 QPixmap (身份比较, 新帧才失效)
        self._scaled_size = None  # 缓存对应的 widget 尺寸 (窗口拖动时失效)
        self._img_size = (0, 0)
        self._select_enabled = False
        self._frozen = False      # 框选期间冻结画面
        self._sel_start = None    # QPoint (widget 坐标)
        self._sel_cur = None
        self._pan_start = None    # 变焦拖动平移起点 (QPoint, widget 坐标)
        self._annotations = []    # 模型输出标注 (overlay.py schema)
        self._overlay = OverlayRenderer()   # 标注图元渲染器 (含解码缓存)
        self._hud = None          # 画面左上角 HUD 文字 (如图像帧率)
        self._hud_algo = None     # HUD 第二行 (算法输出帧率)
        self._depth_provider = None   # fn(x, y, w, h) -> 米|None (悬停读数)

    # ---------- 深度悬停读数 ----------

    def set_depth_provider(self, fn):
        """挂载深度查询回调 fn(x, y, frame_w, frame_h) -> 米|None:
        深度估计运行中鼠标悬停画面在鼠标处显示深度气泡; 返回 None
        (节点没跑/数据过期) 自动隐藏。None 取消挂载。"""
        self._depth_provider = fn

    def _update_depth_tooltip(self, ev):
        """悬停深度气泡: provider 返回新鲜值即显示, 否则隐藏。
        无画面/框选进行中不显示 (无图可读或画面冻结)。"""
        val = None
        if (self._depth_provider is not None and self._pixmap is not None
                and self._sel_start is None):
            p = self._to_image(ev.pos())
            val = self._depth_provider(p.x(), p.y(), *self._img_size)
        if val is None:
            QToolTip.hideText()
        else:
            QToolTip.showText(ev.globalPos(), f'深度 {val:.2f} m', self)

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

    def set_waiting(self, text='等待图像...'):
        """无信号占位: 清掉旧画面/标注/HUD, 等待文字显示在画面中央
        (QLabel 文字与 pixmap 互斥, setText 即清掉画面)。
        新帧到来后 show_frame 自动以画面取代文字。"""
        self._pixmap = None
        self._img_size = (0, 0)
        self._annotations = []
        self._hud = None
        self._hud_algo = None
        self.setText(text)

    def clear_frame(self):
        """清屏回到占位文字。切源时调用: 新源无帧时不残留上一源定格画面。"""
        self.set_waiting()

    # ---------- 框选 ----------

    def set_select_enabled(self, on):
        self._select_enabled = on
        if not on:
            self._cancel_selection()

    def _cancel_selection(self):
        self._frozen = False
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

    def wheelEvent(self, ev):
        """滚轮数字变焦: 上滚放大 / 下滚缩小 (有画面时才生效)。"""
        if self._pixmap is not None:
            d = ev.angleDelta().y()
            if d:
                self.zoom_step.emit(1 if d > 0 else -1)
            ev.accept()
            return
        super().wheelEvent(ev)

    def mousePressEvent(self, ev):
        if (self._select_enabled and self._pixmap is not None
                and ev.button() == Qt.LeftButton):
            self._frozen = True
            self._sel_start = self._sel_cur = ev.pos()
            self._redraw()
        elif self._pixmap is not None and ev.button() == Qt.LeftButton:
            self._pan_start = ev.pos()   # 变焦拖动平移起点 (松手结束)
        super().mousePressEvent(ev)

    def mouseMoveEvent(self, ev):
        if self._sel_start is not None:
            self._sel_cur = ev.pos()
            self._redraw()
        elif self._pan_start is not None:
            d = ev.pos() - self._pan_start
            self._pan_start = ev.pos()
            if d.x() or d.y():           # 逐段发增量, 抓取层折算窗口移动
                self.pan_moved.emit(d.x(), d.y())
        self._update_depth_tooltip(ev)
        super().mouseMoveEvent(ev)

    def leaveEvent(self, ev):
        """鼠标移出画面: 隐藏悬停深度气泡。"""
        QToolTip.hideText()
        super().leaveEvent(ev)

    def mouseReleaseEvent(self, ev):
        if ev.button() == Qt.LeftButton:
            self._pan_start = None
            # 框选松手即确认: 拖出 ≥6px 矩形收框上屏 (否则松手后框继续
            # 跟着鼠标走, 用户单点想"确认"反而重新起框, 框像粘住鼠标);
            # 框太小视为误点, 只取消画框并解冻, 留在框选模式可重拖
            if self._select_enabled and self._sel_start is not None:
                if self._sel_cur is not None and \
                        abs(self._sel_cur.x() - self._sel_start.x()) >= 6 and \
                        abs(self._sel_cur.y() - self._sel_start.y()) >= 6:
                    self._confirm_selection()
                else:
                    self._sel_start = self._sel_cur = None
                    self._frozen = False
                    self._redraw()
        super().mouseReleaseEvent(ev)

    def mouseDoubleClickEvent(self, ev):
        """双击画面: 框选进行中确认选框, 否则变焦复位到 1.0 (有画面时)。"""
        if self._sel_start is not None:
            self._confirm_selection()
            return
        if self._pixmap is not None and ev.button() == Qt.LeftButton:
            self.zoom_reset.emit()
            ev.accept()
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
        # 缩放缓存: SmoothTransformation 全幅缩放开销大, 而一拍内标注/HUD/
        # 新帧各触发一次 _redraw, 若每次都重缩放, 30fps 下每秒 ~90 次全幅
        # 平滑缩放会占满 GUI 线程 (点击关菜单等事件被排队, 界面手感滞留)。
        # 仅新帧或窗口尺寸变化时重缩放, 其余调用复用缓存, 绘制画在副本上。
        if (self._scaled is None or self._scaled_src is not self._pixmap
                or self._scaled_size != self.size()):
            self._scaled = self._pixmap.scaled(
                self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self._scaled_src = self._pixmap
            self._scaled_size = self.size()
        # 先缩放, 之后所有绘制都在缩放后 pixmap 的坐标系里进行:
        # pixmap 会按 AlignCenter 居中显示, 尺寸 = 原图 * scale (KeepAspectRatio)
        scaled = self._scaled.copy()   # 缓存不可直接画: setPixmap 后重复叠加
        painter = QPainter(scaled)
        if self._sel_start is not None and self._sel_cur is not None:
            # 选择框: widget 坐标 -> pixmap 坐标 (减去居中黑边偏移)
            ox = (self.width() - scaled.width()) // 2
            oy = (self.height() - scaled.height()) // 2
            painter.setPen(QPen(Qt.green, 2))
            painter.drawRect(QRect(self._sel_start, self._sel_cur)
                             .normalized().translated(-ox, -oy))
        if self._annotations:
            self._overlay.draw(
                painter, self._annotations, self._img_size, scaled.width())
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
