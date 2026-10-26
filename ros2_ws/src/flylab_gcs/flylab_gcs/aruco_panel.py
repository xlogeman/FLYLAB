#!/usr/bin/env python3
"""右栏"算法输出"面板: 显示 ArUco 检测/降落的实时结构化输出。

数据来源 /flylab/aruco_info (std_msgs/String JSON, 节点 10Hz 限频发布),
按 src 分组各存最新一条 (见 ros_node._aruco_info_cb):
  {"src": "detect",  "count": 3, "largest": {"id","area","dx","dy","dist"}|null}
      —— aruco_node, area=画面占比, dx/dy=中心偏移 (归一化, 右/下为正),
         dist=tvec z 前向距离 (米, 无位姿 -1)
  {"src": "landing", "state": "IDLE|TRACKING|LANDING", "tracked_id", "count",
   "area", "err", "aligned", "lost", "lost_t", "height", "trend",
   "vel":[f,r,d], "trigger", "trigger_need"} —— aruco_landing_node

超过 INFO_TIMEOUT 无更新的分组自动置灰显示 '—' (节点停止/无输出)。
"""
import time

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QFormLayout, QGroupBox, QLabel, QVBoxLayout

INFO_TIMEOUT = 1.2   # 秒: 超时视为该来源无输出 (节点端 10Hz 发布, 留足裕量)

# 降落状态机 -> 界面文案
LANDING_STATE_TEXT = {
    'IDLE': '待机',
    'TRACKING': '对齐下降',
    'LANDING': 'PX4 自动降落',
}


def _signed(v):
    """带方向符号的归一化偏移文本。"""
    return f'{v:+.2f}'


class ArucoInfoPanel(QGroupBox):
    """右栏算法输出面板 (数据由主窗口轮询 refresh() 驱动刷新)。"""

    def __init__(self, get_info, parent=None):
        super().__init__('算法输出', parent)
        self._get_info = get_info   # -> ros_node.aruco_info ({src: {data, t}})
        self._texts = {}            # value label -> 上次文本 (相同不重绘)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 2, 0, 0)
        lay.setSpacing(2)

        # ---- 检测区 (aruco_node) ----
        self._det_box = QGroupBox('ArUco 检测')
        det_form = self._form(self._det_box)
        self._det_rows = {
            'largest': self._row(det_form, '最大码'),
            'ratio': self._row(det_form, '画面占比'),
            'offset': self._row(det_form, '中心偏移'),
            'dist': self._row(det_form, '距离'),
            'count': self._row(det_form, '检出'),
        }
        lay.addWidget(self._det_box)

        # ---- 降落区 (aruco_landing_node) ----
        self._land_box = QGroupBox('ArUco 降落')
        land_form = self._form(self._land_box)
        self._land_rows = {
            'state': self._row(land_form, '状态'),
            'tracked': self._row(land_form, '跟踪码'),
            'err': self._row(land_form, '对齐误差'),
            'ratio': self._row(land_form, '画面占比'),
            'height': self._row(land_form, '高度'),
            'trend': self._row(land_form, '趋势角'),
            'vel': self._row(land_form, '速度(前,右,下)'),
            'trigger': self._row(land_form, '触发计数'),
        }
        lay.addWidget(self._land_box)
        lay.addStretch(1)

        self.refresh()   # 初始占位

    # ---------- 构建 ----------

    @staticmethod
    def _form(box):
        form = QFormLayout(box)
        form.setContentsMargins(6, 2, 6, 4)
        form.setSpacing(1)
        form.setLabelAlignment(Qt.AlignRight)
        return form

    @staticmethod
    def _row(form, name):
        key = QLabel(name + ':')
        val = QLabel('—')
        val.setStyleSheet('font-family: monospace;')
        form.addRow(key, val)
        return val

    # ---------- 刷新 ----------

    def _set(self, label, text):
        """值变化才 setText (33ms 轮询下避免无谓重绘)。"""
        if self._texts.get(id(label)) != text:
            self._texts[id(label)] = text
            label.setText(text)

    def _stale_style(self, box, stale):
        """超时分组标题置灰 (数据恢复自动还原)。"""
        box.setStyleSheet('QGroupBox::title { color: #99a; }' if stale else '')

    def refresh(self):
        info = self._get_info() or {}
        now_m = time.monotonic()
        self._update_detect(info.get('detect'), now_m)
        self._update_landing(info.get('landing'), now_m)

    def _update_detect(self, entry, now_m):
        stale = entry is None or now_m - entry['t'] > INFO_TIMEOUT
        self._stale_style(self._det_box, stale)
        if stale:
            for label in self._det_rows.values():
                self._set(label, '—')
            return
        d = entry['data']
        largest = d.get('largest')
        if largest:
            dx, dy = largest.get('dx', 0.0), largest.get('dy', 0.0)
            self._set(self._det_rows['largest'], f"id={largest.get('id', -1)}")
            self._set(self._det_rows['ratio'],
                      f"{100.0 * largest.get('area', 0.0):.1f}%")
            self._set(self._det_rows['offset'],
                      f"x {_signed(dx)}  y {_signed(dy)}")
            dist = largest.get('dist', -1.0)
            self._set(self._det_rows['dist'],
                      f'{dist:.2f}m' if dist and dist > 0 else '— (无位姿)')
        else:
            for key in ('largest', 'ratio', 'offset', 'dist'):
                self._set(self._det_rows[key], '—')
        self._set(self._det_rows['count'], f"{d.get('count', 0)} 个")

    def _update_landing(self, entry, now_m):
        stale = entry is None or now_m - entry['t'] > INFO_TIMEOUT
        self._stale_style(self._land_box, stale)
        if stale:
            for label in self._land_rows.values():
                self._set(label, '—')
            return
        d = entry['data']
        state = d.get('state', 'IDLE')
        if d.get('lost'):
            lost_t = d.get('lost_t', 0.0)
            state_text = f'目标丢失 (悬停 {lost_t:.1f}s)'
        else:
            state_text = LANDING_STATE_TEXT.get(state, state)
        self._set(self._land_rows['state'], state_text)
        tid = d.get('tracked_id', -1)
        self._set(self._land_rows['tracked'],
                  f'id={tid}' if tid >= 0 else '—')
        if state == 'TRACKING' and not d.get('lost'):
            self._set(self._land_rows['err'],
                      f"{d.get('err', 0.0):.3f} "
                      f"({'达标' if d.get('aligned') else '未达标'})")
            self._set(self._land_rows['ratio'],
                      f"{100.0 * d.get('area', 0.0):.1f}%")
            trend = d.get('trend')
            self._set(self._land_rows['trend'],
                      f'{trend:.1f}°' if trend is not None else '—')
            vel = d.get('vel')
            self._set(self._land_rows['vel'],
                      f'{vel[0]:+.2f} {vel[1]:+.2f} {vel[2]:+.2f}'
                      if vel else '—')
            trigger = d.get('trigger', 0)
            need = d.get('trigger_need', 0)
            self._set(self._land_rows['trigger'],
                      f'{trigger}/{need}' if need else '—')
        else:
            for key in ('err', 'ratio', 'trend', 'vel', 'trigger'):
                self._set(self._land_rows[key], '—')
        height = d.get('height', -1.0)
        self._set(self._land_rows['height'],
                  f'{height:.2f}m' if height and height > 0 else '—')
