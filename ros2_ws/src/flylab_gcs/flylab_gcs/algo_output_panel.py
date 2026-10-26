#!/usr/bin/env python3
"""右栏"算法信息"面板 —— 通用算法信息显示区 (注册表驱动)。

默认空白; 运行哪个算法 (或视觉反控降落) 才动态显示对应分组, 停止即移除。
数据来源 /flylab/aruco_info (std_msgs/String JSON, 节点 10Hz 限频发布),
按 src 分组各存最新一条 (见 ros_node._aruco_info_cb), 超时自动置灰显示 '—'。

数据 schema 由各算法自定义, 只要求顶层:
  {"src": "<分组key>", ...字段...}

---------- 接入新算法 (三步) ----------
1. 算法节点往 /flylab/aruco_info 发 {"src": "<key>", ...} (建议 10Hz 限频;
   参数名 info_topic, 参照 aruco_node.publishInfo)
2. 本文件 SECTIONS 注册一项: title + fields 字段表
   (label, extract) —— extract(data) 返回显示文本, None 显示 '—',
   格式化逻辑完全自定义
3. config.ALGORITHMS 该算法条目加 'panel': '<key>' —— 运行时面板自动
   显示该分组, 停止自动移除

内置分组:
  detect  —— aruco_node 最大码摘要
  landing —— aruco_landing_node 降落状态 (视觉反控按钮按下时显示)
"""
import time

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QGridLayout, QFormLayout, QGroupBox, QLabel

INFO_TIMEOUT = 1.2   # 秒: 超时视为该来源无输出 (节点端 10Hz 发布, 留足裕量)

# 降落状态机 -> 界面文案
_LANDING_STATE_TEXT = {
    'IDLE': '待机',
    'TRACKING': '对齐下降',
    'LANDING': 'PX4 自动降落',
}


def _landing_state_text(data):
    """降落状态 -> 界面文案 (分组标题用)。"""
    if data.get('lost'):
        return f"目标丢失 (悬停 {data.get('lost_t', 0.0):.1f}s)"
    return _LANDING_STATE_TEXT.get(data.get('state', 'IDLE'), '待机')


def _detect_fields(data):
    """aruco_node 最大码摘要 -> {字段名: 文本|None}。"""
    out = {'最大码': None, '尺寸': None, '画面占比': None, '中心偏移': None,
           '距离': None, '实际宽度': None,
           '检出': f"{data.get('count', 0)} 个"}
    largest = data.get('largest')
    if largest:
        dx, dy = largest.get('dx', 0.0), largest.get('dy', 0.0)
        dist = largest.get('dist', -1.0)
        wm = largest.get('wm', -1.0)
        out['最大码'] = f"id={largest.get('id', -1)}"
        out['尺寸'] = f"{largest.get('w', 0)}x{largest.get('h', 0)}px"
        out['画面占比'] = f"{100.0 * largest.get('area', 0.0):.1f}%"
        out['中心偏移'] = f'({dx:+.2f}, {dy:+.2f})'
        out['距离'] = f'{dist:.2f}m' if dist and dist > 0 else '— (无位姿)'
        out['实际宽度'] = f'{wm:.2f}m' if wm and wm > 0 else '— (无位姿)'
    return out


def _landing_fields(data):
    """aruco_landing_node 降落状态 -> {字段名: 文本|None}。"""
    state = data.get('state', 'IDLE')
    out = {
        '跟踪码': None,
        '尺寸': None,
        '实际宽度': None,
        '对齐误差': None,
        '画面占比': None,
        '高度': None,
        '趋势角': None,
        '速度': None,
        '触发计数': None,
    }
    tid = data.get('tracked_id', -1)
    out['跟踪码'] = f'id={tid}' if tid >= 0 else None
    if state == 'TRACKING' and not data.get('lost'):
        w, h_px = data.get('w', 0), data.get('h', 0)
        wm = data.get('wm', -1.0)
        if w and h_px:
            out['尺寸'] = f'{w}x{h_px}px'
            out['实际宽度'] = f'{wm:.2f}m' if wm and wm > 0 else '— (无位姿)'
        out['对齐误差'] = (
            f"{data.get('err', 0.0):.3f} "
            f"({'达标' if data.get('aligned') else '未达标'})")
        out['画面占比'] = f"{100.0 * data.get('area', 0.0):.1f}%"
        trend = data.get('trend')
        out['趋势角'] = f'{trend:.1f}°' if trend is not None else None
        vel = data.get('vel')
        out['速度'] = (
            f'({vel[0]:+.2f}, {vel[1]:+.2f}, {vel[2]:+.2f})' if vel else None)
        need = data.get('trigger_need', 0)
        out['触发计数'] = (
            f"{data.get('trigger', 0)}/{need}" if need else None)
    height = data.get('height', -1.0)
    out['高度'] = f'{height:.2f}m' if height and height > 0 else None
    return out


# 分组注册表: key (= 消息 src) -> {title, title_fn, fields}
# fields: [(label, extract)] —— extract(data) -> 文本 | None (显示 '—')
# 行序在并列显示的分组间保持语义对应 (同行 = 同类信息), 新分组接入时照此排列
SECTIONS = {
    'detect': {
        'title': 'ArUco 检测',
        'fields': [
            ('最大码', lambda d: _detect_fields(d)['最大码']),
            ('尺寸', lambda d: _detect_fields(d)['尺寸']),
            ('实际宽度', lambda d: _detect_fields(d)['实际宽度']),
            ('画面占比', lambda d: _detect_fields(d)['画面占比']),
            ('中心偏移', lambda d: _detect_fields(d)['中心偏移']),
            ('距离', lambda d: _detect_fields(d)['距离']),
            ('检出', lambda d: _detect_fields(d)['检出']),
        ],
    },
    'landing': {
        'title': 'ArUco 降落',
        # 状态不占行位, 动态显示在分组标题里 (对齐下降/目标丢失/…)
        'title_fn': lambda d: (
            'ArUco 降落 · ' + _landing_state_text(d)) if d else 'ArUco 降落',
        'fields': [
            ('跟踪码', lambda d: _landing_fields(d)['跟踪码']),
            ('尺寸', lambda d: _landing_fields(d)['尺寸']),
            ('实际宽度', lambda d: _landing_fields(d)['实际宽度']),
            ('画面占比', lambda d: _landing_fields(d)['画面占比']),
            ('对齐误差', lambda d: _landing_fields(d)['对齐误差']),
            ('高度', lambda d: _landing_fields(d)['高度']),
            ('趋势角', lambda d: _landing_fields(d)['趋势角']),
            ('速度', lambda d: _landing_fields(d)['速度']),
            ('触发计数', lambda d: _landing_fields(d)['触发计数']),
        ],
    },
}


class _Section(QGroupBox):
    """单个算法分组: 固定字段表 + 超时置灰; title_fn 可随数据动态改标题。"""

    def __init__(self, spec, parent=None):
        super().__init__(spec['title'], parent)
        self._fields = spec['fields']
        self._title_fn = spec.get('title_fn')
        self._texts = {}   # 行 -> 上次文本 (相同不重绘)
        form = QFormLayout(self)
        form.setContentsMargins(6, 2, 6, 4)
        form.setSpacing(1)
        form.setLabelAlignment(Qt.AlignRight)
        self._extracts = []
        self._vals = []
        for lbl, extract in self._fields:
            key = QLabel(lbl + ':')
            val = QLabel('—')
            val.setStyleSheet('font-family: monospace;')
            form.addRow(key, val)
            self._extracts.append(extract)
            self._vals.append(val)
        self.update_entry(None, time.monotonic())   # 初始占位

    def update_entry(self, entry, now_m):
        """entry = ros_node 里该 src 的最新数据 {data, t} | None。"""
        data = None
        if entry is not None and now_m - entry['t'] <= INFO_TIMEOUT:
            data = entry['data']
        self._set_stale(data is None)
        if self._title_fn is not None:
            title = self._title_fn(data)
            if title != self.title():
                self.setTitle(title)
        if data is None:
            for val in self._vals:
                self._set(val, '—')
            return
        for val, extract in zip(self._vals, self._extracts):
            text = extract(data)
            self._set(val, text if text is not None else '—')

    def _set(self, label, text):
        if self._texts.get(id(label)) != text:
            self._texts[id(label)] = text
            label.setText(text)

    def _set_stale(self, stale):
        style = 'QGroupBox::title { color: #99a; }' if stale else ''
        if self._texts.get('style') != style:
            self._texts['style'] = style
            self.setStyleSheet(style)


class AlgoOutputPanel(QGroupBox):
    """右栏算法信息面板: set_active() 决定显示哪些分组 (随运行中算法变化),
    refresh() 按各分组最新数据刷新 (33ms 轮询驱动, 内部去重)。"""

    def __init__(self, get_info, parent=None):
        super().__init__('算法信息', parent)
        self._get_info = get_info   # -> ros_node.aruco_info ({src: {data, t}})
        self._sections = {}         # key -> _Section (仅当前显示的)
        self._order = []            # 当前显示顺序 (key 列表)
        # 激活的分组横向并排一行; 网格列等宽拉伸 —— 列宽固定平分,
        # 不随内容变化 (没数据时列也不会缩窄)
        self._lay = QGridLayout(self)
        self._lay.setContentsMargins(0, 2, 0, 0)
        self._lay.setSpacing(4)

    def set_active(self, keys):
        """切换显示的分组: keys 里的动态创建 (未注册的忽略), 其余移除。"""
        for key in list(self._sections):
            if key not in keys:
                self._remove(key)
        for key in keys:
            if key not in self._sections and key in SECTIONS:
                self._add(key)

    def refresh(self):
        info = self._get_info() or {}
        now_m = time.monotonic()
        for key, section in self._sections.items():
            section.update_entry(info.get(key), now_m)

    def _add(self, key):
        self._sections[key] = _Section(SECTIONS[key])
        self._order.append(key)
        self._relayout()

    def _remove(self, key):
        section = self._sections.pop(key)
        self._lay.removeWidget(section)
        section.deleteLater()
        self._order.remove(key)
        self._relayout()

    def _relayout(self):
        """按当前顺序重排网格: 每列等宽拉伸, 多余列拉伸清零。"""
        for col, key in enumerate(self._order):
            self._lay.addWidget(self._sections[key], 0, col)
        for col in range(self._lay.columnCount()):
            self._lay.setColumnStretch(col, 1 if col < len(self._order) else 0)
