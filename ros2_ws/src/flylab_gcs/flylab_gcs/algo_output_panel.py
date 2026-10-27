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
from PyQt5.QtWidgets import QGridLayout, QHBoxLayout, QFormLayout, QGroupBox, \
    QLabel, QSlider, QSpinBox, QWidget

from . import config

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
    out = {'码号': None, '尺寸': None, '占比': None, '偏移': None,
           '实宽': None, '位置': None,
           '检出': f"{data.get('count', 0)} 个"}
    largest = data.get('largest')
    if largest:
        dx, dy = largest.get('dx', 0.0), largest.get('dy', 0.0)
        wm = largest.get('wm', -1.0)
        out['码号'] = f"id={largest.get('id', -1)}"
        out['尺寸'] = f"{largest.get('w', 0)}x{largest.get('h', 0)}px"
        out['占比'] = f"{100.0 * largest.get('area', 0.0):.1f}%"
        out['偏移'] = f'({dx:+.2f}, {dy:+.2f})'
        out['实宽'] = f'{wm:.2f}m' if wm and wm > 0 else '— (无位姿)'
        out['位置'] = _tvec_text(largest)
    return out


def _tvec_text(data):
    """tvec=[x,y,z] (相机系, x右 y下 z前, 米) -> 显示文本; 无位姿返回占位。"""
    tvec = data.get('tvec')
    if tvec and len(tvec) == 3:
        return f'({tvec[0]:+.2f}, {tvec[1]:+.2f}, {tvec[2]:.2f})'
    return '— (无位姿)'


def _landing_fields(data):
    """aruco_landing_node 降落状态 -> {字段名: 文本|None}。"""
    state = data.get('state', 'IDLE')
    out = {
        '码号': None,
        '尺寸': None,
        '实宽': None,
        '位置': None,
        '误差': None,
        '占比': None,
        '趋势角': None,
        '航向': None,
        '速度': None,
        '计数': None,
    }
    tid = data.get('tracked_id', -1)
    out['码号'] = f'id={tid}' if tid >= 0 else None
    if state == 'TRACKING' and not data.get('lost'):
        w, h_px = data.get('w', 0), data.get('h', 0)
        wm = data.get('wm', -1.0)
        if w and h_px:
            out['尺寸'] = f'{w}x{h_px}px'
            out['实宽'] = f'{wm:.2f}m' if wm and wm > 0 else '— (无位姿)'
        out['位置'] = _tvec_text(data)
        out['误差'] = (
            f"{data.get('err', 0.0):.3f} "
            f"({'达标' if data.get('aligned') else '未达标'})")
        out['占比'] = f"{100.0 * data.get('area', 0.0):.1f}%"
        trend = data.get('trend')
        out['趋势角'] = f'{trend:.1f}°' if trend is not None else None
        if data.get('yaw_en'):
            if data.get('yaw_locked'):
                out['航向'] = f"已锁定 (偏置 {data.get('yaw_off', 0.0):.0f}°)"
            else:
                out['航向'] = (
                    f"{data.get('yaw_err', 0.0):+.1f}° "
                    f"({'对齐中' if data.get('aligned') else '待对齐'})")
        vel = data.get('vel')
        out['速度'] = (
            f'({vel[0]:+.2f}, {vel[1]:+.2f}, {vel[2]:+.2f})' if vel else None)
        need = data.get('trigger_need', 0)
        out['计数'] = (
            f"{data.get('trigger', 0)}/{need}" if need else None)
    return out


# 分组注册表: key (= 消息 src) -> {title, title_fn, fields}
# fields: [(label, extract)] —— extract(data) -> 文本 | None (显示 '—')
# 行序在并列显示的分组间保持语义对应 (同行 = 同类信息), 新分组接入时照此排列
SECTIONS = {
    'detect': {
        'title': 'ArUco 检测',
        'fields': [
            ('码号', lambda d: _detect_fields(d)['码号']),
            ('尺寸', lambda d: _detect_fields(d)['尺寸']),
            ('实宽', lambda d: _detect_fields(d)['实宽']),
            ('位置', lambda d: _detect_fields(d)['位置']),
            ('占比', lambda d: _detect_fields(d)['占比']),
            ('偏移', lambda d: _detect_fields(d)['偏移']),
            ('检出', lambda d: _detect_fields(d)['检出']),
        ],
    },
    'landing': {
        'title': 'ArUco 降落',
        # 状态不占行位, 动态显示在分组标题里 (对齐下降/目标丢失/…)
        'title_fn': lambda d: (
            'ArUco 降落 · ' + _landing_state_text(d)) if d else 'ArUco 降落',
        'fields': [
            ('码号', lambda d: _landing_fields(d)['码号']),
            ('尺寸', lambda d: _landing_fields(d)['尺寸']),
            ('实宽', lambda d: _landing_fields(d)['实宽']),
            ('位置', lambda d: _landing_fields(d)['位置']),
            ('占比', lambda d: _landing_fields(d)['占比']),
            ('误差', lambda d: _landing_fields(d)['误差']),
            ('趋势角', lambda d: _landing_fields(d)['趋势角']),
            ('航向', lambda d: _landing_fields(d)['航向']),
            ('速度', lambda d: _landing_fields(d)['速度']),
            ('计数', lambda d: _landing_fields(d)['计数']),
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
            key = QLabel(lbl + '：')
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

    def __init__(self, get_info, set_marker_length=None,
                 set_landing_yaw=None, parent=None):
        super().__init__('算法信息', parent)
        self._get_info = get_info   # -> ros_node.aruco_info ({src: {data, t}})
        self._sections = {}         # key -> _Section (仅当前显示的)
        self._order = []            # 当前显示顺序 (key 列表)
        # 激活的分组横向并排一行; 网格列等宽拉伸 —— 列宽固定平分,
        # 不随内容变化 (没数据时列也不会缩窄)
        self._lay = QGridLayout(self)
        self._lay.setContentsMargins(0, 2, 0, 0)
        self._lay.setSpacing(4)
        # ArUco 码边长滑杆 (常驻, 网格下方一整行): 场景含多种尺寸码时
        # 运行中同步节点 marker_length (位姿解算尺度)
        self._marker_box = None
        if set_marker_length is not None:
            self._build_marker_slider(set_marker_length)
        # 降落朝向选择 (常驻): 机头相对码 0° 方向的落点偏置,
        # 视觉反控降落时运行中同步 aruco_landing_node yaw_offset_deg
        self._yaw_box = None
        if set_landing_yaw is not None:
            self._build_yaw_spin(set_landing_yaw)
        # 底部常驻行 (码边长滑杆 + 降落朝向选择并排): 装进一个横向容器,
        # 整行横跨全部分组列 (见 _relayout); 两者都在才有意义, 单独存在也兼容
        bottom = None
        if self._marker_box is not None or self._yaw_box is not None:
            bottom = QWidget()
            row = QHBoxLayout(bottom)
            row.setContentsMargins(0, 0, 0, 0)
            row.setSpacing(4)
            if self._marker_box is not None:
                row.addWidget(self._marker_box, 2)   # 滑杆占更宽
            if self._yaw_box is not None:
                row.addWidget(self._yaw_box, 1)
        self._bottom_row = bottom

    def _build_marker_slider(self, publish):
        """码边长滑杆: 值域/步进取 config; 拖动才发布 (未动过节点用
        aruco.yaml 的 marker_length, 与视场角同步同一约定)。
        索引式量程 (0~N 档, 每档=一个步长): QSlider 直接用物理值时拖动
        按像素细步 (1px=0.01m), 索引式保证拖动只会落在步长档位上。"""
        box = QGroupBox('ArUco 码边长 (m)')
        box.setToolTip(
            '场景含多种尺寸码时拖动同步算法节点; 先把码摆到已知距离,\n'
            "调到面板\"实际宽度\"≈码的真实边长, 此时\"码位置\"\n"
            '的 z 分量 (末位) 即准确距离')
        lay = QHBoxLayout(box)
        lay.setContentsMargins(6, 2, 6, 4)
        lo_cm = int(round(config.ARUCO_MARKER_LENGTH_MIN * 100))
        step_cm = max(1, int(round(config.ARUCO_MARKER_LENGTH_STEP * 100)))
        n_steps = max(1, (int(round(config.ARUCO_MARKER_LENGTH_MAX * 100)) -
                          lo_cm) // step_cm)
        slider = QSlider(Qt.Horizontal)
        slider.setRange(0, n_steps)
        val = QLabel()
        val.setStyleSheet('font-family: monospace;')
        val.setMinimumWidth(44)

        def on_change(idx):
            cm = lo_cm + idx * step_cm
            val.setText(f'{cm / 100.0:.2f}')
            publish(cm / 100.0)

        slider.valueChanged.connect(on_change)
        # 初始占位不发默认值 (未拖动不发布, 节点用 aruco.yaml 的值)
        default_cm = int(round(config.ARUCO_MARKER_LENGTH_DEFAULT * 100))
        slider.blockSignals(True)
        slider.setValue(max(0, min(n_steps, (default_cm - lo_cm) // step_cm)))
        slider.blockSignals(False)
        val.setText(f'{(lo_cm + slider.value() * step_cm) / 100.0:.2f}')
        lay.addWidget(slider, 1)
        lay.addWidget(val)
        self._marker_box = box

    def _build_yaw_spin(self, publish):
        """降落朝向选择 (QSpinBox): 机头相对码 0° 方向 (码角点 0→1) 的
        落点朝向偏置 (度, 右正), 修改即发布 (latched); 未动过不发布,
        节点用 aruco.yaml 的 yaw_offset_deg (与码边长同步同一约定)。
        keyboardTracking 关闭: 键入过程不发布, 步进/回车/失焦才生效。"""
        box = QGroupBox('降落朝向偏置 (°)')
        box.setToolTip(
            '视觉反控降落时机头相对码 0° 方向 (码角点 0→1, 即码图案 x 轴)\n'
            '的落点朝向, 右转为正; 0 = 机头对齐码 0° 方向。\n'
            '运行中修改立即生效: 飞机会悬停转到新朝向再继续下降')
        lay = QHBoxLayout(box)
        lay.setContentsMargins(6, 2, 6, 4)
        spin = QSpinBox()
        spin.setRange(config.LANDING_YAW_OFFSET_MIN, config.LANDING_YAW_OFFSET_MAX)
        spin.setSingleStep(config.LANDING_YAW_OFFSET_STEP)
        spin.setSuffix('°')
        spin.setKeyboardTracking(False)
        # 初始显示默认值不发 (未动过不发布, 节点用 aruco.yaml 的值;
        # setValue 放在 connect 之前, 不触发 valueChanged)
        spin.setValue(config.LANDING_YAW_OFFSET_DEFAULT)
        spin.valueChanged.connect(publish)
        lay.addWidget(spin, 1)   # 必须加入布局, 否则游离控件不显示 (此前缺失)
        self._yaw_spin = spin
        self._yaw_box = box

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
        """按当前顺序重排网格: 每列等宽拉伸, 多余列拉伸清零;
        底部常驻行 (码边长滑杆 + 降落朝向选择并排) 横跨全部分组列。"""
        for col, key in enumerate(self._order):
            self._lay.addWidget(self._sections[key], 0, col)
        for col in range(self._lay.columnCount()):
            self._lay.setColumnStretch(col, 1 if col < len(self._order) else 0)
        if self._bottom_row is not None:
            self._lay.addWidget(
                self._bottom_row, 1, 0, 1, max(1, self._lay.columnCount()))
