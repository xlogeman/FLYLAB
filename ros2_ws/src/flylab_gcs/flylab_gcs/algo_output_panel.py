#!/usr/bin/env python3
"""右栏"算法信息"面板 —— 通用分组显示框架 (注册表在 panel_sections.SECTIONS)。

运行哪个算法 (或视觉反控降落) 才动态显示对应分组, 停止即移除; 数据来源
/flylab/algo_info (std_msgs/String JSON, 节点 10Hz 限频发布), 按 src 分组
各存最新一条 (见 ros_node._algo_info_cb), 超时自动置灰显示 '—'。

职责分工: "算法 JSON -> 显示文本" 的提取逻辑与分组注册表 (SECTIONS) 在
panel_sections.py —— 新算法接入只改注册表, 不碰框架; 本文件只做 Qt 渲染
(字段行 / 表格模式 / 图片行 / 超时置灰 / 顶部调节行显隐)。顶部调节行
(码边长/朝向偏置/字典/人脸任务/追踪俯角距离等参数控件) 由
latched_sync.LATCHED_PARAMS 声明表驱动 (见 _ParamControl) —— 新增同步
参数只改声明表, 不碰本文件。

分组 spec 契约 (注册表项, 本框架按其渲染):
  title     分组标题 (必填)
  title_fn  可选, 随数据动态改标题 (如降落状态/追踪状态)
  fields    字段模式: [(label, extract[, opts])], opts {'wrap': N} 换行固定行高
  table     表格模式: 列数; rows [(属性, extract(item, col))],
            可选 summary (标签, extract(data)) 汇总行跨全部数据列
  image     True 时分组顶部固定高度 (IMG_ROW_HEIGHT) 图片行,
            数据 "img" (base64 JPEG) 等比 letterbox 自适应显示
"""
import base64
import time

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QComboBox, QDoubleSpinBox, QGridLayout, \
    QHBoxLayout, QFormLayout, QGroupBox, QLabel, QSlider, QSpinBox, \
    QSizePolicy, QWidget

from .latched_sync import LATCHED_PARAMS
from .panel_sections import HIDE_ROW, SECTIONS, landing_spec

INFO_TIMEOUT = 1.2   # 秒: 超时视为该来源无输出 (节点端 10Hz 发布, 留足裕量)
IMG_ROW_HEIGHT = 195  # px: 分组图片行固定高度 (letterbox 留边自适应显示)


class _ImageRow(QLabel):
    """分组图片行: 固定高度显示区, 图片等比缩放完整放入并居中 (letterbox
    留边), 任意分辨率/长宽比的效果图都自适应, 不会把分组撑高挤出可视区。
    水平 Ignored: 宽度完全交给布局, 图片也不反向撑宽分组列;
    拉伸窗口自动重缩放。解码开销可忽略 (节点端 10Hz 限频 + JPEG)。"""

    def __init__(self, height, parent=None):
        super().__init__(parent)
        self._full = None   # 原始 QPixmap (重缩放基准)
        self.setFixedHeight(height)
        self.setAlignment(Qt.AlignCenter)
        self.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)

    def set_image(self, pixmap):
        self._full = pixmap
        self._rescale()

    def clear_image(self):
        if self._full is not None:
            self._full = None
            self.clear()

    def _rescale(self):
        if self._full is None:
            return
        # FastTransformation: 25Hz 高频刷新下比 Smooth 省一半 GUI 线程开销,
        # 320px 缩略图缩到 ~200px 显示区, 两种插值肉眼无差
        self.setPixmap(self._full.scaled(
            max(1, self.width()), max(1, self.height()),
            Qt.KeepAspectRatio, Qt.FastTransformation))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._rescale()


class _Section(QGroupBox):
    """单个算法分组: 固定字段表 + 超时置灰; title_fn 可随数据动态改标题。
    表格模式 (spec['table'] = 列数): 每个目标一列并列显示, 属性行共享
    标签 (spec['rows'], extract(item, col) -> 文本|None), 目标数不足时
    整列隐藏 —— 供实例分割等多目标摘要用。"""

    def __init__(self, spec, parent=None):
        super().__init__(spec['title'], parent)
        self._title_fn = spec.get('title_fn')
        self._texts = {}   # 行 -> 上次文本 (相同不重绘)
        self._last_img = None   # 上次渲染的 "img" base64 (相同帧不重复解码)
        # 两种模式共用 update_entry 的公共前缀, 先统一初始化默认值
        self._img = None
        self._extracts = []
        self._rows = []   # (key, val) 标签对, HIDE_ROW 时整行 setVisible(False)
        self._vals = []
        self._table_cols = spec.get('table', 0)
        if self._table_cols:
            self._build_table(spec)
            self.update_entry(None, time.monotonic())   # 初始占位
            return
        self._fields = spec['fields']
        form = QFormLayout(self)
        form.setContentsMargins(6, 2, 6, 4)
        form.setSpacing(1)
        form.setLabelAlignment(Qt.AlignRight)
        # 图片行 (spec['image']): 置于字段表上方, 跨两列整行;
        # 固定高度等比留边显示, 分组高度不随图片分辨率变化
        self._img = _ImageRow(IMG_ROW_HEIGHT) if spec.get('image') else None
        if self._img is not None:
            form.addRow(self._img)
        for lbl, extract, *rest in self._fields:
            opts = rest[0] if rest else {}
            key = QLabel(lbl + '：')
            val = QLabel('—')
            lines = opts.get('wrap')
            if lines:
                # 长文本字段 (如图例): 换行 + 固定 N 行高 + 宽度交给布局 ——
                # 文本长度逐帧变化时不改变本行尺寸, 右栏各分区不跳动。
                # 字体用 setFont 直设 (不走样式表): 行高按同一字体的度量
                # 精确锁定, 否则样式表字体与默认字体度量不一致, 行底留白
                # (视觉上与下一行之间多出一行)
                font = val.font()
                font.setFamily('monospace')
                val.setFont(font)
                val.setWordWrap(True)
                val.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
                # 行距用 lineSpacing (含 leading, 与富文本/QTextDocument
                # 实际行距一致), height() 不含 leading 会裁掉末行下伸部
                val.setFixedHeight(val.fontMetrics().lineSpacing() * lines)
                val.setAlignment(Qt.AlignLeft | Qt.AlignTop)
                key.setAlignment(Qt.AlignLeft | Qt.AlignTop)
            else:
                val.setStyleSheet('font-family: monospace;')
            form.addRow(key, val)
            self._extracts.append(extract)
            self._rows.append((key, val))
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
        if self._img is not None:
            b64 = data.get('img') if data else None
            if b64 != self._last_img:   # 33ms 轮询 vs 数据帧率不同步, 去重防重复解码
                self._last_img = b64
                pix = QPixmap()
                if b64 and pix.loadFromData(base64.b64decode(b64)):
                    self._img.set_image(pix)
                else:
                    self._img.clear_image()
        if self._table_cols:
            self._update_table(data)
            return
        if data is None:
            for val in self._vals:
                self._set(val, '—')
            return
        for (key, val), extract in zip(self._rows, self._extracts):
            text = extract(data)
            # HIDE_ROW: 该数据源无此字段, 整行隐藏 (两侧都藏行才不留空)
            hidden = text is HIDE_ROW
            key.setVisible(not hidden)
            val.setVisible(not hidden)
            if not hidden:
                self._set(val, text if text is not None else '—')

    def _build_table(self, spec):
        """表格模式构造: 每个目标一列 (列数 = spec['table']), 属性行
        共享左列标签; 行定义 spec['rows']: [(属性名, extract(item, col))]。
        可选 spec['summary']: (标签, extract(data)) 汇总行 (如检出数),
        置于表格上方、值跨全部数据列。"""
        self._table_extracts = [extract for _lbl, extract in spec['rows']]
        self._table_summary = spec.get('summary')
        self._table_summary_val = None
        self._table_cells = []   # [row][col] -> QLabel
        grid = QGridLayout(self)
        grid.setContentsMargins(6, 2, 6, 4)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(1)
        grid.setColumnStretch(0, 0)
        mono = 'font-family: monospace;'
        row = 0
        if self._table_summary is not None:
            lbl, _extract = self._table_summary
            grid.addWidget(QLabel(lbl + '：'), row, 0)
            val = QLabel('—')
            val.setStyleSheet(mono)
            grid.addWidget(val, row, 1, 1, self._table_cols)
            self._table_summary_val = val
            row = 1
        for col in range(self._table_cols):
            grid.setColumnStretch(col + 1, 1)
        for r, (lbl, _extract) in enumerate(spec['rows'], start=row + 1):
            key = QLabel(lbl + '：')
            grid.addWidget(key, r, 0)
            cells = []
            for col in range(self._table_cols):
                val = QLabel('—')
                val.setStyleSheet(mono)
                grid.addWidget(val, r, col + 1)
                cells.append(val)
            self._table_cells.append(cells)

    def _update_table(self, data):
        """表格模式刷新: data = {"items": [目标...]} (节点端按置信度降序,
        取前 table_cols 列)。目标数不足时整列隐藏; 数据超时 (None) 保持
        布局只把值置 '—' (与字段模式同约定, 置灰由标题色表达)。"""
        if data is None:
            if self._table_summary_val is not None:
                self._set(self._table_summary_val, '—')
            for cells in self._table_cells:
                for val in cells:
                    self._set(val, '—')
            return
        if self._table_summary_val is not None:
            text = self._table_summary[1](data)
            self._set(self._table_summary_val,
                      text if text is not None else '—')
        items = data.get('items', []) if data else []
        n = min(len(items), self._table_cols)
        for col in range(self._table_cols):
            visible = col < n
            for cells in self._table_cells:
                cells[col].setVisible(visible)
        for r, extract in enumerate(self._table_extracts):
            for col in range(n):
                text = extract(items[col], col)
                self._set(self._table_cells[r][col],
                          text if text is not None else '—')

    def _set(self, label, text):
        if self._texts.get(id(label)) != text:
            self._texts[id(label)] = text
            label.setText(text)

    def _set_stale(self, stale):
        style = 'QGroupBox::title { color: #99a; }' if stale else ''
        if self._texts.get('style') != style:
            self._texts['style'] = style
            self.setStyleSheet(style)


class _ParamControl:
    """latched 声明表驱动的顶部调节控件 (QGroupBox 包装)。
    按条目的 kind 字段构造四种控件之一, 值域/候选/文案全部来自声明表;
    修改即经 publish(key, value) 下发 (latched)。

    各控件共同遵守 "未动过不发布" 约定: 初始值在 connect 之前设置,
    不触发发布 —— 节点用各自 yaml/ROS 参数默认值。
      slider  索引式量程 (0~N 档, 每档 = 一个 step): QSlider 直接用
              物理值时拖动按像素细步, 索引式保证只落在步长档位上
      spin    QSpinBox (整数, keyboardTracking 关闭: 键入过程不发布,
              步进/回车/失焦才生效)
      spin_f  QDoubleSpinBox (小数, 同上)
      combo   QComboBox, choices 为 list (候选即发布值) 或 dict
              (键=发布值, 值=显示名); activated 信号只响应用户选择,
              含重复选择

    auto_sync 参数 (追踪俯视角度): 用户未动过 (touched=False) 时程序
    持续把显示值同步为云台当前俯角 (set_programmatic, blockSignals
    屏蔽, 不发布不置 touched) —— 首次点悬停跟拍时伺服要把云台转到
    注视目标的角度, 只在分组显示瞬间取值会拿到"调整前"的旧角度。
    """

    def __init__(self, param, publish):
        self.param = param
        self.touched = False   # 用户动过输入 (仅 auto_sync 用)
        self._publish = publish   # (key, value) -> 发布
        box = QGroupBox(param.title)
        if param.tooltip:
            box.setToolTip(param.tooltip)
        lay = QHBoxLayout(box)
        lay.setContentsMargins(6, 2, 6, 4)
        build = {'slider': self._build_slider, 'spin': self._build_spin,
                 'spin_f': self._build_spin_f, 'combo': self._build_combo}
        build[param.kind](box, lay)
        self.box = box

    def _on_change(self, value):
        if self.param.auto_sync:
            self.touched = True
        self._publish(self.param.key, value)

    # ---------- 各 kind 的构造 ----------

    def _build_slider(self, box, lay):
        p = self.param
        lo, step = float(p.lo), float(p.step)
        n_steps = max(1, int(round((float(p.hi) - lo) / step)))
        slider = QSlider(Qt.Horizontal)
        slider.setRange(0, n_steps)
        val = QLabel()
        val.setStyleSheet('font-family: monospace;')
        val.setMinimumWidth(44)

        def on_change(idx):
            v = round(lo + idx * step, p.decimals)
            val.setText(f'{v:.{p.decimals}f}')
            self._on_change(v)

        slider.valueChanged.connect(on_change)
        # 初始占位不发默认值 (未拖动不发布)
        idx = int(round((float(p.default) - lo) / step))
        slider.blockSignals(True)
        slider.setValue(max(0, min(n_steps, idx)))
        slider.blockSignals(False)
        val.setText(f'{round(lo + slider.value() * step, p.decimals):.{p.decimals}f}')
        lay.addWidget(slider, 1)
        lay.addWidget(val)

    def _build_spin(self, box, lay):
        p = self.param
        spin = QSpinBox()
        spin.setRange(int(p.lo), int(p.hi))
        spin.setSingleStep(int(p.step))
        spin.setSuffix(p.suffix)
        spin.setKeyboardTracking(False)
        spin.setFixedWidth(p.width)
        # 初始显示默认值不发 (setValue 放在 connect 之前, 不触发
        # valueChanged)
        spin.setValue(int(p.default))
        spin.valueChanged.connect(self._on_change)
        self._spin = spin   # auto_sync 程序同步用 (set_programmatic)
        lay.addWidget(spin)   # 固定宽度, 不随行宽拉伸 (加入布局防游离不显示)
        lay.addStretch(1)     # 剩余宽度留白, 分组框收窄到内容宽度

    def _build_spin_f(self, box, lay):
        p = self.param
        spin = QDoubleSpinBox()
        spin.setRange(float(p.lo), float(p.hi))
        spin.setSingleStep(float(p.step))
        spin.setDecimals(p.decimals)
        spin.setSuffix(p.suffix)
        spin.setKeyboardTracking(False)
        spin.setFixedWidth(p.width)
        # 初始显示默认值不发 (setValue 放在 connect 之前, 不触发
        # valueChanged)
        spin.setValue(float(p.default))
        spin.valueChanged.connect(self._on_change)
        self._spin = spin   # auto_sync 程序同步用 (set_programmatic)
        lay.addWidget(spin)   # 固定宽度, 不随行宽拉伸 (加入布局防游离不显示)
        lay.addStretch(1)     # 剩余宽度留白, 分组框收窄到内容宽度

    def _build_combo(self, box, lay):
        p = self.param
        combo = QComboBox()
        if isinstance(p.choices, dict):
            # 键 = 发布值, 值 = 显示名; 显示名反查发布值
            combo.addItems(list(p.choices.values()))
            name_to_key = {v: k for k, v in p.choices.items()}
            default = p.choices.get(
                p.default, next(iter(p.choices.values())))
            combo.setCurrentText(default)
            combo.activated[str].connect(   # 用户选择才发布 (含重复选择)
                lambda name: self._on_change(
                    name_to_key.get(name, p.default)))
        else:
            combo.addItems(p.choices)
            combo.setCurrentText(p.default)
            combo.activated[str].connect(self._on_change)
        lay.addWidget(combo, 1)

    # ---------- auto_sync 支持 ----------

    def set_programmatic(self, value):
        """程序同步显示值 (跟随云台俯角): 屏蔽信号, 不发布、不置
        touched —— 用户动输入框才发布, 节点契约不变。"""
        spin = self._spin
        spin.blockSignals(True)
        spin.setValue(value)
        spin.blockSignals(False)

    def visible_for(self, keys, landing_kind):
        return self.param.visible_for(keys, landing_kind)


class AlgoOutputPanel(QGroupBox):
    """右栏算法信息面板: set_active() 决定显示哪些分组 (随运行中算法变化),
    refresh() 按各分组最新数据刷新 (33ms 轮询驱动, 内部去重)。"""

    def __init__(self, get_info, publish_param,
                 get_gimbal_pitch=None, parent=None):
        super().__init__('算法信息', parent)
        self._get_info = get_info   # -> ros_node.algo_info ({src: {data, t}})
        self._get_gimbal_pitch = get_gimbal_pitch   # () -> 俯角度 (下正)|None
        self._sections = {}         # key -> _Section (仅当前显示的)
        self._order = []            # 当前显示顺序 (key 列表)
        self._landing_kind = None   # 反控降落跟随的检测器类型 (aruco|fractal|None)
        # 激活的分组横向并排一行; 网格列等宽拉伸 —— 列宽固定平分,
        # 不随内容变化 (没数据时列也不会缩窄)
        self._lay = QGridLayout(self)
        self._lay.setContentsMargins(0, 2, 0, 0)
        self._lay.setSpacing(4)
        # 顶部调节行 (latched 声明表驱动): 按声明表顺序生成有控件条目
        # (kind != None), 发布统一走 publish_param(key, value); 装进
        # 一个横向容器整行横跨全部分组列, 置于算法分组上方 (见 _relayout);
        # 可见性随激活分组 (声明表的 groups 字段, 见 _update_top_visibility),
        # 无相关算法运行时整行隐藏
        self._controls = {
            p.key: _ParamControl(p, publish_param)
            for p in LATCHED_PARAMS if p.kind is not None}
        top = None
        if self._controls:
            top = QWidget()
            row = QHBoxLayout(top)
            row.setContentsMargins(0, 0, 0, 0)
            row.setSpacing(4)
            for p in LATCHED_PARAMS:
                if p.kind is not None:
                    row.addWidget(self._controls[p.key].box, p.stretch)
        self._top_row = top
        self._update_top_visibility([])   # 初始无分组, 整行隐藏

    def _sync_auto_controls(self):
        """auto_sync 参数 (追踪俯视角度) 同步为云台当前俯角 (限幅):
        仅用户未动过 (touched=False) 且控件可见时调用 (refresh 轮询 +
        track 分组显示时, 见 _update_top_visibility); 云台不可用/非
        AirSim 源跳过保持现值。程序同步是"跟随显示"而非用户输入:
        set_programmatic 屏蔽信号, 不发布、不置 touched (用户动输入框
        才发布, 节点契约不变)。首次点悬停跟拍时伺服要把云台转到注视
        目标的角度, 只在分组显示瞬间取值会拿到"调整前"的旧角度 ——
        轮询里持续同步到云台收敛, 输入框自然停在调整后的角度。"""
        if self._get_gimbal_pitch is None:
            return
        p = self._get_gimbal_pitch()
        if p is None:
            return
        for ctrl in self._controls.values():
            if ctrl.param.auto_sync and ctrl.box.isVisible() \
                    and not ctrl.touched:
                v = int(round(max(ctrl.param.lo,
                                  min(ctrl.param.hi, p))))
                ctrl.set_programmatic(v)

    def set_active(self, keys, landing_kind=None):
        """切换显示的分组: keys 里的动态创建 (未注册的忽略), 其余移除;
        顶部调节行的可见性同步跟随 (无相关分组时整行隐藏)。
        landing_kind = 反控降落跟随的检测器类型 ('aruco'|'fractal'|None,
        随运行中的算法; None = 无反控或检测节点已停, 按 aruco 兜底) ——
        landing 分组标题与字典/分形配置二选一靠它区分, 类型变化时重建
        landing 分组 (标题 ArUco 降落 / fractal 降落)。"""
        prev_kind = self._landing_kind or 'aruco'
        self._landing_kind = landing_kind
        for key in list(self._sections):
            if key not in keys:
                self._remove(key)
        # landing 标题随检测器类型: 类型变化时移除重建 (下次循环重加)
        if 'landing' in self._sections and \
                prev_kind != (self._landing_kind or 'aruco'):
            self._remove('landing')
        for key in keys:
            if key not in self._sections and key in SECTIONS:
                self._add(key)
        self._update_top_visibility(keys)

    def _update_top_visibility(self, keys):
        """顶部调节行随激活分组显隐: 每个参数控件的归属由声明表 groups
        字段声明 (见 _ParamControl.visible_for) —— 字典切换属于 ArUco
        检测 / aruco 降落; 分形配置属于 fractal 检测 / fractal 降落,
        landing 分组的归属靠 _landing_kind (反控降落的检测器类型, 见
        set_active) 区分, 不同时出现。全无则整行隐藏。"""
        landing_kind = self._landing_kind or 'aruco'
        any_on = False
        for ctrl in self._controls.values():
            on = ctrl.visible_for(keys, landing_kind)
            ctrl.box.setVisible(on)
            any_on = any_on or on
        if self._top_row is not None:
            self._top_row.setVisible(any_on)
        # auto_sync 参数 (俯视角度) 显示即跟云台当前俯角 (持续同步见
        # refresh): 须在整行可见性确定之后同步 —— _sync_auto_controls
        # 以 isVisible() 为守卫, top_row 还隐藏时同步会被跳过
        if any_on:
            self._sync_auto_controls()

    def refresh(self):
        info = self._get_info() or {}
        now_m = time.monotonic()
        for key, section in self._sections.items():
            section.update_entry(info.get(key), now_m)
        # auto_sync 参数 (俯视角度) 持续跟随云台当前俯角 (未动过且控件
        # 显示时): 云台收敛后输入框即停在调整后的角度; 用户一动输入框
        # (touched) 即停止跟随
        self._sync_auto_controls()

    def _add(self, key):
        # landing 分组标题随反控降落的检测器类型 (ArUco 降落 / fractal 降落)
        if key == 'landing' and (self._landing_kind or 'aruco') == 'fractal':
            spec = landing_spec('fractal')
        else:
            spec = SECTIONS[key]
        self._sections[key] = _Section(spec)
        self._order.append(key)
        self._relayout()

    def _remove(self, key):
        section = self._sections.pop(key)
        self._lay.removeWidget(section)
        section.deleteLater()
        self._order.remove(key)
        self._relayout()

    def _relayout(self):
        """按当前顺序重排网格: 顶部调节行 (latched 声明表生成的参数控件,
        见 __init__) 在第 0 行横跨全部分组列, 算法分组在其下
        (第 1 行) —— 可选项在上、算法输出在下; 每列等宽拉伸, 多余列拉伸
        清零; 可见性由 _update_top_visibility 决定 (隐藏的组件不占布局空间)。
        跨列数用 len(_order) 而非 _lay.columnCount(): 后者是布局当前状态,
        首次激活时还没加入分组会取到旧值, 控件行被压进单列把第 0 列
        (先加入的分组) 撑宽、其余分组变窄。"""
        span = max(1, len(self._order))
        if self._top_row is not None:
            self._lay.addWidget(self._top_row, 0, 0, 1, span)
        for col, key in enumerate(self._order):
            self._lay.addWidget(self._sections[key], 1, col)
        for col in range(self._lay.columnCount()):
            self._lay.setColumnStretch(col, 1 if col < len(self._order) else 0)
