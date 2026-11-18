#!/usr/bin/env python3
"""右栏"算法信息"面板 —— 通用分组显示框架 (注册表在 panel_sections.SECTIONS)。

运行哪个算法 (或视觉反控降落) 才动态显示对应分组, 停止即移除; 数据来源
/flylab/aruco_info (std_msgs/String JSON, 节点 10Hz 限频发布), 按 src 分组
各存最新一条 (见 ros_node._aruco_info_cb), 超时自动置灰显示 '—'。

职责分工: "算法 JSON -> 显示文本" 的提取逻辑与分组注册表 (SECTIONS) 在
panel_sections.py —— 新算法接入只改注册表, 不碰框架; 本文件只做 Qt 渲染
(字段行 / 表格模式 / 图片行 / 超时置灰 / 顶部调节行显隐)。

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
from PyQt5.QtWidgets import QComboBox, QGridLayout, QHBoxLayout, QFormLayout, \
    QGroupBox, QLabel, QSlider, QSpinBox, QSizePolicy, QWidget

from . import config
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


class AlgoOutputPanel(QGroupBox):
    """右栏算法信息面板: set_active() 决定显示哪些分组 (随运行中算法变化),
    refresh() 按各分组最新数据刷新 (33ms 轮询驱动, 内部去重)。"""

    def __init__(self, get_info, set_marker_length=None,
                 set_landing_yaw=None, set_aruco_dict=None,
                 set_face_task=None, set_face_topk=None,
                 set_fractal_config=None, parent=None):
        super().__init__('算法信息', parent)
        self._get_info = get_info   # -> ros_node.aruco_info ({src: {data, t}})
        self._sections = {}         # key -> _Section (仅当前显示的)
        self._order = []            # 当前显示顺序 (key 列表)
        self._landing_kind = None   # 反控降落跟随的检测器类型 (aruco|fractal|None)
        # 激活的分组横向并排一行; 网格列等宽拉伸 —— 列宽固定平分,
        # 不随内容变化 (没数据时列也不会缩窄)
        self._lay = QGridLayout(self)
        self._lay.setContentsMargins(0, 2, 0, 0)
        self._lay.setSpacing(4)
        # ArUco 码边长滑杆 (跟随 ArUco 分组显隐, 分组网格上方一整行): 场景含
        # 多种尺寸码时运行中同步节点 marker_length (位姿解算尺度)
        self._marker_box = None
        if set_marker_length is not None:
            self._build_marker_slider(set_marker_length)
        # 降落朝向选择 (仅降落分组激活时显示): 机头相对码 0° 方向的落点
        # 偏置, 视觉反控降落时运行中同步 aruco_landing_node yaw_offset_deg
        self._yaw_box = None
        if set_landing_yaw is not None:
            self._build_yaw_spin(set_landing_yaw)
        # 字典切换 (跟随 ArUco 分组显隐): 运行中同步 aruco_node /
        # aruco_landing_node 的 dict_name (下一帧检测生效)
        self._dict_box = None
        if set_aruco_dict is not None:
            self._build_dict_combo(set_aruco_dict)
        # 人脸任务切换 (跟随 face 分组显隐): 运行中同步 face_detect_node 的
        # FaceXFormer 任务 (下一帧生效)
        self._face_task_box = None
        if set_face_task is not None:
            self._build_face_task_combo(set_face_task)
        # 人脸任务张数 (跟随 face 分组显隐): 置信度前 N 张 / 全部批量跑任务
        self._face_topk_box = None
        if set_face_topk is not None:
            self._build_face_topk_combo(set_face_topk)
        # 分形配置切换 (跟随 fractal 分组显隐): 运行中同步 fractal_node 的
        # fractal_config (重建检测器, 下一帧检测生效)
        self._fractal_box = None
        if set_fractal_config is not None:
            self._build_fractal_combo(set_fractal_config)
        # 目标追踪无面板控件: track 分组激活期间画面框选常开 (主窗口
        # 控制), 拖出矩形松开即自动开始追踪, 停止靠工具栏"停止算法"
        # 顶部调节行 (码边长滑杆 + 降落朝向选择 + 字典切换 + 分形配置并排):
        # 装进一个横向容器, 整行横跨全部分组列, 置于算法分组上方 (见 _relayout);
        # 可见性跟随激活分组 (见 _update_top_visibility), 无相关算法运行时整行隐藏
        top = None
        if self._marker_box is not None or self._yaw_box is not None or \
                self._dict_box is not None or self._face_task_box is not None or \
                self._face_topk_box is not None or \
                self._fractal_box is not None:
            top = QWidget()
            row = QHBoxLayout(top)
            row.setContentsMargins(0, 0, 0, 0)
            row.setSpacing(4)
            if self._marker_box is not None:
                row.addWidget(self._marker_box, 2)   # 滑杆占更宽
            if self._yaw_box is not None:
                row.addWidget(self._yaw_box, 1)
            if self._dict_box is not None:
                row.addWidget(self._dict_box, 1)
            if self._face_task_box is not None:
                row.addWidget(self._face_task_box, 1)
            if self._face_topk_box is not None:
                row.addWidget(self._face_topk_box, 1)
            if self._fractal_box is not None:
                row.addWidget(self._fractal_box, 1)
        self._top_row = top
        self._update_top_visibility([])   # 初始无分组, 整行隐藏

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
        box = QGroupBox('朝向偏置 (°)')
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
        spin.setFixedWidth(78)   # 收窄: 三位数 + ° 足够, 省出底部行宽度给滑杆
        # 初始显示默认值不发 (未动过不发布, 节点用 aruco.yaml 的值;
        # setValue 放在 connect 之前, 不触发 valueChanged)
        spin.setValue(config.LANDING_YAW_OFFSET_DEFAULT)
        spin.valueChanged.connect(publish)
        lay.addWidget(spin)   # 固定宽度, 不随行宽拉伸 (加入布局防游离不显示)
        lay.addStretch(1)     # 剩余宽度留白, 分组框收窄到内容宽度
        self._yaw_box = box

    def _build_dict_combo(self, publish):
        """ArUco 字典切换 (QComboBox): 候选取 config.ARUCO_DICTS, 选择即
        发布 (latched); 未动过不发布, 节点用 aruco.yaml 的 dict_name
        (与码边长同步同一约定)。激活信号过滤: 程序设初值不触发发布。"""
        box = QGroupBox('ArUco 字典')
        box.setToolTip(
            '场景换了码的字典 (如 4x4 换 6x6) 时下拉切换, 运行中同步\n'
            '检测/降落节点, 下一帧检测生效; 降落中切换会重新选择目标')
        lay = QHBoxLayout(box)
        lay.setContentsMargins(6, 2, 6, 4)
        combo = QComboBox()
        combo.addItems(config.ARUCO_DICTS)
        # 初始显示默认值不发 (未选择不发布, 节点用 aruco.yaml 的值;
        # setCurrentText 放在 connect 之前, 不触发 activated)
        combo.setCurrentText(config.ARUCO_DICT_DEFAULT)
        combo.activated[str].connect(publish)   # 用户选择才发布 (含重复选择, 传字典名)
        lay.addWidget(combo, 1)
        self._dict_box = box

    def _build_face_task_combo(self, publish):
        """人脸下游任务切换 (QComboBox): 候选取 config.FACE_TASKS (键=发
        给节点的任务名, 值=显示名), 选择即发布 (latched); 未动过不发布,
        节点用 facex_task 参数的值 (与 ArUco 字典同步同一约定)。激活信号
        过滤: 程序设初值不触发发布。"""
        box = QGroupBox('人脸任务')
        box.setToolTip(
            'FaceXFormer 对检出人脸跑的下游分析任务 (张数见"人脸数量"),\n'
            '下拉运行中切换, 下一帧生效:\n'
            '人像分割/68 关键点/头部姿态/人脸属性/年龄性别/点可见性')
        lay = QHBoxLayout(box)
        lay.setContentsMargins(6, 2, 6, 4)
        combo = QComboBox()
        combo.addItems(list(config.FACE_TASKS.values()))
        # 初始显示默认值不发 (未选择不发布, 节点用 facex_task 参数的值;
        # setCurrentText 放在 connect 之前, 不触发 activated)
        default_name = config.FACE_TASKS.get(
            config.FACE_TASK_DEFAULT, config.FACE_TASK_DEFAULT)
        combo.setCurrentText(default_name)
        # 用户选择才发布 (含重复选择, 传任务名 -> 显示名反查任务 id)
        name_to_id = {v: k for k, v in config.FACE_TASKS.items()}

        def on_activated(name):
            publish(name_to_id.get(name, config.FACE_TASK_DEFAULT))

        combo.activated[str].connect(on_activated)
        lay.addWidget(combo, 1)
        self._face_task_box = box

    def _build_face_topk_combo(self, publish):
        """人脸任务张数切换 (QComboBox): 候选取 config.FACE_TOPKS (键=发给
        节点的数值字符串 '0'=全部, 值=显示名), 选择即发布 (latched); 未动
        过不发布, 节点用 facex_topk 参数的值。激活信号过滤: 程序设初值不
        触发发布。"""
        box = QGroupBox('人脸数量')
        box.setToolTip(
            'FaceXFormer 批量跑任务的人脸张数 (按置信度排序):\n'
            '前 1/2/3/5 张或全部检出脸; 脸多时拼批一次推理,\n'
            '下拉运行中切换, 下一帧生效')
        lay = QHBoxLayout(box)
        lay.setContentsMargins(6, 2, 6, 4)
        combo = QComboBox()
        combo.addItems(list(config.FACE_TOPKS.values()))
        # 初始显示默认值不发 (未选择不发布, 节点用 facex_topk 参数的值)
        combo.setCurrentText(config.FACE_TOPKS.get(
            config.FACE_TOPK_DEFAULT,
            next(iter(config.FACE_TOPKS.values()))))
        # 用户选择才发布 (含重复选择, 显示名反查数值字符串)
        name_to_key = {v: k for k, v in config.FACE_TOPKS.items()}

        def on_activated(name):
            publish(int(name_to_key.get(name, config.FACE_TOPK_DEFAULT)))

        combo.activated[str].connect(on_activated)
        lay.addWidget(combo, 1)
        self._face_topk_box = box

    def _build_fractal_combo(self, publish):
        """fractal 分形配置切换 (QComboBox): 候选取 config.FRACTAL_CONFIGS
        (内置 2L~5L 四档), 选择即发布 (latched), 节点重建检测器下一帧生效;
        未动过不发布, 节点用 aruco.yaml 的 fractal_config (与 ArUco 字典
        同步同一约定)。激活信号过滤: 程序设初值不触发发布。"""
        box = QGroupBox('fractal 配置')
        box.setToolTip(
            '分形码嵌套层数 (2L~5L, 数字越大层数越多, 远距/遮挡越鲁棒,\n'
            '内层码越小), 场景换了码时下拉切换, 运行中同步 fractal_node,\n'
            '下一帧检测生效')
        lay = QHBoxLayout(box)
        lay.setContentsMargins(6, 2, 6, 4)
        combo = QComboBox()
        combo.addItems(config.FRACTAL_CONFIGS)
        # 初始显示默认值不发 (未选择不发布, 节点用 aruco.yaml 的值;
        # setCurrentText 放在 connect 之前, 不触发 activated)
        combo.setCurrentText(config.FRACTAL_CONFIG_DEFAULT)
        combo.activated[str].connect(publish)   # 用户选择才发布 (含重复选择)
        lay.addWidget(combo, 1)
        self._fractal_box = box

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
        """顶部调节行随激活分组显隐: 字典切换属于 ArUco 检测 / aruco 降落;
        分形配置属于 fractal 检测 / fractal 降落 —— 两者互斥, 靠
        _landing_kind (反控降落的检测器类型, 见 set_active) 区分 landing
        分组归属, 不同时出现。码边长滑杆对两条链路都有意义 (同一降落坪);
        降落朝向偏置属于任何反控降落与 fractal 检测; 人脸任务只属于 face;
        全无则整行隐藏。"""
        landing_on = 'landing' in keys
        landing_kind = self._landing_kind or 'aruco'
        face_on = 'face' in keys
        aruco_on = 'aruco' in keys or (landing_on and landing_kind == 'aruco')
        fractal_on = 'fractal' in keys or (landing_on and landing_kind == 'fractal')
        if self._marker_box is not None:
            self._marker_box.setVisible(aruco_on or fractal_on)
        if self._yaw_box is not None:
            self._yaw_box.setVisible(landing_on or fractal_on)
        if self._dict_box is not None:
            self._dict_box.setVisible(aruco_on)
        if self._face_task_box is not None:
            self._face_task_box.setVisible(face_on)
        if self._face_topk_box is not None:
            self._face_topk_box.setVisible(face_on)
        if self._fractal_box is not None:
            self._fractal_box.setVisible(fractal_on)
        if self._top_row is not None:
            self._top_row.setVisible(
                aruco_on or face_on or fractal_on)

    def refresh(self):
        info = self._get_info() or {}
        now_m = time.monotonic()
        for key, section in self._sections.items():
            section.update_entry(info.get(key), now_m)

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
        """按当前顺序重排网格: 顶部调节行 (码边长滑杆 + 降落朝向选择 +
        字典切换 + 分形配置并排) 在第 0 行横跨全部分组列, 算法分组在其下
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
