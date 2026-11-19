#!/usr/bin/env python3
"""ULG 飞行日志分析窗口 —— PX4 .ulg 离线分析 (独立 QMainWindow, 非模态)。

布局:
  顶部工具条: 日志文件下拉 (自动扫描 SITL 日志目录) + 打开文件 + 状态/信息
  左栏: 话题树 (按字段勾选即添加曲线) + 字段搜索
  右侧 QTabWidget:
    曲线   预置模板下拉 (悬停振荡 FFT 诊断/高度/姿态/油门...) + 勾选的自选曲线,
           matplotlib 画布 (NavigationToolbar 缩放/平移)
    消息   飞控 ERROR/WARN/INFO 日志消息时间线
    参数   初始参数表 + 名字过滤

解析在后台线程执行 (大文件 1~2s), 经 pyqtSignal 回 GUI 线程刷新。
数据层见 ulg_analyzer.py (纯 pyulog/numpy, 不碰 Qt)。
"""
import os
import threading
import traceback

import numpy as np
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtWidgets import (QComboBox, QFileDialog, QHBoxLayout, QLabel,
                             QLineEdit, QMainWindow, QSplitter, QTabWidget,
                             QToolButton, QTreeWidget, QTreeWidgetItem,
                             QVBoxLayout, QWidget)
from PyQt5.QtGui import QColor

import matplotlib
matplotlib.use('Qt5Agg')
# 中文标题/标签: 系统装了 Noto CJK 才不豆腐块, 缺失时回退默认并接受警告
matplotlib.rcParams['font.sans-serif'] = ['Noto Sans CJK JP', 'Noto Sans CJK SC',
                                          'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False
from matplotlib.backends.backend_qt5agg import (FigureCanvasQTAgg,
                                                NavigationToolbar2QT)
from matplotlib.figure import Figure

from . import config
from . import ulg_analyzer as ua

# 勾选自选曲线的上限 (超过提示, 防止 figure 卡顿)
MAX_CURVES = 12

_CURVE_COLORS = ['#1f77b4', '#d62728', '#2ca02c', '#9467bd',
                 '#ff7f0e', '#8c564b', '#17becf', '#7f7f7f']

# ---------- 图内文字随窗口缩放 ----------
# 基准: 画布 1000px 宽时字号 = 基准值 × 1; 缩放系数夹在 [0.7, 2.5]。
# 只在 resize 防抖后就地改字号属性, 不重绘数据 (缩放/平移视图不丢)
FONT_BASE_WIDTH = 1000
_FONT_BASE = {   # (标题, 刻度, 图例, 标注, 总标题)
    'title': 9.0, 'tick': 7.0, 'legend': 7.0, 'annot': 8.0, 'suptitle': 10.0,
    'empty': 12.0,
}
FONT_SCALE_MIN, FONT_SCALE_MAX = 0.7, 2.5


class _Canvas(FigureCanvasQTAgg):
    """matplotlib 画布: resize 事件转信号 (窗口拖动时防抖后重排字号)。"""

    resized = pyqtSignal()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.resized.emit()


class UlgAnalyzerWindow(QMainWindow):
    """日志分析窗口: 一次创建常驻, 重复打开 bring-to-front。"""

    # 解析完成/失败 (后台线程 emit, GUI 线程接收刷新)
    _parsed = pyqtSignal(object, str)
    _load_failed = pyqtSignal(str, str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle('ULG 飞行日志分析')
        self.resize(1300, 800)
        self._ul = None            # 当前 ULog 对象
        self._path = None          # 当前文件路径
        self._loading = False      # 解析进行中 (防并发重复解析)
        self._selected = []        # 勾选自选曲线 [(topic_id, field)] 有序
        self._tree_updating = False   # 树批量填充/勾选联动时挂起 itemChanged

        self._parsed.connect(self._on_parsed)
        self._load_failed.connect(self._on_load_failed)
        self._build_ui()
        self._refresh_files()

    # ---------- UI 组装 ----------

    def _build_ui(self):
        central = QWidget()
        lay = QVBoxLayout(central)
        lay.setContentsMargins(6, 6, 6, 6)
        lay.setSpacing(4)

        # 顶部工具条: 文件下拉 + 刷新 + 打开文件 + 信息行
        top = QHBoxLayout()
        self.file_combo = QComboBox()
        self.file_combo.setMinimumWidth(320)
        self.file_combo.currentIndexChanged.connect(self._on_file_selected)
        refresh_btn = QToolButton()
        refresh_btn.setText('刷新')
        refresh_btn.clicked.connect(self._refresh_files)
        open_btn = QToolButton()
        open_btn.setText('打开文件...')
        open_btn.clicked.connect(self._on_open_file)
        top.addWidget(QLabel('日志:'))
        top.addWidget(self.file_combo, stretch=1)
        top.addWidget(refresh_btn)
        top.addWidget(open_btn)
        lay.addLayout(top)
        self.info_label = QLabel('未加载日志')
        self.info_label.setStyleSheet('color: #555;')
        lay.addWidget(self.info_label)

        # 左话题树 + 右 Tab 页
        splitter = QSplitter(Qt.Horizontal)
        lay.addWidget(splitter, stretch=1)
        splitter.addWidget(self._build_topic_panel())
        self.tabs = QTabWidget()
        splitter.addWidget(self.tabs)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([300, 900])
        self.tabs.addTab(self._build_plot_tab(), '曲线')
        self.tabs.addTab(self._build_msg_tab(), '消息')
        self.tabs.addTab(self._build_param_tab(), '参数')
        self.setCentralWidget(central)

    def _build_topic_panel(self):
        """左栏: 字段搜索 + 话题树 (字段可勾选 → 添加/移除曲线)。"""
        panel = QWidget()
        lay = QVBoxLayout(panel)
        lay.setContentsMargins(0, 0, 0, 0)
        self.topic_filter = QLineEdit()
        self.topic_filter.setPlaceholderText('搜索话题/字段...')
        self.topic_filter.setClearButtonEnabled(True)
        self.topic_filter.textChanged.connect(self._filter_tree)
        lay.addWidget(self.topic_filter)
        self.topic_tree = QTreeWidget()
        self.topic_tree.setHeaderLabels(['话题 / 字段'])
        self.topic_tree.itemChanged.connect(self._on_tree_item_changed)
        lay.addWidget(self.topic_tree)
        return panel

    def _build_plot_tab(self):
        tab = QWidget()
        lay = QVBoxLayout(tab)
        lay.setContentsMargins(0, 4, 0, 0)
        # 模板行: 预置下拉 + 清空勾选
        bar = QHBoxLayout()
        bar.addWidget(QLabel('模板:'))
        self.preset_combo = QComboBox()
        self.preset_combo.addItems(list(ua.PRESETS.keys()))
        self.preset_combo.currentIndexChanged.connect(
            lambda _: self._redraw_plots())
        clear_btn = QToolButton()
        clear_btn.setText('清空自选曲线')
        clear_btn.clicked.connect(self._clear_selected)
        bar.addWidget(self.preset_combo, stretch=1)
        bar.addWidget(clear_btn)
        lay.addLayout(bar)
        self.note_label = QLabel('')
        self.note_label.setStyleSheet('color: #a05a00;')
        self.note_label.setWordWrap(True)
        lay.addWidget(self.note_label)
        # matplotlib 画布 + 导航条 (缩放/平移/回位); resize 防抖后图内
        # 标题/图例/刻度字号随画布宽度缩放 (_apply_font_scale)
        self.fig = Figure(figsize=(8, 5), dpi=100)
        self.canvas = _Canvas(self.fig)
        self.canvas.setStyleSheet('background: white;')
        self.navbar = NavigationToolbar2QT(self.canvas, tab)
        lay.addWidget(self.navbar)
        lay.addWidget(self.canvas, stretch=1)
        self._font_timer = QTimer(self)          # resize 防抖 (拖动结束才重排)
        self._font_timer.setSingleShot(True)
        self._font_timer.setInterval(150)
        self._font_timer.timeout.connect(self._apply_font_scale)
        self.canvas.resized.connect(self._font_timer.start)
        self._draw_empty('选择模板或勾选左侧话题字段绘制曲线')
        return tab

    def _build_msg_tab(self):
        tab = QWidget()
        lay = QVBoxLayout(tab)
        lay.setContentsMargins(0, 4, 0, 0)
        self.msg_tree = QTreeWidget()
        self.msg_tree.setHeaderLabels(['时间 (s)', '级别', '消息'])
        self.msg_tree.setColumnWidth(0, 100)
        self.msg_tree.setAlternatingRowColors(True)
        lay.addWidget(self.msg_tree)
        return tab

    def _build_param_tab(self):
        tab = QWidget()
        lay = QVBoxLayout(tab)
        lay.setContentsMargins(0, 4, 0, 0)
        self.param_filter = QLineEdit()
        self.param_filter.setPlaceholderText('按参数名过滤...')
        self.param_filter.setClearButtonEnabled(True)
        self.param_filter.textChanged.connect(self._refresh_params)
        lay.addWidget(self.param_filter)
        self.param_tree = QTreeWidget()
        self.param_tree.setHeaderLabels(['参数', '值'])
        self.param_tree.setColumnWidth(0, 320)
        self.param_tree.setAlternatingRowColors(True)
        lay.addWidget(self.param_tree)
        return tab

    # ---------- 文件加载 ----------

    def _refresh_files(self):
        """重扫 SITL 日志目录, 保持当前选中文件 (列表变化才重建下拉)。"""
        files = ua.scan_log_dir(config.ULG_LOG_ROOT)
        names = [ua.display_name(f, config.ULG_LOG_ROOT) for f in files]
        cur = self.file_combo.currentData()
        if names == [self.file_combo.itemText(i)
                     for i in range(self.file_combo.count())]:
            return
        self.file_combo.blockSignals(True)
        self.file_combo.clear()
        for f, name in zip(files, names):
            self.file_combo.addItem(name, f)
        if cur is not None:
            i = files.index(cur) if cur in files else -1
            if i >= 0:
                self.file_combo.setCurrentIndex(i)
        self.file_combo.blockSignals(False)
        if files and self._path is None:
            self._load_path(files[0])   # 首次自动加载最新日志

    def _on_file_selected(self, _):
        path = self.file_combo.currentData()
        if path:
            self._load_path(path)

    def _default_open_dir(self):
        """文件对话框默认目录: SITL 日志根目录; 手动开过外部文件时退其所在
        目录, 日志根不存在再退家目录。"""
        if self._path and os.path.isfile(self._path) \
                and not self._path.startswith(config.ULG_LOG_ROOT):
            return os.path.dirname(self._path)
        root = config.ULG_LOG_ROOT
        return root if os.path.isdir(root) else os.path.expanduser('~')

    def _on_open_file(self):
        path, _ = QFileDialog.getOpenFileName(
            self, '选择 ULG 日志', self._default_open_dir(),
            'ULog (*.ulg);;所有文件 (*)')
        if path:
            self._load_path(path)

    def _load_path(self, path):
        """后台线程解析 (防重入: 解析中忽略新请求)。"""
        if self._loading:
            return
        self._loading = True
        self._path = path
        self.info_label.setText(f'解析中: {ua.display_name(path, config.ULG_LOG_ROOT)} ...')
        threading.Thread(target=self._parse_worker, args=(path,),
                         daemon=True).start()

    def _parse_worker(self, path):
        try:
            ul = ua.load_ulog(path)
        except Exception:
            self._load_failed.emit(path, traceback.format_exc(limit=3))
            return
        self._parsed.emit(ul, path)

    def _on_load_failed(self, path, tb):
        self._loading = False
        self.info_label.setText(f'解析失败: {path}')
        print(f'ULG 解析失败 {path}:\n{tb}')

    def _on_parsed(self, ul, path):
        """解析完成 (GUI 线程): 填充话题树/消息/参数 + 默认模板出图。"""
        self._loading = False
        self._ul = ul
        self._path = path
        self._selected = []
        self.info_label.setText(ua.ulog_info(ul, path))
        self._fill_topic_tree()
        self._fill_messages()
        self.param_filter.setText('')   # 触发参数表重填
        self._refresh_params()
        self._redraw_plots()

    # ---------- 话题树 ----------

    def _fill_topic_tree(self):
        self._tree_updating = True
        self.topic_tree.clear()
        tree = ua.topic_tree(self._ul)
        for topic_id, fields in tree.items():
            t_item = QTreeWidgetItem([topic_id])
            t_item.setFlags(t_item.flags() & ~Qt.ItemIsSelectable)
            for f in fields:
                f_item = QTreeWidgetItem([f])
                f_item.setData(0, Qt.UserRole, (topic_id, f))
                f_item.setFlags(f_item.flags() | Qt.ItemIsUserCheckable)
                f_item.setCheckState(0, Qt.Unchecked)
                t_item.addChild(f_item)
            self.topic_tree.addTopLevelItem(t_item)
        self._tree_updating = False
        self._filter_tree()

    def _filter_tree(self):
        """按搜索词过滤话题树: 话题名或任一字段命中则整棵话题保留。"""
        kw = self.topic_filter.text().lower()
        for i in range(self.topic_tree.topLevelItemCount()):
            t_item = self.topic_tree.topLevelItem(i)
            topic_hit = kw in t_item.text(0).lower() if kw else True
            visible = topic_hit
            for j in range(t_item.childCount()):
                c = t_item.child(j)
                hit = (not kw) or topic_hit or kw in c.text(0).lower()
                c.setHidden(not hit)
                visible = visible or hit
            t_item.setHidden(not visible)

    def _on_tree_item_changed(self, item, _col):
        """字段勾选 → 维护自选曲线列表 (有序) 并重绘。"""
        if self._tree_updating:
            return
        key = item.data(0, Qt.UserRole)
        if key is None:
            return
        checked = item.checkState(0) == Qt.Checked
        if checked and key not in self._selected:
            if len(self._selected) >= MAX_CURVES:
                self._tree_updating = True
                item.setCheckState(0, Qt.Unchecked)
                self._tree_updating = False
                self.note_label.setText(
                    f'最多同时显示 {MAX_CURVES} 条自选曲线')
                return
            self._selected.append(key)
        elif not checked and key in self._selected:
            self._selected.remove(key)
        self._redraw_plots()

    def _clear_selected(self):
        """清空自选曲线 (树勾选态同步复位)。"""
        self._tree_updating = True
        for i in range(self.topic_tree.topLevelItemCount()):
            t = self.topic_tree.topLevelItem(i)
            for j in range(t.childCount()):
                t.child(j).setCheckState(0, Qt.Unchecked)
        self._tree_updating = False
        self._selected = []
        self._redraw_plots()

    # ---------- 消息 / 参数 ----------

    def _fill_messages(self):
        self.msg_tree.clear()
        for t, level, text in ua.logged_messages(self._ul):
            item = QTreeWidgetItem([f'{t:.2f}', level, text])
            if level in ('ERROR', 'CRIT', 'ALERT'):
                item.setForeground(2, QColor('#c0392b'))
            elif level == 'WARN':
                item.setForeground(2, QColor('#a05a00'))
            self.msg_tree.addTopLevelItem(item)

    def _refresh_params(self):
        self.param_tree.clear()
        kw = self.param_filter.text()
        for name, val in ua.filter_params(self._ul, kw):
            self.param_tree.addTopLevelItem(QTreeWidgetItem([name, str(val)]))

    # ---------- 绘图 ----------

    def _draw_empty(self, text):
        self.fig.clf()
        ax = self.fig.add_subplot(111)
        ax.axis('off')
        txt = ax.text(0.5, 0.5, text, ha='center', va='center',
                      fontsize=_FONT_BASE['empty'],
                      color='#888', transform=ax.transAxes)
        txt._font_base = _FONT_BASE['empty']   # 缩放基准标记 (见 _apply_font_scale)
        self.canvas.draw_idle()
        self._apply_font_scale()

    def _apply_font_scale(self):
        """图内文字 (标题/刻度/图例/标注/总标题) 随画布宽度缩放。
        就地改已绘制元素的字号属性, 不重建 figure —— NavigationToolbar
        的缩放/平移视图得以保留; 缩放系数 = 画布宽 / 基准宽 (夹限)。"""
        scale = max(FONT_SCALE_MIN,
                    min(FONT_SCALE_MAX,
                        self.canvas.width() / FONT_BASE_WIDTH))
        for ax in self.fig.axes:
            if ax.get_title():
                ax.title.set_fontsize(_FONT_BASE['title'] * scale)
            for lbl in (ax.get_xticklabels() + ax.get_yticklabels()):
                lbl.set_fontsize(_FONT_BASE['tick'] * scale)
            ax.xaxis.label.set_fontsize(_FONT_BASE['tick'] * scale)
            ax.yaxis.label.set_fontsize(_FONT_BASE['tick'] * scale)
            leg = ax.get_legend()
            if leg is not None:
                for txt in leg.get_texts():
                    txt.set_fontsize(_FONT_BASE['legend'] * scale)
            for txt in ax.texts:   # annotate/空态提示等 (各有自己的基准)
                txt.set_fontsize(getattr(txt, '_font_base',
                                         _FONT_BASE['annot']) * scale)
        st = getattr(self.fig, '_suptitle', None)
        if st is not None:
            st.set_fontsize(_FONT_BASE['suptitle'] * scale)
        self.fig.tight_layout()
        self.canvas.draw_idle()

    def _redraw_plots(self):
        """按当前模板 (+ 自选曲线) 重建整个 figure。"""
        if self._ul is None:
            self._draw_empty('未加载日志')
            return
        self.note_label.setText('')
        preset = self.preset_combo.currentText()
        try:
            if preset == '悬停振荡诊断 (FFT)':
                self._plot_hover_diagnosis()
            elif preset == '姿态 (欧拉角)':
                self._plot_attitude()
            else:
                self._plot_preset(preset)
        except Exception:
            self._draw_empty('绘图出错, 详见终端')
            print('ULG 绘图异常:\n' + traceback.format_exc(limit=5))

    def _plot_preset(self, preset):
        """普通模板: 模板定义的子图 + 自选曲线各占一行。"""
        groups = []
        for title, curves in ua.PRESETS[preset]:
            cs = []
            for topic_id, field, label in curves:
                s = ua.get_series(self._ul, topic_id, field)
                if s is not None:
                    cs.append((s[0], s[1], label))
            if cs:
                groups.append((title, cs))
        groups += self._selected_groups()
        self._draw_groups(groups)

    def _selected_groups(self):
        """勾选自选曲线 → 单曲线子图组 (话题.字段 为标签)。"""
        groups = []
        for topic_id, field in self._selected:
            s = ua.get_series(self._ul, topic_id, field)
            if s is not None:
                groups.append(('', [(s[0], s[1], f'{topic_id}.{field}')]))
        return groups

    def _plot_attitude(self):
        """姿态模板: 四元数 → 欧拉角, 实际与设定各占一行。"""
        groups = []
        for topic_id, title in (
                ('vehicle_attitude[0]', '姿态实际 (deg)'),
                ('vehicle_attitude_setpoint[0]', '姿态设定 (deg)')):
            rows = ua.euler_series(self._ul, topic_id,
                                   'q' if 'setpoint' not in topic_id else 'q_d')
            if rows:
                groups.append((title, rows))
        groups += self._selected_groups()
        if not groups:
            self._draw_empty('日志中无姿态四元数数据')
            return
        self._draw_groups(groups)

    def _plot_hover_diagnosis(self):
        """悬停振荡诊断: 2x2 (z 曲线 / vz 曲线 / z 频谱 / vz 频谱) + 摘要。"""
        d = ua.hover_diagnosis(self._ul)
        self.note_label.setText(d['note'] or '无离地飞行段')
        if d['t'] is None:
            self._draw_empty(d['note'] or '日志中没有离地飞行段')
            return
        self.fig.clf()
        axes = self.fig.subplots(2, 2)
        ax = axes[0][0]
        ax.plot(d['t'], d['z'], color='#1f77b4', lw=0.8)
        ax.set_title('高度 z (NED 下+) / m', fontsize=9)
        ax.grid(alpha=0.3)
        ax = axes[0][1]
        if d['vz'] is not None:
            ax.plot(d['t_vz'], d['vz'], color='#d62728', lw=0.8)
        ax.set_title('垂直速度 vz (下+) / m/s', fontsize=9)
        ax.grid(alpha=0.3)
        if d['f'] is not None:
            ax = axes[1][0]
            ax.semilogy(d['f'][1:], d['amp_z'][1:], color='#1f77b4', lw=0.8)
            if d['f_dom_z'] is not None:
                ax.axvline(d['f_dom_z'], color='#d62728', ls='--', lw=0.8)
                ax.annotate(f'{d["f_dom_z"]:.2f} Hz',
                            (d['f_dom_z'], d['amp_z'][np.argmin(
                                np.abs(d['f'] - d['f_dom_z']))]),
                            color='#d62728', fontsize=8)
            ax.set_title('z 幅值谱 (Hann 窗)', fontsize=9)
            ax.set_xlabel('Hz')
            ax.grid(alpha=0.3)
            ax = axes[1][1]
            if d['amp_vz'] is not None:
                ax.semilogy(d['f'][1:], d['amp_vz'][1:], color='#d62728',
                            lw=0.8)
            if d['f_dom_vz'] is not None:
                ax.axvline(d['f_dom_vz'], color='#1f77b4', ls='--', lw=0.8)
                ax.annotate(f'{d["f_dom_vz"]:.2f} Hz',
                            (d['f_dom_vz'], d['amp_vz'][np.argmin(
                                np.abs(d['f'] - d['f_dom_vz']))]),
                            color='#1f77b4', fontsize=8)
            ax.set_title('vz 幅值谱 (Hann 窗)', fontsize=9)
            ax.set_xlabel('Hz')
            ax.grid(alpha=0.3)
        self.fig.suptitle('悬停振荡诊断 (最长离地段)', fontsize=10)
        self.fig.tight_layout()
        self._apply_font_scale()

    def _draw_groups(self, groups):
        """通用多子图绘制: 每组一行, 组内多条曲线同轴叠画。"""
        if not groups:
            self._draw_empty('没有可绘制的数据 (话题/字段在日志中不存在)')
            return
        self.fig.clf()
        n = len(groups)
        axes = self.fig.subplots(n, 1, squeeze=False)
        for ax, (title, curves) in zip(axes[:, 0], groups):
            for k, (t, y, label) in enumerate(curves):
                ax.plot(t, y, lw=0.8,
                        color=_CURVE_COLORS[k % len(_CURVE_COLORS)],
                        label=label)
            if title:
                ax.set_title(title, fontsize=9)
            ax.grid(alpha=0.3)
            if len(curves) > 1:
                ax.legend(fontsize=7, loc='best', ncol=min(len(curves), 4))
        self.fig.tight_layout()
        self._apply_font_scale()
