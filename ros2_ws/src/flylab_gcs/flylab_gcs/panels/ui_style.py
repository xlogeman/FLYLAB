#!/usr/bin/env python3
"""工具栏按钮统一样式 + 菜单选中态绿点 (各功能域面板模块共用, 见 main_window 的各 mixin)。"""

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QIcon, QPainter, QPixmap

# 状态灯/绿点点亮色 (按钮点亮、菜单选中态共用)
GREEN_COLOR = '#2d8a3e'
# 菜单"停止当前算法"项的标记色 (与绿点区分, 一眼看出是停止动作)
RED_COLOR = '#c0392b'

# QToolButton 点亮态: 动态属性 on=true (状态灯: PX4 运行中 / 链路开启 /
# 飞控控制中 / 算法运行中), 由 main_window._set_btn_on 切换。
# 注意: 菜单项一律不用 setCheckable —— 可勾选菜单项会被 Qt 画成空心
# 方块 (白框) 且 QSS 无法消除; 菜单选中/运行态统一用绿点图标标记
# (见 green_dot_icon / set_menu_icon)
TOOL_BTN_STYLE = """
    QToolButton { min-height: 24px; padding: 2px 10px; border: 1px solid #bbb;
                  border-radius: 4px; background: #f6f6f6; }
    QToolButton:hover { background: #e8e8e8; }
    QToolButton:checked { background-color: #2d8a3e; color: white;
                          border-color: #2d8a3e; }
    QToolButton[on="true"] { background-color: #2d8a3e; color: white;
                             border-color: #2d8a3e; }
"""


def green_dot_icon(color=GREEN_COLOR):
    """运行中/选中态的圆点图标 (纯绘制, 无需图标资源); 默认绿色 (状态灯
    同色), 传 color 可出其他色 (停止项用红色)。"""
    pm = QPixmap(12, 12)
    pm.fill(Qt.transparent)
    p = QPainter(pm)
    p.setRenderHint(QPainter.Antialiasing)
    p.setPen(Qt.NoPen)
    p.setBrush(QColor(color))
    p.drawEllipse(2, 2, 8, 8)
    p.end()
    return QIcon(pm)


def set_menu_icon(act, on):
    """按布尔量设置菜单项绿点图标 (on=False 清空)。"""
    act.setIcon(green_dot_icon() if on else QIcon())


def set_stop_menu_icon(act, on):
    """停止动作菜单项的红点图标: 运行中亮红点, 无算法运行时清空
    (置灰用 setEnabled, 图标无需区分)。"""
    act.setIcon(green_dot_icon(RED_COLOR) if on else QIcon())
