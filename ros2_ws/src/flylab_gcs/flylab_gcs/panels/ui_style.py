#!/usr/bin/env python3
"""工具栏按钮统一样式 (各功能域面板模块共用, 见 main_window 的各 mixin)。"""

# QToolButton 点亮态: 动态属性 on=true (状态灯: PX4 运行中 / 链路开启 /
# 飞控控制中 / 算法运行中), 由 main_window._set_btn_on 切换
TOOL_BTN_STYLE = """
    QToolButton { min-height: 24px; padding: 2px 10px; border: 1px solid #bbb;
                  border-radius: 4px; background: #f6f6f6; }
    QToolButton:hover { background: #e8e8e8; }
    QToolButton:checked { background-color: #2d8a3e; color: white;
                          border-color: #2d8a3e; }
    QToolButton[on="true"] { background-color: #2d8a3e; color: white;
                             border-color: #2d8a3e; }
"""
