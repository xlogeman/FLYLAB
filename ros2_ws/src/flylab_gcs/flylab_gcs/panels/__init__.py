"""工具栏功能域面板 (MainWindow 的 mixin 模块) —— 仅 main_window 引用。

每个文件一个功能域: 机型/PX4、通信链路、飞行控制、视觉算法、飞行链路、
视觉反控 (含接管控制菜单本体 + 门控应用)、目标追踪反控 (子项构建与启停
流程)、手动遥控; offboard 互斥决策表在 offboard_gates.py (三功能共用),
按钮样式常量在 ui_style.py。
"""
