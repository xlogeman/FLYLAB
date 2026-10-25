#!/usr/bin/env python3
"""QGC 式遥测数值条 —— 画面下方六格数值 (日志框之上)。

数据来自 VehicleController 心跳采样的 get_telemetry() (AirSim kinematics,
不依赖 mavros/uxrce 链路): 水平速度/垂直速度/高度(相对起飞点)/航向/
累计行程/飞行时间。航向连接 PX4 即有效 (不等起飞), 其余未起飞时
显示 "--", 行程/时长保留上次飞行 (下次起飞清零重计)。
"""
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QFrame, QHBoxLayout, QLabel, QVBoxLayout, QWidget


class TelemetryBar(QFrame):
    """六格遥测条: 每格上小字名称、下等宽字体数值。"""

    ITEMS = [   # (key, 名称, 单位, 数值格式)
        ('hs',   '水平速度', 'm/s', '{:.1f}'),
        ('vs',   '垂直速度', 'm/s', '{:+.1f}'),
        ('alt',  '高度',     'm',   '{:.1f}'),
        ('hdg',  '航向',     '°',   '{:.0f}'),
        ('dist', '累计行程', 'm',   '{:.0f}'),
        ('ftime', '飞行时间', '',    None),
    ]

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFrameShape(QFrame.StyledPanel)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(8, 2, 8, 2)
        lay.setSpacing(0)
        self._cells = {}
        for i, (key, name, unit, fmt) in enumerate(self.ITEMS):
            if i:
                sep = QLabel('|')
                sep.setStyleSheet('color: #ccc;')
                lay.addWidget(sep)
            cell = QWidget()
            v = QVBoxLayout(cell)
            v.setContentsMargins(0, 0, 0, 0)
            v.setSpacing(0)
            lab = QLabel(name)
            lab.setStyleSheet('color: #888; font-size: 10px;')
            lab.setAlignment(Qt.AlignHCenter)
            val = QLabel('--')
            val.setStyleSheet(
                'font-family: monospace; font-size: 14px; font-weight: bold;')
            val.setAlignment(Qt.AlignHCenter)
            v.addWidget(lab)
            v.addWidget(val)
            lay.addWidget(cell, stretch=1)
            self._cells[key] = (val, unit, fmt)

    def update_values(self, tele):
        """刷新六格 (tele 为 VehicleController.get_telemetry() 的 dict)。"""
        flying = tele.get('flying', False)
        for key, (val, unit, fmt) in self._cells.items():
            v = tele.get(key)
            if v is None:
                text = '--:--' if key == 'ftime' else '--'   # 尚无有效数据
            elif key == 'ftime':
                text = (f'{int(v) // 60:02d}:{int(v) % 60:02d}'
                        if v > 0 else '--:--')
            elif not flying and v == 0 and key != 'hdg':
                text = '--'          # 未起飞且为零: 高度/速度/行程显示 "--" (航向连接即有效)
            else:
                text = fmt.format(v)
                if unit:
                    text += f' {unit}'
            if val.text() != text:
                val.setText(text)
