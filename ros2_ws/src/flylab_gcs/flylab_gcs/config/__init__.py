#!/usr/bin/env python3
"""flylab_gcs 全局配置 —— 按域拆分的包, 改对应子模块。

子模块 (经下方 star 导入全部平铺到 config.* 命名空间, 调用方
`from . import config` 用法不变):
  paths.py      路径/端口/窗口常量 (vision 参数目录、PX4 脚本、ULG 日志)
  topics.py     ROS 话题名 + QoS 开关/新鲜度/超时 (与节点侧一一对应)
  gimbal.py     云台/航向跟随控制律 + 数字变焦/FOV (teleop 偏航上限同源于此)
  ui_ranges.py  面板控件值域 + 枚举注册表 + 手柄映射 (latched 参数界面侧)
  commands.py   进程启动命令 + 指令字符串 + 反控模式枚举 + 飞控命令
  registry.py   算法/飞行链路/机型注册表 (新算法、新链路、新机型加这里)

latched 参数同步管道的话题/值域在 topics/ui_ranges, 控件声明见
latched_sync.LATCHED_PARAMS —— 新增一个同步参数共改三处, 界面/发布侧零改动。
"""

from .paths import *          # noqa: F401,F403
from .gimbal import *         # noqa: F401,F403
from .topics import *         # noqa: F401,F403
from .ui_ranges import *      # noqa: F401,F403
from .commands import *       # noqa: F401,F403
from .registry import *       # noqa: F401,F403
