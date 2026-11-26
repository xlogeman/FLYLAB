#!/usr/bin/env python3
"""offboard 功能互斥决策表 —— 纯决策, 不碰任何 UI 对象。

接管控制下的三个 offboard 功能 (手动遥控 / 视觉反控降落 / 目标追踪反控)
同一时刻只运行一个, 门控规则原来内联在 landing_panel 的
_update_takeover_gates 里 (if/elif 分支直接 setEnabled), 主窗口飞控按钮
门控又把互斥条件抄了一遍。现在决策收敛为本模块的 decide_offboard:
输入运行状态, 输出 OffboardDecision (各功能的 enabled/锁定/联动),
调用方只负责把决策落到 UI 上 —— 加新 offboard 功能 (或子模式) 只改
这里的决策表, 两个消费方 (landing_panel 门控轮询 / main_window 飞控
按钮) 不动。

决策表 (优先级 = 判定顺序):
  遥控运行中     锁链路, 云台不锁 (摇杆要转云台), 反控可点 (接管时
                 自动取消遥控, 防炸机出口)
  视觉反控降落中 锁链路, 锁云台, 模式互斥 (现役模式保持可点再点即
                 取消, 另一模式置灰), 遥控可点 (抢回)
  目标追踪反控中 锁链路, 锁云台, 遥控可点 (抢回), 视觉反控置灰;
                 子模式 (打击/跟随) 可用性前提 = 悬停跟拍运行中
  空闲           链路可用, 反控按各自完整条件 (算法运行中 + 已解锁
                 升空 + AirSim 源 + 链路已开, 由调用方算好传入)

航向跟随按钮 (双语义): 手动遥控中 = 航向跟随外环开关 (默认开且锁定),
目标追踪反控中 = 机头跟随开关 (经 /flylab/track_follow 下发节点);
两种语义都需 AirSim 源在线。
"""


class OffboardDecision:
    """一次门控决策的输出 (各字段语义见 decide_offboard)。"""

    def __init__(self):
        self.link_enabled = False        # 通信链路按钮 (任一 offboard 运行中锁)
        self.gimbal_locked = False       # 云台/变焦锁定 (交给反控算法)
        self.teleop_enabled = False      # 手动遥控菜单项
        self.landing_enabled = False     # 视觉反控整组 (降落运行中恒 True 以便取消)
        self.landing_mode_mutex = False  # 降落运行中: 非现役模式置灰 (互斥)
        self.gimbal_act_enabled = False  # 悬停跟拍项 (目标追踪反控运行中恒 True)
        self.stub_enabled = False        # 打击/跟随四子项 (前提 = 悬停跟拍运行中)
        self.follow_mode = 'teleop'      # 跟随按钮语义: 'teleop' | 'track'
        self.follow_available = False    # 跟随按钮可用 (双语义共用门控)
        self.btn_on = False              # 接管按钮状态灯 (任一功能运行中)


def decide_offboard(*, link_ok, landing_ok, takeover_ok, airsim_ok,
                    teleop_live, teleop_on, landing_on, gimbal_on, stub_on):
    """按当前运行状态产出 offboard 门控决策。

    输入:
      link_ok      通信链路已选 (mavros/uxrce)
      landing_ok   视觉反控完整条件满足 (码检测运行中 + 已解锁升空 +
                   AirSim 源 + 链路已开, 调用方采集)
      takeover_ok  目标追踪反控完整条件满足 (追踪算法运行中 + 同上)
      airsim_ok    AirSim 源在线且云台姿态可得 (跟随按钮前提)
      teleop_live  遥控接管成功 (offboard 通道可用, teleop 状态回传)
      teleop_on / landing_on / gimbal_on / stub_on
                   各菜单项当前勾选状态 (gimbal=悬停跟拍,
                   stub=俯冲打击两项/机体跟随两项任一)

    注意: 悬停跟拍弹起时运行中的子项自动弹起 (前提失效兜底联动) 由调用
    方在调用本函数【之前】执行并把弹起后的 stub_on 传入 —— 取消与决策
    同拍生效, 否则会按过期的 track_on 多锁一拍 (33ms)。
    """
    d = OffboardDecision()
    track_on = gimbal_on or stub_on
    d.btn_on = teleop_on or landing_on or track_on
    d.follow_mode = 'track' if track_on else 'teleop'
    # 跟随按钮: 遥控中 (需接管成功 + 链路) 或目标追踪反控中可开, 均需 AirSim 源
    d.follow_available = ((teleop_live and link_ok) or track_on) and airsim_ok
    # 打击/跟随子项前提 = 悬停跟拍运行中, 各分支同一规则
    d.stub_enabled = gimbal_on

    if teleop_on:
        # 遥控运行中: 锁链路; 云台不锁 (摇杆要转云台); 反控仍可点
        # (接管时自动取消遥控), 遥控项保持可点 (再点即取消)
        d.link_enabled = False
        d.teleop_enabled = True
        d.landing_enabled = landing_ok
        d.gimbal_act_enabled = takeover_ok
    elif landing_on:
        # 视觉反控降落中: 锁链路 + 云台; 遥控可点 (抢回); 模式互斥
        # (运行中不存在模式切换, 只能先取消再换); 追踪反控置灰 (双保险)
        d.link_enabled = False
        d.gimbal_locked = True
        d.teleop_enabled = link_ok
        d.landing_enabled = True
        d.landing_mode_mutex = True
    elif track_on:
        # 目标追踪反控中: 锁链路 + 云台; 遥控可点 (抢回); 视觉反控置灰
        # (双保险); 悬停跟拍保持可点 (再点即取消交还)
        d.link_enabled = False
        d.gimbal_locked = True
        d.teleop_enabled = link_ok
        d.landing_enabled = False
        d.gimbal_act_enabled = True
    else:
        # 空闲: 链路可用, 反控按完整条件门控
        d.link_enabled = True
        d.teleop_enabled = link_ok
        d.landing_enabled = landing_ok
        d.gimbal_act_enabled = takeover_ok
    return d
