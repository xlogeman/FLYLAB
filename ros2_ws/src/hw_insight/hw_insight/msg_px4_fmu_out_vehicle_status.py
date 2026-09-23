#!/usr/bin/env python3

'''
# AI初版，没有认为校验
# Encodes the system state of the vehicle published by commander
# 对指挥官发布的飞行器系统状态进行编码

uint64 timestamp # 自系统启动以来的时间（微秒）

uint64 armed_time # 解锁时间戳（微秒）
uint64 takeoff_time # 起飞时间戳（微秒）

uint8 arming_state # 解锁状态
uint8 ARMING_STATE_DISARMED = 1 # 未解锁状态
uint8 ARMING_STATE_ARMED    = 2 # 已解锁状态

uint8 latest_arming_reason # 最近一次解锁原因
uint8 latest_disarming_reason # 最近一次上锁原因
uint8 ARM_DISARM_REASON_TRANSITION_TO_STANDBY = 0 # 过渡到待命状态的解锁/上锁原因.当无人机系统需要从其他状态（如关机、故障恢复等）切换到 “待命状态”（可操作但未起飞）时触发的解锁或上锁操作。例如系统初始化完成后进入可操作状态时的解锁，或任务结束后切换回低功耗待命状态时的上锁。
uint8 ARM_DISARM_REASON_RC_STICK = 1 # 遥控器摇杆操作的解锁/上锁原因
uint8 ARM_DISARM_REASON_RC_SWITCH = 2 # 遥控器开关操作的解锁/上锁原因
uint8 ARM_DISARM_REASON_COMMAND_INTERNAL = 3 # 内部命令的解锁/上锁原因
uint8 ARM_DISARM_REASON_COMMAND_EXTERNAL = 4 # 外部命令的解锁/上锁原因
uint8 ARM_DISARM_REASON_MISSION_START = 5 # 任务开始的解锁/上锁原因
uint8 ARM_DISARM_REASON_SAFETY_BUTTON = 6 # 安全按钮操作的解锁/上锁原因
uint8 ARM_DISARM_REASON_AUTO_DISARM_LAND = 7 # 自动降落上锁原因
uint8 ARM_DISARM_REASON_AUTO_DISARM_PREFLIGHT = 8 # 起飞前自动上锁原因
uint8 ARM_DISARM_REASON_KILL_SWITCH = 9 # 急停开关操作的解锁/上锁原因
uint8 ARM_DISARM_REASON_LOCKDOWN = 10 # 锁定状态的解锁/上锁原因
uint8 ARM_DISARM_REASON_FAILURE_DETECTOR = 11 # 故障检测触发的解锁/上锁原因
uint8 ARM_DISARM_REASON_SHUTDOWN = 12 # 关机操作的解锁/上锁原因
uint8 ARM_DISARM_REASON_UNIT_TEST = 13 # 单元测试的解锁/上锁原因

uint64 nav_state_timestamp # 当前导航状态激活的时间

uint8 nav_state_user_intention # 用户选择的模式（在故障保护情况下可能与nav_state不同）

uint8 nav_state # 当前激活的模式
uint8 NAVIGATION_STATE_MANUAL = 0 # 手动模式
uint8 NAVIGATION_STATE_ALTCTL = 1 # 高度控制模式
uint8 NAVIGATION_STATE_POSCTL = 2 # 位置控制模式
uint8 NAVIGATION_STATE_AUTO_MISSION = 3 # 自动任务模式
uint8 NAVIGATION_STATE_AUTO_LOITER = 4 # 自动悬停模式
uint8 NAVIGATION_STATE_AUTO_RTL = 5 # 自动返航模式
uint8 NAVIGATION_STATE_POSITION_SLOW = 6 # 慢速位置控制模式
uint8 NAVIGATION_STATE_FREE5 = 7 # 自由模式5
uint8 NAVIGATION_STATE_FREE4 = 8 # 自由模式4
uint8 NAVIGATION_STATE_FREE3 = 9 # 自由模式3
uint8 NAVIGATION_STATE_ACRO = 10 # 特技模式
uint8 NAVIGATION_STATE_FREE2 = 11 # 自由模式2
uint8 NAVIGATION_STATE_DESCEND = 12 # 下降模式（无位置控制）
uint8 NAVIGATION_STATE_TERMINATION = 13 # 终止模式
uint8 NAVIGATION_STATE_OFFBOARD = 14 # 外部控制模式
uint8 NAVIGATION_STATE_STAB = 15 # 稳定模式
uint8 NAVIGATION_STATE_FREE1 = 16 # 自由模式1
uint8 NAVIGATION_STATE_AUTO_TAKEOFF = 17 # 自动起飞模式
uint8 NAVIGATION_STATE_AUTO_LAND = 18 # 自动降落模式
uint8 NAVIGATION_STATE_AUTO_FOLLOW_TARGET = 19 # 自动跟随模式
uint8 NAVIGATION_STATE_AUTO_PRECLAND = 20 # 精确降落模式（有降落目标）
uint8 NAVIGATION_STATE_ORBIT = 21 # 环绕模式
uint8 NAVIGATION_STATE_AUTO_VTOL_TAKEOFF = 22 # 垂直起降自动起飞模式（过渡并建立悬停）
uint8 NAVIGATION_STATE_EXTERNAL1 = 23 # 外部模式1
uint8 NAVIGATION_STATE_EXTERNAL2 = 24 # 外部模式2
uint8 NAVIGATION_STATE_EXTERNAL3 = 25 # 外部模式3
uint8 NAVIGATION_STATE_EXTERNAL4 = 26 # 外部模式4
uint8 NAVIGATION_STATE_EXTERNAL5 = 27 # 外部模式5
uint8 NAVIGATION_STATE_EXTERNAL6 = 28 # 外部模式6
uint8 NAVIGATION_STATE_EXTERNAL7 = 29 # 外部模式7
uint8 NAVIGATION_STATE_EXTERNAL8 = 30 # 外部模式8
uint8 NAVIGATION_STATE_MAX = 31 # 最大导航状态值

uint8 executor_in_charge # 当前负责的模式执行器（0=自动驾驶仪）

uint32 valid_nav_states_mask # 所有有效导航状态值的位掩码
uint32 can_set_nav_states_mask # 用户可以选择的所有模式的位掩码

# 检测到的故障位掩码
uint16 failure_detector_status
uint16 FAILURE_NONE = 0 # 无故障
uint16 FAILURE_ROLL = 1 # 横滚故障 (1 << 0)
uint16 FAILURE_PITCH = 2 # 俯仰故障 (1 << 1)
uint16 FAILURE_ALT = 4 # 高度故障 (1 << 2)
uint16 FAILURE_EXT = 8 # 外部故障 (1 << 3)
uint16 FAILURE_ARM_ESC = 16 # 电机电调解锁故障 (1 << 4)
uint16 FAILURE_BATTERY = 32 # 电池故障 (1 << 5)
uint16 FAILURE_IMBALANCED_PROP = 64 # 螺旋桨不平衡故障 (1 << 6)
uint16 FAILURE_MOTOR = 128 # 电机故障 (1 << 7)

uint8 hil_state # 硬件在环状态
uint8 HIL_STATE_OFF = 0 # 硬件在环关闭状态
uint8 HIL_STATE_ON = 1 # 硬件在环开启状态

# 如果是垂直起降飞行器，那么在作为多旋翼飞行时，值为VEHICLE_TYPE_ROTARY_WING；作为固定翼飞行时，值为VEHICLE_TYPE_FIXED_WING
uint8 vehicle_type # 飞行器类型
uint8 VEHICLE_TYPE_UNKNOWN = 0 # 未知飞行器类型
uint8 VEHICLE_TYPE_ROTARY_WING = 1 # 旋翼飞行器类型
uint8 VEHICLE_TYPE_FIXED_WING = 2 # 固定翼飞行器类型
uint8 VEHICLE_TYPE_ROVER = 3 # 地面漫游车类型
uint8 VEHICLE_TYPE_AIRSHIP = 4 # 飞艇类型

uint8 FAILSAFE_DEFER_STATE_DISABLED = 0 # 故障保护延迟状态禁用
uint8 FAILSAFE_DEFER_STATE_ENABLED = 1 # 故障保护延迟状态启用
uint8 FAILSAFE_DEFER_STATE_WOULD_FAILSAFE = 2 # 故障保护已延迟，但会触发故障保护

bool failsafe # 如果系统处于故障保护状态（如：自动返航、悬停、终止等），则为true
bool failsafe_and_user_took_over # 如果系统处于故障保护状态，但用户接管了控制权，则为true
uint8 failsafe_defer_state # FAILSAFE_DEFER_STATE_* 中的一个值

# 链路丢失
bool gcs_connection_lost # 与地面站的数据链路丢失
uint8 gcs_connection_lost_counter # 统计与地面站连接丢失的唯一事件次数
bool high_latency_data_link_lost # 如果高延迟数据链路（如RockBlock铱星9603遥测模块）丢失，则设置为true

# 垂直起降标志
bool is_vtol # 如果系统具备垂直起降能力，则为true
bool is_vtol_tailsitter # 如果系统在从多旋翼模式过渡到固定翼模式时进行90°俯仰向下旋转，则为true
bool in_transition_mode # 如果垂直起降飞行器正在进行模式转换，则为true
bool in_transition_to_fw # 如果垂直起降飞行器正在从多旋翼模式过渡到固定翼模式，则为true

# MAVLink标识
uint8 system_type # 系统类型，包含MAVLink的MAV_TYPE
uint8 system_id # 系统ID，包含MAVLink的系统ID字段
uint8 component_id # 子系统/组件ID，包含MAVLink的组件ID字段

bool safety_button_available # 如果连接了安全按钮，则设置为true
bool safety_off # 如果安全开关关闭，则设置为true

bool power_input_valid # 如果输入电源有效，则设置
bool usb_connected # 一旦从USB链路接收到遥测数据，设置为true（永不清除）

bool open_drone_id_system_present # 是否存在开放式无人机标识系统
bool open_drone_id_system_healthy # 开放式无人机标识系统是否正常

bool parachute_system_present # 是否存在降落伞系统
bool parachute_system_healthy # 降落伞系统是否正常

bool avoidance_system_required # 如果通过COM_OBS_AVOID参数启用了避障系统，则设置为true
bool avoidance_system_valid # 避障系统的状态

bool rc_calibration_in_progress # 遥控器校准是否正在进行
bool calibration_enabled # 是否启用校准

bool pre_flight_checks_pass # 如果所有起飞前必要检查都通过，则为true
'''

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import  VehicleStatus

SHOW_LEVEL_MIN = 0
SHOW_LEVEL_NORMAL = 1
SHOW_LEVEL_MAX = 2
SHOW_LEVEL_DEBUG = 3

class px4_fmu_out_vehicle_status:
    def __init__(self):
        # 初始化arming_state映射
        self.arming_state_map = {
            1: "ARMING_STATE_DISARMED:(锁定，不能起飞)",
            2: "ARMING_STATE_ARMED:(解锁，可以起飞)"
        }

        # 初始化arming/disarming原因映射
        self.arm_disarm_reason_map = {
            0: "ARM_DISARM_REASON_TRANSITION_TO_STANDBY:(切换为待命状态)",
            1: "ARM_DISARM_REASON_RC_STICK:(遥控器摇杆特定操作)",
            2: "ARM_DISARM_REASON_RC_SWITCH:(遥控器解锁开关操作)",
            3: "ARM_DISARM_REASON_COMMAND_INTERNAL:(内部命令，由飞控系统内部逻辑自动生成的指令)",
            4: "ARM_DISARM_REASON_COMMAND_EXTERNAL:(外部命令，由地面站或其他外部系统发出的指令)",
            5: "ARM_DISARM_REASON_MISSION_START:(任务开始)",
            6: "ARM_DISARM_REASON_SAFETY_BUTTON:(无人机机身物理安全按钮)",
            7: "ARM_DISARM_REASON_AUTO_DISARM_LAND:(降落后自动锁定)",
            8: "ARM_DISARM_REASON_AUTO_DISARM_PREFLIGHT:(起飞前自动锁定)",
            9: "ARM_DISARM_REASON_KILL_SWITCH:(急停开关)",
            10: "ARM_DISARM_REASON_LOCKDOWN:(出发锁定逻辑)",
            11: "ARM_DISARM_REASON_FAILURE_DETECTOR:(故障检测)",
            12: "ARM_DISARM_REASON_SHUTDOWN:(关机操作)",
            13: "ARM_DISARM_REASON_UNIT_TEST:(单元测试)"
        }

        # 初始化nav_state映射
        self.nav_state_map = {
            0: "NAVIGATION_STATE_MANUAL:(摇杆完全控制所有运动，飞控仅提供基础姿态稳定，无自动位置/高度保持)",
            1: "NAVIGATION_STATE_ALTCTL:(摇杆控制水平方向运动，飞控自动维持高度)",
            2: "NAVIGATION_STATE_POSCTL:(摇杆控制速度或位移，飞控自动保持位置和高度)",
            3: "NAVIGATION_STATE_AUTO_MISSION:(自动任务模式，按照事先编制的飞行计划飞行)",
            4: "NAVIGATION_STATE_AUTO_LOITER:(自动盘旋模式，用来在目标点进行盘旋，方便对目标点进行长时间观测)",
            5: "NAVIGATION_STATE_AUTO_RTL:(自动返航模式，比如丢失联系，电池电量过低等)",
            6: "NAVIGATION_STATE_POSITION_SLOW:(慢速模式，降低速度提高操作精度？？？)",
            7: "NAVIGATION_STATE_FREE5:(自由模式5)",
            8: "NAVIGATION_STATE_FREE4:(自由模式4)",
            9: "NAVIGATION_STATE_FREE3:(自由模式3)",
            10: "NAVIGATION_STATE_ACRO:(特技模式，和手动模式相比，摇杆的含义不同，能够完成各种特技动作，且飞控不限制飞行器的姿态)",
            11: "NAVIGATION_STATE_FREE2:(自由模式2)",
            12: "NAVIGATION_STATE_DESCEND:(垂直下降模式，飞控不控制位置，只控制高度按照设定的速度下降)",
            13: "NAVIGATION_STATE_TERMINATION:(终止模式，无人机立即关闭所有电机，自由坠落（多旋翼）或滑翔降落（固定翼）)",
            14: "NAVIGATION_STATE_OFFBOARD:(外部控制模式，由地面站或其他外部系统控制飞行器的运动)",
            15: "NAVIGATION_STATE_STAB:(自稳模式,手动控制姿态角，飞控自动补偿外力干扰比如风)",
            16: "NAVIGATION_STATE_FREE1:(自由模式1)",
            17: "NAVIGATION_STATE_AUTO_TAKEOFF:(自动起飞模式)",
            18: "NAVIGATION_STATE_AUTO_LAND:(自动降落模式)",
            19: "NAVIGATION_STATE_AUTO_FOLLOW_TARGET:(自动跟随模式)",
            20: "NAVIGATION_STATE_AUTO_PRECLAND:(自动精确定点着陆模式)",
            21: "NAVIGATION_STATE_ORBIT:(环绕轨道模式)",
            22: "NAVIGATION_STATE_AUTO_VTOL_TAKEOFF:(专为复合翼/VTOL无人机设计，包含多阶段：垂直起飞→过渡至固定翼模式→爬升到巡航高度)",
            23: "NAVIGATION_STATE_EXTERNAL1:(外部模式1)",
            24: "NAVIGATION_STATE_EXTERNAL2:(外部模式2)",
            25: "NAVIGATION_STATE_EXTERNAL3:(外部模式3)",
            26: "NAVIGATION_STATE_EXTERNAL4:(外部模式4)",
            27: "NAVIGATION_STATE_EXTERNAL5:(外部模式5)",
            28: "NAVIGATION_STATE_EXTERNAL6:(外部模式6)",
            29: "NAVIGATION_STATE_EXTERNAL7:(外部模式7)",
            30: "NAVIGATION_STATE_EXTERNAL8:(外部模式8)",
            31: "NAVIGATION_STATE_MAX:(最大边界)"
        }
        # 初始化故障检测状态映射（位掩码）
        self.failure_mask_map = {
            1: "FAILURE_ROLL:(横滚控制故障)",
            2: "FAILURE_PITCH:(俯仰控制故障)",
            4: "FAILURE_ALT:(高度控制故障)",
            8: "FAILURE_EXT:(外部故障)",
            16: "FAILURE_ARM_ESC:(电调解锁失败)",
            32: "FAILURE_BATTERY:(电池故障)",
            64: "FAILURE_IMBALANCED_PROP:(螺旋桨不平衡故障)",
            128: "FAILURE_MOTOR:(电机故障)"
        }
        # 初始化failsafe_defer_state映射
        self.failsafe_defer_state_map = {
            0: "FAILSAFE_DEFER_STATE_DISABLED:(系统严格遵循预设的故障保护规则，检测到故障条件时立即执行保护动作)",
            1: "FAILSAFE_DEFER_STATE_ENABLED:(故障保障机制也不会立刻被触发，而是会被延迟，给予系统或操作员一定的时间来处理问题)",
            2: "FAILSAFE_DEFER_STATE_WOULD_FAILSAFE:(系统保持当前状态不触发保护，但会记录故障状态，以便后分析和处理)"
        }
        # 初始化hil_state映射
        self.hil_state_map = {
            0: "HIL_STATE_OFF:(硬件在环关闭)",
            1: "HIL_STATE_ON:(硬件在环开启)"
        }
        # 初始化vehicle_type映射
        self.vehicle_type_map = {
            0: "VEHICLE_TYPE_UNKNOWN:(未知)",
            1: "VEHICLE_TYPE_ROTARY_WING:(旋翼)",
            2: "VEHICLE_TYPE_FIXED_WING:(固定翼)",
            3: "VEHICLE_TYPE_ROVER:(地面漫游车)",
            4: "VEHICLE_TYPE_AIRSHIP:(飞艇)"
        }


    # 解析锁定状态，ARM=解锁可飞行，DISARM=锁定不可飞行
    def decode_arming_state(self, value):
        return self.arming_state_map.get(value, "未定义数值")

    # 解析解锁/上锁原因
    def decode_arm_disarm_reason(self, value):
        return self.arm_disarm_reason_map.get(value, "未定义数值")

    # 解析导航状态
    def decode_nav_state(self, value):
        return self.nav_state_map.get(value, "未定义数值")
    # 解析导航状态位图
    def decode_nav_state_bitmap(self, value):
        if value == 0:
            return ["无导航模式"]
        navs = []
        for state, name in self.nav_state_map.items():
            if value & (1<<state) :
                navs.append(name)
        return navs if navs else ["未知导航模式值, "]

    # 解析故障检测位图
    def decode_failure_status(self, value):
        if value == 0:
            return ["无故障，一切OK"]
        failures = []
        for mask, name in self.failure_mask_map.items():
            if value & mask:
                failures.append(name + ",")
        return failures if failures else ["未知故障数值"]

    # 解析HIL状态
    def decode_hil_state(self, value):
        return self.hil_state_map.get(value, "未定义数值")

    # 解析vehicle_type状态
    def decode_vehicle_type(self, value):
        return self.vehicle_type_map.get(value, "未定义数值")

    # 解析failsafe_defer_state状态
    def decode_failsafe_defer_state(self, value):
        return self.failsafe_defer_state_map.get(value, "未定义数值")

    # 解析并打印vehicle_status消息
    def pretty_print(self, vehicle_status, show_level):
        print(f"\n\n\n============== Vehicle Status (飞行器状态报告级别{show_level}) ==============")
        print(f"系统时间: {vehicle_status.timestamp/1000000:.4f} s")
        print(f"锁定状态: {self.decode_arming_state(vehicle_status.arming_state)}")
        print(f"解锁时间: {vehicle_status.armed_time/1000000:.4f} s")
        print(f"起飞时间: {vehicle_status.takeoff_time/1000000:.4f} s")

        print(f"导航激活时间: {vehicle_status.nav_state_timestamp/1000000:.4f} s")
        print(f"用户配置的导航模式: {self.decode_nav_state(vehicle_status.nav_state_user_intention)}")
        print(f"当前实际导航模式: {self.decode_nav_state(vehicle_status.nav_state)}")

        print("检测到的故障: " , end='')
        failures = self.decode_failure_status(vehicle_status.failure_detector_status)
        for failure in failures:
            print("%s, " % failure, end='')
        print(f"\n检测保护延迟状态: {self.decode_failsafe_defer_state(vehicle_status.failsafe_defer_state)}")
        print(f"是否处于故障保护状态: {vehicle_status.failsafe}")
        print(f"用户是否接管了控制权: {vehicle_status.failsafe_and_user_took_over}")

        if show_level>=SHOW_LEVEL_NORMAL: 
            print(f"最近一次解锁原因: {self.decode_arm_disarm_reason(vehicle_status.latest_arming_reason)}")
            print(f"最近一次锁定原因: {self.decode_arm_disarm_reason(vehicle_status.latest_disarming_reason)}")
            print(f"起飞前检查通过: {vehicle_status.pre_flight_checks_pass}")
            print(f"MAVLINK--system_type: {vehicle_status.system_type}")
            print(f"MAVLINK--system_id: {vehicle_status.system_id}")
            print(f"MAVLINK--component_id: {vehicle_status.component_id}")
            print(f"丢失和地面站的连接: {vehicle_status.gcs_connection_lost}")
            print(f"丢失和地面站的连接次数: {vehicle_status.gcs_connection_lost_counter}")
            print(f"高延迟数据链路丢失: {vehicle_status.high_latency_data_link_lost}")

        if show_level>=SHOW_LEVEL_MAX: 
            print(f"硬件在环状态: {self.decode_hil_state(vehicle_status.hil_state)}")
            print(f"当前负责的执行器: {vehicle_status.executor_in_charge}, (0=Autopilot)")
            print(f"飞行器类型: {self.decode_vehicle_type(vehicle_status.vehicle_type)}")
            print(f"是否配备物理安全按钮: {vehicle_status.safety_button_available}")
            print(f"安全开关是否关闭(安全机制是否解除): {vehicle_status.safety_off}")
            print(f"输入电源是否有效: {vehicle_status.power_input_valid}")
            print(f"USB是否连接(收到遥测数据): {vehicle_status.usb_connected}")
            print(f"开放式无人机标识系统存在: {vehicle_status.open_drone_id_system_present}")
            print(f"开放式无人机标识系统正常: {vehicle_status.open_drone_id_system_healthy}")
            print(f"降落伞系统存在: {vehicle_status.parachute_system_present}")
            print(f"降落伞系统正常: {vehicle_status.parachute_system_healthy}")
            print(f"避障系统要求启动: {vehicle_status.avoidance_system_required}")
            print(f"避障系统有效: {vehicle_status.avoidance_system_valid}")
            print(f"遥控器校准功能启用: {vehicle_status.calibration_enabled}")
            print(f"遥控器校准进行中: {vehicle_status.rc_calibration_in_progress}")
        
        if show_level>=SHOW_LEVEL_DEBUG: 
            print(f"合法的导航模式:")
            navs = self.decode_nav_state_bitmap(vehicle_status.valid_nav_states_mask)
            for nav in navs:
                print(f"\t{nav}")
            print(f"可选择的导航模式:")
            navs = self.decode_nav_state_bitmap(vehicle_status.can_set_nav_states_mask)
            for nav in navs:
                print(f"\t{nav}")

            print(f"VTOL类型(可转换旋翼固定翼): {vehicle_status.is_vtol}")
            print(f"VTOL是否支持尾坐式垂直起降: {vehicle_status.is_vtol_tailsitter}")
            print(f"VOTL是否处于模式转换状态: {vehicle_status.in_transition_mode}")
            print(f"VOTL是否正在从旋翼转换到固定翼模式: {vehicle_status.in_transition_to_fw}")


class decode_msg(Node):
    def __init__(self) -> None:
        super().__init__('decode_px4_fmu_out_vehicle_status')

        self.declare_parameter('decode_px4_show_level', 0)

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.subscription_callback, qos_profile)

        self.timer = self.create_timer(100, self.timer_callback)

    def subscription_callback(self, msg: VehicleStatus) -> None:
        # 解析并打印vehicle_status消息
        show_level = self.get_parameter('decode_px4_show_level').get_parameter_value().integer_value
        decoder = px4_fmu_out_vehicle_status()
        decoder.pretty_print(msg, show_level)

    def timer_callback(self) -> None:
        time.sleep(1.5)


def main(args=None) -> None:
    rclpy.init(args=args)
    decode_handle = decode_msg()
    rclpy.spin(decode_handle)
    decode_handle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
