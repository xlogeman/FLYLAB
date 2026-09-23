#!/usr/bin/env python3

#import keyboard
import math
import sys
import tty
import termios
import select
import time
import rclpy
from rclpy.node import Node
from hw_interface.msg import HWSimpleKeyboardInfo
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

X_STEP = 20
Y_STEP = 20
Z_STEP = 5
YAW_STEP = 0.04
YAW_MAX = 6.28

# 阻塞方式读取按键 ##############################################################
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


# 非阻塞方式读取按键 ##############################################################
def getch_with_timeout(timeout=0.1):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            ch = sys.stdin.read(1)
            return ch
        else:
            #print(f"{timeout}秒内没有获取到按键")
            return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


class KeyboardPosition(Node):
    def __init__(self) -> None:
        super().__init__('keyboard_position')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.vehicle_local_position = VehicleLocalPosition()
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.init = True

        self.sub = self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.sub_callback, qos_profile)
        self.pub = self.create_publisher(HWSimpleKeyboardInfo, "/hw_insight/keyboard_postion", 1) 
        self.timer1 = self.create_timer(0.2, self.timer_callback_pub)
        self.timer2 = self.create_timer(0.1, self.timer_callback_key)

    def sub_callback(self, vehicle_local_position):
        self.vehicle_local_position = vehicle_local_position
        if self.init :
            self.init = False
            self.x = self.vehicle_local_position.x
            self.y = self.vehicle_local_position.y
            self.z = self.vehicle_local_position.z           

    def timer_callback_key(self):
        key = getch_with_timeout(0.5)
        # 处理按键值
        if key == 'w' or key == 'W':
            self.x = self.vehicle_local_position.x + X_STEP
            print("前进")
        elif key == 's' or key == 'S':
            self.x = self.vehicle_local_position.x - X_STEP
            print("后退")
        elif key == 'a' or key == 'A':
            self.y = self.vehicle_local_position.y - Y_STEP
            print("左移")
        elif key == 'd' or key == 'D':
            self.y = self.vehicle_local_position.y + Y_STEP
            print("右移")
        elif key == 'i' or key == 'I':
            self.z = self.vehicle_local_position.z - Z_STEP
            print("上升")
        elif key == 'k' or key == 'K':
            self.z = self.vehicle_local_position.z + Z_STEP
            print("下降")
        elif key == 'j' or key == 'J':
            self.yaw = (YAW_MAX + self.yaw - YAW_STEP) % YAW_MAX
            print("左转")
        elif key == 'l' or key == 'L':
            self.yaw = (self.yaw + YAW_STEP) % YAW_MAX
            print("右转")
        elif key == 'h' or key == 'H':
            self.x = self.vehicle_local_position.x
            self.y = self.vehicle_local_position.y
            self.z = self.vehicle_local_position.z
            print("停止")
        else:
            # 退出按键
            if key is not None:
                key_ascii = ord(key)
                if key_ascii == 27 or key_ascii == 3:  # ESC键, Ctrl+C
                    exit(0)

    def timer_callback_pub(self):
        msg = HWSimpleKeyboardInfo()
        msg.x = self.x
        msg.y = self.y
        msg.z = self.z
        msg.yaw = self.yaw
        self.pub.publish(msg)


def main(args=None) -> None:
    print('Start keyboard position control node...')
    rclpy.init(args=args)
    keyboard_position = KeyboardPosition()
    rclpy.spin(keyboard_position)
    keyboard_position.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
