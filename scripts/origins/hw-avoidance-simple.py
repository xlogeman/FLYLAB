#!/usr/bin/env python3

#版权声明：
#本代码版权归B站用户 hw-insight 所有(B站唯一ID：bili_6958687484)。
#允许个人出于学习目的免费使用本代码内容。
#禁止任何单位或个人以抄袭、搬运、改编、复制等方式将本教程用于商业用途，包括但不限于课程销售、广告推广、产品宣传、自媒体运营等。
#未经授权的商业使用行为将构成侵权，版权方保留通过法律途径追究侵权者民事赔偿责任及其他法律责任的权利。如需商业合作，请联系获取授权。

# 安装命令: pip3 install mavsdk
# 官网链接:      https://mavsdk.mavlink.io/main/en/python/quickstart.html
# API使用说明:   http://mavsdk-python-docs.s3-website.eu-central-1.amazonaws.com/index.html


import math
import asyncio
from mavsdk import System
from mavsdk import mission_raw
from mavsdk.telemetry import (DistanceSensor, PositionNed)
from mavsdk.offboard import (OffboardError, AccelerationNed, PositionNedYaw, PositionGlobalYaw, VelocityBodyYawspeed)
from mavsdk.mission import (MissionItem, MissionPlan)

glb_leave_home = False  # 全局变量 保存是否离开了home，在home附近是不会触发避障逻辑

glb_mission_current = 0   # 全局变量 保存最新进度
glb_mission_total = 0     # 全局变量 保存任务总数

glb_distance_front = 1000.0  # 全局变量 保存前方距离
glb_distance_left = 1000.0   # 全局变量 保存左前方距离，航向左转45都
glb_distance_right = 1000.0  # 全局变量 保存右前方距离，航向右转45度
glb_distance_up = 1000.0     # 全局变量 保存上前方距离，航向上转45度
glb_distance_down = 1000.0   # 全局变量 保存下前方方距离，航向下转45度
glb_distance_min = 1000.0    # 全局变量 保存所有传感器中的最小距离

# 定期打印所有信息 ################################################################
async def periodically_print():
    while True:
        print(f"\n\nProgress: {glb_mission_current}/{glb_mission_total}")
        print(f"Distances: \n"
              f"Front: {glb_distance_front}m \n"
              f"Back : {glb_distance_back}m \n"
              f"Left : {glb_distance_left}m \n"
              f"Right: {glb_distance_right}m \n"
              f"Up   : {glb_distance_up}m \n"
              f"Down : {glb_distance_down}m \n")
        await asyncio.sleep(1)  # 每秒打印一次


# 监控是否离开了home ##############################################################
async def check_leave_home(drone: System):
    global glb_leave_home
        # 等待获取Home点坐标
    async for home in drone.telemetry.home():
        home_lat = home.latitude_deg
        home_lon = home.longitude_deg
        break
    while True:
        # 获取当前位置坐标
        async for position in drone.telemetry.position():
            current_lat = position.latitude_deg
            current_lon = position.longitude_deg
            break
        #print(f"Home坐标: ({home_lat:.9f}, {home_lon:.9f})")
        #print(f"当前坐标: ({current_lat:.9f}, {current_lon:.9f})\n\n")
        # 判断是否在home附近
        if (abs(current_lat-home_lat) + abs(current_lon-home_lon))  > 0.000015:
            #print("离开home")
            glb_leave_home = True
        else:
            #print("在home附近")
            glb_leave_home = False
        await asyncio.sleep(0.03)



# 监控任务进度 #################################################################
async def monitor_progress(drone: System):
    global glb_mission_current
    global glb_mission_total
    async for mission_progress in drone.mission_raw.mission_progress():
        glb_mission_current = mission_progress.current
        glb_mission_total = mission_progress.total
        print(f"Update Mission progress: "
              f"{glb_mission_current}/"
              f"{glb_mission_total}")


# 监控距离传感器 #############################################################
async def monitor_distance(drone: System):
    global glb_distance_front
    global glb_distance_left
    global glb_distance_right
    global glb_distance_up
    global glb_distance_down
    global glb_distance_min
    # 获取所有距离传感器信息
    async for distance_sensor in drone.telemetry.distance_sensor():
        # 提取传感器姿态角（单位：度）
        roll = distance_sensor.orientation.roll_deg
        pitch = distance_sensor.orientation.pitch_deg
        yaw = distance_sensor.orientation.yaw_deg
        current_distance = distance_sensor.current_distance_m

        # 根据姿态角判断方向，后续优化为初始化时判断一下，判断通过就不再监控
        if math.isclose(yaw, 0.0) and math.isclose(pitch, 0.0) and math.isclose(roll, 0.0):
            # 前
            glb_distance_front = current_distance
        elif math.isclose(yaw, 45.0) and math.isclose(pitch, 0.0) and math.isclose(roll, 0.0):
            # 右
            glb_distance_right = current_distance
        elif math.isclose(yaw, 135.0) and math.isclose(pitch, 0.0) and math.isclose(roll, 0.0):
            # 左
            glb_distance_left = current_distance
        elif math.isclose(yaw, 45.0) and math.isclose(pitch, 0.0) and math.isclose(roll, 90.0):
            # 下
            glb_distance_down = current_distance
        elif math.isclose(yaw, 135.0) and math.isclose(pitch, 0.0) and math.isclose(roll, 90.0):
            # 上
            glb_distance_up = current_distance
        else:
            print("未知距离他传感器方位\n")
        
        # 获取所有传感器中最小的值
        if glb_distance_front < glb_distance_left:
            tmp_distance_min = glb_distance_front
        else:
            tmp_distance_min = glb_distance_left
        if tmp_distance_min > glb_distance_right:
            tmp_distance_min = glb_distance_right
        if tmp_distance_min > glb_distance_up:
            tmp_distance_min = glb_distance_up
        if tmp_distance_min > glb_distance_down:
            tmp_distance_min = glb_distance_down
        glb_distance_min = tmp_distance_min
        #print(f"最小距离传感器: {glb_distance_min}")


# 简单避障逻辑，遇到障碍上升 #############################################################
AVOIDANCE_TRIGGER_DISTANCE = 7.0   # 避障触发距离阈值
AVOIDANCE_RELEASE_DISTANCE = 15.0  # 避障解除距离阈值

# 方法1：速度避障逻辑，遇到障碍物进行爬升，直到和障碍物的距离大于阈值 ------------------------
async def speed_avoidance(drone: System):
    # 适当后退远离障碍物，避免抵住障碍物导致无法爬升
    print("--加载速度避障逻辑")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    await drone.offboard.start()
    print("--适当倒退")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(
            -0.1,    # X轴速度（前后方向）
            0.0,     # Y轴速度（左右方向）
            0.0,     # Z轴速度（上下方向）
            0.0      # Yaw角速度（机体旋转绕Z轴旋转，0表示保持航向不变）
        )
    )
    #await drone.offboard.set_acceleration_ned(AccelerationNed(0.000000, 0.000000, -1.0))
    await asyncio.sleep(0.5)    # 让无人机后退0.5秒，远离障碍物
    # 设置爬升加速度
    print("--稳定姿态")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(2)
    print("--开始爬升")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, -1.7, 0.0))
    while True:
        # 判断前方距离
        if glb_distance_min > AVOIDANCE_RELEASE_DISTANCE :
            break   # 如果前方距离大于阈值，则停止持续爬升逻辑
        await asyncio.sleep(1)
    await asyncio.sleep(1.7)  # 继续爬升1.7秒，留出足够余量
    print("--障碍物已远离，停止爬升!")

    # 停止Offboard模式，继续执行任务
    print("--退出避障逻辑")
    print("恢复执行任务...")
    await drone.offboard.stop()
    await drone.mission.start_mission()

# 避障主循环 ------------------------
async def obstacle_avoidance(drone: System):
    while True:
        #print(f"chcek{glb_distance_front}")
        if glb_leave_home and glb_distance_min < AVOIDANCE_TRIGGER_DISTANCE :
            print(f"近距离发现障碍物{glb_distance_min:.2f}，停止前进!")
            await drone.mission.pause_mission()
            #await asyncio.sleep(1)
            await speed_avoidance(drone) # 调用速度避障逻辑
        await asyncio.sleep(0.04)  # 每0.1秒检查一次


# 主函数 ######################################################################
async def run():
    # 连接到飞行器
    drone = System()
    await drone.connect(system_address="udp://:14540")  # 在14540端口上等待PX4的连接
    print("等待连接...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("已连接!\n")
            break

    # 启动所有任务
    task_check_leave_home = asyncio.create_task(check_leave_home(drone))
    #task_monitor_progress = asyncio.create_task(monitor_progress(drone))
    task_monitor_distance = asyncio.create_task(monitor_distance(drone))
    task_obstacle_avoidance = asyncio.create_task(obstacle_avoidance(drone))
    #task_periodically_print = asyncio.create_task(periodically_print())
    await asyncio.gather(task_check_leave_home, task_monitor_distance, task_obstacle_avoidance)


# 运行主函数
if __name__ == "__main__":
    asyncio.run(run())