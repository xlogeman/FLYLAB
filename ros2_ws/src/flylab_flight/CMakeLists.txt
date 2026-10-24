cmake_minimum_required(VERSION 3.8)
project(flylab_flight)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(mars_quadrotor_msgs REQUIRED)

# octomap 动态清理: 订阅 /airsim_node/PX4/odom_local，
# 周期调用 /octomap_server/clear_bbox 清理无人机周围的旧地图
add_executable(octomap_dynamic src/octomap_dynamic.cpp)
ament_target_dependencies(octomap_dynamic
  rclcpp
  nav_msgs
  octomap_msgs
)

# VINS 起飞联动开关: 监听 /fmu/out/vehicle_status,
# 在 arming/disarming 时开关 VINS 的 IMU/相机并重启估计器 (mono/stereo 共用)
add_executable(vins_arming_switch src/vins_arming_switch.cpp)
ament_target_dependencies(vins_arming_switch
  rclcpp
  px4_msgs
  std_msgs
)

# ==============================================================================
# AirSim 直连传感器节点 (src/sensors/, 算法无关, 可插拔):
#   airsim_camera  通用相机 (深度/双目/单目, 参数化)
#   airsim_imu     IMU (FRD 原生 + FLU 标准双话题, 统一时间轴)
#   airsim_odom    真值里程计 (ENU) + world->base_link TF
#   airsim_nav     低频导航传感器 (GPS/气压计/磁力计)
#   airsim_lidar   3D LiDAR 原始点云 (传感器系 NED + 逐点时间模型)
# 与 airsim_ros_pkgs 同款接法: 直接编译 AirLib/rpclib/MavLinkCom 源码并链接
# ==============================================================================
set(AIRSIM_ROOT /root/flylab/Cosys-AirSim)
set(RPC_LIB_INCLUDES ${AIRSIM_ROOT}/external/rpclib/rpclib-2.3.1/include)
add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" flylab_rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" flylab_MavLinkCom)
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" flylab_AirLib)

# 定义一个 AirSim 直连传感器节点的辅助函数 (相同的 include/链接/编译选项)
function(add_airsim_sensor_node target src)
  add_executable(${target} ${src})
  target_include_directories(${target} PRIVATE
    ${AIRSIM_ROOT}/AirLib/include
    ${AIRSIM_ROOT}/AirLib/deps/eigen3
    ${RPC_LIB_INCLUDES}
    ${AIRSIM_ROOT}/MavLinkCom/include
    ${AIRSIM_ROOT}/MavLinkCom/common_utils
  )
  target_link_libraries(${target} AirLib ${OpenCV_LIBS} Threads::Threads)
  ament_target_dependencies(${target}
    rclcpp
    sensor_msgs
    geometry_msgs
    tf2
    ${ARGN}
  )
  # AirSim 头文件不满足 -Wpedantic, 对该目标关闭
  target_compile_options(${target} PRIVATE -Wno-pedantic -Wnoexcept)
endfunction()

add_airsim_sensor_node(airsim_camera src/sensors/airsim_camera.cpp cv_bridge)
add_airsim_sensor_node(airsim_imu    src/sensors/airsim_imu.cpp)
add_airsim_sensor_node(airsim_odom   src/sensors/airsim_odom.cpp nav_msgs tf2 tf2_ros tf2_geometry_msgs)
add_airsim_sensor_node(airsim_nav    src/sensors/airsim_nav.cpp)
add_airsim_sensor_node(airsim_lidar  src/sensors/airsim_lidar.cpp)

# 轨迹跟踪节点: 接收规划器轨迹并转成 PX4 setpoint 下发 (uXRCE-DDS 链路)
add_executable(tracker_uxrce src/control/tracker_uxrce.cpp)
ament_target_dependencies(tracker_uxrce
  rclcpp
  px4_msgs
  geometry_msgs
  quadrotor_msgs
)

# 轨迹跟踪节点 (MAVROS 版): 接收规划器轨迹并经 MAVROS 下发 PX4
add_executable(tracker_mavros src/control/tracker_mavros.cpp)
ament_target_dependencies(tracker_mavros
  rclcpp
  geometry_msgs
  mavros_msgs
  quadrotor_msgs
)

# SUPER 输出桥接节点: mars_quadrotor_msgs/PositionCommand -> quadrotor_msgs/PositionCommand
add_executable(super_cmd_bridge src/control/super_cmd_bridge.cpp)
ament_target_dependencies(super_cmd_bridge
  rclcpp
  mars_quadrotor_msgs
  quadrotor_msgs
)

# ==============================================================================
# 算法桥接节点 (src/bridges/, 纯 ROS, 不依赖 AirSim):
#   每个算法链路按需选择, 做传感器数据 -> 算法输入的适配 (坐标系/门控/对齐)
# ==============================================================================

# VIO 断桨门控桥接: 图像/IMU 原样转发, 断桨停发、解锁恢复 (OpenVINS 链路)
add_executable(bridge_vio src/bridges/bridge_vio.cpp)
ament_target_dependencies(bridge_vio
  rclcpp
  px4_msgs
  sensor_msgs
)

# FAST-LIO 输入适配桥接: 点云 NED->FLU + 雷达挂载外参静态 TF
add_executable(bridge_fastlio src/bridges/bridge_fastlio.cpp)
ament_target_dependencies(bridge_fastlio
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)

# SUPER 输入适配桥接: 点云传感器系 NED -> world 系 ENU (位姿时间对齐 + 逐点去畸变
# + 机身回波过滤), 并转发真值里程计给 SUPER
add_executable(bridge_super src/bridges/bridge_super.cpp)
ament_target_dependencies(bridge_super
  rclcpp
  sensor_msgs
  nav_msgs
  tf2
)

install(TARGETS
  octomap_dynamic
  vins_arming_switch
  airsim_camera
  airsim_imu
  airsim_odom
  airsim_nav
  airsim_lidar
  bridge_vio
  bridge_fastlio
  bridge_super
  tracker_uxrce
  tracker_mavros
  super_cmd_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  rviz
  configs
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
