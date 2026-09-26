cmake_minimum_required(VERSION 3.8)
project(flylab)

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

set(FLYLAB_DEPENDENCIES
  rclcpp
  px4_msgs
  sensor_msgs
  geometry_msgs
)

# octomap 动态清理: 订阅 /airsim_node/PX4/odometry，
# 周期调用 /octomap_server/clear_bbox 清理无人机周围的旧地图
add_executable(octomap_dynamic src/octomap_dynamic.cpp)
ament_target_dependencies(octomap_dynamic
  rclcpp
  nav_msgs
  octomap_msgs
)

# offboard 速度控制器: 订阅 /flylab/cmd_vel (geometry_msgs/Twist)
add_executable(move_velocity src/move_velocity.cpp)
ament_target_dependencies(move_velocity ${FLYLAB_DEPENDENCIES})

# VINS 起飞联动开关: 监听 /fmu/out/vehicle_status,
# 在 arming/disarming 时开关 VINS 的 IMU/相机并重启估计器 (mono/stereo 共用)
add_executable(vins_arming_switch src/vins_arming_switch.cpp)
ament_target_dependencies(vins_arming_switch
  rclcpp
  px4_msgs
  std_msgs
)

# ==============================================================================
# EGO 链路: AirSim 直连传感器节点 (深度图/IMU/真值里程计/GPS/气压计/磁力计 + TF)
# 与 airsim_ros_pkgs 同款接法: 直接编译 AirLib/rpclib/MavLinkCom 源码并链接
# ==============================================================================
set(AIRSIM_ROOT /root/fucker/Cosys-AirSim)
set(RPC_LIB_INCLUDES ${AIRSIM_ROOT}/external/rpclib/rpclib-2.3.1/include)
add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" flylab_rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" flylab_MavLinkCom)
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" flylab_AirLib)

# EGO 轨迹跟踪节点: 接收 EGO-Planner 轨迹并转成 PX4 setpoint 下发
add_executable(px4_ego_tracker src/px4_ego_tracker.cpp)
ament_target_dependencies(px4_ego_tracker
  rclcpp
  px4_msgs
  geometry_msgs
  quadrotor_msgs
)

# EGO 链路 AirSim 传感器节点: 深度图/IMU/真值里程计/GPS/气压计/磁力计 + TF
add_executable(airsim_sensor_ego src/airsim_sensor_ego.cpp)
target_include_directories(airsim_sensor_ego PRIVATE
  ${AIRSIM_ROOT}/AirLib/include
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
)
target_link_libraries(airsim_sensor_ego AirLib ${OpenCV_LIBS} Threads::Threads)
ament_target_dependencies(airsim_sensor_ego
  rclcpp
  sensor_msgs
  cv_bridge
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
# AirSim 头文件不满足 -Wpedantic, 对该目标关闭
target_compile_options(airsim_sensor_ego PRIVATE -Wno-pedantic -Wnoexcept)

# VIO 链路 AirSim 传感器节点 (OpenVINS MSCKF): 双目相机 + IMU + GPS/气压计/磁力计
add_executable(airsim_sensor_vio src/airsim_sensor_vio.cpp)
target_include_directories(airsim_sensor_vio PRIVATE
  ${AIRSIM_ROOT}/AirLib/include
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
)
target_link_libraries(airsim_sensor_vio AirLib ${OpenCV_LIBS} Threads::Threads)
ament_target_dependencies(airsim_sensor_vio
  rclcpp
  sensor_msgs
  cv_bridge
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_compile_options(airsim_sensor_vio PRIVATE -Wno-pedantic -Wnoexcept)

install(TARGETS
  move_velocity
  octomap_dynamic
  vins_arming_switch
  airsim_sensor_ego
  airsim_sensor_vio
  px4_ego_tracker
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
