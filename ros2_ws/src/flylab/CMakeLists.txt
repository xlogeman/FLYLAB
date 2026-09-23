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

install(TARGETS
  move_velocity
  octomap_dynamic
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
