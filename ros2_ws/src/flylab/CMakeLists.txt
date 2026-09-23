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

set(FLYLAB_DEPENDENCIES
  rclcpp
  px4_msgs
  sensor_msgs
  geometry_msgs
)

# offboard 速度控制器: 订阅 /flylab/cmd_vel (geometry_msgs/Twist)
add_executable(move_velocity src/move_velocity.cpp)
ament_target_dependencies(move_velocity ${FLYLAB_DEPENDENCIES})

install(TARGETS
  move_velocity
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
