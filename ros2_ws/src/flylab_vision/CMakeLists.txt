cmake_minimum_required(VERSION 3.10)
project(flylab_vision)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc aruco calib3d)

# ---------- 核心算法库 (无 ROS 依赖) ----------
add_library(aruco_detector src/aruco/aruco_detector.cpp)
target_include_directories(aruco_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(aruco_detector ${OpenCV_LIBS})

# ---------- ROS2 节点: 地面站视觉工具链路 ----------
add_executable(aruco_node src/aruco/aruco_node.cpp)
target_link_libraries(aruco_node aruco_detector)
ament_target_dependencies(aruco_node rclcpp sensor_msgs std_msgs)

# ---------- ROS2 节点: 飞行控制 (ArUco 视觉降落 / 后续目标追踪控制等, 执行层见 include/control/) ----------
add_executable(aruco_landing_node src/aruco/aruco_landing_node.cpp)
target_link_libraries(aruco_landing_node aruco_detector)
ament_target_dependencies(aruco_landing_node rclcpp sensor_msgs std_msgs px4_msgs geometry_msgs mavros_msgs)

install(TARGETS aruco_detector aruco_node aruco_landing_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
