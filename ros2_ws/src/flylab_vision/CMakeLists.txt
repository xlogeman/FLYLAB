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
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs aruco calib3d dnn)
find_package(ament_index_cpp REQUIRED)

# ---------- onnxruntime (第三方预编译包, yolo 推理后端) ----------
# 系统 OpenCV 4.5.x DNN 解析不了 yolo26 的 attention Split 节点, 推理交给 ort;
# dnn 组件仍保留 (只借用 cv::dnn::NMSBoxes 做 NMS)
set(ORT_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/onnxruntime)

# ---------- 核心算法库 (无 ROS 依赖) ----------
add_library(aruco_detector src/aruco/aruco_detector.cpp)
target_include_directories(aruco_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(aruco_detector ${OpenCV_LIBS})

# ---------- 核心算法库: YOLO 目标检测 (无 ROS 依赖, onnxruntime 推理) ----------
add_library(yolo_detector src/yolo/yolo_detector.cpp)
target_include_directories(yolo_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${ORT_ROOT}/include
  $<INSTALL_INTERFACE:include>)
target_link_libraries(yolo_detector ${OpenCV_LIBS} ${ORT_ROOT}/lib/libonnxruntime.so)

# ---------- 核心算法库: YOLO 图像分类 (无 ROS 依赖, onnxruntime 推理) ----------
add_library(yolo_classifier src/yolo/yolo_classifier.cpp)
target_include_directories(yolo_classifier PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${ORT_ROOT}/include
  $<INSTALL_INTERFACE:include>)
target_link_libraries(yolo_classifier ${OpenCV_LIBS} ${ORT_ROOT}/lib/libonnxruntime.so)

# ---------- 核心算法库: YOLO 单目深度估计 (无 ROS 依赖, onnxruntime 推理) ----------
add_library(yolo_depth src/yolo/yolo_depth.cpp)
target_include_directories(yolo_depth PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${ORT_ROOT}/include
  $<INSTALL_INTERFACE:include>)
target_link_libraries(yolo_depth ${OpenCV_LIBS} ${ORT_ROOT}/lib/libonnxruntime.so)

# ---------- 核心算法库: YOLO OBB 旋转框检测 (无 ROS 依赖, onnxruntime 推理) ----------
add_library(yolo_obb src/yolo/yolo_obb.cpp)
target_include_directories(yolo_obb PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${ORT_ROOT}/include
  $<INSTALL_INTERFACE:include>)
target_link_libraries(yolo_obb ${OpenCV_LIBS} ${ORT_ROOT}/lib/libonnxruntime.so)

# ---------- 核心算法库: YOLO 姿态估计 (无 ROS 依赖, onnxruntime 推理) ----------
add_library(yolo_pose src/yolo/yolo_pose.cpp)
target_include_directories(yolo_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${ORT_ROOT}/include
  $<INSTALL_INTERFACE:include>)
target_link_libraries(yolo_pose ${OpenCV_LIBS} ${ORT_ROOT}/lib/libonnxruntime.so)

# ---------- 核心算法库: YOLO 实例分割 (无 ROS 依赖, onnxruntime 推理) ----------
# 链接 yolo_detector 复用 coco_names() (分割同为 COCO 80 类)
add_library(yolo_segment src/yolo/yolo_segment.cpp)
target_include_directories(yolo_segment PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${ORT_ROOT}/include
  $<INSTALL_INTERFACE:include>)
target_link_libraries(yolo_segment yolo_detector ${OpenCV_LIBS})

# ---------- ROS2 节点: YOLO 目标检测 (默认模型 share/<pkg>/models/detect/) ----------
add_executable(yolo_detect_node src/yolo/yolo_node.cpp)
target_link_libraries(yolo_detect_node yolo_detector)
ament_target_dependencies(
  yolo_detect_node rclcpp sensor_msgs std_msgs ament_index_cpp)

# ---------- ROS2 节点: YOLO 图像分类 (默认模型 share/<pkg>/models/classify/) ----------
add_executable(yolo_cls_node src/yolo/yolo_cls_node.cpp)
target_link_libraries(yolo_cls_node yolo_classifier)
ament_target_dependencies(
  yolo_cls_node rclcpp sensor_msgs std_msgs ament_index_cpp)

# ---------- ROS2 节点: YOLO 单目深度估计 (默认模型 share/<pkg>/models/depth/) ----------
add_executable(yolo_depth_node src/yolo/yolo_depth_node.cpp)
target_link_libraries(yolo_depth_node yolo_depth)
ament_target_dependencies(
  yolo_depth_node rclcpp sensor_msgs std_msgs ament_index_cpp)

# ---------- ROS2 节点: YOLO OBB 旋转框检测 (默认模型 share/<pkg>/models/obb/) ----------
add_executable(yolo_obb_node src/yolo/yolo_obb_node.cpp)
target_link_libraries(yolo_obb_node yolo_obb)
ament_target_dependencies(
  yolo_obb_node rclcpp sensor_msgs std_msgs ament_index_cpp)

# ---------- ROS2 节点: YOLO 姿态估计 (默认模型 share/<pkg>/models/pose/) ----------
add_executable(yolo_pose_node src/yolo/yolo_pose_node.cpp)
target_link_libraries(yolo_pose_node yolo_pose)
ament_target_dependencies(
  yolo_pose_node rclcpp sensor_msgs std_msgs ament_index_cpp)

# ---------- ROS2 节点: YOLO 实例分割 (默认模型 share/<pkg>/models/segment/) ----------
add_executable(yolo_segment_node src/yolo/yolo_segment_node.cpp)
target_link_libraries(yolo_segment_node yolo_segment)
ament_target_dependencies(
  yolo_segment_node rclcpp sensor_msgs std_msgs ament_index_cpp)
# 安装后 libonnxruntime 在 lib/ 根目录, 各目标按相对位置解析
set_target_properties(yolo_detector PROPERTIES INSTALL_RPATH "$ORIGIN")
set_target_properties(yolo_classifier PROPERTIES INSTALL_RPATH "$ORIGIN")
set_target_properties(yolo_depth PROPERTIES INSTALL_RPATH "$ORIGIN")
set_target_properties(yolo_obb PROPERTIES INSTALL_RPATH "$ORIGIN")
set_target_properties(yolo_pose PROPERTIES INSTALL_RPATH "$ORIGIN")
set_target_properties(yolo_segment PROPERTIES INSTALL_RPATH "$ORIGIN")
set_target_properties(yolo_detect_node PROPERTIES INSTALL_RPATH "$ORIGIN/../;$ORIGIN")
set_target_properties(yolo_cls_node PROPERTIES INSTALL_RPATH "$ORIGIN/../;$ORIGIN")
set_target_properties(yolo_depth_node PROPERTIES INSTALL_RPATH "$ORIGIN/../;$ORIGIN")
set_target_properties(yolo_obb_node PROPERTIES INSTALL_RPATH "$ORIGIN/../;$ORIGIN")
set_target_properties(yolo_pose_node PROPERTIES INSTALL_RPATH "$ORIGIN/../;$ORIGIN")
set_target_properties(yolo_segment_node PROPERTIES INSTALL_RPATH "$ORIGIN/../;$ORIGIN")

# ---------- ROS2 节点: 地面站视觉工具链路 ----------
add_executable(aruco_node src/aruco/aruco_node.cpp)
target_link_libraries(aruco_node aruco_detector)
ament_target_dependencies(aruco_node rclcpp sensor_msgs std_msgs)

# ---------- ROS2 节点: 飞行控制 (ArUco 视觉降落 / 后续目标追踪控制等, 执行层见 include/control/) ----------
add_executable(aruco_landing_node src/aruco/aruco_landing_node.cpp)
target_link_libraries(aruco_landing_node aruco_detector)
ament_target_dependencies(aruco_landing_node rclcpp sensor_msgs std_msgs px4_msgs geometry_msgs mavros_msgs)

# 模型权重装到 share/<pkg>/models/, yolo_detect_node 经 ament_index 解析默认路径
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME})
# onnxruntime 动态库 (含版本符号链接) 装到 lib/, 供 rpath 解析
install(DIRECTORY ${ORT_ROOT}/lib/
  DESTINATION lib
  FILES_MATCHING PATTERN "libonnxruntime*")

install(TARGETS aruco_detector yolo_detector yolo_classifier yolo_depth yolo_obb yolo_pose yolo_segment
  aruco_node yolo_detect_node yolo_cls_node yolo_depth_node yolo_obb_node yolo_pose_node yolo_segment_node aruco_landing_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
