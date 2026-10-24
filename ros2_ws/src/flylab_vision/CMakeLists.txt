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
set(ORT_LIB ${ORT_ROOT}/lib/libonnxruntime.so)

# ---------- 节点层公共工具 (to_bgr / 默认模型路径解析) ----------
# 仅供 ROS 节点链接; 算法库保持无 ROS 依赖, 不链接本目标
add_library(vision_common src/common/image_utils.cpp)
target_include_directories(vision_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(vision_common ${OpenCV_LIBS})
ament_target_dependencies(vision_common sensor_msgs ament_index_cpp)

# ---------- 算法库模板: add_library + 头文件路径 + ort 头 + rpath ----------
# 额外参数为需要链接的库 (ORT_LIB / 其他算法库); 算法库本身纯 OpenCV, 无 ROS 依赖
function(flylab_algo target src)
  add_library(${target} ${src})
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_include_directories(${target} SYSTEM PRIVATE ${ORT_ROOT}/include)
  target_link_libraries(${target} ${OpenCV_LIBS} ${ARGN})
  set_target_properties(${target} PROPERTIES INSTALL_RPATH "$ORIGIN")
endfunction()

# ---------- ROS 展示节点模板: add_executable + 公共依赖 + rpath ----------
# 额外参数为需要链接的算法库; 节点自动链接 vision_common (图像转换/模型路径),
# 通信链路封装见 include/common/display_vision_node.hpp
function(flylab_node target src)
  add_executable(${target} ${src})
  target_link_libraries(${target} vision_common ${ARGN})
  ament_target_dependencies(${target} rclcpp sensor_msgs std_msgs ament_index_cpp)
  set_target_properties(${target} PROPERTIES INSTALL_RPATH "$ORIGIN/../;$ORIGIN")
endfunction()

# ---------- 核心算法库 (无 ROS 依赖) ----------
flylab_algo(aruco_detector src/aruco/aruco_detector.cpp)

# 各任务一个子目录: src/yolo/<task>/ (算法库 + 展示节点成对放置)
flylab_algo(yolo_detector src/yolo/detector/yolo_detector.cpp ${ORT_LIB})
flylab_algo(yolo_classifier src/yolo/classifier/yolo_classifier.cpp ${ORT_LIB})
flylab_algo(yolo_depth src/yolo/depth/yolo_depth.cpp ${ORT_LIB})
flylab_algo(yolo_obb src/yolo/obb/yolo_obb.cpp ${ORT_LIB})
flylab_algo(yolo_pose src/yolo/pose/yolo_pose.cpp ${ORT_LIB})
# 链接 yolo_detector 复用 coco_names() (分割同为 COCO 80 类)
flylab_algo(yolo_segment src/yolo/segment/yolo_segment.cpp yolo_detector ${ORT_LIB})
flylab_algo(yolo_semantic src/yolo/semantic/yolo_semantic.cpp ${ORT_LIB})

# ---------- ROS2 展示节点 (地面站视觉工具链路) ----------
flylab_node(aruco_node src/aruco/aruco_node.cpp aruco_detector)
flylab_node(yolo_detector_node src/yolo/detector/yolo_detector_node.cpp yolo_detector)
flylab_node(yolo_cls_node src/yolo/classifier/yolo_cls_node.cpp yolo_classifier)
flylab_node(yolo_depth_node src/yolo/depth/yolo_depth_node.cpp yolo_depth)
flylab_node(yolo_obb_node src/yolo/obb/yolo_obb_node.cpp yolo_obb)
flylab_node(yolo_pose_node src/yolo/pose/yolo_pose_node.cpp yolo_pose)
flylab_node(yolo_segment_node src/yolo/segment/yolo_segment_node.cpp yolo_segment)
flylab_node(yolo_semantic_node src/yolo/semantic/yolo_semantic_node.cpp yolo_semantic)

# ---------- ROS2 节点: ArUco 视觉降落 (业务节点, 执行层见 include/control/) ----------
# 不走 flylab_node 模板: 控制链路依赖与展示型节点不同
add_executable(aruco_landing_node src/aruco/aruco_landing_node.cpp)
target_link_libraries(aruco_landing_node aruco_detector vision_common)
ament_target_dependencies(
  aruco_landing_node rclcpp sensor_msgs std_msgs px4_msgs geometry_msgs mavros_msgs)

# ---------- ROS2 节点: 手动遥控 (方向键 → offboard 速度+偏航角速率) ----------
add_executable(teleop_node src/teleop/teleop_node.cpp)
target_link_libraries(teleop_node vision_common)
ament_target_dependencies(teleop_node rclcpp std_msgs px4_msgs geometry_msgs mavros_msgs)

# 模型权重装到 share/<pkg>/models/, 各节点经 ament_index 解析默认路径。
# 只装 .onnx 推理模型: .pt 是训练权重 (~200MB), 运行时无用, 不随包拷贝
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.onnx")
# onnxruntime 动态库 (含版本符号链接) 装到 lib/, 供 rpath 解析
install(DIRECTORY ${ORT_ROOT}/lib/
  DESTINATION lib
  FILES_MATCHING PATTERN "libonnxruntime*")

install(TARGETS vision_common
  aruco_detector yolo_detector yolo_classifier yolo_depth yolo_obb yolo_pose yolo_segment yolo_semantic
  aruco_node yolo_detector_node yolo_cls_node yolo_depth_node yolo_obb_node yolo_pose_node yolo_segment_node yolo_semantic_node aruco_landing_node teleop_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
