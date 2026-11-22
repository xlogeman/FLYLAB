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

# ---------- aruco 3.1.12 (第三方库, 分形码检测后端; UCO aruco, GPLv3) ----------
# 分形码 = 大码内嵌套逐层缩小的小码, 远距/遮挡鲁棒 (Case2 关键点跟踪接管位姿);
# 源码 + 预编译库在 thirdparty/aruco-3.1.12 (裁剪自 /root/FractalMarkers)
set(ARUCO3_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/aruco-3.1.12)
set(ARUCO3_LIB ${ARUCO3_ROOT}/build/libaruco.so)

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
# 分形码检测: 包装上游 aruco::FractalDetector, 输出复用 Detection;
# SYSTEM PUBLIC 让上游头文件的告警静默, 下游节点经传递拿到 include 路径
flylab_algo(fractal_detector src/aruco/fractal_detector.cpp ${ARUCO3_LIB})
target_include_directories(fractal_detector SYSTEM PUBLIC ${ARUCO3_ROOT}/src)

# 任务一个子目录: src/<task>/ (命名与 models/<task>/ 对齐, 同一任务可挂
# 多种算法/模型变体), 算法库 + 展示节点成对放置
flylab_algo(yolo_detector src/detect/yolo_detector.cpp ${ORT_LIB})
flylab_algo(yolo_classifier src/classify/yolo_classifier.cpp ${ORT_LIB})
flylab_algo(yolo_depth src/depth/yolo_depth.cpp ${ORT_LIB})
flylab_algo(yolo_obb src/obb/yolo_obb.cpp ${ORT_LIB})
flylab_algo(yolo_pose src/pose/yolo_pose.cpp ${ORT_LIB})
# 链接 yolo_detector 复用 coco_names() (分割同为 COCO 80 类)
flylab_algo(yolo_segment src/segment/yolo_segment.cpp yolo_detector ${ORT_LIB})
flylab_algo(yolo_semantic src/semantic/yolo_semantic.cpp ${ORT_LIB})
# 目标追踪任务 (src/track/, 对应 models/track/): ReID 单目标跟踪
# (链接 yolo_detector 复用 YoloDetection/色板) + ORTrack 模板匹配式
flylab_algo(yolo_tracker src/track/yolo_tracker.cpp yolo_detector ${ORT_LIB})
# 目标追踪: ORTrack 模板匹配式单目标跟踪 (CVPR 2025, 纯模板+搜索区前向)
flylab_algo(ortrack src/track/ortrack.cpp ${ORT_LIB})
# 手部姿态: WiLoR (CVPR 2025) 两级管线 —— YOLO 手部检测 + 逐手 MANO
# 回归, 21 关键点经逆仿射回原图 (include/wilor/, src/wilor/)
flylab_algo(wilor src/wilor/wilor.cpp ${ORT_LIB})
# 人脸检测任务 (src/face/, 对应 models/face/): insightface SCRFD-10G
# (9 输出 = 3 尺度 score/框距离/5 关键点距离, distance 解码 + NMS)
flylab_algo(scrfd_detector src/face/scrfd_detector.cpp ${ORT_LIB})
# 人脸下游任务: FaceXFormer (ICCV 2025) 统一模型 6 任务 —— 分割/68 点/
# 头部姿态/CelebA 属性/年龄性别种族/29 点可见性 (单模型 8 输出按任务解码)
flylab_algo(facexformer src/face/facexformer.cpp ${ORT_LIB})

# ---------- ROS2 展示节点 (地面站视觉工具链路) ----------
flylab_node(aruco_node src/aruco/aruco_node.cpp aruco_detector)
flylab_node(fractal_node src/aruco/fractal_node.cpp fractal_detector aruco_detector)
flylab_node(yolo_detector_node src/detect/yolo_detector_node.cpp yolo_detector)
flylab_node(yolo_cls_node src/classify/yolo_cls_node.cpp yolo_classifier)
flylab_node(yolo_depth_node src/depth/yolo_depth_node.cpp yolo_depth)
flylab_node(yolo_obb_node src/obb/yolo_obb_node.cpp yolo_obb)
flylab_node(yolo_pose_node src/pose/yolo_pose_node.cpp yolo_pose)
flylab_node(yolo_segment_node src/segment/yolo_segment_node.cpp yolo_segment)
flylab_node(yolo_semantic_node src/semantic/yolo_semantic_node.cpp yolo_semantic)
flylab_node(yolo_track_node src/track/yolo_track_node.cpp yolo_tracker yolo_detector)
flylab_node(ortrack_node src/track/ortrack_node.cpp ortrack yolo_classifier)
# 两个追踪节点接入了反控接管层 (control/track_takeover.hpp → px4_offboard_client),
# 补 PX4 执行链路依赖 (展示节点模板默认只有 sensor/std_msgs)
ament_target_dependencies(yolo_track_node px4_msgs geometry_msgs mavros_msgs)
ament_target_dependencies(ortrack_node px4_msgs geometry_msgs mavros_msgs)
flylab_node(wilor_node src/wilor/wilor_node.cpp wilor)
flylab_node(face_detect_node src/face/face_detect_node.cpp scrfd_detector facexformer)

# ---------- ROS2 节点: ArUco 视觉降落 (业务节点, 执行层见 include/control/) ----------
# 不走 flylab_node 模板: 控制链路依赖与展示型节点不同
add_executable(aruco_landing_node src/aruco/aruco_landing_node.cpp)
target_link_libraries(aruco_landing_node aruco_detector fractal_detector vision_common)
ament_target_dependencies(
  aruco_landing_node rclcpp sensor_msgs std_msgs px4_msgs geometry_msgs mavros_msgs)

# ---------- ROS2 节点: 手动遥控 (方向键 → offboard 速度+偏航角速率) ----------
add_executable(teleop_node src/teleop/teleop_node.cpp)
target_link_libraries(teleop_node vision_common)
ament_target_dependencies(teleop_node rclcpp std_msgs px4_msgs geometry_msgs mavros_msgs)

# WiLoR 模型 (models/hand/): 只装 detector.onnx + wilor_fp16_dyn.onnx
# (动态 batch 版, 默认; 旧静态 wilor_fp16.onnx 保留可经参数回退), 共 ~2.5GB;
# MANO 网格拓扑 (原 mano_faces.npy) 已烘焙
# 进 include/wilor/mano_faces.hpp, 节点经 ament share 解析,
# detector_model/wilor_model 参数可覆盖
install(DIRECTORY models/hand
  DESTINATION share/${PROJECT_NAME}/models)

# 模型权重装到 share/<pkg>/models/, 各节点经 ament_index 解析默认路径。
# 只装 .onnx 推理模型: .pt 是训练权重 (~200MB), 运行时无用, 不随包拷贝
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.onnx")
# 参数配置 (configs/aruco.yaml 等) 装到 share/<pkg>/configs/, 地面站经 ament 解析携带
install(DIRECTORY configs
  DESTINATION share/${PROJECT_NAME})
# onnxruntime 动态库 (含版本符号链接) 装到 lib/, 供 rpath 解析
install(DIRECTORY ${ORT_ROOT}/lib/
  DESTINATION lib
  FILES_MATCHING PATTERN "libonnxruntime*")

# aruco 3.1.12 动态库 (含版本符号链接) 装到 lib/, 供 rpath 解析
install(DIRECTORY ${ARUCO3_ROOT}/build/
  DESTINATION lib
  FILES_MATCHING PATTERN "libaruco*")

install(TARGETS vision_common
  aruco_detector fractal_detector yolo_detector yolo_classifier yolo_depth yolo_obb yolo_pose yolo_segment yolo_semantic yolo_tracker ortrack wilor scrfd_detector facexformer
  aruco_node fractal_node yolo_detector_node yolo_cls_node yolo_depth_node yolo_obb_node yolo_pose_node yolo_segment_node yolo_semantic_node yolo_track_node ortrack_node wilor_node face_detect_node aruco_landing_node teleop_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
