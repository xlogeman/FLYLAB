// Copyright 2026 flylab
#ifndef FLYLAB_VISION__COMMON__IMAGE_UTILS_HPP_
#define FLYLAB_VISION__COMMON__IMAGE_UTILS_HPP_

#include <string>

#include <opencv2/core.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace flylab_vision
{

/// sensor_msgs/Image -> BGR cv::Mat (手动转换, 不依赖 cv_bridge);
/// 失败返回空 Mat
cv::Mat to_bgr(const sensor_msgs::msg::Image & msg);

/// 默认模型路径: ament share 目录下 models/<sub>/<file>
/// (路径由 CMakeLists 安装, 见 install(DIRECTORY models ...));
/// 包 share 目录解析失败返回空串
std::string default_model_path(const char * sub, const char * file);

}  // namespace flylab_vision

#endif  // FLYLAB_VISION__COMMON__IMAGE_UTILS_HPP_
