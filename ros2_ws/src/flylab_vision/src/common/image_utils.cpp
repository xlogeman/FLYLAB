// Copyright 2026 flylab
/// 节点层公共工具实现 —— 仅供 ROS 节点使用, 算法库保持无 ROS 依赖
#include "common/image_utils.hpp"

#include <exception>

#include <opencv2/imgproc.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace flylab_vision
{

cv::Mat to_bgr(const sensor_msgs::msg::Image & msg)
{
  int ch = 0;
  int code = 0;  // 非 0 表示需要 cvtColor
  if (msg.encoding == "bgr8") {
    ch = 3;
  } else if (msg.encoding == "rgb8") {
    ch = 3;
    code = cv::COLOR_RGB2BGR;
  } else if (msg.encoding == "mono8") {
    ch = 1;
    code = cv::COLOR_GRAY2BGR;
  } else {
    return {};
  }
  const size_t expect = static_cast<size_t>(msg.height) * msg.width * ch;
  if (msg.height == 0 || msg.width == 0 || msg.data.size() < expect) {
    return {};
  }
  const int type = ch == 3 ? CV_8UC3 : CV_8UC1;
  // 用消息的 step (行字节数) 构造, 兼容行尾有填充的编码器
  cv::Mat m(
    msg.height, msg.width, type,
    const_cast<uint8_t *>(msg.data.data()), msg.step);
  cv::Mat out = m.clone();  // 数据来自消息, 拷贝一份脱离其生命周期
  if (code) {
    cv::cvtColor(out, out, code);
  }
  return out;
}

std::string default_model_path(const char * sub, const char * file)
{
  try {
    return ament_index_cpp::get_package_share_directory("flylab_vision") +
           "/models/" + sub + "/" + file;
  } catch (const std::exception &) {
    return "";
  }
}

}  // namespace flylab_vision
