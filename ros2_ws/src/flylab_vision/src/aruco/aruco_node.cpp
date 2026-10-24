// Copyright 2026 flylab
/// ArUco 检测 ROS2 节点 —— 接入地面站视觉工具体系 (flylab_tools 既有约定)。
///
/// 链路 (话题名与 flylab_tools/config.py 保持一致, 均可用参数覆盖):
///   1. 图像输入: 地面站把"当前显示的帧" (AirSim 相机/本机摄像头/视频
///      文件, 用户看到什么就转什么) 发到 /flylab/display/image_raw,
///      本节点订阅它处理
///   2. 工具按钮: 地面站往 /flylab/vision_cmd 发 aruco_start / aruco_stop,
///      本节点据此启停检测 (常驻运行, 不用每次重启节点)
///   3. 画面叠加: 每帧把检测结果转成 JSON 标注发往 /flylab/annotations
///      (schema 见 flylab_tools/overlay.py), 地面站自动叠加到画面上
///
/// 运行:
///   ros2 run flylab_vision aruco_node
///   ros2 run flylab_vision aruco_node --ros-args
///       -p image_topic:=/airsim/cam/left_cam/image_raw -p dict_name:=DICT_6X6_250

#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include <opencv2/imgproc.hpp>

#include "aruco/aruco_detector.hpp"

namespace flylab_vision
{

namespace
{
constexpr const char kCmdStart[] = "aruco_start";
constexpr const char kCmdStop[] = "aruco_stop";

/// sensor_msgs/Image -> BGR cv::Mat (手动转换, 不依赖 cv_bridge);
/// 失败返回空 Mat
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
}  // namespace

class ArucoNode : public rclcpp::Node
{
public:
  ArucoNode()
  : Node("aruco_detector")
  {
    // ---------- 参数 ----------
    const auto image_topic = declare_parameter<std::string>(
      "image_topic", "/flylab/display/image_raw");
    const auto cmd_topic = declare_parameter<std::string>("cmd_topic", "/flylab/vision_cmd");
    const auto anno_topic =
      declare_parameter<std::string>("annotation_topic", "/flylab/annotations");
    const auto dict_name = declare_parameter<std::string>("dict_name", "DICT_4X4_1000");
    const auto marker_length = declare_parameter<double>("marker_length", 0.2);
    const auto fov_deg = declare_parameter<double>("fov_deg", 60.0);
    const auto calib_path = declare_parameter<std::string>("calib", "");
    image_topic_ = image_topic;

    DetectorConfig cfg;
    cfg.dict_name = dict_name;
    cfg.marker_length = marker_length;
    cfg.fov_deg = fov_deg;
    if (!calib_path.empty()) {
      load_calib(calib_path, cfg.K, cfg.D);   // 失败抛异常, 直接终止暴露配置问题
    }
    detector_ = std::make_unique<ArucoDetector>(cfg);
    if (!detector_->calibrated()) {
      RCLCPP_WARN(
        get_logger(),
        "未提供标定 (calib 参数), 内参按 FOV 估算, 位姿解算仅供参考");
    }

    // ---------- 通信 ----------
    anno_pub_ = create_publisher<std_msgs::msg::String>(anno_topic, 10);
    cmd_sub_ = create_subscription<std_msgs::msg::String>(
      cmd_topic, 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {on_cmd(*msg);});
    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {on_image(*msg);});

    RCLCPP_INFO(
      get_logger(),
      "ArUco 检测节点就绪: 字典 %s, 边长 %.2fm, 订阅 %s, 等待 %s 指令",
      dict_name.c_str(), marker_length, image_topic.c_str(), kCmdStart);

    // 诊断统计: 每 2 秒汇报收帧/检出情况, 收不到帧时主动告警
    stats_timer_ = create_wall_timer(std::chrono::seconds(2), [this]() {
        if (!enabled_) {
          return;
        }
        if (rx_since_log_ == 0) {
          RCLCPP_WARN(
            get_logger(),
            "近 2 秒未收到任何图像帧! 检查: 1) 地面站是否在运行 2) 画面源"
            "是否有帧 3) 图像话题 %s 是否一致",
            image_topic_.c_str());
        }
        rx_since_log_ = 0;
      });
  }

private:
  void on_cmd(const std_msgs::msg::String & msg)
  {
    if (msg.data == kCmdStart && !enabled_) {
      enabled_ = true;
      RCLCPP_INFO(get_logger(), "检测已启动");
    } else if (msg.data == kCmdStop && enabled_) {
      enabled_ = false;
      std_msgs::msg::String clear;
      clear.data = flylab_vision::to_annotations_json({});  // 清自己的分组标注
      anno_pub_->publish(clear);
      RCLCPP_INFO(get_logger(), "检测已停止");
    }
  }

  void on_image(const sensor_msgs::msg::Image & msg)
  {
    if (!enabled_) {
      return;
    }
    if (!got_first_frame_) {
      got_first_frame_ = true;
      RCLCPP_INFO(
        get_logger(), "收到首帧图像: %ux%u (%s)",
        msg.width, msg.height, msg.encoding.c_str());
    }
    const cv::Mat bgr = to_bgr(msg);
    if (bgr.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "图像数据不完整或编码不支持 (%s), 跳过", msg.encoding.c_str());
      return;
    }
    const auto results = detector_->detect(bgr);
    rx_since_log_ += 1;
    std_msgs::msg::String out;
    out.data = to_annotations_json(results);
    anno_pub_->publish(out);
  }

  std::unique_ptr<ArucoDetector> detector_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr anno_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  std::string image_topic_;
  uint32_t rx_since_log_ = 0;    // 距上次统计周期收到的帧数
  bool enabled_ = false;
  bool got_first_frame_ = false;
};

}  // namespace flylab_vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<flylab_vision::ArucoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
