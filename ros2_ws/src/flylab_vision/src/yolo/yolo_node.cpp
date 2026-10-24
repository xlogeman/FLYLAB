// Copyright 2026 flylab
/// YOLO 目标检测 ROS2 节点 —— 接入地面站视觉工具体系 (flylab_tools 既有约定)。
///
/// 链路 (话题名与 flylab_tools/config.py 保持一致, 均可用参数覆盖):
///   1. 图像输入: 地面站把"当前显示的帧" (AirSim 相机/本机摄像头/视频
///      文件, 用户看到什么就转什么) 发到 /flylab/display/image_raw,
///      本节点订阅它处理
///   2. 工具按钮: 地面站往 /flylab/vision_cmd 发 detect_start / detect_stop,
///      本节点据此启停检测 (常驻运行, 不用每次重启节点)
///   3. 画面叠加: 每帧把检测结果转成 JSON 标注发往 /flylab/annotations
///      (schema 见 flylab_tools/overlay.py, algo 组名 "yolo"),
///      地面站自动叠加到画面上
///
/// 模型默认取 share/flylab_vision/models/detect/yolo26s.onnx
/// (CMakeLists 安装), 可用 model 参数覆盖。
///
/// 运行:
///   ros2 run flylab_vision yolo_detect_node
///   ros2 run flylab_vision yolo_detect_node --ros-args
///       -p model:=/path/to/xx.onnx -p conf:=0.35 -p device:=cuda
#include <algorithm>
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include <opencv2/imgproc.hpp>

#include "yolo/yolo_detector.hpp"

namespace flylab_vision
{

namespace
{
constexpr const char kCmdStart[] = "detect_start";
constexpr const char kCmdStop[] = "detect_stop";
constexpr const char kAlgoGroup[] = "yolo";

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

/// 默认模型路径: ament share 目录下的 models/detect/yolo26s.onnx
std::string default_model_path()
{
  try {
    return ament_index_cpp::get_package_share_directory("flylab_vision") +
           "/models/detect/yolo26s.onnx";
  } catch (const std::exception &) {
    return "";
  }
}
}  // namespace

class YoloNode : public rclcpp::Node
{
public:
  YoloNode()
  : Node("yolo_detect_node")
  {
    // ---------- 参数 ----------
    const auto model = declare_parameter<std::string>("model", default_model_path());
    const auto input_size = declare_parameter<int>("input_size", 640);
    const auto conf = declare_parameter<double>("conf", 0.25);
    const auto iou = declare_parameter<double>("iou", 0.45);
    const auto device = declare_parameter<std::string>("device", "cpu");
    // 逗号分隔的类别名过滤器, 空 = 全部 80 类
    const auto classes = declare_parameter<std::string>("classes", "");
    const auto image_topic = declare_parameter<std::string>(
      "image_topic", "/flylab/display/image_raw");
    const auto cmd_topic = declare_parameter<std::string>("cmd_topic", "/flylab/vision_cmd");
    const auto anno_topic =
      declare_parameter<std::string>("annotation_topic", "/flylab/annotations");
    image_topic_ = image_topic;

    // ---------- 模型 (启动即加载, 失败抛异常直接终止暴露配置问题) ----------
    YoloConfig cfg;
    cfg.model_path = model;
    cfg.input_size = input_size;
    cfg.conf_thres = static_cast<float>(conf);
    cfg.iou_thres = static_cast<float>(iou);
    cfg.use_cuda = device == "cuda";
    if (!classes.empty()) {
      std::stringstream ss(classes);
      std::string name;
      while (std::getline(ss, name, ',')) {
        const auto & names = coco_names();
        const auto it = std::find(names.begin(), names.end(), name);
        if (it == names.end()) {
          throw std::invalid_argument("未知类别名: " + name);
        }
        cfg.class_filter.push_back(static_cast<int>(it - names.begin()));
      }
    }
    detector_ = std::make_unique<YoloDetector>(cfg);
    RCLCPP_INFO(
      get_logger(),
      "模型加载完成: %s [%s, input %d, conf %.2f, iou %.2f]",
      model.c_str(), detector_->cuda_active() ? "cuda" : "cpu",
      cfg.input_size, cfg.conf_thres, cfg.iou_thres);
    if (cfg.use_cuda && !detector_->cuda_active()) {
      RCLCPP_WARN(
        get_logger(), "CUDA EP 注册失败, 已回退 CPU: %s",
        detector_->cuda_error().c_str());
    }

    // ---------- 通信 ----------
    anno_pub_ = create_publisher<std_msgs::msg::String>(anno_topic, 10);
    cmd_sub_ = create_subscription<std_msgs::msg::String>(
      cmd_topic, 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {on_cmd(*msg);});
    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {on_image(*msg);});

    RCLCPP_INFO(
      get_logger(),
      "YOLO 检测节点就绪: 订阅 %s, 等待 %s 指令",
      image_topic.c_str(), kCmdStart);

    // 诊断: 每 2 秒检查收帧情况, 收不到帧时主动告警
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
  void publish_items(const std::vector<YoloDetection> & items)
  {
    std_msgs::msg::String out;
    out.data = to_annotations_json(items, kAlgoGroup);
    anno_pub_->publish(out);
  }

  void on_cmd(const std_msgs::msg::String & msg)
  {
    if (msg.data == kCmdStart && !enabled_) {
      enabled_ = true;
      RCLCPP_INFO(get_logger(), "检测已启动");
    } else if (msg.data == kCmdStop && enabled_) {
      enabled_ = false;
      publish_items({});   // 清自己的分组标注
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
    const auto t0 = std::chrono::steady_clock::now();
    const auto results = detector_->detect(bgr);
    if (!first_infer_done_) {
      first_infer_done_ = true;
      const auto ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0).count();
      RCLCPP_INFO(get_logger(), "首帧推理耗时 %ldms, 检出 %zu 个目标",
        ms, results.size());
    }
    rx_since_log_ += 1;
    publish_items(results);
  }

  std::unique_ptr<YoloDetector> detector_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr anno_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  std::string image_topic_;
  uint32_t rx_since_log_ = 0;   // 距上次统计周期收到的帧数
  bool enabled_ = false;
  bool got_first_frame_ = false;
  bool first_infer_done_ = false;
};

}  // namespace flylab_vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<flylab_vision::YoloNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
