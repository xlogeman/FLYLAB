// Copyright 2026 flylab
/// YOLO 图像分类 ROS2 节点 —— 接入地面站视觉工具体系 (flylab_tools 既有约定)。
///
/// 与 yolo_node.cpp 同构, 区别只在算法 (整图分类 vs 目标检测) 和标注形式
/// (左上角 top-k 文字行 vs 目标框)。链路 (话题名与 flylab_tools/config.py
/// 保持一致, 均可用参数覆盖):
///   1. 图像输入: 订阅地面站"当前显示的帧" /flylab/display/image_raw
///   2. 工具按钮: 地面站往 /flylab/vision_cmd 发 cls_start / cls_stop,
///      本节点据此启停推理 (常驻运行, 不用每次重启节点)
///   3. 画面叠加: 每帧把 top-k 结果转成 JSON 标注发往 /flylab/annotations
///      (schema 见 flylab_tools/overlay.py, algo 组名 "yolo_cls",
///      与检测 "yolo" 分组独立, 可并存)
///
/// 模型默认取 share/flylab_vision/models/classify/yolo26s-cls.onnx
/// (CMakeLists 安装), 输入尺寸自动取模型固定值, 标签自动取模型内嵌
/// metadata, 都可用参数覆盖。
///
/// 运行:
///   ros2 run flylab_vision yolo_cls_node
///   ros2 run flylab_vision yolo_cls_node --ros-args
///       -p model:=/path/to/xx.onnx -p topk:=3 -p device:=cuda
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include <opencv2/imgproc.hpp>

#include "yolo/yolo_classifier.hpp"

namespace flylab_vision
{

namespace
{
constexpr const char kCmdStart[] = "cls_start";
constexpr const char kCmdStop[] = "cls_stop";
constexpr const char kAlgoGroup[] = "yolo_cls";

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

/// 默认模型路径: ament share 目录下的 models/classify/yolo26s-cls.onnx
std::string default_model_path()
{
  try {
    return ament_index_cpp::get_package_share_directory("flylab_vision") +
           "/models/classify/yolo26s-cls.onnx";
  } catch (const std::exception &) {
    return "";
  }
}
}  // namespace

class YoloClsNode : public rclcpp::Node
{
public:
  YoloClsNode()
  : Node("yolo_cls_node")
  {
    // ---------- 参数 ----------
    const auto model = declare_parameter<std::string>("model", default_model_path());
    // 0 = 自动取模型固定输入尺寸 (yolo26s-cls 导出为 224)
    const auto input_size = declare_parameter<int>("input_size", 0);
    const auto topk = declare_parameter<int>("topk", 3);
    const auto device = declare_parameter<std::string>("device", "cpu");
    const auto labels = declare_parameter<std::string>("labels", "");
    const auto image_topic = declare_parameter<std::string>(
      "image_topic", "/flylab/display/image_raw");
    const auto cmd_topic = declare_parameter<std::string>("cmd_topic", "/flylab/vision_cmd");
    const auto anno_topic =
      declare_parameter<std::string>("annotation_topic", "/flylab/annotations");
    image_topic_ = image_topic;

    // ---------- 模型 (启动即加载, 失败抛异常直接终止暴露配置问题) ----------
    ClsConfig cfg;
    cfg.model_path = model;
    cfg.input_size = input_size;
    cfg.topk = topk;
    cfg.labels_file = labels;
    cfg.use_cuda = device == "cuda";
    classifier_ = std::make_unique<YoloClassifier>(cfg);
    RCLCPP_INFO(
      get_logger(),
      "模型加载完成: %s [%s, input %d, classes %d, topk %d]",
      model.c_str(), classifier_->cuda_active() ? "cuda" : "cpu",
      classifier_->input_size(), classifier_->num_classes(), cfg.topk);
    if (cfg.use_cuda && !classifier_->cuda_active()) {
      RCLCPP_WARN(
        get_logger(), "CUDA EP 注册失败, 已回退 CPU: %s",
        classifier_->cuda_error().c_str());
    }

    // ---------- 通信 ----------
    anno_pub_ = create_publisher<std_msgs::msg::String>(anno_topic, 10);
    cmd_sub_ = create_subscription<std_msgs::msg::String>(
      cmd_topic, 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {on_cmd(*msg);});
    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::QoS(rclcpp::KeepLast(2)).reliable(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {on_image(*msg);});

    RCLCPP_INFO(
      get_logger(),
      "YOLO 分类节点就绪: 订阅 %s, 等待 %s 指令",
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
  void publish_preds(const std::vector<ClsPrediction> & preds, int frame_width)
  {
    std_msgs::msg::String out;
    out.data = to_cls_annotations_json(preds, kAlgoGroup, frame_width);
    anno_pub_->publish(out);
  }

  void on_cmd(const std_msgs::msg::String & msg)
  {
    if (msg.data == kCmdStart && !enabled_) {
      enabled_ = true;
      RCLCPP_INFO(get_logger(), "分类已启动");
    } else if (msg.data == kCmdStop && enabled_) {
      enabled_ = false;
      publish_preds({}, 0);   // 清自己的分组标注
      RCLCPP_INFO(get_logger(), "分类已停止");
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
    const auto results = classifier_->classify(bgr);
    if (!first_infer_done_) {
      first_infer_done_ = true;
      const auto ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0).count();
      RCLCPP_INFO(get_logger(), "首帧推理耗时 %ldms, top1: %s %.2f",
        ms,
        results.empty() ? "-" : results.front().name.c_str(),
        results.empty() ? 0.f : results.front().conf);
    }
    rx_since_log_ += 1;
    publish_preds(results, msg.width);
  }

  std::unique_ptr<YoloClassifier> classifier_;
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
  auto node = std::make_shared<flylab_vision::YoloClsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
