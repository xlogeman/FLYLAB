// Copyright 2026 flylab
/// YOLO 语义分割 ROS2 节点 —— 接入地面站视觉工具体系 (flylab_tools 既有约定)。
///
/// 链路 (与 yolo_segment_node 相同, 话题名与 flylab_tools/config.py 保持一致,
/// 均可用参数覆盖):
///   1. 图像输入: 地面站把"当前显示的帧"发到 /flylab/display/image_raw,
///      本节点订阅它处理
///   2. 工具按钮: 地面站往 /flylab/vision_cmd 发 semantic_start / semantic_stop,
///      本节点据此启停
///   3. 画面叠加: 每帧把类别图转成 JSON 标注发往 /flylab/annotations
///      (algo 组名 "semantic", 可视化同深度估计: cityscapes 配色画中画
///      小窗贴右下角 + 类别图例文字在左下角)
///
/// 模型默认取 share/flylab_vision/models/semantic/yolo26s-sem.onnx
/// (CMakeLists 安装), Cityscapes 19 类语义分割, 输入尺寸自动取模型固定值
/// (1024), 都可用参数覆盖。
///
/// 运行:
///   ros2 run flylab_vision yolo_semantic_node
///   ros2 run flylab_vision yolo_semantic_node --ros-args
///       -p model:=/path/to/xx.onnx -p pip_scale:=0.33 -p device:=cuda
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

#include "yolo/yolo_semantic.hpp"

namespace flylab_vision
{

namespace
{
constexpr const char kCmdStart[] = "semantic_start";
constexpr const char kCmdStop[] = "semantic_stop";
constexpr const char kAlgoGroup[] = "semantic";

/// sensor_msgs/Image -> BGR cv::Mat (手动转换, 不依赖 cv_bridge); 失败返回空 Mat
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

/// 默认模型路径: ament share 目录下的 models/semantic/yolo26s-sem.onnx
std::string default_model_path()
{
  try {
    return ament_index_cpp::get_package_share_directory("flylab_vision") +
           "/models/semantic/yolo26s-sem.onnx";
  } catch (const std::exception &) {
    return "";
  }
}
}  // namespace

class YoloSemanticNode : public rclcpp::Node
{
public:
  YoloSemanticNode()
  : Node("yolo_semantic_node")
  {
    // ---------- 参数 ----------
    const auto model = declare_parameter<std::string>("model", default_model_path());
    // 0 = 自动取模型固定输入尺寸 (yolo26s-sem 导出为 1024)
    const auto input_size = declare_parameter<int>("input_size", 0);
    const auto device = declare_parameter<std::string>("device", "cpu");
    // 逗号分隔的类别名过滤器 (cityscapes 类别名), 空 = 全部 19 类
    const auto classes = declare_parameter<std::string>("classes", "");
    // 画中画窗宽占原帧比例 (0.1~1.0 自动夹取, 同深度估计 pip_scale)
    const auto pip_scale = declare_parameter<double>("pip_scale", 0.33);
    // 类别图中值滤波核 (奇数, 去椒盐噪声), 0/1 = 关闭
    const auto smooth = declare_parameter<int>("smooth", 5);
    const auto image_topic = declare_parameter<std::string>(
      "image_topic", "/flylab/display/image_raw");
    const auto cmd_topic = declare_parameter<std::string>("cmd_topic", "/flylab/vision_cmd");
    const auto anno_topic =
      declare_parameter<std::string>("annotation_topic", "/flylab/annotations");
    image_topic_ = image_topic;
    pip_scale_ = pip_scale;

    // ---------- 模型 (启动即加载, 失败抛异常直接终止暴露配置问题) ----------
    YoloSemanticConfig cfg;
    cfg.model_path = model;
    cfg.input_size = input_size;
    cfg.smooth_kernel = smooth;
    cfg.use_cuda = device == "cuda";
    if (!classes.empty()) {
      std::stringstream ss(classes);
      std::string name;
      while (std::getline(ss, name, ',')) {
        const auto & names = cityscapes_names();
        const auto it = std::find(names.begin(), names.end(), name);
        if (it == names.end()) {
          throw std::invalid_argument("未知类别名: " + name);
        }
        cfg.class_filter.push_back(static_cast<int>(it - names.begin()));
      }
    }
    segmenter_ = std::make_unique<YoloSemanticSegmenter>(cfg);
    RCLCPP_INFO(
      get_logger(),
      "模型加载完成: %s [%s, input %d, %zu 类]",
      model.c_str(), segmenter_->cuda_active() ? "cuda" : "cpu",
      segmenter_->input_size(),
      cfg.class_filter.empty() ? cityscapes_names().size() : cfg.class_filter.size());
    if (cfg.use_cuda && !segmenter_->cuda_active()) {
      RCLCPP_WARN(
        get_logger(), "CUDA EP 注册失败, 已回退 CPU: %s",
        segmenter_->cuda_error().c_str());
    }

    // ---------- 通信 ----------
    // 配色图 base64 消息体积大 (几十 KB), 队列限深防积压旧标注
    anno_pub_ = create_publisher<std_msgs::msg::String>(anno_topic, rclcpp::QoS(5));
    cmd_sub_ = create_subscription<std_msgs::msg::String>(
      cmd_topic, 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {on_cmd(*msg);});
    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {on_image(*msg);});

    RCLCPP_INFO(
      get_logger(),
      "YOLO 语义分割节点就绪: 订阅 %s, 等待 %s 指令",
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
  void publish_result(const YoloSemanticResult & result)
  {
    std_msgs::msg::String out;
    out.data = to_semantic_annotations_json(result, kAlgoGroup, pip_scale_);
    anno_pub_->publish(out);
  }

  void on_cmd(const std_msgs::msg::String & msg)
  {
    if (msg.data == kCmdStart && !enabled_) {
      enabled_ = true;
      RCLCPP_INFO(get_logger(), "语义分割已启动");
    } else if (msg.data == kCmdStop && enabled_) {
      enabled_ = false;
      publish_result(YoloSemanticResult{});   // 清自己的分组标注
      RCLCPP_INFO(get_logger(), "语义分割已停止");
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
    const YoloSemanticResult result = segmenter_->segment(bgr);
    if (!first_infer_done_) {
      first_infer_done_ = true;
      const auto ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0).count();
      RCLCPP_INFO(
        get_logger(), "首帧推理耗时 %ldms, 检出 %zu 类", ms, result.present.size());
    }
    rx_since_log_ += 1;
    publish_result(result);
  }

  std::unique_ptr<YoloSemanticSegmenter> segmenter_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr anno_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  std::string image_topic_;
  double pip_scale_ = 0.33;
  uint32_t rx_since_log_ = 0;   // 距上次统计周期收到的帧数
  bool enabled_ = false;
  bool got_first_frame_ = false;
  bool first_infer_done_ = false;
};

}  // namespace flylab_vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<flylab_vision::YoloSemanticNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
