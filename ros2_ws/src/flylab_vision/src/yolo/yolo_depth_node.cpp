// Copyright 2026 flylab
/// YOLO 单目深度估计 ROS2 节点 —— 接入地面站视觉工具体系 (flylab_tools 既有约定)。
///
/// 与 yolo_node.cpp / yolo_cls_node.cpp 同构, 区别在算法 (逐像素深度) 和
/// 标注形式: 稠密输出经 turbo 伪彩 + JPEG(base64) 以 image 图元画中画
/// 显示 (相机画面右下角不透明小窗, 不遮挡原画面), 另带左下角深度范围
/// 文字。链路 (话题名与 flylab_tools/config.py 保持一致, 均可用参数覆盖):
///   1. 图像输入: 订阅地面站"当前显示的帧" /flylab/display/image_raw
///   2. 工具按钮: 地面站往 /flylab/vision_cmd 发 depth_start / depth_stop,
///      本节点据此启停推理 (常驻运行, 不用每次重启节点)
///   3. 画面叠加: 每帧把伪彩深度图转成 JSON 标注发往 /flylab/annotations
///      (schema 见 flylab_tools/overlay.py, algo 组名 "yolo_depth",
///      与检测/分类分组独立, 可并存)
///
/// 模型默认取 share/flylab_vision/models/depth/yolo26s-depth.onnx
/// (CMakeLists 安装), 输入尺寸自动取模型固定值 (768), 都可用参数覆盖。
/// 注: Ultralytics depth 标准导出输出为米制绝对深度 (log 头 exp + 米制
/// 校准已烘焙进图内), 左下角深度范围可直接当米读; 绝对尺度按预训练
/// 验证集标定, 换相机/换场景存在全局偏差属正常。
///
/// 运行:
///   ros2 run flylab_vision yolo_depth_node
///   ros2 run flylab_vision yolo_depth_node --ros-args
///       -p model:=/path/to/xx.onnx -p pip_scale:=0.5 -p device:=cuda
#include <chrono>
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include <opencv2/imgproc.hpp>

#include "yolo/yolo_depth.hpp"

namespace flylab_vision
{

namespace
{
constexpr const char kCmdStart[] = "depth_start";
constexpr const char kCmdStop[] = "depth_stop";
constexpr const char kAlgoGroup[] = "yolo_depth";

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

/// 默认模型路径: ament share 目录下的 models/depth/yolo26s-depth.onnx
std::string default_model_path()
{
  try {
    return ament_index_cpp::get_package_share_directory("flylab_vision") +
           "/models/depth/yolo26s-depth.onnx";
  } catch (const std::exception &) {
    return "";
  }
}
}  // namespace

class YoloDepthNode : public rclcpp::Node
{
public:
  YoloDepthNode()
  : Node("yolo_depth_node")
  {
    // ---------- 参数 ----------
    const auto model = declare_parameter<std::string>("model", default_model_path());
    // 0 = 自动取模型固定输入尺寸 (yolo26s-depth-log 导出为 768)
    const auto input_size = declare_parameter<int>("input_size", 0);
    const auto device = declare_parameter<std::string>("device", "cpu");
    // 画中画小窗宽度占原帧比例
    const auto pip_scale = declare_parameter<double>("pip_scale", 0.33);
    const auto jpeg_quality = declare_parameter<int>("jpeg_quality", 70);
    const auto image_topic = declare_parameter<std::string>(
      "image_topic", "/flylab/display/image_raw");
    const auto cmd_topic = declare_parameter<std::string>("cmd_topic", "/flylab/vision_cmd");
    const auto anno_topic =
      declare_parameter<std::string>("annotation_topic", "/flylab/annotations");
    image_topic_ = image_topic;
    pip_scale_ = pip_scale;
    jpeg_quality_ = jpeg_quality;

    // ---------- 模型 (启动即加载, 失败抛异常直接终止暴露配置问题) ----------
    DepthConfig cfg;
    cfg.model_path = model;
    cfg.input_size = input_size;
    cfg.use_cuda = device == "cuda";
    estimator_ = std::make_unique<YoloDepthEstimator>(cfg);
    RCLCPP_INFO(
      get_logger(),
      "模型加载完成: %s [%s, input %d]",
      model.c_str(), estimator_->cuda_active() ? "cuda" : "cpu",
      estimator_->input_size());
    if (cfg.use_cuda && !estimator_->cuda_active()) {
      RCLCPP_WARN(
        get_logger(), "CUDA EP 注册失败, 已回退 CPU: %s",
        estimator_->cuda_error().c_str());
    }

    // ---------- 通信 ----------
    // 伪彩图 base64 消息体积大 (几十 KB), 队列限深防积压旧标注
    anno_pub_ = create_publisher<std_msgs::msg::String>(anno_topic, rclcpp::QoS(5));
    cmd_sub_ = create_subscription<std_msgs::msg::String>(
      cmd_topic, 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {on_cmd(*msg);});
    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {on_image(*msg);});

    RCLCPP_INFO(
      get_logger(),
      "YOLO 深度估计节点就绪: 订阅 %s, 等待 %s 指令",
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
  void publish_result(const DepthResult & result)
  {
    std_msgs::msg::String out;
    out.data = to_depth_annotations_json(
      result, kAlgoGroup, pip_scale_, jpeg_quality_);
    anno_pub_->publish(out);
  }

  void on_cmd(const std_msgs::msg::String & msg)
  {
    if (msg.data == kCmdStart && !enabled_) {
      enabled_ = true;
      RCLCPP_INFO(get_logger(), "深度估计已启动");
    } else if (msg.data == kCmdStop && enabled_) {
      enabled_ = false;
      publish_result(DepthResult{});   // 清自己的分组标注
      RCLCPP_INFO(get_logger(), "深度估计已停止");
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
    const DepthResult result = estimator_->estimate(bgr);
    if (!first_infer_done_) {
      first_infer_done_ = true;
      const auto ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0).count();
      RCLCPP_INFO(
        get_logger(), "首帧推理耗时 %ldms, 深度范围 %.2f ~ %.2f",
        ms, result.dmin, result.dmax);
    }
    rx_since_log_ += 1;
    publish_result(result);
  }

  std::unique_ptr<YoloDepthEstimator> estimator_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr anno_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  std::string image_topic_;
  double pip_scale_ = 0.33;
  int jpeg_quality_ = 70;
  uint32_t rx_since_log_ = 0;   // 距上次统计周期收到的帧数
  bool enabled_ = false;
  bool got_first_frame_ = false;
  bool first_infer_done_ = false;
};

}  // namespace flylab_vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<flylab_vision::YoloDepthNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
