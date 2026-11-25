// Copyright 2026 flylab
/// YOLO 图像分类 ROS2 节点 —— 接入地面站视觉工具体系 (基类 common/display_vision_node.hpp)。
///
/// 与 yolo_detector_node.cpp 同构, 区别只在算法 (整图分类 vs 目标检测) 和
/// 输出形式 (右栏面板表格 vs 目标框, 画面无叠加)。链路:
///   1. 图像输入: 订阅地面站"当前显示的帧" /flylab/display/image_raw
///   2. 工具按钮: /flylab/vision_cmd 收 cls_start / cls_stop 启停
///   3. 面板数据: {"src":"cls", ...} 发 /flylab/algo_info (~30Hz 限频,
///      top-k 结果按置信度降序进右栏"算法信息"表格, 画面不叠加标注)
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
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "common/display_vision_node.hpp"
#include "classify/yolo_classifier.hpp"

namespace flylab_vision
{

class YoloClsNode final : public DisplayVisionNode
{
public:
  YoloClsNode()
  : DisplayVisionNode("yolo_cls_node",
      {"cls_start", "cls_stop", "yolo_cls", "分类"})
  {
    // ---------- 参数 ----------
    const auto model = declare_parameter<std::string>(
      "model", default_model_path("classify", "yolo26s-cls.onnx"));
    // 0 = 自动取模型固定输入尺寸 (yolo26s-cls 导出为 224)
    const auto input_size = declare_parameter<int>("input_size", 0);
    const auto topk = declare_parameter<int>("topk", 3);
    const auto device = declare_parameter<std::string>("device", "cpu");
    const auto labels = declare_parameter<std::string>("labels", "");
    const auto info_topic =
      declare_parameter<std::string>("info_topic", "/flylab/algo_info");

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

    log_ready("YOLO 分类");

    // ---------- 通信 (基类之外: 右栏算法信息面板数据) ----------
    info_pub_ = create_publisher<std_msgs::msg::String>(info_topic, 10);
  }

protected:
  /// 分类结果只进右栏面板, 画面不叠加 (返回空串 = 本帧不发标注)
  std::string process(const cv::Mat & bgr, int /*frame_width*/) override
  {
    const auto preds = classifier_->classify(bgr);
    last_name_ = preds.empty() ? "-" : preds.front().name;
    last_conf_ = preds.empty() ? 0.f : preds.front().conf;
    publish_info(preds);
    return "";
  }

  std::string clear_json() const override
  {
    // 从未发过画面标注, 无需清屏
    return "";
  }

  void log_first_infer(long ms) override
  {
    RCLCPP_INFO(
      get_logger(), "首帧推理耗时 %ldms, top1: %s %.2f",
      ms, last_name_.c_str(), last_conf_);
  }

private:
  /// 面板数据 {"src":"cls", "items":[...]}: ~30Hz 限频 (对齐地面站 33ms
  /// 轮询, 值不变时面板端自动去重)。items = 按置信度降序的 top-k (面板
  /// 表格取前 3 列), 每项: 类别名 + softmax 置信度
  void publish_info(const std::vector<ClsPrediction> & preds)
  {
    const auto now = std::chrono::steady_clock::now();
    if (now - info_last_ < std::chrono::milliseconds(33)) {
      return;
    }
    info_last_ = now;
    const size_t n = std::min<size_t>(preds.size(), 3);
    std::ostringstream os;
    os << "{\"src\":\"cls\",\"items\":[";
    for (size_t i = 0; i < n; ++i) {
      if (i > 0) {
        os << ',';
      }
      os << "{\"cls\":\"" << preds[i].name << "\","
         << "\"conf\":" << num(preds[i].conf, 2) << "}";
    }
    os << "]}";
    std_msgs::msg::String out;
    out.data = os.str();
    info_pub_->publish(out);
  }

  static std::string num(double v, int prec = 1)
  {
    std::ostringstream os;
    os << std::fixed << std::setprecision(prec) << v;
    return os.str();
  }

  std::unique_ptr<YoloClassifier> classifier_;
  std::string last_name_;
  float last_conf_ = 0.f;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr info_pub_;
  std::chrono::steady_clock::time_point info_last_;
};

}  // namespace flylab_vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<flylab_vision::YoloClsNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
