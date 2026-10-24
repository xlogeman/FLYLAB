// Copyright 2026 flylab
/// YOLO 图像分类 ROS2 节点 —— 接入地面站视觉工具体系 (基类 common/display_vision_node.hpp)。
///
/// 与 yolo_node.cpp 同构, 区别只在算法 (整图分类 vs 目标检测) 和标注形式
/// (左上角 top-k 文字行 vs 目标框)。链路:
///   1. 图像输入: 订阅地面站"当前显示的帧" /flylab/display/image_raw
///   2. 工具按钮: /flylab/vision_cmd 收 cls_start / cls_stop 启停
///   3. 画面叠加: top-k 结果转 JSON 标注发 /flylab/annotations
///      (组名 "yolo_cls", 与检测 "yolo" 分组独立, 可并存)
///
/// 模型默认取 share/flylab_vision/models/classify/yolo26s-cls.onnx
/// (CMakeLists 安装), 输入尺寸自动取模型固定值, 标签自动取模型内嵌
/// metadata, 都可用参数覆盖。
///
/// 运行:
///   ros2 run flylab_vision yolo_cls_node
///   ros2 run flylab_vision yolo_cls_node --ros-args
///       -p model:=/path/to/xx.onnx -p topk:=3 -p device:=cuda
#include <memory>
#include <string>
#include <vector>

#include "common/display_vision_node.hpp"
#include "yolo/yolo_classifier.hpp"

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
  }

protected:
  std::string process(const cv::Mat & bgr, int frame_width) override
  {
    const auto preds = classifier_->classify(bgr);
    last_name_ = preds.empty() ? "-" : preds.front().name;
    last_conf_ = preds.empty() ? 0.f : preds.front().conf;
    return to_cls_annotations_json(preds, algo_group(), frame_width);
  }

  std::string clear_json() const override
  {
    return to_cls_annotations_json(
      std::vector<ClsPrediction>{}, algo_group(), 0);
  }

  void log_first_infer(long ms) override
  {
    RCLCPP_INFO(
      get_logger(), "首帧推理耗时 %ldms, top1: %s %.2f",
      ms, last_name_.c_str(), last_conf_);
  }

private:
  std::unique_ptr<YoloClassifier> classifier_;
  std::string last_name_;
  float last_conf_ = 0.f;
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
