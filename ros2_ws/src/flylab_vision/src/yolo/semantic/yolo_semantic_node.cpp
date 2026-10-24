// Copyright 2026 flylab
/// YOLO 语义分割 ROS2 节点 —— 接入地面站视觉工具体系 (基类 common/display_vision_node.hpp)。
///
/// 链路 (与 yolo_segment_node 相同, 话题名与 flylab_tools/config.py 保持一致,
/// 均可用参数覆盖):
///   1. 图像输入: 订阅地面站"当前显示的帧" /flylab/display/image_raw
///   2. 工具按钮: /flylab/vision_cmd 收 semantic_start / semantic_stop 启停
///   3. 画面叠加: 类别图转 JSON 标注发 /flylab/annotations
///      (组名 "semantic", 可视化同深度估计: cityscapes 配色画中画
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
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "common/display_vision_node.hpp"
#include "yolo/yolo_semantic.hpp"

namespace flylab_vision
{

class YoloSemanticNode final : public DisplayVisionNode
{
public:
  YoloSemanticNode()
  // 配色图 base64 体积大, large_payload 让标注发布 QoS 限深防积压旧标注
  : DisplayVisionNode("yolo_semantic_node",
      {"semantic_start", "semantic_stop", "semantic", "语义分割", true})
  {
    // ---------- 参数 ----------
    const auto model = declare_parameter<std::string>(
      "model", default_model_path("semantic", "yolo26s-sem.onnx"));
    // 0 = 自动取模型固定输入尺寸 (yolo26s-sem 导出为 1024)
    const auto input_size = declare_parameter<int>("input_size", 0);
    const auto device = declare_parameter<std::string>("device", "cpu");
    // 逗号分隔的类别名过滤器 (cityscapes 类别名), 空 = 全部 19 类
    const auto classes = declare_parameter<std::string>("classes", "");
    // 画中画窗宽占原帧比例 (0.1~1.0 自动夹取, 同深度估计 pip_scale)
    const auto pip_scale = declare_parameter<double>("pip_scale", 0.33);
    // 类别图中值滤波核 (奇数, 去椒盐噪声), 0/1 = 关闭
    const auto smooth = declare_parameter<int>("smooth", 5);
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

    log_ready("YOLO 语义分割");
  }

protected:
  std::string process(const cv::Mat & bgr, int /*frame_width*/) override
  {
    const YoloSemanticResult result = segmenter_->segment(bgr);
    last_classes_ = result.present.size();
    return to_semantic_annotations_json(result, algo_group(), pip_scale_);
  }

  std::string clear_json() const override
  {
    return to_semantic_annotations_json(
      YoloSemanticResult{}, algo_group(), pip_scale_);
  }

  void log_first_infer(long ms) override
  {
    RCLCPP_INFO(
      get_logger(), "首帧推理耗时 %ldms, 检出 %zu 类", ms, last_classes_);
  }

private:
  std::unique_ptr<YoloSemanticSegmenter> segmenter_;
  double pip_scale_ = 0.33;
  size_t last_classes_ = 0;
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
