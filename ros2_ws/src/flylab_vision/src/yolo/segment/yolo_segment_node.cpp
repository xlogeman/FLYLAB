// Copyright 2026 flylab
/// YOLO 实例分割 ROS2 节点 —— 接入地面站视觉工具体系 (基类 common/display_vision_node.hpp)。
///
/// 链路 (与 yolo_node 相同, 话题名与 flylab_tools/config.py 保持一致, 均可用参数覆盖):
///   1. 图像输入: 订阅地面站"当前显示的帧" /flylab/display/image_raw
///   2. 工具按钮: /flylab/vision_cmd 收 seg_start / seg_stop 启停
///   3. 画面叠加: 分割结果转 JSON 标注发 /flylab/annotations
///      (组名 "seg", 掩码轮廓多边形 + 检测框 + 类别置信度文字)
///
/// 模型默认取 share/flylab_vision/models/segment/yolo26s-seg.onnx
/// (CMakeLists 安装), 可用 model 参数覆盖; 默认输入边长 640。
///
/// 运行:
///   ros2 run flylab_vision yolo_segment_node
///   ros2 run flylab_vision yolo_segment_node --ros-args
///       -p model:=/path/to/xx.onnx -p conf:=0.35 -p device:=cuda
#include <algorithm>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "common/display_vision_node.hpp"
#include "yolo/yolo_detector.hpp"
#include "yolo/yolo_segment.hpp"

namespace flylab_vision
{

class YoloSegmentNode final : public DisplayVisionNode
{
public:
  YoloSegmentNode()
  : DisplayVisionNode("yolo_segment_node",
      {"seg_start", "seg_stop", "seg", "实例分割"})
  {
    // ---------- 参数 ----------
    const auto model = declare_parameter<std::string>(
      "model", default_model_path("segment", "yolo26s-seg.onnx"));
    const auto input_size = declare_parameter<int>("input_size", 640);
    const auto conf = declare_parameter<double>("conf", 0.25);
    const auto iou = declare_parameter<double>("iou", 0.45);
    const auto mask_thres = declare_parameter<double>("mask_thres", 0.5);
    const auto device = declare_parameter<std::string>("device", "cpu");
    // 逗号分隔的类别名过滤器, 空 = 全部 80 类
    const auto classes = declare_parameter<std::string>("classes", "");

    // ---------- 模型 (启动即加载, 失败抛异常直接终止暴露配置问题) ----------
    YoloSegmentConfig cfg;
    cfg.model_path = model;
    cfg.input_size = input_size;
    cfg.conf_thres = static_cast<float>(conf);
    cfg.iou_thres = static_cast<float>(iou);
    cfg.mask_thres = static_cast<float>(mask_thres);
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
    detector_ = std::make_unique<YoloSegmentDetector>(cfg);
    RCLCPP_INFO(
      get_logger(),
      "模型加载完成: %s [%s, input %d, conf %.2f, iou %.2f, mask %.2f]",
      model.c_str(), detector_->cuda_active() ? "cuda" : "cpu",
      cfg.input_size, cfg.conf_thres, cfg.iou_thres, cfg.mask_thres);
    if (cfg.use_cuda && !detector_->cuda_active()) {
      RCLCPP_WARN(
        get_logger(), "CUDA EP 注册失败, 已回退 CPU: %s",
        detector_->cuda_error().c_str());
    }

    log_ready("YOLO 实例分割");
  }

protected:
  std::string process(const cv::Mat & bgr, int /*frame_width*/) override
  {
    const auto results = detector_->segment(bgr);
    last_count_ = results.size();
    return to_annotations_json(results, algo_group());
  }

  std::string clear_json() const override
  {
    return to_annotations_json(
      std::vector<YoloSegmentDetection>{}, algo_group());
  }

  void log_first_infer(long ms) override
  {
    RCLCPP_INFO(
      get_logger(), "首帧推理耗时 %ldms, 检出 %zu 个实例", ms, last_count_);
  }

private:
  std::unique_ptr<YoloSegmentDetector> detector_;
  size_t last_count_ = 0;
};

}  // namespace flylab_vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<flylab_vision::YoloSegmentNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
