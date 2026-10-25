// Copyright 2026 flylab
/// YOLO 单目深度估计 ROS2 节点 —— 接入地面站视觉工具体系 (基类 common/display_vision_node.hpp)。
///
/// 与 yolo_node.cpp 同构, 区别在算法 (逐像素深度) 和标注形式: 稠密输出经
/// turbo 伪彩 + JPEG(base64) 以 image 图元画中画显示 (相机画面右下角不透明
/// 小窗, 不遮挡原画面), 另带左下角深度范围文字。链路:
///   1. 图像输入: 订阅地面站"当前显示的帧" /flylab/display/image_raw
///   2. 工具按钮: /flylab/vision_cmd 收 depth_start / depth_stop 启停
///   3. 画面叠加: 伪彩深度图转 JSON 标注发 /flylab/annotations
///      (组名 "yolo_depth", 与检测/分类分组独立, 可并存)
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
#include <memory>
#include <string>

#include "common/display_vision_node.hpp"
#include "yolo/yolo_depth.hpp"

namespace flylab_vision
{

class YoloDepthNode final : public DisplayVisionNode
{
public:
  YoloDepthNode()
  // 伪彩图 base64 体积大, large_payload 让标注发布 QoS 限深防积压旧标注
  : DisplayVisionNode("yolo_depth_node",
      {"depth_start", "depth_stop", "yolo_depth", "深度估计", true})
  {
    // ---------- 参数 ----------
    const auto model = declare_parameter<std::string>(
      "model", default_model_path("depth", "yolo26s-depth.onnx"));
    // 0 = 自动取模型固定输入尺寸 (yolo26s-depth 导出为 768)
    const auto input_size = declare_parameter<int>("input_size", 0);
    const auto device = declare_parameter<std::string>("device", "cpu");
    // 画中画小窗宽度占原帧比例
    const auto pip_scale = declare_parameter<double>("pip_scale", 0.33);
    const auto jpeg_quality = declare_parameter<int>("jpeg_quality", 70);
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

    log_ready("YOLO 深度估计");
  }

protected:
  std::string process(const cv::Mat & bgr, int /*frame_width*/) override
  {
    const DepthResult result = estimator_->estimate(bgr);
    last_dmin_ = result.dmin;
    last_dmax_ = result.dmax;
    return to_depth_annotations_json(
      result, algo_group(), pip_scale_, jpeg_quality_);
  }

  std::string clear_json() const override
  {
    return to_depth_annotations_json(
      DepthResult{}, algo_group(), pip_scale_, jpeg_quality_);
  }

  void log_first_infer(long ms) override
  {
    RCLCPP_INFO(
      get_logger(), "首帧推理耗时 %ldms, 深度范围 %.2f ~ %.2f",
      ms, last_dmin_, last_dmax_);
  }

private:
  std::unique_ptr<YoloDepthEstimator> estimator_;
  double pip_scale_ = 0.33;
  int jpeg_quality_ = 70;
  float last_dmin_ = 0.f;
  float last_dmax_ = 0.f;
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
