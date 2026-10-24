// Copyright 2026 flylab
/// YOLO Pose (人体关键点) 估计 ROS2 节点 —— 接入地面站视觉工具体系 (基类 common/display_vision_node.hpp)。
///
/// 链路 (与 yolo_obb_node 相同, 话题名与 flylab_tools/config.py 保持一致, 均可用参数覆盖):
///   1. 图像输入: 订阅地面站"当前显示的帧" /flylab/display/image_raw
///   2. 工具按钮: /flylab/vision_cmd 收 pose_start / pose_stop 启停
///   3. 画面叠加: 检测结果转 JSON 标注发 /flylab/annotations
///      (组名 "pose", 骨架连线 + 可见关键点; 框仅用于内部 NMS 不绘制)
///
/// 模型默认取 share/flylab_vision/models/pose/yolo26s-pose.onnx
/// (CMakeLists 安装), 可用 model 参数覆盖; 默认输入边长 640。
///
/// 运行:
///   ros2 run flylab_vision yolo_pose_node
///   ros2 run flylab_vision yolo_pose_node --ros-args
///       -p model:=/path/to/xx.onnx -p conf:=0.35 -p device:=cuda
#include <memory>
#include <string>
#include <vector>

#include "common/display_vision_node.hpp"
#include "yolo/yolo_pose.hpp"

namespace flylab_vision
{

class YoloPoseNode final : public DisplayVisionNode
{
public:
  YoloPoseNode()
  : DisplayVisionNode("yolo_pose_node",
      {"pose_start", "pose_stop", "pose", "姿态估计"})
  {
    // ---------- 参数 ----------
    const auto model = declare_parameter<std::string>(
      "model", default_model_path("pose", "yolo26s-pose.onnx"));
    const auto input_size = declare_parameter<int>("input_size", 640);
    const auto conf = declare_parameter<double>("conf", 0.25);
    const auto iou = declare_parameter<double>("iou", 0.45);
    const auto kpt_conf = declare_parameter<double>("kpt_conf", 0.5);
    const auto device = declare_parameter<std::string>("device", "cpu");

    // ---------- 模型 (启动即加载, 失败抛异常直接终止暴露配置问题) ----------
    YoloPoseConfig cfg;
    cfg.model_path = model;
    cfg.input_size = input_size;
    cfg.conf_thres = static_cast<float>(conf);
    cfg.iou_thres = static_cast<float>(iou);
    cfg.kpt_conf = static_cast<float>(kpt_conf);
    cfg.use_cuda = device == "cuda";
    detector_ = std::make_unique<YoloPoseDetector>(cfg);
    RCLCPP_INFO(
      get_logger(),
      "模型加载完成: %s [%s, input %d, conf %.2f, iou %.2f, kpt_conf %.2f, %d kpts]",
      model.c_str(), detector_->cuda_active() ? "cuda" : "cpu",
      cfg.input_size, cfg.conf_thres, cfg.iou_thres, cfg.kpt_conf,
      detector_->kpt_count());
    if (cfg.use_cuda && !detector_->cuda_active()) {
      RCLCPP_WARN(
        get_logger(), "CUDA EP 注册失败, 已回退 CPU: %s",
        detector_->cuda_error().c_str());
    }

    log_ready("YOLO Pose");
  }

protected:
  std::string process(const cv::Mat & bgr, int /*frame_width*/) override
  {
    const auto results = detector_->detect(bgr);
    last_count_ = results.size();
    return to_annotations_json(results, algo_group());
  }

  std::string clear_json() const override
  {
    return to_annotations_json(std::vector<YoloPoseDetection>{}, algo_group());
  }

  void log_first_infer(long ms) override
  {
    RCLCPP_INFO(
      get_logger(), "首帧推理耗时 %ldms, 检出 %zu 个人体", ms, last_count_);
  }

private:
  std::unique_ptr<YoloPoseDetector> detector_;
  size_t last_count_ = 0;
};

}  // namespace flylab_vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<flylab_vision::YoloPoseNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
