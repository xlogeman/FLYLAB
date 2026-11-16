// Copyright 2026 flylab
/// SCRFD 人脸检测 ROS2 节点 —— 接入地面站视觉工具体系 (基类 common/display_vision_node.hpp)。
///
/// 链路 (话题名与 flylab_gcs/config.py 保持一致, 均可用参数覆盖):
///   1. 图像输入: 订阅地面站"当前显示的帧" /flylab/display/image_raw
///   2. 工具按钮: /flylab/vision_cmd 收 face_start / face_stop 启停
///   3. 画面叠加: 人脸框 + 5 关键点转 JSON 标注发 /flylab/annotations
///      (组名 "face")
///   4. 面板数据: {"src":"face", ...} 发 /flylab/aruco_info (~30Hz 限频,
///      摘要 = 人脸数 + 前 3 张脸 [按置信度降序] 的置信度/位置/尺寸/框占比)
///
/// 模型默认取 share/flylab_vision/models/face/scrfd_10g_bnkps.onnx
/// (CMakeLists 安装), 可用 model 参数覆盖。
///
/// 运行:
///   ros2 run flylab_vision face_detect_node
///   ros2 run flylab_vision face_detect_node --ros-args
///       -p model:=/path/to/xx.onnx -p conf:=0.5 -p device:=cuda
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "common/display_vision_node.hpp"
#include "face/scrfd_detector.hpp"

namespace flylab_vision
{

class FaceDetectNode final : public DisplayVisionNode
{
public:
  FaceDetectNode()
  : DisplayVisionNode("face_detect_node",
      {"face_start", "face_stop", "face", "人脸检测"})
  {
    // ---------- 参数 ----------
    const auto model = declare_parameter<std::string>(
      "model", default_model_path("face", "scrfd_10g_bnkps.onnx"));
    const auto input_size = declare_parameter<int>("input_size", 640);
    const auto conf = declare_parameter<double>("conf", 0.5);
    const auto iou = declare_parameter<double>("iou", 0.4);
    const auto device = declare_parameter<std::string>("device", "cpu");
    const auto info_topic =
      declare_parameter<std::string>("info_topic", "/flylab/aruco_info");

    // ---------- 模型 (启动即加载, 失败抛异常直接终止暴露配置问题) ----------
    ScrfdConfig cfg;
    cfg.model_path = model;
    cfg.input_size = input_size;
    cfg.conf_thres = static_cast<float>(conf);
    cfg.iou_thres = static_cast<float>(iou);
    cfg.use_cuda = device == "cuda";
    detector_ = std::make_unique<ScrfdDetector>(cfg);
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

    log_ready("SCRFD 人脸检测");

    // ---------- 通信 (基类之外: 右栏算法信息面板数据) ----------
    info_pub_ = create_publisher<std_msgs::msg::String>(info_topic, 10);
  }

protected:
  std::string process(const cv::Mat & bgr, int /*frame_width*/) override
  {
    const auto faces = detector_->detect(bgr);
    last_count_ = faces.size();
    publish_info(faces, static_cast<double>(bgr.cols) * bgr.rows);
    return to_annotations_json(faces, algo_group());
  }

  std::string clear_json() const override
  {
    return to_annotations_json(std::vector<ScrfdFace>{}, algo_group());
  }

  void log_first_infer(long ms) override
  {
    RCLCPP_INFO(
      get_logger(), "首帧推理耗时 %ldms, 检出 %zu 张人脸", ms, last_count_);
  }

private:
  /// 面板数据 {"src":"face", "items":[...]}: ~30Hz 限频 (对齐地面站 33ms
  /// 轮询, 值不变时面板端自动去重)。items = 按置信度降序的前 3 张脸
  /// (右栏表格 3 列), 每项: 置信度/框中心/框尺寸/框占比
  /// (占比 = 框面积 / 画幅像素, 体现目标在画面中的大小)
  void publish_info(
    const std::vector<ScrfdFace> & faces, double frame_area)
  {
    const auto now = std::chrono::steady_clock::now();
    if (now - info_last_ < std::chrono::milliseconds(33)) {
      return;
    }
    info_last_ = now;
    std::vector<const ScrfdFace *> sorted;
    sorted.reserve(faces.size());
    for (const auto & f : faces) {
      sorted.push_back(&f);
    }
    std::sort(sorted.begin(), sorted.end(),
      [](const ScrfdFace * a, const ScrfdFace * b) {
        return a->conf > b->conf;
      });
    const size_t n = std::min<size_t>(sorted.size(), 3);
    std::ostringstream os;
    // count = 全部检出数 (可能 > 3, 不能用 len(items) 代替, 面板汇总行用)
    os << "{\"src\":\"face\",\"count\":" << faces.size() << ",\"items\":[";
    for (size_t i = 0; i < n; ++i) {
      const auto & f = *sorted[i];
      float area_pct = 0.f;
      if (frame_area > 0) {
        area_pct = static_cast<float>(100.0 * f.box.area() / frame_area);
      }
      if (i > 0) {
        os << ',';
      }
      os << "{\"conf\":" << num(f.conf, 2) << ","
         << "\"x\":" << num(f.box.x + f.box.width / 2.f, 0) << ","
         << "\"y\":" << num(f.box.y + f.box.height / 2.f, 0) << ","
         << "\"w\":" << num(f.box.width, 0) << ","
         << "\"h\":" << num(f.box.height, 0) << ","
         << "\"area\":" << num(area_pct, 1) << "}";
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

  std::unique_ptr<ScrfdDetector> detector_;
  size_t last_count_ = 0;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr info_pub_;
  std::chrono::steady_clock::time_point info_last_;
};

}  // namespace flylab_vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<flylab_vision::FaceDetectNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
