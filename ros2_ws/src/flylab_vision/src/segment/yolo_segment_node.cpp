// Copyright 2026 flylab
/// YOLO 实例分割 ROS2 节点 —— 接入地面站视觉工具体系 (基类 common/display_vision_node.hpp)。
///
/// 链路 (与 yolo_node 相同, 话题名与 flylab_gcs/config.py 保持一致, 均可用参数覆盖):
///   1. 图像输入: 订阅地面站"当前显示的帧" /flylab/display/image_raw
///   2. 工具按钮: /flylab/vision_cmd 收 seg_start / seg_stop 启停
///   3. 画面叠加: 分割结果转 JSON 标注发 /flylab/annotations
///      (组名 "seg", 掩码轮廓多边形 + 检测框 + 类别置信度文字)
///   4. 面板数据: {"src":"seg", ...} 发 /flylab/algo_info (~30Hz 限频,
///      摘要 = 实例数 + 主目标 [最高置信度] 的类别/位置/尺寸/掩码占比)
///
/// 模型默认取 share/flylab_vision/models/segment/yolo26s-seg.onnx
/// (CMakeLists 安装), 可用 model 参数覆盖; 默认输入边长 640。
///
/// 运行:
///   ros2 run flylab_vision yolo_segment_node
///   ros2 run flylab_vision yolo_segment_node --ros-args
///       -p model:=/path/to/xx.onnx -p conf:=0.35 -p device:=cuda
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "common/display_vision_node.hpp"
#include "detect/yolo_detector.hpp"
#include "segment/yolo_segment.hpp"

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
    const auto info_topic =
      declare_parameter<std::string>("info_topic", "/flylab/algo_info");

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

    // ---------- 通信 (基类之外: 右栏算法信息面板数据) ----------
    info_pub_ = create_publisher<std_msgs::msg::String>(info_topic, 10);
  }

protected:
  std::string process(const cv::Mat & bgr, int /*frame_width*/) override
  {
    const auto results = detector_->segment(bgr);
    last_count_ = results.size();
    publish_info(results, static_cast<double>(bgr.cols) * bgr.rows);
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
  /// 面板数据 {"src":"seg", "items":[...]}: ~30Hz 限频 (对齐地面站 33ms
  /// 轮询, 值不变时面板端自动去重)。items = 按置信度降序的前 3 实例
  /// (右栏表格 3 列), 每项: 类别/置信度/框中心/框尺寸/掩码占比
  /// (占比 = 掩码像素数 / 画幅像素, 体现实例在画面中的大小)
  void publish_info(
    const std::vector<YoloSegmentDetection> & results, double frame_area)
  {
    const auto now = std::chrono::steady_clock::now();
    if (now - info_last_ < std::chrono::milliseconds(33)) {
      return;
    }
    info_last_ = now;
    std::vector<const YoloSegmentDetection *> sorted;
    sorted.reserve(results.size());
    for (const auto & r : results) {
      sorted.push_back(&r);
    }
    std::sort(sorted.begin(), sorted.end(),
      [](const YoloSegmentDetection * a, const YoloSegmentDetection * b) {
        return a->conf > b->conf;
      });
    const size_t n = std::min<size_t>(sorted.size(), 3);
    std::ostringstream os;
    // count = 全部检出数 (可能 > 3, 不能用 len(items) 代替, 面板汇总行用)
    os << "{\"src\":\"seg\",\"count\":" << results.size() << ",\"items\":[";
    for (size_t i = 0; i < n; ++i) {
      const auto & d = *sorted[i];
      float area_pct = 0.f;
      if (frame_area > 0 && !d.mask.empty()) {
        area_pct = static_cast<float>(
          100.0 * cv::countNonZero(d.mask) / frame_area);
      }
      if (i > 0) {
        os << ',';
      }
      os << "{\"cls\":\"" << d.name << "\","
         << "\"conf\":" << num(d.conf, 2) << ","
         << "\"x\":" << num(d.box.x + d.box.width / 2.f, 0) << ","
         << "\"y\":" << num(d.box.y + d.box.height / 2.f, 0) << ","
         << "\"w\":" << num(d.box.width, 0) << ","
         << "\"h\":" << num(d.box.height, 0) << ","
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

  std::unique_ptr<YoloSegmentDetector> detector_;
  size_t last_count_ = 0;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr info_pub_;
  std::chrono::steady_clock::time_point info_last_;
};

}  // namespace flylab_vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<flylab_vision::YoloSegmentNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
