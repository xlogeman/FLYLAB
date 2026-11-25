// Copyright 2026 flylab
/// YOLO OBB (旋转框) 检测 ROS2 节点 —— 接入地面站视觉工具体系 (基类 common/display_vision_node.hpp)。
///
/// 链路 (与 yolo_detector_node 相同, 话题名与 flylab_gcs/config.py 保持一致, 均可用参数覆盖):
///   1. 图像输入: 订阅地面站"当前显示的帧" /flylab/display/image_raw
///   2. 工具按钮: /flylab/vision_cmd 收 obb_start / obb_stop 启停
///   3. 画面叠加: 检测结果转 JSON 标注发 /flylab/annotations
///      (组名 "obb", 4 角点闭合多边形 + 类别置信度文字)
///   4. 面板数据: {"src":"obb", ...} 发 /flylab/algo_info (~30Hz 限频,
///      摘要 = 检出数 + 前 3 旋转框 [按置信度降序] 的类别/位置/尺寸/角度)
///
/// 模型默认取 share/flylab_vision/models/obb/yolo26s-obb.onnx
/// (CMakeLists 安装), 可用 model 参数覆盖; 默认输入边长 1024。
///
/// 运行:
///   ros2 run flylab_vision yolo_obb_node
///   ros2 run flylab_vision yolo_obb_node --ros-args
///       -p model:=/path/to/xx.onnx -p conf:=0.35 -p device:=cuda
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "common/display_vision_node.hpp"
#include "obb/yolo_obb.hpp"

namespace flylab_vision
{

class YoloObbNode final : public DisplayVisionNode
{
public:
  YoloObbNode()
  : DisplayVisionNode("yolo_obb_node",
      {"obb_start", "obb_stop", "obb", "OBB 检测"})
  {
    // ---------- 参数 ----------
    const auto model = declare_parameter<std::string>(
      "model", default_model_path("obb", "yolo26s-obb.onnx"));
    const auto input_size = declare_parameter<int>("input_size", 1024);
    const auto conf = declare_parameter<double>("conf", 0.25);
    const auto iou = declare_parameter<double>("iou", 0.45);
    const auto device = declare_parameter<std::string>("device", "cpu");
    // 逗号分隔的类别名过滤器, 空 = 全部 15 类
    const auto classes = declare_parameter<std::string>("classes", "");
    const auto info_topic =
      declare_parameter<std::string>("info_topic", "/flylab/algo_info");

    // ---------- 模型 (启动即加载, 失败抛异常直接终止暴露配置问题) ----------
    YoloObbConfig cfg;
    cfg.model_path = model;
    cfg.input_size = input_size;
    cfg.conf_thres = static_cast<float>(conf);
    cfg.iou_thres = static_cast<float>(iou);
    cfg.use_cuda = device == "cuda";
    if (!classes.empty()) {
      std::stringstream ss(classes);
      std::string name;
      while (std::getline(ss, name, ',')) {
        const auto & names = dota_names();
        const auto it = std::find(names.begin(), names.end(), name);
        if (it == names.end()) {
          throw std::invalid_argument("未知类别名: " + name);
        }
        cfg.class_filter.push_back(static_cast<int>(it - names.begin()));
      }
    }
    detector_ = std::make_unique<YoloObbDetector>(cfg);
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

    log_ready("YOLO OBB");

    // ---------- 通信 (基类之外: 右栏算法信息面板数据) ----------
    info_pub_ = create_publisher<std_msgs::msg::String>(info_topic, 10);
  }

protected:
  std::string process(const cv::Mat & bgr, int /*frame_width*/) override
  {
    const auto results = detector_->detect(bgr);
    last_count_ = results.size();
    publish_info(results, static_cast<double>(bgr.cols) * bgr.rows);
    return to_annotations_json(results, algo_group());
  }

  std::string clear_json() const override
  {
    return to_annotations_json(std::vector<YoloObbDetection>{}, algo_group());
  }

  void log_first_infer(long ms) override
  {
    RCLCPP_INFO(
      get_logger(), "首帧推理耗时 %ldms, 检出 %zu 个目标", ms, last_count_);
  }

private:
  /// 面板数据 {"src":"obb", "items":[...]}: ~30Hz 限频 (对齐地面站 33ms
  /// 轮询, 值不变时面板端自动去重)。items = 按置信度降序的前 3 旋转框
  /// (右栏表格 3 列), 每项: 类别/置信度/框中心/宽高/旋转角
  /// (RotatedRect::angle, 度; 面积恒为 w*h 与角度无关, 面板不单独显示占比)
  void publish_info(
    const std::vector<YoloObbDetection> & results, double /*frame_area*/)
  {
    const auto now = std::chrono::steady_clock::now();
    if (now - info_last_ < std::chrono::milliseconds(33)) {
      return;
    }
    info_last_ = now;
    std::vector<const YoloObbDetection *> sorted;
    sorted.reserve(results.size());
    for (const auto & r : results) {
      sorted.push_back(&r);
    }
    std::sort(sorted.begin(), sorted.end(),
      [](const YoloObbDetection * a, const YoloObbDetection * b) {
        return a->conf > b->conf;
      });
    const size_t n = std::min<size_t>(sorted.size(), 3);
    std::ostringstream os;
    // count = 全部检出数 (可能 > 3, 不能用 len(items) 代替, 面板汇总行用)
    os << "{\"src\":\"obb\",\"count\":" << results.size() << ",\"items\":[";
    for (size_t i = 0; i < n; ++i) {
      const auto & d = *sorted[i];
      if (i > 0) {
        os << ',';
      }
      os << "{\"cls\":\"" << d.name << "\","
         << "\"conf\":" << num(d.conf, 2) << ","
         << "\"x\":" << num(d.box.center.x, 0) << ","
         << "\"y\":" << num(d.box.center.y, 0) << ","
         << "\"w\":" << num(d.box.size.width, 0) << ","
         << "\"h\":" << num(d.box.size.height, 0) << ","
         << "\"angle\":" << num(d.box.angle, 1) << "}";
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

  std::unique_ptr<YoloObbDetector> detector_;
  size_t last_count_ = 0;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr info_pub_;
  std::chrono::steady_clock::time_point info_last_;
};

}  // namespace flylab_vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<flylab_vision::YoloObbNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
