// Copyright 2026 flylab
/// YOLO Pose (人体关键点) 估计 ROS2 节点 —— 接入地面站视觉工具体系 (基类 common/display_vision_node.hpp)。
///
/// 链路 (与 yolo_obb_node 相同, 话题名与 flylab_gcs/config.py 保持一致, 均可用参数覆盖):
///   1. 图像输入: 订阅地面站"当前显示的帧" /flylab/display/image_raw
///   2. 工具按钮: /flylab/vision_cmd 收 pose_start / pose_stop 启停
///   3. 画面叠加: 检测结果转 JSON 标注发 /flylab/annotations
///      (组名 "pose", 骨架连线 + 可见关键点; 框仅用于内部 NMS 不绘制)
///   4. 面板数据: {"src":"pose", "items":[...]} 发 /flylab/algo_info
///      (~30Hz 限频, 按置信度降序前 3 人体: 置信度/位置/尺寸/可见点)
///
/// 模型默认取 share/flylab_vision/models/pose/yolo26s-pose.onnx
/// (CMakeLists 安装), 可用 model 参数覆盖; 默认输入边长 640。
///
/// 运行:
///   ros2 run flylab_vision yolo_pose_node
///   ros2 run flylab_vision yolo_pose_node --ros-args
///       -p model:=/path/to/xx.onnx -p conf:=0.35 -p device:=cuda
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "common/display_vision_node.hpp"
#include "common/gcs_topics.hpp"
#include "pose/yolo_pose.hpp"

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
    const auto info_topic =
      declare_parameter<std::string>("info_topic", topics::ALGO_INFO_TOPIC);

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

    // ---------- 通信 (基类之外: 右栏算法信息面板数据) ----------
    kpt_conf_ = static_cast<float>(kpt_conf);
    info_pub_ = create_publisher<std_msgs::msg::String>(info_topic, 10);
  }

protected:
  std::string process(const cv::Mat & bgr, int /*frame_width*/) override
  {
    const auto results = detector_->detect(bgr);
    last_count_ = results.size();
    // 置信度降序 (画面编号与面板列序共用同一排序): 所有人都按排名标号
    // (1 = 置信度最高), 前 3 进面板表格
    std::vector<std::pair<const YoloPoseDetection *, size_t>> ranked;
    ranked.reserve(results.size());
    for (size_t i = 0; i < results.size(); ++i) {
      ranked.emplace_back(&results[i], i);
    }
    std::sort(ranked.begin(), ranked.end(),
      [](const auto & a, const auto & b) {return a.first->conf > b.first->conf;});
    std::vector<int> ranks(results.size(), 0);
    for (size_t i = 0; i < ranked.size(); ++i) {
      ranks[ranked[i].second] = static_cast<int>(i) + 1;
    }
    publish_info(ranked);
    return to_annotations_json(results, algo_group(), ranks);
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
  /// 面板数据 {"src":"pose", "items":[...]}: ~30Hz 限频 (对齐地面站 33ms
  /// 轮询, 值不变时面板端自动去重)。items = 画面编号序 (置信度降序) 的
  /// 前 3 人体, 每项: 置信度/框中心/框尺寸/可见点 (可见点按 kpt_conf
  /// 阈值计数, 与画面绘制同阈值); 编号 1 起始与画面标注对应
  void publish_info(
    const std::vector<std::pair<const YoloPoseDetection *, size_t>> & ranked)
  {
    const auto now = std::chrono::steady_clock::now();
    if (now - info_last_ < std::chrono::milliseconds(33)) {
      return;
    }
    info_last_ = now;
    const size_t n = std::min<size_t>(ranked.size(), 3);
    std::ostringstream os;
    os << "{\"src\":\"pose\",\"count\":" << ranked.size() << ",\"items\":[";
    for (size_t i = 0; i < n; ++i) {
      const auto & d = *ranked[i].first;
      int visible = 0;
      for (const float c : d.kpt_conf) {
        if (c >= kpt_conf_) {
          ++visible;
        }
      }
      if (i > 0) {
        os << ',';
      }
      os << "{\"conf\":" << num(d.conf, 2) << ","
         << "\"x\":" << num(d.box.x + d.box.width / 2.f, 0) << ","
         << "\"y\":" << num(d.box.y + d.box.height / 2.f, 0) << ","
         << "\"w\":" << num(d.box.width, 0) << ","
         << "\"h\":" << num(d.box.height, 0) << ","
         << "\"kpts\":" << visible << ","
         << "\"kpts_total\":" << d.kpt_conf.size() << "}";
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

  std::unique_ptr<YoloPoseDetector> detector_;
  size_t last_count_ = 0;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr info_pub_;
  float kpt_conf_ = 0.5f;   // 可见点计数阈值 (同绘制阈值)
  std::chrono::steady_clock::time_point info_last_;
};

}  // namespace flylab_vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<flylab_vision::YoloPoseNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
