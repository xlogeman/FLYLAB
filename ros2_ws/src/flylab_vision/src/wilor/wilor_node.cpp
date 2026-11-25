// Copyright 2026 flylab
/// WiLoR 手部姿态 (21 关键点 + MANO 网格) 估计 ROS2 节点 —— 接入地面站
/// 视觉工具体系 (基类 common/display_vision_node.hpp)。
///
/// 链路 (话题名与 flylab_gcs/config.py 保持一致, 均可用参数覆盖):
///   1. 图像输入: 订阅地面站"当前显示的帧" /flylab/display/image_raw
///   2. 工具按钮: /flylab/vision_cmd 收 wilor_start / wilor_stop 启停
///   3. 画面叠加: 左右手 bbox + 6 段骨架 polyline + MANO 网格 778 顶点
///      点云, 转 JSON 标注发 /flylab/annotations (组名 "wilor")
///   4. 面板数据: {"src":"wilor", "img":网格渲染图, "items":[...]} 发
///      /flylab/algo_info (10Hz 限频, 前 2 只手: 左右手/置信度/位置/尺寸)
///
/// 模型默认取 share/flylab_vision/models/hand/ 下 detector.onnx +
/// wilor_fp16_dyn.onnx (CMakeLists 安装; 动态 batch, 多手拼批一次回归:
/// 单手 ~32ms, batch=4 摊到 ~19ms/手, 旧静态 wilor_fp16.onnx 单手 28ms),
/// 可用 detector_model / wilor_model 参数覆盖 (喂回旧静态模型自动逐手)。
///
/// 运行:
///   ros2 run flylab_vision wilor_node --ros-args -p device:=cuda
#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "common/display_vision_node.hpp"
#include "wilor/wilor.hpp"

namespace flylab_vision
{

class WilorNode final : public DisplayVisionNode
{
public:
  WilorNode()
  : DisplayVisionNode("wilor_node",
      {"wilor_start", "wilor_stop", "wilor", "手部姿态"})
  {
    // ---------- 参数 ----------
    const auto detector_model = declare_parameter<std::string>(
      "detector_model", default_model_path("hand", "detector.onnx"));
    const auto wilor_model = declare_parameter<std::string>(
      "wilor_model", default_model_path("hand", "wilor_fp16_dyn.onnx"));
    const auto conf = declare_parameter<double>("conf", 0.3);
    const auto iou = declare_parameter<double>("iou", 0.45);
    const auto det_stride = declare_parameter<int>("det_stride", 1);
    const auto device = declare_parameter<std::string>("device", "cpu");
    const auto info_topic =
      declare_parameter<std::string>("info_topic", "/flylab/algo_info");

    // ---------- 模型 (启动即加载, 失败抛异常直接终止暴露配置问题) ----------
    WilorConfig cfg;
    cfg.detector_path = detector_model;
    cfg.wilor_path = wilor_model;
    cfg.conf_thres = static_cast<float>(conf);
    cfg.iou_thres = static_cast<float>(iou);
    cfg.det_stride = det_stride;
    cfg.use_cuda = device == "cuda";
    pipeline_ = std::make_unique<WilorPipeline>(cfg);
    RCLCPP_INFO(
      get_logger(),
      "模型加载完成: detector %s, wilor %s [%s, conf %.2f, %s]",
      detector_model.c_str(), wilor_model.c_str(),
      pipeline_->cuda_active() ? "cuda" : "cpu", cfg.conf_thres,
      pipeline_->batch_dynamic() ? "动态batch" : "静态逐手");
    if (cfg.use_cuda && !pipeline_->cuda_active()) {
      RCLCPP_WARN(
        get_logger(), "CUDA EP 注册失败, 已回退 CPU: %s",
        pipeline_->cuda_error().c_str());
    }

    log_ready("WiLoR 手部姿态");

    // ---------- 通信 (基类之外: 右栏算法信息面板数据) ----------
    info_pub_ = create_publisher<std_msgs::msg::String>(info_topic, 10);
  }

protected:
  std::string process(const cv::Mat & bgr, int /*frame_width*/) override
  {
    const auto hands = pipeline_->detect(bgr);
    last_count_ = hands.size();
    publish_info(hands);
    return to_annotations_json(hands, algo_group());
  }

  std::string clear_json() const override
  {
    return to_annotations_json(std::vector<WilorHand>{}, algo_group());
  }

  void log_first_infer(long ms) override
  {
    RCLCPP_INFO(
      get_logger(), "首帧推理耗时 %ldms, 检出 %zu 只手", ms, last_count_);
  }

private:
  /// 面板数据 {"src":"wilor", "img":..., "items":[...]}: 10Hz 限频 (右栏
  /// 面板 1.2s 超时置灰)。"img" = 首只手的 MANO 网格渲染小图 (base64
  /// JPEG, 分组顶部固定高度图片行, 见 algo_output_panel SECTIONS['wilor']
  /// 的 'image': True); items = 前 2 只手 (表格 2 列):
  /// 左右手 / 置信度 / 框左上角 / 框尺寸
  void publish_info(const std::vector<WilorHand> & hands)
  {
    const auto now = std::chrono::steady_clock::now();
    if (now - info_last_ < std::chrono::milliseconds(100)) {
      return;
    }
    info_last_ = now;
    const size_t n = std::min<size_t>(hands.size(), 2);
    std::ostringstream os;
    os << "{\"src\":\"wilor\",\"count\":" << hands.size();
    if (!hands.empty()) {
      const auto img = to_mesh_jpeg(hands.front());
      if (!img.empty()) {
        os << ",\"img\":\"" << img << "\"";
      }
    }
    os << ",\"items\":[";
    for (size_t i = 0; i < n; ++i) {
      const auto & d = hands[i];
      if (i > 0) {
        os << ',';
      }
      os << "{\"side\":\"" << (d.is_right ? "Right" : "Left") << "\","
         << "\"conf\":" << num(d.conf, 2) << ","
         << "\"x\":" << num(d.box.x, 0) << ","
         << "\"y\":" << num(d.box.y, 0) << ","
         << "\"w\":" << num(d.box.width, 0) << ","
         << "\"h\":" << num(d.box.height, 0) << "}";
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

  std::unique_ptr<WilorPipeline> pipeline_;
  size_t last_count_ = 0;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr info_pub_;
  std::chrono::steady_clock::time_point info_last_;
};

}  // namespace flylab_vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<flylab_vision::WilorNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
