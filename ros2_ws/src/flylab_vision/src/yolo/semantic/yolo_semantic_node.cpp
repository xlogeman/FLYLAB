// Copyright 2026 flylab
/// YOLO 语义分割 ROS2 节点 —— 接入地面站视觉工具体系 (基类 common/display_vision_node.hpp)。
///
/// 与 yolo_segment_node.cpp 同构, 区别在算法 (逐像素类别图) 和输出形式:
/// 可视化结果 (cityscapes 配色类别图 + 类别图例) 全部在地面站右栏"算法
/// 信息"面板显示 (信息话题与 ArUco 检测/深度估计同一 /flylab/aruco_info,
/// 面板按 src 分组), 画面上不画任何东西 (标注只发空 items 供 HUD"解算 X"
/// 帧率统计)。链路:
///   1. 图像输入: 订阅地面站"当前显示的帧" /flylab/display/image_raw
///   2. 工具按钮: /flylab/vision_cmd 收 semantic_start / semantic_stop 启停
///   3. 右栏面板: 配色缩略图 + 类别图例 JSON 随推理帧率发布 (info_hz 可限频)
///      发到 /flylab/aruco_info (组名 "semantic", 与检测/降落/深度分组独立,
///      可并存)
///   4. 画面叠加: 仅发空 items 标注 (帧率心跳), 不画任何图元
///
/// 模型默认取 share/flylab_vision/models/semantic/yolo26s-sem.onnx
/// (CMakeLists 安装), Cityscapes 19 类语义分割, 输入尺寸自动取模型固定值
/// (1024), 都可用参数覆盖。
///
/// 运行:
///   ros2 run flylab_vision yolo_semantic_node
///   ros2 run flylab_vision yolo_semantic_node --ros-args
///       -p model:=/path/to/xx.onnx -p device:=cuda
#include <algorithm>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <std_msgs/msg/string.hpp>

#include "common/display_vision_node.hpp"
#include "yolo/yolo_semantic.hpp"

namespace flylab_vision
{

class YoloSemanticNode final : public DisplayVisionNode
{
public:
  YoloSemanticNode()
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
    // 类别图中值滤波核 (奇数, 去椒盐噪声), 0/1 = 关闭
    const auto smooth = declare_parameter<int>("smooth", 5);
    // 右栏面板数据: 配色缩略图宽度 (px); 0 = 不发图只发文字图例
    const auto panel_img_width = declare_parameter<int>("panel_img_width", 320);
    // 面板数据限频 (Hz); 0 = 随推理帧率发布 (类别图每帧都在变, 与图传同
    // 帧率; ArUco 这类文字摘要变化慢才适合固定 10Hz 限频)
    const auto info_hz = declare_parameter<double>("info_hz", 0.0);
    info_min_dt_ = info_hz > 0.0 ? 1.0 / info_hz : 0.0;
    panel_img_width_ = panel_img_width;

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

    // ---------- 地面站右栏"算法信息"面板数据 ----------
    // 与 ArUco 检测同一话题, 面板按 src 分组 ("semantic"), 话题名可用参数覆盖
    const auto info_topic = declare_parameter<std::string>(
      "info_topic", "/flylab/aruco_info");
    info_pub_ = create_publisher<std_msgs::msg::String>(info_topic, 10);

    log_ready("YOLO 语义分割");
  }

protected:
  std::string process(const cv::Mat & bgr, int /*frame_width*/) override
  {
    const YoloSemanticResult result = segmenter_->segment(bgr);
    last_classes_ = result.present.size();
    publishInfo(result, bgr.cols, bgr.rows);
    // 画面上不画任何图元; 仍发空 items 标注作心跳
    // (地面站 HUD"解算 X"帧率统计依赖标注消息流)
    return std::string("{\"algo\":\"") + algo_group() + "\",\"items\":[]}";
  }

  std::string clear_json() const override
  {
    // 空 items = 清掉自己的分组标注
    return std::string("{\"algo\":\"") + algo_group() + "\",\"items\":[]}";
  }

  void log_first_infer(long ms) override
  {
    RCLCPP_INFO(
      get_logger(), "首帧推理耗时 %ldms, 检出 %zu 类", ms, last_classes_);
  }

private:
  std::unique_ptr<YoloSemanticSegmenter> segmenter_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr info_pub_;  // 面板数据
  rclcpp::Time last_info_pub_{0, 0, RCL_ROS_TIME};                // info_hz 限频
  double info_min_dt_ = 0.0;                                      // 0 = 每帧发
  int panel_img_width_ = 320;
  size_t last_classes_ = 0;

  /// 右栏"算法信息"面板: 类别图例 + cityscapes 配色缩略图 (默认随推理帧率
  /// 发布, info_hz 参数可限频)。
  /// {"src":"semantic","nclass","w","h","present":[[名,占比%]...],"img"|缺省}
  void publishInfo(const YoloSemanticResult & result, int frame_w, int frame_h)
  {
    const auto t = now();
    if (info_min_dt_ > 0.0 && (t - last_info_pub_).seconds() < info_min_dt_) {
      return;
    }
    last_info_pub_ = t;
    std_msgs::msg::String out;
    out.data = to_semantic_info_json(
      result, frame_w, frame_h, panel_img_width_);
    info_pub_->publish(out);
  }
};

}  // namespace flylab_vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<flylab_vision::YoloSemanticNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
