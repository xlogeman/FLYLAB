// Copyright 2026 flylab
/// YOLO 单目深度估计 ROS2 节点 —— 接入地面站视觉工具体系 (基类 common/display_vision_node.hpp)。
///
/// 与 yolo_detector_node.cpp 同构, 区别在算法 (逐像素深度) 和输出形式:
/// 可视化结果 (turbo 伪彩深度图 + 深度摘要) 全部在地面站右栏"算法信息"
/// 面板显示 (信息话题与 ArUco 检测同一 /flylab/aruco_info, 面板按 src
/// 分组), 画面上不画任何东西 (标注只发空 items 供 HUD"解算 X"
/// 帧率统计)。链路:
///   1. 图像输入: 订阅地面站"当前显示的帧" /flylab/display/image_raw
///   2. 工具按钮: /flylab/vision_cmd 收 depth_start / depth_stop 启停
///   3. 右栏面板: 伪彩缩略图 + 摘要 JSON 随推理帧率发布 (info_hz 可限频)
///      发到 /flylab/aruco_info (组名 "depth", 与检测/降落分组独立, 可并存)
///   4. 画面叠加: 仅发空 items 标注 (帧率心跳), 不画任何图元
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
///       -p model:=/path/to/xx.onnx -p device:=cuda
#include <memory>
#include <string>

#include <std_msgs/msg/string.hpp>

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
    const auto jpeg_quality = declare_parameter<int>("jpeg_quality", 70);
    // 右栏面板数据: 伪彩缩略图宽度 (px); 0 = 不发图只发文字摘要
    const auto panel_img_width = declare_parameter<int>("panel_img_width", 320);
    // 面板数据限频 (Hz); 0 = 随推理帧率发布 (深度每帧都在变, 与图传同帧率;
    // ArUco 这类文字摘要变化慢才适合固定 10Hz 限频)
    const auto info_hz = declare_parameter<double>("info_hz", 0.0);
    info_min_dt_ = info_hz > 0.0 ? 1.0 / info_hz : 0.0;
    jpeg_quality_ = jpeg_quality;
    panel_img_width_ = panel_img_width;

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

    // ---------- 地面站右栏"算法信息"面板数据 ----------
    // 与 ArUco 检测同一话题, 面板按 src 分组 ("depth"), 话题名可用参数覆盖
    const auto info_topic = declare_parameter<std::string>(
      "info_topic", "/flylab/aruco_info");
    info_pub_ = create_publisher<std_msgs::msg::String>(info_topic, 10);

    log_ready("YOLO 深度估计");
  }

protected:
  std::string process(const cv::Mat & bgr, int /*frame_width*/) override
  {
    const DepthResult result = estimator_->estimate(bgr);
    last_dmin_ = result.dmin;
    last_dmax_ = result.dmax;
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
      get_logger(), "首帧推理耗时 %ldms, 深度范围 %.2f ~ %.2f",
      ms, last_dmin_, last_dmax_);
  }

private:
  std::unique_ptr<YoloDepthEstimator> estimator_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr info_pub_;  // 面板数据
  rclcpp::Time last_info_pub_{0, 0, RCL_ROS_TIME};                // info_hz 限频
  double info_min_dt_ = 0.0;                                      // 0 = 每帧发
  int jpeg_quality_ = 70;
  int panel_img_width_ = 320;
  float last_dmin_ = 0.f;
  float last_dmax_ = 0.f;

  /// 右栏"算法信息"面板: 深度摘要 + turbo 伪彩缩略图 (默认随推理帧率发布,
  /// info_hz 参数可限频)。
  /// {"src":"depth","dmin","dmax","dmean","dctr","w","h","img"|缺省}
  void publishInfo(const DepthResult & result, int frame_w, int frame_h)
  {
    const auto t = now();
    if (info_min_dt_ > 0.0 && (t - last_info_pub_).seconds() < info_min_dt_) {
      return;
    }
    last_info_pub_ = t;
    std_msgs::msg::String out;
    out.data = to_depth_info_json(
      result, frame_w, frame_h, panel_img_width_, jpeg_quality_);
    info_pub_->publish(out);
  }
};

}  // namespace flylab_vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<flylab_vision::YoloDepthNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
