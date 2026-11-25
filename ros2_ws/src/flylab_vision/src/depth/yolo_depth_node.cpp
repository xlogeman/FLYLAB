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
///
/// 深度图输出: /flylab/depth_map (sensor_msgs/Image, 16UC1 毫米, best-effort)
/// 供地面站"鼠标悬停画面读深度"。全分辨率 float32 每帧数 MB, 按 map_scale
/// 降采样后以毫米整型发出 (毫米精度对悬停读数绰绰有余, 65535mm 上限外
/// 饱和); 降采样最近邻采样, 值不被邻域混合。仅订阅者在线时才发布。
#include <memory>
#include <string>

#include <opencv2/imgproc.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <std_msgs/msg/string.hpp>

#include "common/display_vision_node.hpp"
#include "depth/yolo_depth.hpp"

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

    // 深度图话题 (悬停读数用): map_scale 降采样比例 (0/1 = 全分辨率,
    // 0.5 = 长宽各半, 4 倍体积收益; 最近邻缩放值不被邻域混合)
    const auto map_topic = declare_parameter<std::string>(
      "map_topic", "/flylab/depth_map");
    map_scale_ = declare_parameter<double>("map_scale", 0.5);
    if (map_scale_ <= 0.0 || map_scale_ > 1.0) {
      map_scale_ = 1.0;
    }
    // best-effort + 限深: 数 MB 级消息积压无意义, 最新帧语义即可;
    // 无订阅者 (地面站没开) 时 process() 直接跳过, 零开销
    map_pub_ = create_publisher<sensor_msgs::msg::Image>(
      map_topic, rclcpp::SensorDataQoS());

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
    publishMap(result);
    publishInfo(result, bgr.cols, bgr.rows);
    // 画面上不画任何图元; 仍发空 items 标注作心跳
    // (地面站 HUD"解算 X"帧率统计依赖标注消息流)
    return heartbeat_json();
  }

  std::string clear_json() const override
  {
    // 空 items = 清掉自己的分组标注 (与停止前的心跳同串)
    return heartbeat_json();
  }

  void log_first_infer(long ms) override
  {
    RCLCPP_INFO(
      get_logger(), "首帧推理耗时 %ldms, 深度范围 %.2f ~ %.2f",
      ms, last_dmin_, last_dmax_);
  }

private:
  /// 空 items 心跳/清屏标注串 (画面不画图元, 只供地面站 HUD 帧率统计)
  std::string heartbeat_json() const
  {
    return std::string("{\"algo\":\"") + algo_group() + "\",\"items\":[]}";
  }

  std::unique_ptr<YoloDepthEstimator> estimator_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr info_pub_;  // 面板数据
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr map_pub_; // 深度图
  double map_scale_ = 0.5;                                        // 降采样比例
  rclcpp::Time last_info_pub_{0, 0, RCL_ROS_TIME};                // info_hz 限频
  double info_min_dt_ = 0.0;                                      // 0 = 每帧发
  int jpeg_quality_ = 70;
  int panel_img_width_ = 320;
  float last_dmin_ = 0.f;
  float last_dmax_ = 0.f;

  /// 深度图发布 (悬停读数): CV_32F 米 -> 16UC1 毫米 (负值截 0, 上限
  /// 65535mm ≈ 65.5m 饱和), map_scale 最近邻降采样后发出。
  /// 与面板信息流独立: 无订阅者时整体跳过 (缩放/转换都不做)
  void publishMap(const DepthResult & result)
  {
    if (!map_pub_ || map_pub_->get_subscription_count() == 0 ||
        result.depth.empty()) {
      return;
    }
    const cv::Mat & full = result.depth;
    cv::Mat small;
    if (map_scale_ < 1.0 - 1e-6) {
      cv::resize(full, small, cv::Size(), map_scale_, map_scale_,
                 cv::INTER_NEAREST);
    } else {
      small = full;
    }
    cv::Mat mm;
    small.convertTo(mm, CV_16UC1, 1000.0);   // saturate_cast 自动截断越界值
    sensor_msgs::msg::Image msg;
    msg.height = mm.rows;
    msg.width = mm.cols;
    msg.encoding = "16UC1";
    msg.is_bigendian = false;
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(mm.step[0]);
    msg.header.frame_id = "depth";
    msg.header.stamp = now();
    msg.data.assign(
      mm.data, mm.data + mm.total() * mm.elemSize());
    map_pub_->publish(msg);
  }

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
