// Copyright 2026 flylab
/// ArUco 检测 ROS2 节点 —— 接入地面站视觉工具体系 (基类 common/display_vision_node.hpp)。
///
/// 链路 (话题名与 flylab_gcs/config.py 保持一致, 均可用参数覆盖):
///   1. 图像输入: 订阅地面站"当前显示的帧" /flylab/display/image_raw
///   2. 工具按钮: /flylab/vision_cmd 收 aruco_start / aruco_stop 启停
///   3. 画面叠加: 检测结果转 JSON 标注发 /flylab/annotations
///      (schema 见 flylab_gcs/overlay.py), 地面站自动叠加到画面上
///
/// 运行:
///   ros2 run flylab_vision aruco_node
///   ros2 run flylab_vision aruco_node --ros-args --params-file 
///       <install>/flylab_vision/share/flylab_vision/configs/aruco.yaml
///   也可 -p 覆盖单项: -p image_topic:=/airsim/cam/left_cam/image_raw -p dict_name:=DICT_6X6_250

#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>

#include "common/display_vision_node.hpp"
#include "aruco/aruco_detector.hpp"

namespace flylab_vision
{

class ArucoNode final : public DisplayVisionNode
{
public:
  ArucoNode()
  // aruco 无首帧推理日志诉求, 且结果走默认分组 (to_annotations_json 默认 "aruco")
  : DisplayVisionNode("aruco_detector",
      {"aruco_start", "aruco_stop", "aruco", "检测", false, false})
  {
    // ---------- 参数 ----------
    const auto dict_name = declare_parameter<std::string>("dict_name", "DICT_4X4_1000");
    const auto marker_length = declare_parameter<double>("marker_length", 0.2);
    const auto fov_deg = declare_parameter<double>("fov_deg", 60.0);
    const auto calib_path = declare_parameter<std::string>("calib", "");

    DetectorConfig cfg;
    cfg.dict_name = dict_name;
    cfg.marker_length = marker_length;
    cfg.fov_deg = fov_deg;
    if (!calib_path.empty()) {
      load_calib(calib_path, cfg.K, cfg.D);   // 失败抛异常, 直接终止暴露配置问题
    }
    detector_ = std::make_unique<ArucoDetector>(cfg);
    if (!detector_->calibrated()) {
      RCLCPP_WARN(
        get_logger(),
        "未提供标定 (calib 参数), 内参按 FOV 估算, 位姿解算仅供参考");
    }

    // ---------- 地面站视场角同步 ----------
    // 地面站调整视场滑杆即发布当前 FOV (latched, 后启动也能收到最近值);
    // 未标定时按 FOV 估算内参, 实时跟随。话题名可用参数覆盖。
    const auto fov_topic =
      declare_parameter<std::string>("fov_topic", "/flylab/camera_fov");
    fov_deg_ = fov_deg;
    fov_sub_ = create_subscription<std_msgs::msg::Float64>(
      fov_topic, 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {on_fov(*msg);});

    // ---------- 降落节点跟踪码同步 (MAX 让位) ----------
    // 降落中每帧广播正在跟踪的码 ID: 若恰为画面最大码, 本节点不画洋红
    // MAX, 该码由降落节点橙色 TRACK 独占, 避免两组标注叠在同一码上;
    // 广播停发 (降落结束/节点退出) 超时后自动恢复 MAX
    const auto tracked_id_topic = declare_parameter<std::string>(
      "tracked_id_topic", "/flylab/landing_tracked_id");
    tracked_id_sub_ = create_subscription<std_msgs::msg::Int32>(
      tracked_id_topic, 10,
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        tracked_id_rx_ = msg->data;
        tracked_id_rx_time_ = now();
      });

    // ---------- 右栏"算法输出"面板数据 ----------
    // 结构化检测摘要 (最大码 id/占比/偏移/距离), 地面站右栏面板显示;
    // 10Hz 限频, 话题名可用参数覆盖
    const auto info_topic =
      declare_parameter<std::string>("info_topic", "/flylab/aruco_info");
    info_pub_ = create_publisher<std_msgs::msg::String>(info_topic, 10);

    // 就绪日志带字典/边长信息, 不用基类 log_ready
    RCLCPP_INFO(
      get_logger(),
      "ArUco 检测节点就绪: 字典 %s, 边长 %.2fm, 订阅 %s, 等待 %s 指令",
      dict_name.c_str(), marker_length, image_topic().c_str(), "aruco_start");
  }

protected:
  std::string process(const cv::Mat & bgr, int /*frame_width*/) override
  {
    auto results = detector_->detect(bgr);
    // 画面最大码 (真实最大; 让位只影响可视化, 不影响面板摘要)
    const Detection * largest = nullptr;
    double largest_area = -1.0;
    for (const auto & d : results) {
      const double a = d.bbox_wh.x * d.bbox_wh.y;
      if (a > largest_area) {
        largest_area = a;
        largest = &d;
      }
    }
    // 右栏"算法输出"面板数据: 最大码摘要 (10Hz 限频)
    publishInfo(bgr.cols, bgr.rows, results.size(), largest);
    // 画面最大码用强调样式 (洋红 + MAX 后缀), 一眼看出谁最大。
    // 让位: 最大码恰为降落节点正在跟踪的码 (广播 1s 内有效) 时,
    // 不画 MAX —— 该码由降落节点橙色 TRACK 独占, 防止两组标注叠画
    const Detection * vis = largest;
    const bool landing_tracking = tracked_id_rx_ >= 0 &&
      (now() - tracked_id_rx_time_).seconds() < kTrackedIdTimeoutSec;
    if (landing_tracking && vis != nullptr && vis->id == tracked_id_rx_) {
      vis = nullptr;
    }
    return to_annotations_json(results, "aruco", vis, "MAX");
  }

  std::string clear_json() const override
  {
    return to_annotations_json(std::vector<Detection>{});
  }

private:
  static constexpr double kTrackedIdTimeoutSec = 1.0;  // 跟踪码广播有效期

  void on_fov(const std_msgs::msg::Float64 & msg)
  {
    // 已标定 (真实内参) 或非法值不跟随; 值未变化跳过 (滑杆拖动/重发布降噪)
    if (detector_->calibrated() || msg.data <= 0.0 || msg.data == fov_deg_) {
      return;
    }
    fov_deg_ = msg.data;
    detector_->set_fov(msg.data);
    // RCLCPP_INFO(
    //   get_logger(), "视场角同步: %.1f° (内参按 FOV 估算, 来自地面站)",
    //   msg.data);
  }

  /// 码的实际物理宽度 (米): w_px × Z / fx (针孔模型), 无位姿返回 -1。
  /// fx 与检测器同源: 有标定用标定 K, 否则按当前同步 FOV 估算
  double physicalWidth(double img_w, const Detection & d) const
  {
    if (!d.has_pose || img_w <= 0.0) {
      return -1.0;
    }
    const double fx = detector_->calibrated() ? detector_->fx() :
      img_w / (2.0 * std::tan(fov_deg_ * CV_PI / 360.0));
    return d.bbox_wh.x * d.tvec[2] / fx;
  }

  /// 右栏"算法输出"面板: 最大码摘要 JSON (10Hz 限频)。
  /// {"src":"detect","count":N,"largest":{"id","area","w","h","dx","dy","dist","wm"}|null}
  /// area=画面占比, w/h=外接框宽高 (原图像素), dx/dy=中心相对画面中心
  /// 偏移 (归一化, 右/下为正), dist=tvec z 前向距离 (米, 无位姿 -1),
  /// wm=实际物理宽度 (米, 由像素宽+距离+内参换算, 无位姿 -1)
  void publishInfo(double img_w, double img_h, size_t count,
    const Detection * largest)
  {
    (void)img_h;
    const auto t = now();
    if ((t - last_info_pub_).seconds() < 0.1) {
      return;
    }
    last_info_pub_ = t;
    std::ostringstream js;
    js << "{\"src\":\"detect\",\"count\":" << count << ",\"largest\":";
    if (largest == nullptr) {
      js << "null}";
    } else {
      const double area = img_w > 0.0 ?
        (largest->bbox_wh.x * largest->bbox_wh.y) /
        (img_w * (img_h > 0.0 ? img_h : 1.0)) : 0.0;
      js << std::fixed << std::setprecision(3)
         << "{\"id\":" << largest->id
         << ",\"area\":" << area
         << ",\"w\":" << static_cast<int>(largest->bbox_wh.x + 0.5)
         << ",\"h\":" << static_cast<int>(largest->bbox_wh.y + 0.5)
         << ",\"dx\":" << (largest->center_norm.x - 0.5)
         << ",\"dy\":" << (largest->center_norm.y - 0.5)
         << ",\"dist\":" << (largest->has_pose ? largest->tvec[2] : -1.0)
         << ",\"wm\":" << physicalWidth(img_w, *largest)
         << "}}";
    }
    std_msgs::msg::String out;
    out.data = js.str();
    info_pub_->publish(out);
  }

  std::unique_ptr<ArucoDetector> detector_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr fov_sub_;
  double fov_deg_ = 60.0;   // 当前生效的 FOV (同步去重用)
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr tracked_id_sub_;
  int tracked_id_rx_ = -1;  // 降落节点正在跟踪的码 ID (最新广播值)
  rclcpp::Time tracked_id_rx_time_{0, 0, RCL_ROS_TIME};  // 广播到达时刻 (有效期判定)
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr info_pub_;  // 面板数据
  rclcpp::Time last_info_pub_{0, 0, RCL_ROS_TIME};                // 10Hz 限频
};

}  // namespace flylab_vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<flylab_vision::ArucoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
