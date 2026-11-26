// Copyright 2026 flylab
/// 分形码检测 ROS2 节点 —— 接入地面站视觉工具体系 (基类 common/display_vision_node.hpp)
///
/// 分形码 = 大码内嵌套逐层缩小的同 id 码 (UCO aruco 3.1.12): 远距离看内层
/// 小码、近距离看外层, 遮挡时关键点跟踪接管位姿 (Case1/Case2 见
/// fractal_detector.hpp)。链路同 aruco_node:
///   1. 图像输入: 订阅地面站"当前显示的帧" /flylab/display/image_raw
///   2. 工具按钮: /flylab/vision_cmd 收 fractal_start / fractal_stop 启停
///   3. 画面叠加: 检测结果转 JSON 标注发 /flylab/annotations (分组 "fractal")
///   4. 右栏面板: 检测摘要 JSON 发 /flylab/algo_info (src="fractal", 10Hz)
///
/// 地面站运行中同步 (全部 latched, 与 aruco_node 同一约定):
///   视场角 /flylab/camera_fov、码边长 /flylab/aruco_marker_length (共用
///   同一滑杆, 与 ArUco 是同一个降落坪)、分形配置 /flylab/fractal_config
///   (FRACTAL_2L_6~5L_6 或 yml 路径)
///
/// 运行:
///   ros2 run flylab_vision fractal_node --ros-args --params-file
///       <install>/flylab_vision/share/flylab_vision/configs/aruco.yaml
///   也可 -p 覆盖单项: -p fractal_config:=FRACTAL_3L_6 -p marker_length:=0.5

#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include "common/display_vision_node.hpp"
#include "common/gcs_topics.hpp"
#include "aruco/fractal_detector.hpp"

namespace flylab_vision
{

class FractalNode final : public DisplayVisionNode
{
public:
  FractalNode()
  // 分形码无首帧推理日志诉求, 结果走独立分组 "fractal"
  : DisplayVisionNode("fractal_detector",
      {"fractal_start", "fractal_stop", "fractal", "fractal检测", false, false})
  {
    // ---------- 参数 ----------
    const auto fractal_config =
      declare_parameter<std::string>("fractal_config", "FRACTAL_5L_6");
    const auto marker_length = declare_parameter<double>("marker_length", 0.2);
    const auto fov_deg = declare_parameter<double>("fov_deg", 60.0);
    const auto calib_path = declare_parameter<std::string>("calib", "");
    const auto fov_topic =
      declare_parameter<std::string>("fov_topic", topics::CAMERA_FOV_TOPIC);
    const auto marker_len_topic = declare_parameter<std::string>(
      "marker_length_topic", topics::ARUCO_MARKER_LENGTH_TOPIC);
    const auto config_topic =
      declare_parameter<std::string>("fractal_config_topic", topics::FRACTAL_CONFIG_TOPIC);
    const auto info_topic =
      declare_parameter<std::string>("info_topic", topics::ALGO_INFO_TOPIC);

    // ---------- 降落节点跟踪层同步 (MAX 让位) ----------
    // 降落中每帧广播正在跟踪的层 ID: 若恰为画面最大层, 本节点不画洋红
    // MAX, 该层由降落节点橙色 TRACK 独占, 避免两组标注叠在同一层上;
    // 广播停发 (降落结束/节点退出) 超时后自动恢复 MAX (与 aruco_node 同款)
    const auto tracked_id_topic = declare_parameter<std::string>(
      "tracked_id_topic", topics::LANDING_TRACKED_ID_TOPIC);
    tracked_id_sub_ = create_subscription<std_msgs::msg::Int32>(
      tracked_id_topic, 10,
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        tracked_id_rx_ = msg->data;
        tracked_id_rx_time_ = now();
      });

    FractalDetectorConfig cfg;
    cfg.config_name = fractal_config;
    cfg.marker_length = marker_length;   // 外层码物理边长 (米)
    cfg.fov_deg = fov_deg;
    if (!calib_path.empty()) {
      load_calib(calib_path, cfg.K, cfg.D);   // 失败抛异常, 直接终止暴露配置问题
    }
    detector_ = std::make_unique<FractalDetector>(cfg);
    if (!detector_->calibrated()) {
      RCLCPP_WARN(
        get_logger(),
        "未提供标定 (calib 参数), 内参按 FOV 估算, 位姿解算仅供参考");
    }

    // ---------- 地面站视场角同步 (latched, 未标定时按 FOV 估算内参) ----------
    fov_deg_ = fov_deg;
    fov_sub_ = create_subscription<std_msgs::msg::Float64>(
      fov_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        if (detector_->calibrated() || msg->data <= 0.0 || msg->data == fov_deg_) {
          return;
        }
        fov_deg_ = msg->data;
        detector_->set_fov(msg->data);
      });

    // ---------- 地面站码边长同步 (latched, 与 ArUco 检测共用同一滑杆) ----------
    marker_length_ = marker_length;
    marker_len_sub_ = create_subscription<std_msgs::msg::Float64>(
      marker_len_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        if (msg->data <= 0.0 || msg->data == marker_length_) {
          return;
        }
        marker_length_ = msg->data;
        detector_->set_marker_length(msg->data);
        RCLCPP_INFO(
          get_logger(), "外层码边长同步: %.2fm (位姿尺度随之更新, 来自地面站)",
          msg->data);
      });

    // ---------- 地面站分形配置同步 (latched): 运行中换配置重建检测器 ----------
    config_name_ = fractal_config;
    config_sub_ = create_subscription<std_msgs::msg::String>(
      config_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {on_config(*msg);});

    // ---------- 右栏"算法信息"面板数据 (10Hz 限频) ----------
    info_pub_ = create_publisher<std_msgs::msg::String>(info_topic, 10);

    // 就绪日志带配置/边长信息, 不用基类 log_ready
    RCLCPP_INFO(
      get_logger(),
      "fractal检测节点就绪: 配置 %s, 外层边长 %.2fm, 订阅 %s, 等待 %s 指令",
      config_name_.c_str(), marker_length, image_topic().c_str(), "fractal_start");
  }

protected:
  std::string process(const cv::Mat & bgr, int /*frame_width*/) override
  {
    auto results = detector_->detect(bgr);
    // 画面最大层强调 (洋红 MAX): Case2 虚拟框 (kp_tracked) 不参与,
    // 遮挡场景由 KP 标签 + 位姿轴表达跟踪结果
    const Detection * largest = nullptr;
    double largest_area = -1.0;
    for (const auto & d : results) {
      if (d.kp_tracked) {
        continue;
      }
      const double a = d.bbox_wh.x * d.bbox_wh.y;
      if (a > largest_area) {
        largest_area = a;
        largest = &d;
      }
    }
    // 右栏"算法信息"面板数据: 检测摘要 (10Hz 限频)
    publishInfo(bgr.cols, bgr.rows, results.size(),
      detector_->last_kp_tracked(), largest);
    // 让位: 最大层恰为降落节点正在跟踪的层 (广播 1s 内有效) 时,
    // 不画 MAX —— 该层由降落节点橙色 TRACK 独占, 防止两组标注叠画
    const Detection * vis = largest;
    const bool landing_tracking = tracked_id_rx_ >= 0 &&
      (now() - tracked_id_rx_time_).seconds() < kTrackedIdTimeoutSec;
    if (landing_tracking && vis != nullptr && vis->id == tracked_id_rx_) {
      vis = nullptr;
    }
    return to_annotations_json(results, algo_group(), vis, "MAX");
  }

  std::string clear_json() const override
  {
    return to_annotations_json(std::vector<Detection>{}, algo_group());
  }

private:
  static constexpr double kTrackedIdTimeoutSec = 1.0;  // 跟踪层广播有效期
  /// 地面站分形配置同步: 换配置下一帧生效 (重建检测器);
  /// 未知配置拒绝并保持原配置
  void on_config(const std_msgs::msg::String & msg)
  {
    if (msg.data.empty() || msg.data == config_name_) {
      return;
    }
    FractalDetectorConfig cfg = detector_->config();
    cfg.config_name = msg.data;
    try {
      auto det = std::make_unique<FractalDetector>(cfg);
      detector_ = std::move(det);
      config_name_ = msg.data;
      RCLCPP_INFO(
        get_logger(), "分形配置同步: %s (下一帧检测生效, 来自地面站)",
        config_name_.c_str());
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(), "忽略无效分形配置 '%s': %s", msg.data.c_str(), e.what());
    }
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

  /// 右栏"算法信息"面板: 检测摘要 JSON (10Hz 限频)。
  /// {"src":"fractal","count":N,"kp":bool,"largest":{...}|null}
  /// largest 与 aruco 同构 (面板字段提取复用): id/area/w/h/dx/dy/dist/wm/
  /// tvec; kp = Case2 关键点跟踪 (遮挡/远距), 此时 largest 为 null
  void publishInfo(double img_w, double img_h, size_t count,
    bool kp, const Detection * largest)
  {
    const auto t = now();
    if ((t - last_info_pub_).seconds() < 0.1) {
      return;
    }
    last_info_pub_ = t;
    std::ostringstream js;
    js << std::fixed << std::setprecision(3)
       << "{\"src\":\"fractal\",\"count\":" << count << ",\"kp\":" << (kp ? 1 : 0)
       << ",\"largest\":";
    if (largest == nullptr) {
      js << "null}";
    } else {
      const double area = img_w > 0.0 ?
        (largest->bbox_wh.x * largest->bbox_wh.y) /
        (img_w * (img_h > 0.0 ? img_h : 1.0)) : 0.0;
      js << "{\"id\":" << largest->id
         << ",\"area\":" << area
         << ",\"w\":" << static_cast<int>(largest->bbox_wh.x + 0.5)
         << ",\"h\":" << static_cast<int>(largest->bbox_wh.y + 0.5)
         << ",\"dx\":" << (largest->center_norm.x - 0.5)
         << ",\"dy\":" << (largest->center_norm.y - 0.5)
         << ",\"dist\":" << (largest->has_pose ? largest->tvec[2] : -1.0)
         << ",\"wm\":" << physicalWidth(img_w, *largest);
      if (largest->has_pose) {
        js << ",\"tvec\":[" << largest->tvec[0] << ',' << largest->tvec[1]
           << ',' << largest->tvec[2] << ']';
      }
      js << "}}";
    }
    std_msgs::msg::String out;
    out.data = js.str();
    info_pub_->publish(out);
  }

  std::unique_ptr<FractalDetector> detector_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr fov_sub_;
  double fov_deg_ = 60.0;   // 当前生效的 FOV (同步去重用)
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr marker_len_sub_;
  double marker_length_ = 0.2;  // 当前生效的码边长 (同步去重用)
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr config_sub_;
  std::string config_name_;     // 当前生效的分形配置 (同步去重用)
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr tracked_id_sub_;
  int tracked_id_rx_ = -1;  // 降落节点正在跟踪的层 ID (最新广播值)
  rclcpp::Time tracked_id_rx_time_{0, 0, RCL_ROS_TIME};  // 广播到达时刻 (有效期判定)
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr info_pub_;  // 面板数据
  rclcpp::Time last_info_pub_{0, 0, RCL_ROS_TIME};                // 10Hz 限频
};

}  // namespace flylab_vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<flylab_vision::FractalNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
