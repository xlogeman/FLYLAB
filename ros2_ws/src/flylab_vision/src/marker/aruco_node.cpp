// Copyright 2026 flylab
/// ArUco 检测 ROS2 节点 —— 接入地面站视觉工具体系 (基类 common/display_vision_node.hpp)。
///
/// 链路 (话题名与 flylab_gcs/config.py 保持一致, 均可用参数覆盖):
///   1. 图像输入: 订阅地面站"当前显示的帧" /flylab/display/image_raw
///   2. 工具按钮: /flylab/vision_cmd 收 aruco_start / aruco_stop 启停
///   3. 画面叠加: 检测结果转 JSON 标注发 /flylab/annotations
///      (schema 见 flylab_gcs/overlay.py), 地面站自动叠加到画面上
///
/// 地面站运行时参数同步 (码边长/视场角/字典) 与降落跟踪 ID 接收复用
/// control/marker/marker_link.hpp 共用组件 (三节点单源)。
///
/// 运行:
///   ros2 run flylab_vision aruco_node
///   ros2 run flylab_vision aruco_node --ros-args --params-file
///       <install>/flylab_vision/share/flylab_vision/configs/aruco.yaml
///   也可 -p 覆盖单项: -p image_topic:=/airsim/cam/left_cam/image_raw -p dict_name:=DICT_6X6_250

#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <std_msgs/msg/string.hpp>

#include "common/display_vision_node.hpp"
#include "common/gcs_topics.hpp"
#include "marker/aruco_detector.hpp"
#include "control/marker/marker_link.hpp"

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

    // ---------- 地面站运行时参数同步 (三节点共用组件) ----------
    // 码边长: 场景含多种尺寸码时滑杆同步位姿解算尺度 (检测不受影响);
    // 视场角: 未标定时按 FOV 估算内参, 实时跟随 (已标定拒跟);
    // 字典: 下拉切换下一帧生效, 未知名字拒绝并保持原字典
    sync_ = std::make_unique<MarkerParamSync>(*this,
      MarkerParamSync::Hooks{
        [this](double v) {detector_->set_marker_length(v); return true;},
        [this](double v) {
          if (detector_->calibrated()) {
            return false;
          }
          detector_->set_fov(v);
          return true;
        },
        [this](const std::string & name) {
          try {
            detector_->set_dictionary(name);
          } catch (const std::exception & e) {
            RCLCPP_WARN(
              get_logger(), "忽略无效字典 '%s': %s", name.c_str(), e.what());
            return false;
          }
          return true;
        },
        nullptr},
      marker_length, fov_deg, dict_name, "");

    // ---------- 降落节点跟踪码同步 (MAX 让位, 共用组件) ----------
    // 降落中每帧广播正在跟踪的码 ID: 若恰为画面最大码, 本节点不画洋红
    // MAX, 该码由降落节点橙色 TRACK 独占, 避免两组标注叠在同一码上;
    // 广播停发 (降落结束/节点退出) 超时后自动恢复 MAX
    tracked_id_rx_ = std::make_unique<TrackedIdRx>(*this);

    // ---------- 右栏"算法输出"面板数据 ----------
    // 结构化检测摘要 (最大码 id/占比/偏移/距离), 地面站右栏面板显示;
    // 10Hz 限频, 话题名可用参数覆盖
    const auto info_topic =
      declare_parameter<std::string>("info_topic", topics::ALGO_INFO_TOPIC);
    info_pub_ = create_publisher<std_msgs::msg::String>(info_topic, 10);

    // 就绪日志带字典/边长信息, 不用基类 log_ready
    RCLCPP_INFO(
      get_logger(),
      "ArUco 检测节点就绪: 字典 %s, 边长 %.2fm, 订阅 %s, 等待 %s 指令",
      sync_->dict_name().c_str(), sync_->marker_length(),
      image_topic().c_str(), "aruco_start");
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
    const int tracked = tracked_id_rx_->current();
    if (tracked >= 0 && vis != nullptr && vis->id == tracked) {
      vis = nullptr;
    }
    return to_annotations_json(results, "aruco", vis, "MAX");
  }

  std::string clear_json() const override
  {
    return to_annotations_json(std::vector<Detection>{});
  }

private:
  /// 右栏"算法输出"面板: 最大码摘要 JSON (10Hz 限频)。
  /// {"src":"detect","count":N,"largest":{...}|null} —— largest 字段见
  /// marker_link.hpp largest_fields_json 注释
  void publishInfo(double img_w, double img_h, size_t count,
    const Detection * largest)
  {
    const auto t = now();
    if ((t - last_info_pub_).seconds() < 0.1) {
      return;
    }
    last_info_pub_ = t;
    std::ostringstream js;
    js << "{\"src\":\"aruco\",\"count\":" << count << ",\"largest\":";
    if (largest == nullptr) {
      js << "null}";
    } else {
      js << largest_fields_json(img_w, img_h, *largest,
        physical_width(img_w, detector_->calibrated(), detector_->fx(),
          sync_->fov_deg(), *largest))
         << "}";
    }
    std_msgs::msg::String out;
    out.data = js.str();
    info_pub_->publish(out);
  }

  std::unique_ptr<ArucoDetector> detector_;
  std::unique_ptr<MarkerParamSync> sync_;        // 地面站参数同步 (共用组件)
  std::unique_ptr<TrackedIdRx> tracked_id_rx_;   // 降落跟踪 ID 接收 (共用组件)
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
