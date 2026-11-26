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
/// 地面站运行时参数同步 (码边长/视场角/分形配置) 与降落跟踪 ID 接收复用
/// control/marker/marker_link.hpp 共用组件 (三节点单源); 码边长/视场角与
/// ArUco 检测共用同一滑杆 (同一个降落坪)。
///
/// 运行:
///   ros2 run flylab_vision fractal_node --ros-args --params-file
///       <install>/flylab_vision/share/flylab_vision/configs/aruco.yaml
///   也可 -p 覆盖单项: -p fractal_config:=FRACTAL_3L_6 -p marker_length:=0.5

#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <std_msgs/msg/string.hpp>

#include "common/display_vision_node.hpp"
#include "common/gcs_topics.hpp"
#include "marker/fractal_detector.hpp"
#include "control/marker/marker_link.hpp"

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
    const auto info_topic =
      declare_parameter<std::string>("info_topic", topics::ALGO_INFO_TOPIC);

    // ---------- 模型 (启动即加载, 失败抛异常直接终止暴露配置问题) ----------
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

    // ---------- 地面站运行时参数同步 (三节点共用组件) ----------
    // 码边长: 与 ArUco 检测共用同一滑杆, 运行中同步外层码位姿尺度;
    // 视场角: 未标定时按 FOV 估算内参, 实时跟随 (已标定拒跟);
    // 分形配置: 运行中换配置重建检测器 (码图案集合下一帧生效)
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
        nullptr,
        [this](const std::string & name) {
          FractalDetectorConfig rcfg = detector_->config();
          rcfg.config_name = name;
          try {
            detector_ = std::make_unique<FractalDetector>(rcfg);
          } catch (const std::exception & e) {
            RCLCPP_WARN(
              get_logger(), "忽略无效分形配置 '%s': %s",
              name.c_str(), e.what());
            return false;
          }
          return true;
        }},
      marker_length, fov_deg, "", fractal_config);

    // ---------- 降落节点跟踪层同步 (MAX 让位, 共用组件) ----------
    // 降落中每帧广播正在跟踪的层 ID: 若恰为画面最大层, 本节点不画洋红
    // MAX, 该层由降落节点橙色 TRACK 独占, 避免两组标注叠在同一层上;
    // 广播停发 (降落结束/节点退出) 超时后自动恢复 MAX (与 aruco_node 同款)
    tracked_id_rx_ = std::make_unique<TrackedIdRx>(*this);

    // ---------- 右栏"算法信息"面板数据 (10Hz 限频) ----------
    info_pub_ = create_publisher<std_msgs::msg::String>(info_topic, 10);

    // 就绪日志带配置/边长信息, 不用基类 log_ready
    RCLCPP_INFO(
      get_logger(),
      "fractal检测节点就绪: 配置 %s, 外层边长 %.2fm, 订阅 %s, 等待 %s 指令",
      sync_->fractal_config().c_str(), sync_->marker_length(),
      image_topic().c_str(), "fractal_start");
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
    const int tracked = tracked_id_rx_->current();
    if (tracked >= 0 && vis != nullptr && vis->id == tracked) {
      vis = nullptr;
    }
    return to_annotations_json(results, algo_group(), vis, "MAX");
  }

  std::string clear_json() const override
  {
    return to_annotations_json(std::vector<Detection>{}, algo_group());
  }

private:
  /// 右栏"算法信息"面板: 检测摘要 JSON (10Hz 限频)。
  /// {"src":"fractal","count":N,"kp":bool,"largest":{...}|null}
  /// largest 与 aruco 同构 (字段见 marker_link.hpp largest_fields_json
  /// 注释); kp = Case2 关键点跟踪 (遮挡/远距), 此时 largest 为 null
  void publishInfo(double img_w, double img_h, size_t count,
    bool kp, const Detection * largest)
  {
    const auto t = now();
    if ((t - last_info_pub_).seconds() < 0.1) {
      return;
    }
    last_info_pub_ = t;
    std::ostringstream js;
    js << "{\"src\":\"fractal\",\"count\":" << count << ",\"kp\":" << (kp ? 1 : 0)
       << ",\"largest\":";
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

  std::unique_ptr<FractalDetector> detector_;
  std::unique_ptr<MarkerParamSync> sync_;        // 地面站参数同步 (共用组件)
  std::unique_ptr<TrackedIdRx> tracked_id_rx_;   // 降落跟踪 ID 接收 (共用组件)
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
