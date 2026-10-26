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
///   ros2 run flylab_vision aruco_node --ros-args --params-file \
///       <install>/flylab_vision/share/flylab_vision/configs/aruco.yaml
///   也可 -p 覆盖单项: -p image_topic:=/airsim/cam/left_cam/image_raw -p dict_name:=DICT_6X6_250

#include <memory>
#include <string>

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

    // 就绪日志带字典/边长信息, 不用基类 log_ready
    RCLCPP_INFO(
      get_logger(),
      "ArUco 检测节点就绪: 字典 %s, 边长 %.2fm, 订阅 %s, 等待 %s 指令",
      dict_name.c_str(), marker_length, image_topic().c_str(), "aruco_start");
  }

protected:
  std::string process(const cv::Mat & bgr, int /*frame_width*/) override
  {
    return to_annotations_json(detector_->detect(bgr));
  }

  std::string clear_json() const override
  {
    return to_annotations_json(std::vector<Detection>{});
  }

private:
  std::unique_ptr<ArucoDetector> detector_;
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
