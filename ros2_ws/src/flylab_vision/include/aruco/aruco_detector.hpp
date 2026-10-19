// Copyright 2026 flylab
#ifndef FLYLAB_VISION__ARUCO_DETECTOR_HPP_
#define FLYLAB_VISION__ARUCO_DETECTOR_HPP_

#include <map>
#include <string>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

namespace flylab_vision
{

/// 单个 marker 的检测结果 (一帧可含多个)
struct Detection
{
  int id = -1;
  std::vector<cv::Point2f> corners;  // 4 角点: 左上/右上/右下/左下 (像素)
  cv::Point2f center_px{0.f, 0.f};   // 中心像素坐标
  cv::Point2f center_norm{0.f, 0.f}; // 中心归一化坐标
  cv::Point2f bbox_wh{0.f, 0.f};     // 外接框宽高
  cv::Vec3d rvec{0.0, 0.0, 0.0};     // 相机系位姿 (x右 y下 z前, 米)
  cv::Vec3d tvec{0.0, 0.0, 0.0};
  bool has_pose = false;             // solvePnP 是否成功
  double yaw_img_deg = 0.0;          // 图像 2D 方向角 (顶边中点相对中心, 粗对准用)
  // 位姿轴端点投影: [原点, x端, y端, z端] (像素); has_pose 时有效,
  // 供可视化画 RGB 轴
  std::vector<cv::Point2f> axis_pts;
};

/// 检测器配置
struct DetectorConfig
{
  std::string dict_name = "DICT_4X4_1000";  // 6x6 码用 DICT_6X6_250
  double marker_length = 0.2;               // 码物理边长 (米), 影响 3D 位姿解算尺度
  double fov_deg = 60.0;                    // 无标定时按水平视场角估算内参
  double axis_len_m = 0.1;                  // 绘制位姿轴的长度 (米)
  cv::Mat K;                                // 相机内参 3x3; 空 = 按图像分辨率+FOV 估算
  cv::Mat D;                                // 畸变系数; 空 = 无畸变
};

/// ArUco 检测器 —— 纯算法, 不含 ROS / 窗口显示。
/// 单实例单线程使用; 检测与绘制可分开调用。
class ArucoDetector
{
public:
  explicit ArucoDetector(const DetectorConfig & config);

  /// 单帧检测 (BGR), 返回所有 marker 的结果; 无码返回空
  std::vector<Detection> detect(const cv::Mat & bgr);

  /// 是否使用了真实标定 (false 时内参为估算值, 位姿仅供参考)
  bool calibrated() const {return !config_.K.empty();}

  const DetectorConfig & config() const {return config_;}

private:
  cv::Mat K_for(const cv::Mat & bgr) const;  // 有标定用标定, 否则按分辨率+FOV 估算

  DetectorConfig config_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> params_;
};

/// 读取 OpenCV/SpireCV 格式标定 yaml (camera_matrix + distortion_coefficients)。
/// 失败抛 std::runtime_error。
void load_calib(const std::string & path, cv::Mat & K, cv::Mat & D);

/// 字典名 -> 预定义字典; 未知名字抛 std::invalid_argument
cv::Ptr<cv::aruco::Dictionary> make_dictionary(const std::string & name);

/// 检测结果 -> 地面站叠加标注 JSON (schema 见 flylab_tools/overlay.py)。
/// 输出为按算法分组的消息 {"algo": algo, "items": [...]}, 每个 marker:
///   四边形轮廓 polyline (绿) + 首角点/中心 point (红) + id 距离 text (黄)
///   + 位姿轴 3 条 polyline (x红/y绿/z蓝, has_pose 时)
/// 坐标为原图像素, 图像源须与地面站显示的相机/分辨率一致。
std::string to_annotations_json(
  const std::vector<Detection> & results, const std::string & algo = "aruco");

}  // namespace flylab_vision

#endif  // FLYLAB_VISION__ARUCO_DETECTOR_HPP_
