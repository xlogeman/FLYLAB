// Copyright 2026 flylab
#ifndef FLYLAB_VISION__ARUCO__FRACTAL_DETECTOR_HPP_
#define FLYLAB_VISION__ARUCO__FRACTAL_DETECTOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include <aruco.h>  // thirdparty/aruco-3.1.12 (UCO aruco, GPLv3)

#include "marker/aruco_detector.hpp"  // Detection / load_calib 复用

namespace flylab_vision
{

/// 分形检测器配置
struct FractalDetectorConfig
{
  std::string config_name = "FRACTAL_5L_6";  // 内置配置名或 fractal_create 的 yml 路径
  double marker_length = 0.2;                // 外层码物理边长 (米), 位姿解算尺度
  double fov_deg = 60.0;                     // 无标定时按水平视场角估算内参
  cv::Mat K;                                 // 相机内参 3x3; 空 = 按分辨率+FOV 估算
  cv::Mat D;                                 // 畸变系数; 空 = 无畸变
};

/// 分形码检测器 —— 包装 UCO aruco 3.1.12 的 FractalDetector,
/// 输出复用 Detection (与 ArUcoDetector 同构, 标注/降落子系统零改动接入)。
/// 纯算法, 不含 ROS / 窗口显示; 单实例单线程使用。
/// 两级工作模式 (上游库约定):
///   Case1 正常检测: 逐层内码字典匹配, 每层输出一个 Detection (轮廓+id),
///                   全码共用一份位姿 (挂在画面最大层, 各层中心重合)
///   Case2 关键点跟踪: 外层轮廓被遮挡/过小时 detect 失败, 由上一帧位姿 +
///                   关键点匹配接管 —— 输出单个虚拟 Detection (id=-1,
///                   角点为外框投影, kp_tracked=true), 位姿连续不掉
class FractalDetector
{
public:
  /// config_name 非法 (非内置配置且文件打不开) 抛 std::runtime_error
  explicit FractalDetector(const FractalDetectorConfig & config);

  /// 单帧检测 (BGR); 无结果返回空
  std::vector<Detection> detect(const cv::Mat & bgr);

  /// 是否使用真实标定 (false 时内参为估算值, 位姿仅供参考)
  bool calibrated() const {return !config_.K.empty();}

  /// 焦距 fx (像素, 标定 K 的 (0,0))。未标定返回 0 占位 —— 调用点
  /// (physical_width 实参求值) 不看 calibrated 标志, fx() 自身必须安全;
  /// 所有调用点都在 calibrated()==true 时才使用该值
  double fx() const {return config_.K.empty() ? 0.0 : config_.K.at<double>(0, 0);}

  /// 运行中更新 FOV (度): 下一帧生效; 仅无标定时有意义
  void set_fov(double fov_deg) {config_.fov_deg = fov_deg;}

  /// 运行中更新外层码边长 (米): 位姿解算尺度下一帧生效, 检测本身不受影响
  void set_marker_length(double length_m) {config_.marker_length = length_m;}

  /// 上一帧是否走了 Case2 关键点跟踪 (detect 失败但位姿成功)
  bool last_kp_tracked() const {return kp_tracked_;}

  /// 当前生效配置 (换配置重建检测器时沿用内参/边长)
  const FractalDetectorConfig & config() const {return config_;}

private:
  /// 上游 setParams 缓存内参, 分辨率/FOV/边长变化时重设
  void sync_params(const cv::Mat & bgr);

  FractalDetectorConfig config_;
  cv::Size synced_size_;
  double synced_marker_length_ = -1.0;
  double synced_fov_ = -1.0;
  std::unique_ptr<aruco::FractalDetector> det_;
  bool kp_tracked_ = false;
};

}  // namespace flylab_vision

#endif  // FLYLAB_VISION__ARUCO__FRACTAL_DETECTOR_HPP_
