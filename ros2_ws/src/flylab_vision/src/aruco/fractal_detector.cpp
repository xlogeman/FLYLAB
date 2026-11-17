// Copyright 2026 flylab
#include "aruco/fractal_detector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <opencv2/calib3d.hpp>

namespace flylab_vision
{

namespace
{

// 与 aruco_detector.cpp 同款: 无标定时按分辨率+FOV 估算内参
cv::Mat intrinsics_from_fov(int w, int h, double fov_deg)
{
  const double fx = w / (2.0 * std::tan(fov_deg * CV_PI / 360.0));
  return (cv::Mat_<double>(3, 3) <<
          fx, 0.0, w / 2.0,
          0.0, fx, h / 2.0,
          0.0, 0.0, 1.0);
}

cv::Mat no_distortion()
{
  return cv::Mat::zeros(1, 5, CV_64F);
}

}  // namespace

FractalDetector::FractalDetector(const FractalDetectorConfig & config)
: config_(config)
{
  det_ = std::make_unique<aruco::FractalDetector>();
  try {
    det_->setConfiguration(config_.config_name);
  } catch (const cv::Exception & e) {
    throw std::runtime_error(
      "分形码配置无效 '" + config_.config_name +
      "' (内置: FRACTAL_2L_6/3L_6/4L_6/5L_6, 或 fractal_create 生成的 yml 路径): " +
      e.what());
  }
}

void FractalDetector::sync_params(const cv::Mat & bgr)
{
  const cv::Size size = bgr.size();
  if (size == synced_size_ &&
    config_.marker_length == synced_marker_length_ &&
    config_.fov_deg == synced_fov_)
  {
    return;
  }
  const cv::Mat K = config_.K.empty() ?
    intrinsics_from_fov(bgr.cols, bgr.rows, config_.fov_deg) : config_.K;
  const cv::Mat D = config_.D.empty() ? no_distortion() : config_.D;
  det_->setParams(
    aruco::CameraParameters(K, D, size),
    static_cast<float>(config_.marker_length));
  synced_size_ = size;
  synced_marker_length_ = config_.marker_length;
  synced_fov_ = config_.fov_deg;
}

std::vector<Detection> FractalDetector::detect(const cv::Mat & bgr)
{
  std::vector<Detection> results;
  if (bgr.empty()) {
    return results;
  }
  sync_params(bgr);
  kp_tracked_ = false;

  // 上游约定: detect 失败仍要调 poseEstimation —— Case2 关键点跟踪接管
  const bool detected = det_->detect(bgr);
  const bool posed = det_->poseEstimation();
  if (!posed) {
    return results;
  }
  kp_tracked_ = !detected;

  const cv::Mat tvec = det_->getTvec();
  const cv::Mat rvec = det_->getRvec();
  if (tvec.empty() || rvec.empty()) {
    return results;
  }
  const cv::Mat K = config_.K.empty() ?
    intrinsics_from_fov(bgr.cols, bgr.rows, config_.fov_deg) : config_.K;
  const cv::Mat D = config_.D.empty() ? no_distortion() : config_.D;

  // 位姿轴端点投影 (供可视化画 RGB 轴); 轴长 = 半边长
  auto fill_pose = [&](Detection & d) {
      d.tvec = cv::Vec3d(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
      d.has_pose = true;
      const double a = config_.marker_length * 0.5;
      cv::Mat pts(4, 1, CV_64FC3);
      pts.at<cv::Vec3d>(0) = {0, 0, 0};
      pts.at<cv::Vec3d>(1) = {a, 0, 0};
      pts.at<cv::Vec3d>(2) = {0, a, 0};
      pts.at<cv::Vec3d>(3) = {0, 0, a};
      cv::Mat proj;
      cv::projectPoints(pts, rvec, tvec, K, D, proj);
      for (int k = 0; k < 4; ++k) {
        d.axis_pts.emplace_back(
          static_cast<float>(proj.at<cv::Vec2d>(k)[0]),
          static_cast<float>(proj.at<cv::Vec2d>(k)[1]));
      }
    };

  // 角点几何量 (中心取角点均值, 与 ArUcoDetector 一致)
  auto finish_geom = [&](Detection & d) {
      cv::Point2f lo(1e9f, 1e9f), hi(-1e9f, -1e9f);
      for (const auto & p : d.corners) {
        lo.x = std::min(lo.x, p.x);
        lo.y = std::min(lo.y, p.y);
        hi.x = std::max(hi.x, p.x);
        hi.y = std::max(hi.y, p.y);
      }
      d.bbox_wh = hi - lo;
      for (const auto & p : d.corners) {
        d.center_px += p;
      }
      d.center_px /= static_cast<float>(d.corners.size());
      d.center_norm = cv::Point2f(d.center_px.x / bgr.cols, d.center_px.y / bgr.rows);
    };

  if (detected) {
    // Case1: 每层一个 Detection (层轮廓 + 层 id); 全码一份位姿挂画面最大层
    const std::vector<aruco::Marker> markers = det_->getMarkers();
    size_t largest = 0;
    double largest_area = -1.0;
    for (size_t i = 0; i < markers.size(); ++i) {
      cv::Point2f lo(1e9f, 1e9f), hi(-1e9f, -1e9f);
      for (const auto & p : markers[i]) {
        lo.x = std::min(lo.x, p.x);
        lo.y = std::min(lo.y, p.y);
        hi.x = std::max(hi.x, p.x);
        hi.y = std::max(hi.y, p.y);
      }
      const double area = static_cast<double>(hi.x - lo.x) * (hi.y - lo.y);
      if (area > largest_area) {
        largest_area = area;
        largest = i;
      }
    }
    for (size_t i = 0; i < markers.size(); ++i) {
      const auto & m = markers[i];
      Detection d;
      d.id = m.id;
      for (const auto & p : m) {
        d.corners.push_back(p);
      }
      finish_geom(d);
      if (i == largest) {
        fill_pose(d);
      }
      results.push_back(std::move(d));
    }
    return results;
  }

  // Case2: 无层轮廓, 投影外框四角生成虚拟 Detection 供叠加
  const double half = config_.marker_length / 2.0;
  const cv::Mat objp = (cv::Mat_<cv::Point3f>(4, 1) <<
                        cv::Point3f(-half, half, 0), cv::Point3f(half, half, 0),
                        cv::Point3f(half, -half, 0), cv::Point3f(-half, -half, 0));
  cv::Mat proj;
  cv::projectPoints(objp, rvec, tvec, K, D, proj);
  Detection d;
  d.id = -1;
  d.kp_tracked = true;
  for (int k = 0; k < 4; ++k) {
    d.corners.emplace_back(
      proj.at<cv::Vec2f>(k)[0], proj.at<cv::Vec2f>(k)[1]);
  }
  finish_geom(d);
  fill_pose(d);
  results.push_back(std::move(d));
  return results;
}

}  // namespace flylab_vision
