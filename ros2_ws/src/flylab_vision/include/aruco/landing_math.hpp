// Copyright 2026 flylab
/// 降落纯数学助手 —— 无 ROS 依赖的纯函数/纯逻辑, 可独立单测
#ifndef FLYLAB_VISION__ARUCO__LANDING_MATH_HPP_
#define FLYLAB_VISION__ARUCO__LANDING_MATH_HPP_

#include <cmath>
#include <vector>

#include <opencv2/core.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

namespace flylab_vision
{

inline double clampd(double v, double lim)
{
  lim = std::abs(lim);
  return std::fmin(std::fmax(v, -lim), lim);
}

/// R(q)·v: 四元数旋转向量 (标准公式, q 为 body←camera 旋转)
inline cv::Point3d quat_rot(const geometry_msgs::msg::Quaternion & q, double x, double y, double z)
{
  // t = 2 * (q_vec × v)
  const double tx = 2.0 * (q.y * z - q.z * y);
  const double ty = 2.0 * (q.z * x - q.x * z);
  const double tz = 2.0 * (q.x * y - q.y * x);
  return {
    x + q.w * tx + (q.y * tz - q.z * ty),
    y + q.w * ty + (q.z * tx - q.x * tz),
    z + q.w * tz + (q.x * ty - q.y * tx)
  };
}

/// 降落趋势角: 位移向量与竖直向下方向的夹角 (度)
inline double landing_angle_deg(const std::vector<cv::Point3d> & positions)
{
  const auto & a = positions.front();
  const auto & b = positions.back();
  const double vx = b.x - a.x, vy = b.y - a.y, vz = b.z - a.z;
  const double len = std::sqrt(vx * vx + vy * vy + vz * vz);
  if (len < 1e-6) {
    return 0.0;  // 位置未变化, 视为已垂直
  }
  double cos_theta = -vz / len;  // ENU 竖直向下 = (0,0,-1)
  cos_theta = std::fmax(-1.0, std::fmin(1.0, cos_theta));
  return std::acos(cos_theta) * 180.0 / M_PI;
}

/// 角度归一化到 [-180, 180)
inline double wrap180(double deg)
{
  deg = std::fmod(deg + 180.0, 360.0);
  if (deg < 0.0) {
    deg += 360.0;
  }
  return deg - 180.0;
}

/// 下降速度调度 (正值=向下): 按 s=(1-面积)² 两段线性插值,
/// 近处(s→0) v_close 慢降, 远处(s→1) v_far 快降
inline double descend_schedule(
  double area, double v_close, double v_far, double area_s_thresh)
{
  double s = std::fmax(0.0, std::fmin(1.0, 1.0 - area));
  s *= s;
  const double v_mid = 0.5 * (v_close + v_far);       // 分段点处速度
  const double t2 = std::fmax(0.01, std::fmin(0.99, area_s_thresh));
  if (s <= t2) {
    return v_close + (v_mid - v_close) * (s / t2);
  }
  return v_mid + (v_far - v_mid) * ((s - t2) / (1.0 - t2));
}

/// 降落趋势角估计器: 滑动窗口内机体位移与竖直方向的夹角 (度),
/// 窗口大小随面积自适应: 距离越远窗口越大
class TrendAngleEstimator
{
public:
  TrendAngleEstimator() = default;

  void setWindow(int min_frames, int max_frames)
  {
    win_min_ = min_frames;
    win_max_ = max_frames;
  }

  /// p: 机体位置 (ENU, 米); pos_valid 为 false 的拍忽略 (位置无效)
  void update(double area, bool pos_valid, const cv::Point3d & p)
  {
    if (!pos_valid) {
      return;
    }
    double win = (1.0 - area) * (1.0 - area) * 12.0;
    win = std::fmax(
      static_cast<double>(win_min_),
      std::fmin(static_cast<double>(win_max_), win));
    const size_t n = std::max<size_t>(2, static_cast<size_t>(std::lround(win)));
    if (positions_.size() >= n) {
      positions_.erase(positions_.begin());
    }
    positions_.push_back(p);
    if (positions_.size() >= n) {
      angle_ = landing_angle_deg(positions_);
    }
  }

  /// 当前趋势角 (度); 窗口数据不足时保持初始值 90 (视为未垂直)
  double angle() const {return angle_;}

  void reset()
  {
    angle_ = 90.0;
    positions_.clear();
  }

private:
  int win_min_ = 5;
  int win_max_ = 50;
  double angle_ = 90.0;                 // 降落趋势角 (度)
  std::vector<cv::Point3d> positions_;  // 滑动窗口 (ENU)
};

}  // namespace flylab_vision

#endif  // FLYLAB_VISION__ARUCO__LANDING_MATH_HPP_
