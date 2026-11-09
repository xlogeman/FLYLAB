// Copyright 2026 flylab
/// 朝向对齐子系统 —— 云台对准码 0° 方向 (+偏置), 机头经跟随通道追随
#ifndef FLYLAB_VISION__ARUCO__YAW_ALIGNER_HPP_
#define FLYLAB_VISION__ARUCO__YAW_ALIGNER_HPP_

#include <cmath>

#include <rclcpp/rclcpp.hpp>

#include "aruco/landing_math.hpp"

namespace flylab_vision
{

/// 一次 update 的输出
struct YawAlignOut
{
  double yaw_err_deg = 0.0;   // 滤波后的云台方位偏差 (度, 右正)
  double yaw_gate = 1.0;      // 航向门控 [0,1]: 关闭对齐或已锁定 = 1
  double gimbal_rate = 0.0;   // 云台偏航速率指令 (°/s, 右正, 0=停转)
  double yaw_rate_rad = 0.0;  // 机头跟随注入偏航角速率 (rad/s, 正=右转)
  bool follow_ok = false;     // 跟随误差本次可用
  bool locked = false;        // 航向已锁定 (含超时强制放行)
  bool just_locked = false;   // 本次 update 恰好进入锁定 (边沿)
  bool turning = false;       // 云台正在转动 (未锁定且未超时)
};

/// 朝向对齐状态机 (纯逻辑, 不持有发布器 —— 云台指令/跟随请求由上层发送):
///   目标: 云台方位 = 码方位 − yaw_offset, 对准后画面中码 0° 方向停在
///   屏幕上方偏右 yaw_offset 处 (右正); 机头经跟随通道同步追随云台
///   (注入偏航角速率 = follow_kp × 反卷误差) —— 收敛态与直接转机头一致
///   (码方位相对机头 = yaw_offset)。不等水平对齐即开始, 与对中/下降并行,
///   边降边转时码持续变大、朝向测量自改善; 测量逐帧 EMA 滤波 (α=0.4);
///   云台对准 (|偏差|≤tol) 且机头跟上 (|跟随误差|≤follow_tol) 连续
///   min_frames 帧后锁定航向; 转向悬置超 timeout_sec 强制放行 (仅放行,
///   不再重试对齐, 人工介入排查)。目标码切换 (嵌套码接管/重选) 后朝向
///   基准失效, 自动复位滤波/计数/锁定按新码重新对齐。
///   航向门控: 取云台偏差与跟随误差的较大者线性斜坡 (任一未跟上都压低
///   下降); 跟随误差不可用时只按云台偏差门控 (由超时保护兜底放行)。
class YawAligner
{
public:
  struct Params
  {
    bool enabled = true;           // 是否启用朝向对齐 (yaw_align)
    double offset_deg = 0.0;       // 朝向偏置 (度, 右正): 机头相对码 0° 方向的落点朝向
    double tol_deg = 5.0;          // 云台对准容差 (度)
    double gain = 2.5;             // 云台速率 P 增益
    double rate_max_deg = 30.0;    // 云台速率限幅 (°/s)
    int min_frames = 5;            // 双对齐连续达标帧数
    double timeout_sec = 30.0;     // 转向悬置超时 (秒, 0=禁用)
    double follow_kp = 0.8;        // 机头跟随注入增益
    double follow_tol_deg = 5.0;   // 机头跟上容差 (度)
    double follow_rate_max_deg = 30.0;  // 跟随注入限幅 (°/s)
    double gate_width_deg = 15.0;  // 航向门控斜坡宽度 (度)
  };

  YawAligner(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock, const Params & p)
  : logger_(logger), clock_(clock), p_(p) {}

  /// 全部复位到未对齐 (降落流程重启/字典变更); 保留偏置
  void reset()
  {
    locked_ = false;
    count_ = 0;
    filt_init_ = false;
    last_id_ = -1;
    wait_t_ = 0.0;
  }

  /// 朝向偏置更新 (地面站同步): 解锁航向按新朝向重新对齐
  void setOffset(double offset_deg)
  {
    if (offset_deg == p_.offset_deg) {
      return;
    }
    p_.offset_deg = offset_deg;
    reset();
  }

  bool locked() const {return locked_;}
  bool enabled() const {return p_.enabled;}
  double offsetDeg() const {return p_.offset_deg;}

  /// 每个控制拍调用一次 (须在云台姿态新鲜时调用, 方位测量才可信)。
  ///   target_id: 当前跟踪码 ID (切换时自动复位重新对齐)
  ///   marker_yaw_deg: 码 0° 方向机体系方位角 (度, 右正)
  ///   gimbal_az_deg: 云台方位角 (度, 右正) = 画面上方旋到机体系的水平方位
  ///   follow_ok / follow_err_deg: 跟随误差可用性与数值 (度, 右+)
  YawAlignOut update(
    double dt, int target_id, double marker_yaw_deg, double gimbal_az_deg,
    bool follow_ok, double follow_err_deg)
  {
    YawAlignOut out;
    out.follow_ok = follow_ok;
    if (!p_.enabled) {
      last_id_ = target_id;
      return out;
    }
    if (target_id != last_id_) {
      // 目标码切换 (嵌套码接管/重选) 后朝向基准失效, 重新对齐
      last_id_ = target_id;
      filt_init_ = false;
      count_ = 0;
      locked_ = false;
    }
    // 方位偏差 (右正): 云台方位需转到 码方位 − 偏置; 逐帧 EMA 滤波
    const double e = wrap180(marker_yaw_deg - p_.offset_deg - gimbal_az_deg);
    const double err =
      filt_init_ ? wrap180(err_f_ + 0.4 * wrap180(e - err_f_)) : e;
    err_f_ = err;
    filt_init_ = true;
    out.yaw_err_deg = err;
    if (locked_) {
      out.locked = true;
      return out;   // 锁定后不再对齐 (门控恒 1, 云台停转)
    }
    // 转向阶段: 请求相机侧反卷 + 误差注入偏航角速率由上层完成;
    // 跟随误差不可用 (非 AirSim 源 / 地面站挂掉) 时仅转云台不转机头,
    // 机头跟不上则永远不满足锁定条件, 由超时保护兜底
    if (!follow_ok) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "\033[33m机头跟随误差不可用 (需地面站 AirSim 相机源 + 跟随广播), "
        "仅转云台不转机头\033[0m");
    }
    const bool aircraft_aligned =
      follow_ok && std::fabs(follow_err_deg) <= p_.follow_tol_deg;
    if (std::fabs(err) <= p_.tol_deg && aircraft_aligned) {
      count_++;
    } else {
      count_ = 0;
    }
    if (count_ >= p_.min_frames) {
      locked_ = true;
      out.locked = true;
      out.just_locked = true;
      RCLCPP_INFO(
        logger_,
        "\033[32m云台已对准码朝向且机头已跟随 (云台偏差 %.1f°, 跟随误差 %.1f°, "
        "偏置 %.0f°), 航向锁定\033[0m",
        err, follow_err_deg, p_.offset_deg);
      return out;
    }
    // 转云台 + 机头跟随 (与对中/下降并行, 下降由航向门控平滑压低)
    out.turning = true;
    out.gimbal_rate = p_.gain * err;
    if (follow_ok) {
      out.yaw_rate_rad = clampd(
        p_.follow_kp * follow_err_deg, p_.follow_rate_max_deg) * M_PI / 180.0;
    }
    // 转向悬置超时保护: 长期收敛不了按当前朝向继续 (人工介入排查)
    wait_t_ += dt;
    if (p_.timeout_sec > 0.0 && wait_t_ > p_.timeout_sec) {
      RCLCPP_WARN(
        logger_,
        "\033[33m朝向对齐悬置超时 (%.0fs, 云台偏差 %.1f°, 跟随误差 %.1f°), "
        "放弃对齐继续下降 (可取消后检查 yaw_gain/follow_kp/容差/"
        "码是否被遮挡/云台是否到限位)\033[0m",
        wait_t_, err, follow_err_deg);
      locked_ = true;   // 仅放行后续流程, 不再重试对齐
      out.locked = true;
      out.just_locked = true;
      out.turning = false;
      out.gimbal_rate = 0.0;
      out.yaw_rate_rad = 0.0;
      return out;
    }
    // 航向门控: 取云台偏差与跟随误差的较大者 (任一未跟上都压低下降);
    // 跟随误差不可用时只按云台偏差门控 (由超时保护兜底放行)
    double gate_err = std::fabs(err);
    if (follow_ok) {
      gate_err = std::fmax(gate_err, std::fabs(follow_err_deg));
    }
    out.yaw_gate = std::fmin(1.0, std::fmax(0.0,
      1.0 - (gate_err - p_.tol_deg) / std::fmax(1e-3, p_.gate_width_deg)));
    return out;
  }

private:
  rclcpp::Logger logger_ = rclcpp::get_logger("yaw_aligner");
  rclcpp::Clock::SharedPtr clock_;
  Params p_;
  bool locked_ = false;       // 航向已锁定 (转 AUTO.LAND 前提之一)
  int count_ = 0;             // 航向偏差连续达标帧数
  double err_f_ = 0.0;        // 航向偏差 EMA 滤波值 (度)
  bool filt_init_ = false;    // 滤波器已初始化 (首帧直通)
  int last_id_ = -1;          // 上次测向的目标码 ID (切换复位用)
  double wait_t_ = 0.0;       // 转向悬置累计时长 (超时保护)
};

}  // namespace flylab_vision

#endif  // FLYLAB_VISION__ARUCO__YAW_ALIGNER_HPP_
