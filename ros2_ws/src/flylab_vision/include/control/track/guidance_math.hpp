// Copyright 2026 flylab
#ifndef FLYLAB_VISION__CONTROL__TRACK__GUIDANCE_MATH_HPP_
#define FLYLAB_VISION__CONTROL__TRACK__GUIDANCE_MATH_HPP_

/// 反控制导共用控制原语 (纯函数, 无 ROS 依赖, 可独立单测)。
///
/// 抽自 track_takeover.hpp 中三种控制环 (机头跟随偏航 / 高度环 / 跟随
/// 距离环) 重复的"抑摆三件套"手法, 语义与原实现逐行保持:
///
///   死区回差 P 律 (hysteresis_p_law): |err| 进入死区停输出, 须偏离
///     1.5× 死区才重启 (回差防测量噪声在死区边界抖动); |err| < slow
///     时增益随误差线性衰减 (中心减速段, 越接近越慢平滑收敛), 超过
///     恢复线性全增益, 输出限幅 ±rate_max
///
///   一阶低通 (lowpass_step): 抑制测量噪声进反演律 (像素偏角经 cotθ /
///     1/tanθ 反演放大), tc = 0 直通; 首拍 (state = NaN) 直接取原始值
///
///   限加速度 (accel_limit_step): 指令斜率限幅, 阶跃平滑成匀加速 ——
///     指令大幅摆动时实际速度会收敛到摆动均值导致闭环失效
///     (02_57_39 俯冲自激振荡教训), 限斜率后实际速度贴指令走
///
///   机体俯仰补偿 (body_pitch_rad): 自 NED→FRD 四元数提取俯仰角, 把
///     机体系相对视线角补偿成世界系视线角 (前倾不补偿 → 距离高估
///     正反馈 / 航迹比视线平)

#include <algorithm>
#include <cmath>

namespace flylab_vision
{

/// 死区回差 + 中心减速 P 律: in_deadband 为迟滞状态 (调用方持有, 随
/// 控制环生命周期复位); 死区内返回 0, 其余返回限幅后的控制量
inline double hysteresis_p_law(
  double err, double kp, double deadband, double slow,
  double rate_max, bool & in_deadband)
{
  const double abs_err = std::fabs(err);
  if (in_deadband) {
    if (abs_err < deadband * 1.5) {
      return 0.0;
    }
    in_deadband = false;
  } else if (abs_err <= deadband) {
    in_deadband = true;
    return 0.0;
  }
  const double u = std::min(1.0, abs_err / slow);
  return std::clamp(kp * err * u, -rate_max, rate_max);
}

/// 一阶低通步进 (返回新状态; state = NaN 视为未初始化直接取原始值)
inline double lowpass_step(double state, double raw, double tc, double dt)
{
  const double alpha = tc > 0.0 ? dt / (tc + dt) : 1.0;
  return std::isnan(state) ? raw : state + alpha * (raw - state);
}

/// 指令限加速度步进 (返回新斜率状态): target 每拍至多向 prev 靠拢
/// accel·dt
inline double accel_limit_step(
  double prev, double target, double accel, double dt)
{
  const double dv_max = accel * dt;
  return std::clamp(target, prev - dv_max, prev + dv_max);
}

/// 机体俯仰角 (rad, 下正) 自 NED→FRD 四元数: asin(2(xz+wy))
/// (已对照恒等/朝东/低头 30° 三基准验证)
inline double body_pitch_rad(double x, double y, double z, double w)
{
  return std::asin(std::clamp(2.0 * (x * z + w * y), -1.0, 1.0));
}

}  // namespace flylab_vision

#endif  // FLYLAB_VISION__CONTROL__TRACK__GUIDANCE_MATH_HPP_
