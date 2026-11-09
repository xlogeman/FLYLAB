// Copyright 2026 flylab
/// 降落目标码选择器 —— 指定 ID 优先 + 最大码回退 (多尺度嵌套码)
#ifndef FLYLAB_VISION__ARUCO__TARGET_SELECTOR_HPP_
#define FLYLAB_VISION__ARUCO__TARGET_SELECTOR_HPP_

#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "aruco/aruco_detector.hpp"

namespace flylab_vision
{

/// 目标选择 (指定 ID 优先 + 最大码回退, 适配多尺度嵌套码):
///   target_id = -1  自动模式: 首帧选画面最大的码并锁定, 持续丢失超
///                   reacquire_sec 重选
///   target_id >= 0  指定模式: 指定 ID 出现即切为对齐目标; 不可见则回退
///                   画面最大码 (fallback=true) 或只认指定 ID
/// 回退目标软锁定防跳变; 指定 ID 重新出现立即切回 (同心码无水平跳变)。
/// 同 ID 多码一律取画面面积最大者, 不随检出顺序跳变。
class TargetSelector
{
public:
  struct Params
  {
    int target_id = 41;           // 指定精准落点码 ID; -1 = 任一 (取画面最大并锁定)
    bool fallback = true;         // 指定 ID 不可见时回退画面最大码
    double reacquire_sec = 0.5;   // 锁定 ID 持续丢失多久后重新选择 (秒)
  };

  TargetSelector(rclcpp::Logger logger, const Params & p)
  : logger_(logger), p_(p) {}

  /// 丢弃锁定重新选目标 (降落流程重启/字典变更)
  void reset()
  {
    tracked_id_ = -1;
    lost_time_ = 0.0;
  }

  int trackedId() const {return tracked_id_;}

  /// 指定 ID 的检出码中取画面面积最大者; 无该 ID 返回 nullptr
  const Detection * largestOfId(
    int id, const std::vector<Detection> & detections) const
  {
    const Detection * best = nullptr;
    double best_area = -1.0;
    for (const auto & d : detections) {
      if (d.id != id) {
        continue;
      }
      const double a = d.bbox_wh.x * d.bbox_wh.y;
      if (a > best_area) {
        best_area = a;
        best = &d;
      }
    }
    return best;
  }

  /// 每个控制拍调用: 返回本拍对齐目标 (无则 nullptr, 上层按目标丢失处理)。
  /// 返回指针指向 detections 内部, 仅本拍有效
  const Detection * pick(double dt, const std::vector<Detection> & detections)
  {
    if (detections.empty()) {
      noteMissed(dt);
      return nullptr;
    }
    // 指定 ID 优先: 出现即切换对齐目标
    if (p_.target_id >= 0) {
      const Detection * m = largestOfId(p_.target_id, detections);
      if (m != nullptr) {
        if (tracked_id_ != p_.target_id) {
          RCLCPP_INFO(
            logger_, "\033[32m对齐目标切换: id=%d (指定目标出现%s)\033[0m",
            p_.target_id, tracked_id_ >= 0 ? ", 从回退码接管" : "");
        }
        tracked_id_ = p_.target_id;
        lost_time_ = 0.0;
        return m;
      }
      if (!p_.fallback) {
        noteMissed(dt);
        return nullptr;  // 严格模式: 只认指定 ID
      }
      // 指定 ID 未出现: 回退跟踪画面最大码 (软锁定, 防跳变)
    }
    // 锁定目标仍在画面: 直接跟它 (指定模式下 tracked_id_ 已是回退码)
    if (tracked_id_ >= 0 && tracked_id_ != p_.target_id) {
      const Detection * m = largestOfId(tracked_id_, detections);
      if (m != nullptr) {
        lost_time_ = 0.0;
        return m;
      }
      // 锁定目标本帧缺失: 短暂丢失 (检测抖动/遮挡) 仍认锁定目标, 悬停等待
      noteMissed(dt);
      if (lost_time_ <= p_.reacquire_sec) {
        return nullptr;
      }
      RCLCPP_INFO(
        logger_, "锁定目标 id=%d 持续丢失 %.1fs, 重新选择", tracked_id_, lost_time_);
      tracked_id_ = -1;
    }
    // (重新)选择: 取画面里面积最大的码并锁定
    const Detection * best = nullptr;
    double best_area = -1.0;
    for (const auto & d : detections) {
      const double a = d.bbox_wh.x * d.bbox_wh.y;
      if (a > best_area) {
        best_area = a;
        best = &d;
      }
    }
    if (best != nullptr) {
      if (p_.target_id >= 0) {
        RCLCPP_INFO(
          logger_, "\033[33m指定目标 id=%d 不可见, 回退画面最大码 id=%d "
          "(共检出 %zu 个码)\033[0m", p_.target_id, best->id, detections.size());
      } else {
        RCLCPP_INFO(
          logger_, "\033[32m选定跟踪目标: id=%d (画面最大, 共检出 %zu 个码, "
          "此后锁定该 ID)\033[0m", best->id, detections.size());
      }
      tracked_id_ = best->id;
      lost_time_ = 0.0;
    }
    return best;
  }

private:
  void noteMissed(double dt)
  {
    if (tracked_id_ >= 0) {
      lost_time_ += dt;
    }
  }

  rclcpp::Logger logger_ = rclcpp::get_logger("target_selector");
  Params p_;
  int tracked_id_ = -1;      // 锁定的目标码 ID (-1 = 未锁定)
  double lost_time_ = 0.0;   // 锁定目标持续丢失时长 (秒)
};

}  // namespace flylab_vision

#endif  // FLYLAB_VISION__ARUCO__TARGET_SELECTOR_HPP_
