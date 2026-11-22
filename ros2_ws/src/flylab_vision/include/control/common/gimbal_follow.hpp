// Copyright 2026 flylab
/// 机头跟随通道 (相机侧反卷) —— aruco 降落 / 目标追踪反控共用组件
#ifndef FLYLAB_VISION__CONTROL__COMMON__GIMBAL_FOLLOW_HPP_
#define FLYLAB_VISION__CONTROL__COMMON__GIMBAL_FOLLOW_HPP_

#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

namespace flylab_vision
{

/// 机头跟随通道封装 (地面站相机侧反卷, 世界锚定 —— 机体转动时相对偏航收敛):
///   请求方向: 发布 /flylab/gimbal_follow (Bool) 请求地面站开/关反卷。
///     开启按 0.5s 保活重发 —— 地面站侧跟随可能被悄悄退出 (gimbal_cmd
///     心跳超时/抓取线程重连复位), 节点无感知, 重发自愈失步; 关闭边沿
///     触发只发一次 (地面站断流超时自退兜底)。
///   误差方向: 订阅 /flylab/gimbal_yaw_err (latched, 后启动即拿当前值;
///     反卷后的云台偏航 = 视线世界方位 − 航向, 右正; NaN = 未跟随/测量
///     过期, 地面站 1s 保活重发), 新鲜度经 err_usable() 判定。
///
/// 与机头的接口是注入偏航角速率 (offboard 单写者原则: 反控期间偏航指令
/// 由算法节点下发, 地面站只做相机侧反卷与误差广播); 通道约定与话题名
/// 两个使用方一致。
///
/// 线程约定: 订阅回调与 request() 调用方在同一回调组串行 (两个使用方
/// 均为默认组), 状态无需加锁。
class GimbalFollow
{
public:
  GimbalFollow(
    rclcpp::Node & node, const std::string & req_topic,
    const std::string & err_topic)
  : node_(node)
  {
    req_pub_ = node.create_publisher<std_msgs::msg::Bool>(req_topic, 10);
    err_sub_ = node.create_subscription<std_msgs::msg::Float64>(
      err_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        err_deg_ = msg->data;
        err_t_ = node_.now();
      });
  }

  /// 跟随请求 (语义见类注释: 开启保活重发, 关闭边沿触发一次)
  void request(bool on)
  {
    const rclcpp::Time t = node_.now();
    const bool changed = (on != requested_);
    const bool keep = (t - req_t_).seconds() >= kKeepaliveSec;
    if (!changed && !(on && keep)) {
      return;
    }
    std_msgs::msg::Bool msg;
    msg.data = on;
    req_pub_->publish(msg);
    requested_ = on;
    req_t_ = t;
  }

  /// 复位请求状态 (流程重启): 只清本地状态不发关闭 —— 上次运行遗留的
  /// 跟随由地面站心跳超时自退
  void reset_request() {requested_ = false;}

  /// 当前是否处于跟随请求中 (调用方保活/联动判定用)
  bool requested() const {return requested_;}

  /// 最近一次广播的跟随误差原始值 (度, 右正; NaN = 未跟随/测量过期)
  double yaw_err_deg() const {return err_deg_;}

  /// 误差是否可用 (非 NaN 且在 fresh_sec 新鲜度内; 地面站 1s 保活 + 裕量)
  bool err_usable(double fresh_sec) const
  {
    return std::isfinite(err_deg_) &&
           (node_.now() - err_t_).seconds() <= fresh_sec;
  }

private:
  static constexpr double kKeepaliveSec = 0.5;  // 请求保活重发周期 (秒)

  rclcpp::Node & node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr req_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr err_sub_;
  bool requested_ = false;                  // 最近发布的请求状态 (去重+保活判定)
  rclcpp::Time req_t_{0, 0, RCL_ROS_TIME};  // 最近请求发布时刻 (保活节流)
  double err_deg_ = std::nan("");           // 最近广播的跟随误差 (NaN=无效)
  rclcpp::Time err_t_{0, 0, RCL_ROS_TIME};  // 误差到达时刻 (新鲜度判定)
};

}  // namespace flylab_vision

#endif  // FLYLAB_VISION__CONTROL__COMMON__GIMBAL_FOLLOW_HPP_
