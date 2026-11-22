// Copyright 2026 flylab
#ifndef FLYLAB_VISION__CONTROL__TRACK_TAKEOVER_HPP_
#define FLYLAB_VISION__CONTROL__TRACK_TAKEOVER_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include "control/px4_offboard_client.hpp"

namespace flylab_vision
{

/// 目标追踪反控接管层 (yolo_track / ortrack 共用): 接管 offboard 拿到飞机
/// 控制权 —— 原地悬停 + 机头跟随 (视线方位收敛), 云台伺服 (GimbalServo)
/// 继续由帧上下文驱动让追踪目标保持在画面中央, 即"悬停跟拍"形态;
/// 飞机跟着目标飞的速度环在 tickEngaged 里扩展 (现悬停占位)。
///
/// 与 aruco_landing_node 同一套组件与流程 (px4_offboard_client, offboard
/// 单写者原则 —— 接管期间偏航指令由本节点注入, 地面站遥控必须退出):
///   激活: takeoverReset → 定时 tick 跑接管状态机 (心跳预热 → 周期请求 →
///         确认), ENGAGED 后持续 sendVelocityFRDYawRate(0,0,0,yaw) 维持
///         offboard 心跳 (速度全零 = 原地悬停, yaw 为机头跟随)
///   交还: requestLoiter 切回 AUTO.LOITER; 上锁 / OFFBOARD 被外部切换 /
///         图像断流 / 接管超时 自动交还并回 idle
/// 机头跟随: 发布 /flylab/gimbal_follow 请求相机侧反卷 (世界系锚点,
/// 0.5s 保活), 订阅 /flylab/gimbal_yaw_err (latched) 按误差 P 律注入偏航
/// 角速率 —— 与 aruco_landing 同一通道约定; 可经 /flylab/track_follow
/// (latched Bool, 地面站云台面板"跟随"按钮) 开关 —— 关闭时机头保持
/// 航向仅注入零偏航, 云台伺服 (相机侧反卷) 不受影响, 目标仍保持画面
/// 中央, 云台偏航被 ±45° 限位收口。
/// 状态回传 /flylab/track_status (engaged/idle/failed), 地面站据此联动按钮。
///
/// 线程约定: activate/deactivate 在指令回调 (默认组), tick 在定时器
/// (默认组) —— 同组串行, 状态无需加锁。
class TrackTakeover
{
public:
  explicit TrackTakeover(rclcpp::Node & node)
  : node_(node)
  {
    // ---------- 参数 ----------
    const auto link = node.declare_parameter<std::string>("link", "mavros");
    const auto control_hz = node.declare_parameter<double>("takeover_hz", 20.0);
    const auto warmup =
      node.declare_parameter<double>("offboard_warmup_sec", 1.0);
    const auto retry =
      node.declare_parameter<double>("offboard_retry_sec", 10.0);
    const auto follow_kp = node.declare_parameter<double>("follow_kp", 1.2);
    const auto follow_rate_max = node.declare_parameter<double>(
      "follow_yaw_rate_max_deg", 45.0);
    const auto follow_deadband = node.declare_parameter<double>(
      "follow_deadband_deg", 2.0);
    const auto follow_slow = node.declare_parameter<double>(
      "follow_slow_deg", 30.0);
    frame_timeout_sec_ = node.declare_parameter<double>("frame_timeout_sec", 2.0);
    warmup_sec_ = warmup;
    retry_sec_ = retry;
    follow_kp_ = follow_kp;
    follow_rate_max_ = follow_rate_max;
    follow_deadband_deg_ = follow_deadband;
    follow_slow_deg_ = follow_slow;

    // ---------- PX4 执行层 (复用组件, 链路按 link 参数选择) ----------
    if (link == "mavros") {
      px4_ = std::make_unique<Px4OffboardClientMavros>(node);
      RCLCPP_INFO(
        node.get_logger(),
        "反控 PX4 执行链路: MAVROS (/mavros/*, fcu_url:=udp://:14542@ 专属实例)");
    } else if (link == "uxrce") {
      px4_ = std::make_unique<Px4OffboardClientUxrce>(node);
      RCLCPP_INFO(
        node.get_logger(),
        "反控 PX4 执行链路: uXRCE-DDS (/fmu/*, 需 MicroXRCEAgent udp4 -p 8888)");
    } else {
      RCLCPP_FATAL(
        node.get_logger(), "未知 link 参数 '%s' (可选 mavros|uxrce)",
        link.c_str());
      throw std::invalid_argument("link must be mavros or uxrce");
    }

    // ---------- 地面站通道 (状态回传 / 机头跟随, 约定同 aruco_landing) ----------
    const auto status_topic = node.declare_parameter<std::string>(
      "status_topic", "/flylab/track_status");
    const auto follow_req_topic = node.declare_parameter<std::string>(
      "follow_req_topic", "/flylab/gimbal_follow");
    const auto follow_err_topic = node.declare_parameter<std::string>(
      "follow_err_topic", "/flylab/gimbal_yaw_err");
    status_pub_ =
      node.create_publisher<std_msgs::msg::String>(status_topic, 10);
    follow_req_pub_ =
      node.create_publisher<std_msgs::msg::Bool>(follow_req_topic, 10);
    follow_err_sub_ = node.create_subscription<std_msgs::msg::Float64>(
      follow_err_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        yaw_err_deg_ = msg->data;   // NaN = 未跟随/测量过期
      });
    // 机头跟随开关 (地面站云台面板"跟随"按钮, latched): 默认开 —— 与
    // 地面站未发布时的旧行为一致; 关闭仅停偏航注入, 云台伺服照常
    const auto follow_switch_topic = node.declare_parameter<std::string>(
      "follow_switch_topic", "/flylab/track_follow");
    follow_switch_sub_ = node.create_subscription<std_msgs::msg::Bool>(
      follow_switch_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (nose_follow_ == msg->data) {
          return;
        }
        nose_follow_ = msg->data;
        RCLCPP_INFO(
          node_.get_logger(), "机头跟随: %s",
          nose_follow_ ? "\033[32m开启 (机头转向视线方向)\033[0m"
                       : "\033[33m关闭 (机头保持航向, 仅云台伺服跟目标)\033[0m");
      });

    // ---------- 控制主循环 (固定频率, 不随图像帧率波动) ----------
    timer_ = node.create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, control_hz)),
      [this] {tick();});
  }

  /// 激活反控 (vision_cmd track_takeover_start): 幂等, 重复激活无副作用
  void activate()
  {
    if (active_) {
      return;
    }
    active_ = true;
    ever_engaged_ = false;
    px4_->takeoverReset(node_.now());
    notify_frame();
    RCLCPP_INFO(
      node_.get_logger(),
      "\033[32m目标追踪反控: 接管 offboard, 预热 %.1fs 后切模式 "
      "(悬停跟拍 + 机头跟随)\033[0m", warmup_sec_);
  }

  /// 交还反控 (vision_cmd track_takeover_stop / 异常自愈): 幂等
  void deactivate(const char * reason)
  {
    if (!active_) {
      return;
    }
    active_ = false;
    if (ever_engaged_ && px4_->armed() && px4_->isOffboard()) {
      RCLCPP_INFO(
        node_.get_logger(), "目标追踪反控结束 (%s), 切回 AUTO_LOITER 交还控制权",
        reason);
      px4_->requestLoiter();
    } else {
      RCLCPP_INFO(node_.get_logger(), "目标追踪反控结束 (%s)", reason);
    }
    ever_engaged_ = false;
    request_follow(false);
    publish_status("idle");
  }

  /// 帧上下文每帧调用 (process 开头): 喂图像断流看门狗
  void notify_frame()
  {
    frame_t_ = std::chrono::steady_clock::now();
  }

  bool active() const {return active_;}

private:
  void tick()
  {
    if (!active_) {
      return;
    }
    // 图像断流自动交还: 算法停/GCS 关时没人发停令, 悬停心跳与云台
    // 伺服都已失依, 交还控制权比维持盲飞安全
    if (std::chrono::steady_clock::now() - frame_t_ >
        std::chrono::duration<double>(frame_timeout_sec_))
    {
      deactivate("图像断流");
      return;
    }
    if (!px4_->gotStatus()) {
      return;   // 链路未就绪, 等 takeoverTick 内部重试
    }
    if (!px4_->armed()) {
      deactivate("飞机已上锁");
      return;
    }
    // 外部切模式 = 有人抢走控制权 (QGC/遥控器), 交还退出
    if (ever_engaged_ && !px4_->isOffboard()) {
      RCLCPP_WARN(
        node_.get_logger(),
        "\033[33mOFFBOARD 被外部切换 (当前模式: %s), 判定为外部接管, "
        "交还控制权并退出反控\033[0m", px4_->modeName().c_str());
      deactivate("外部接管");
      return;
    }
    if (!ever_engaged_) {
      // 接管中: PX4 要求先收到 setpoint 流才接受切模式, takeoverTick
      // 内部已在发心跳+悬停
      const auto res =
        px4_->takeoverTick(node_.now(), warmup_sec_, retry_sec_);
      if (res == Px4OffboardClientBase::Takeover::ENGAGED) {
        ever_engaged_ = true;
        publish_status("engaged");
        RCLCPP_INFO(
          node_.get_logger(),
          "\033[32m已接管 offboard: 悬停跟拍中 (云台伺服保持目标居中)\033[0m");
      } else if (res == Px4OffboardClientBase::Takeover::FAILED) {
        publish_status("failed");
        deactivate("接管超时");
      }
      return;
    }
    // 已接管: 机头跟随 + 悬停 (速度全零), offboard 心跳随 setpoint 持续发
    request_follow(true);
    px4_->sendVelocityFRDYawRate(0.0, 0.0, 0.0, follow_yaw_rate());
  }

  /// 机头跟随偏航角速率 (rad/s, NED 正=右转): 误差 (度, 视线在机头右侧
  /// 为正) × KP, 与转向同号; 开关关闭 / NaN (未跟随/过期) / 死区内为 0。
  /// 收敛段两条抑摆措施 (误差链路有图像+云台伺服延迟, 纯 P 律在零附近
  /// 极限环 = 左右摇摆反复对准):
  ///   1. 死区带回差: |err| < deadband 停, 须偏离 1.5× deadband 才重启
  ///      (回差防测量噪声在死区边界抖动)
  ///   2. 中心减速段: |err| < slow_deg 时增益随误差线性衰减, 越接近
  ///      对准转得越慢, 平滑收敛不冲过; 超过后恢复线性全增益
  double follow_yaw_rate()
  {
    if (!nose_follow_ || std::isnan(yaw_err_deg_)) {
      return 0.0;
    }
    const double err = yaw_err_deg_;
    const double abs_err = std::fabs(err);
    if (in_deadband_) {
      if (abs_err < follow_deadband_deg_ * 1.5) {
        return 0.0;
      }
      in_deadband_ = false;
    } else if (abs_err <= follow_deadband_deg_) {
      in_deadband_ = true;
      return 0.0;
    }
    const double u = std::min(1.0, abs_err / follow_slow_deg_);
    const double rate_deg = std::clamp(
      follow_kp_ * err * u, -follow_rate_max_, follow_rate_max_);
    return rate_deg * M_PI / 180.0;
  }

  /// 机头跟随请求 (相机侧反卷): 开启 0.5s 保活重发 (自愈地面站侧悄悄
  /// 退出的失步), 关闭只发一次
  void request_follow(bool on)
  {
    const auto now = std::chrono::steady_clock::now();
    if (on) {
      if (follow_on_ && now - follow_req_t_ <
          std::chrono::milliseconds(500))
      {
        return;
      }
      follow_req_t_ = now;
    } else if (!follow_on_) {
      return;
    }
    follow_on_ = on;
    std_msgs::msg::Bool m;
    m.data = on;
    follow_req_pub_->publish(m);
  }

  void publish_status(const char * status)
  {
    std_msgs::msg::String m;
    m.data = status;
    status_pub_->publish(m);
  }

  rclcpp::Node & node_;
  std::unique_ptr<Px4OffboardClientBase> px4_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr follow_req_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr follow_err_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr follow_switch_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool active_ = false;
  bool ever_engaged_ = false;        // 曾确认进入 offboard (外部切换判定)
  bool nose_follow_ = true;          // 机头跟随开关 (/flylab/track_follow)
  bool follow_on_ = false;
  double yaw_err_deg_ = std::nan("");
  std::chrono::steady_clock::time_point follow_req_t_;
  std::chrono::steady_clock::time_point frame_t_;
  double warmup_sec_ = 1.0;
  double retry_sec_ = 10.0;
  double follow_kp_ = 1.2;
  double follow_rate_max_ = 45.0;
  double follow_deadband_deg_ = 2.0;   // 对准死区 (度, 回差 1.5×)
  double follow_slow_deg_ = 30.0;      // 中心减速段宽度 (度)
  bool in_deadband_ = false;           // 死区状态 (回差迟滞)
  double frame_timeout_sec_ = 2.0;
};

}  // namespace flylab_vision

#endif  // FLYLAB_VISION__CONTROL__TRACK_TAKEOVER_HPP_
