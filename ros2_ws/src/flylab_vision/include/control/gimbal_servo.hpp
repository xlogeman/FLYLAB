// Copyright 2026 flylab
#ifndef FLYLAB_VISION__CONTROL__GIMBAL_SERVO_HPP_
#define FLYLAB_VISION__CONTROL__GIMBAL_SERVO_HPP_

#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <utility>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace flylab_vision
{

/// 云台视觉伺服 (目标追踪 -> 软云台, 让追踪目标保持在画面中央)。
///
/// 通道约定与 aruco_landing_node 的算法云台通道一致 (flylab_gcs/config.py):
/// 发布 /flylab/gimbal_cmd [yaw_rate, pitch_rate] (°/s), 地面站软云台按
/// 速率持续步进 (保持型指令 —— 停止/丢失/断流必须显式清零, 否则云台
/// 会按最后速率一直转, 见 set_enabled / lost_zero / tick 的兜底)。
/// 只动云台不动飞机: 不接 offboard, 无需通信链路与升空。
///
/// 控制律 (PD, 输入像素误差): 目标框中心相对画面中心的像素偏移经 FOV
/// 换算成角度误差 (小角度 atan, 水平/垂直共用一个焦距像素), 死区内
/// 归零, PD 输出夹紧到限速。方向约定 (与地面站软云台一致):
///   图像 x 右+  -> 云台偏航右+ (yaw_rate 同号)
///   图像 y 下+  -> 云台俯仰向下 (pitch_rate 取负号: pitch+ 为抬头)
///
/// FOV 经 /flylab/camera_fov (latched) 与地面站同步, 未收到前用参数
/// 默认值 (水平视场角, 与 AirSim simGetCameraInfo 同义)。
///
/// 线程约定: update() 在帧上下文 (proc_group_), set_enabled 在指令回调
/// (默认组), tick 在定时器 (默认组) —— MultiThreadedExecutor 下并发,
/// 状态统一加锁。发布即低频小消息, 锁内直发无碍。
class GimbalServo
{
public:
  explicit GimbalServo(rclcpp::Node & node)
  {
    const auto kp = node.declare_parameter<double>("servo_kp", 3.0);
    const auto kd = node.declare_parameter<double>("servo_kd", 0.6);
    const auto max_rate = node.declare_parameter<double>(
      "servo_max_rate_deg", 45.0);
    const auto deadband = node.declare_parameter<double>(
      "servo_deadband_deg", 1.0);
    const auto fov_default = node.declare_parameter<double>(
      "servo_fov_deg", 90.0);
    const auto cmd_topic = node.declare_parameter<std::string>(
      "gimbal_cmd_topic", "/flylab/gimbal_cmd");
    const auto fov_topic = node.declare_parameter<std::string>(
      "camera_fov_topic", "/flylab/camera_fov");

    kp_ = kp;
    kd_ = kd;
    max_rate_ = max_rate;
    deadband_ = deadband;
    fov_ = fov_default;

    pub_ = node.create_publisher<std_msgs::msg::Float64MultiArray>(
      cmd_topic, 10);
    // FOV latched 同步 (与地面站发布端 QoS 对齐: reliable + transient_local),
    // 后启动的节点一接入即拿到当前视场角
    fov_sub_ = node.create_subscription<std_msgs::msg::Float64>(
      fov_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        if (msg->data > 10.0 && msg->data < 170.0) {
          fov_ = msg->data;
        }
      });
    // 兜底心跳: 丢帧兜底 (帧断流后清零速率防云台漂移) + 丢包重发保活
    timer_ = node.create_wall_timer(
      std::chrono::milliseconds(200), [this] {tick();});

    RCLCPP_INFO(
      node.get_logger(),
      "云台伺服就绪: KP %.1f KD %.1f 限速 %.0f°/s 死区 %.1f° "
      "(指令经 %s, FOV 同步自 %s)",
      kp_, kd_, max_rate_, deadband_, cmd_topic.c_str(), fov_topic.c_str());
  }

  bool enabled() const {return enabled_.load();}

  /// 接管控制菜单开/关 (vision_cmd track_gimbal_start/stop)。
  /// 失能立即清零速率: 保持型指令不清零, 云台会按最后速率一直转。
  void set_enabled(bool on)
  {
    bool prev = enabled_.exchange(on);
    if (prev == on) {
      return;
    }
    std::lock_guard<std::mutex> lk(mutex_);
    if (!on && (cmd_.first != 0.0 || cmd_.second != 0.0)) {
      publish_locked(0.0, 0.0);
    }
  }

  /// 帧上下文伺服一步 (调用方保证 frame 非空): box 为追踪目标框
  /// (显示帧像素), has_target=false 视为目标丢失 (清零速率)。
  void update(const cv::Rect2f & box, const cv::Mat & frame, bool has_target)
  {
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lk(mutex_);
    frame_t_ = now;
    if (!enabled_.load()) {
      return;
    }
    if (!has_target || frame.cols < 2 || frame.rows < 2) {
      lost_zero_locked();
      return;
    }
    // 像素误差 -> 角度误差 (焦距像素 = (W/2)/tan(hfov/2), 双轴共用)
    const double W = frame.cols, H = frame.rows;
    const double ex = box.x + box.width * 0.5 - W * 0.5;    // 右+
    const double ey = box.y + box.height * 0.5 - H * 0.5;   // 下+
    const double focal =
      (W * 0.5) / std::tan(fov_ * 0.5 * CV_PI / 180.0);
    const double ax = std::atan(ex / focal) * 180.0 / CV_PI;
    const double ay = std::atan(ey / focal) * 180.0 / CV_PI;
    // PD: 丢失后重捕获不做差分 (dt 失真), 只走 P
    const double dt = was_target_ ?
      std::chrono::duration<double>(now - ctrl_t_).count() : 0.0;
    const double dx = was_target_ ? ax - ax_prev_ : 0.0;
    const double dy = was_target_ ? ay - ay_prev_ : 0.0;
    ax_prev_ = ax;
    ay_prev_ = ay;
    was_target_ = true;
    ctrl_t_ = now;
    const double rx = clamp_rate(pd(ax, dx, dt));
    const double ry = clamp_rate(pd(ay, dy, dt));
    publish_locked(rx, -ry);   // 图像 y 下+ = 俯仰向下 (pitch+ 为抬头, 取负)
  }

private:
  double pd(double err, double derr, double dt) const
  {
    // 死区: 角度误差在此内视为已对准 (悬停抖动不触发无限微转)
    if (std::fabs(err) < deadband_) {
      return 0.0;
    }
    const double d = dt > 1e-3 ? kd_ * derr / dt : 0.0;
    return kp_ * err + d;
  }

  double clamp_rate(double rate) const
  {
    return std::max(-max_rate_, std::min(max_rate_, rate));
  }

  /// 目标丢失: 边沿触发清零 (速率是保持型, 清一次即停)
  void lost_zero_locked()
  {
    was_target_ = false;
    if (cmd_.first != 0.0 || cmd_.second != 0.0) {
      publish_locked(0.0, 0.0);
    }
  }

  void publish_locked(double yaw_rate, double pitch_rate)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {yaw_rate, pitch_rate};
    pub_->publish(msg);
    cmd_ = {yaw_rate, pitch_rate};
  }

  /// 兜底心跳 (200ms): 图像断流 (地面站关/链路卡死) 时帧更新停止,
  /// 保持型速率指令不清零云台会一直转 —— 断流即清零; 帧正常时重发
  /// 当前速率作丢包保活 (小消息, 开销可忽略)
  void tick()
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!enabled_.load()) {
      return;
    }
    const bool stale = frame_t_ == std::chrono::steady_clock::time_point{} ||
      std::chrono::steady_clock::now() - frame_t_ > std::chrono::milliseconds(500);
    if (stale) {
      lost_zero_locked();
      return;
    }
    publish_locked(cmd_.first, cmd_.second);
  }

  // ---------- 参数 (构造后只读) ----------
  double kp_ = 3.0;            // P 增益 (1/s): 角度误差 × KP = 角速率
  double kd_ = 0.6;            // D 增益: 误差变化率阻尼
  double max_rate_ = 45.0;     // 角速率限幅 (°/s, 对齐摇杆满偏量级)
  double deadband_ = 1.0;      // 对准死区 (度)

  std::atomic<bool> enabled_{false};
  mutable std::mutex mutex_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr fov_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  // ---------- 受 mutex_ 保护的状态 ----------
  double fov_ = 90.0;          // 当前水平视场角 (度, latched 同步)
  std::pair<double, double> cmd_{0.0, 0.0};   // 最近下发的速率指令
  bool was_target_ = false;    // 上一帧是否有目标 (差分有效性)
  double ax_prev_ = 0.0, ay_prev_ = 0.0;      // 上一帧角度误差 (D 项)
  std::chrono::steady_clock::time_point ctrl_t_;   // 上一帧控制时刻
  std::chrono::steady_clock::time_point frame_t_;  // 最近一帧到达时刻
};

}  // namespace flylab_vision

#endif  // FLYLAB_VISION__CONTROL__GIMBAL_SERVO_HPP_
