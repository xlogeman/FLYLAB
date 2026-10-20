// Copyright 2026 flylab
#ifndef PX4_OFFBOARD_CLIENT_MAVROS_HPP_
#define PX4_OFFBOARD_CLIENT_MAVROS_HPP_

/// PX4 offboard 执行层 —— MAVROS 链路实现 (与 flylab/tracker_mavros 同一套
/// 话题与服务, 可共用同一 MAVROS 桥, fcu_url:=udp://:14542@ 专属实例)。
///
///   - 状态: /mavros/state; 位置: /mavros/local_position/pose
///   - setpoint: /mavros/setpoint_raw/local (PositionTarget, ENU 语义,
///     插件内部负责 ENU->NED 转换; 不发额外心跳——持续 setpoint 流即心跳)
///   - 模式切换: /mavros/set_mode 服务 (OFFBOARD / AUTO.LAND / AUTO.LOITER)

#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

#include "px4_offboard_client_base.hpp"

namespace flylab_vision
{

class Px4OffboardClientMavros : public Px4OffboardClientBase
{
public:
  explicit Px4OffboardClientMavros(rclcpp::Node & node)
  : node_(node)
  {
    setpoint_pub_ = node_.create_publisher<mavros_msgs::msg::PositionTarget>(
      "/mavros/setpoint_raw/local", 10);

    state_sub_ = node_.create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10,
      [this](const mavros_msgs::msg::State::SharedPtr msg) {
        const bool first_connected = msg->connected && !state_.connected;
        const bool mode_changed = state_.connected && msg->mode != state_.mode;
        state_ = *msg;
        last_update_ = node_.now();
        if (first_connected) {
          got_status_ = true;
          RCLCPP_INFO(
            node_.get_logger(), "\033[32m已连接 PX4 (MAVROS 链路, 当前模式: %s, %s)\033[0m",
            state_.mode.c_str(), state_.armed ? "已解锁" : "未解锁");
        }
        // 模式切换跟踪: 任何模式变化 (含 failsafe 接管) 都打印, 链路问题一眼可见
        if (mode_changed) {
          RCLCPP_INFO(
            node_.get_logger(), "\033[36m飞控模式切换: %s -> %s\033[0m",
            last_mode_.c_str(), state_.mode.c_str());
        }
        last_mode_ = state_.mode;
      });
    pose_sub_ = node_.create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_pose_ = *msg;
        got_pose_ = true;
        last_update_ = node_.now();
      });

    set_mode_client_ = node_.create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
  }

  // ---------------- 状态查询 ----------------
  bool gotStatus() const override {return got_status_;}
  bool armed() const override {return state_.armed;}
  bool isOffboard() const override {return state_.mode == "OFFBOARD";}
  bool isAutoLand() const override {return state_.mode == "AUTO.LAND";}
  const std::string & currentModeName() const {return state_.mode;}

  /// 最近一次飞控数据 (state 或 pose) 距现在是否仍在 timeout_sec 内 (断链判定)
  bool linkFresh(double timeout_sec) const override
  {
    return got_status_ && (node_.now() - last_update_).seconds() <= timeout_sec;
  }

  LocalPosNED localPosition() const override
  {
    LocalPosNED p;
    if (!got_pose_) {
      return p;
    }
    p.xy_valid = p.z_valid = true;
    p.x = current_pose_.pose.position.y;   // ENU -> NED: x_NED = y_ENU
    p.y = current_pose_.pose.position.x;   //            y_NED = x_ENU
    p.z = -current_pose_.pose.position.z;  //            z_NED = -z_ENU
    return p;
  }

  // ---------------- 控制指令 ----------------
  /// 机体系 FRD 速度 (x前 y右 z下) -> FLU 机体系 -> 按当前 yaw 旋到 ENU map 系,
  /// 经 /mavros/setpoint_raw/local 下发 (MAVROS 插件内部 ENU->NED)。
  /// 持续 setpoint 流即 offboard 心跳 (与 tracker_mavros 一致, 无需额外消息)
  void sendVelocityFRD(double fwd, double right, double down) override
  {
    const double bx = fwd;          // FRD x 前 = FLU x 前
    const double by = -right;       // FRD y 右 = FLU -y
    const double bz = -down;        // FRD z 下 = FLU -z
    const double yaw = currentYaw();
    const double c = std::cos(yaw), s = std::sin(yaw);

    mavros_msgs::msg::PositionTarget sp;
    sp.header.stamp = node_.now();
    sp.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    sp.type_mask = kIgnorePosAcc | mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
    sp.velocity.x = c * bx - s * by;
    sp.velocity.y = s * bx + c * by;
    sp.velocity.z = bz;
    sp.yaw = yaw;                   // 保持当前机头
    setpoint_pub_->publish(sp);
  }

  /// 原地悬停: 定点 (位置+yaw, 与 tracker_mavros 悬停保护一致); 无定位时零速度兜底
  void sendHover() override
  {
    mavros_msgs::msg::PositionTarget sp;
    sp.header.stamp = node_.now();
    sp.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    if (got_pose_) {
      sp.type_mask = kIgnoreVelAcc | mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
      sp.position = current_pose_.pose.position;
      sp.yaw = currentYaw();
    } else {
      sp.type_mask = kIgnorePosAcc | mavros_msgs::msg::PositionTarget::IGNORE_YAW |
        mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
      sp.velocity.x = 0.0f;
      sp.velocity.y = 0.0f;
      sp.velocity.z = 0.0f;
    }
    setpoint_pub_->publish(sp);
  }

  void requestOffboard() override {requestMode("OFFBOARD");}
  void requestAutoLand() override {requestMode("AUTO.LAND");}
  void requestLoiter() override {requestMode("AUTO.LOITER");}

  // ---------------- offboard 接管状态机 (与 uXRCE 版同逻辑) ----------------
  void takeoverReset(const rclcpp::Time & now) override
  {
    takeover_start_ = now;
    last_request_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    attempts_ = 0;
    engaged_logged_ = false;
  }

  Takeover takeoverTick(const rclcpp::Time & now, double warmup_sec, double retry_sec) override
  {
    if (isOffboard()) {
      if (!engaged_logged_) {
        engaged_logged_ = true;
        RCLCPP_INFO(
          node_.get_logger(), "\033[32mOFFBOARD 接管成功 (尝试 %u 次, 耗时 %.1fs)\033[0m",
          attempts_, (now - takeover_start_).seconds());
      }
      return Takeover::ENGAGED;
    }
    sendHover();   // 预热/等待期间持续发悬停 setpoint (满足切换条件又保持姿态稳定)
    const double elapsed = (now - takeover_start_).seconds();
    if (elapsed > warmup_sec && (now - last_request_).seconds() > 0.5) {
      attempts_++;
      RCLCPP_INFO(
        node_.get_logger(), "请求进入 OFFBOARD (第 %u 次): 当前模式=%s, "
        "已预热 %.1fs, 悬停 setpoint 持续发送中, 等待 /mavros/set_mode 服务确认 ...",
        attempts_, currentModeName().c_str(), elapsed);
      requestOffboard();
      last_request_ = now;
    }
    if (elapsed > warmup_sec + retry_sec) {
      RCLCPP_WARN(
        node_.get_logger(), "无法进入 OFFBOARD (检查 MAVROS / 是否有其他链路占用 setpoint)");
      return Takeover::FAILED;
    }
    return Takeover::REQUESTING;
  }

private:
  /// 模式切换: /mavros/set_mode 服务 (custom_mode 字符串, 与 tracker_mavros 一致)
  void requestMode(const std::string & mode)
  {
    if (!set_mode_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 2000, "/mavros/set_mode 服务未就绪");
      return;
    }
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = mode;
    set_mode_client_->async_send_request(request,
      [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
        if (!future.get()->mode_sent) {
          RCLCPP_WARN(node_.get_logger(), "切换到 %s 模式请求被拒", mode.c_str());
        }
      });
  }

  double currentYaw() const
  {
    if (!got_pose_) {
      return 0.0;
    }
    const auto & q = current_pose_.pose.orientation;
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  // setpoint type_mask 常用组合
  static constexpr uint16_t kIgnorePosAcc =
    mavros_msgs::msg::PositionTarget::IGNORE_PX | mavros_msgs::msg::PositionTarget::IGNORE_PY |
    mavros_msgs::msg::PositionTarget::IGNORE_PZ | mavros_msgs::msg::PositionTarget::IGNORE_AFX |
    mavros_msgs::msg::PositionTarget::IGNORE_AFY | mavros_msgs::msg::PositionTarget::IGNORE_AFZ;
  static constexpr uint16_t kIgnoreVelAcc =
    mavros_msgs::msg::PositionTarget::IGNORE_VX | mavros_msgs::msg::PositionTarget::IGNORE_VY |
    mavros_msgs::msg::PositionTarget::IGNORE_VZ | mavros_msgs::msg::PositionTarget::IGNORE_AFX |
    mavros_msgs::msg::PositionTarget::IGNORE_AFY | mavros_msgs::msg::PositionTarget::IGNORE_AFZ;

  rclcpp::Node & node_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;

  mavros_msgs::msg::State state_;
  geometry_msgs::msg::PoseStamped current_pose_;   // 当前位置 (ENU)
  std::string last_mode_;
  bool got_status_ = false;
  bool got_pose_ = false;
  rclcpp::Time last_update_{0, 0, RCL_ROS_TIME};   // state/pose 任一到达即刷新 (断链判定)

  // 接管状态机
  rclcpp::Time takeover_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_request_{0, 0, RCL_ROS_TIME};
  uint32_t attempts_ = 0;
  bool engaged_logged_ = false;
};

}  // namespace flylab_vision

#endif  // PX4_OFFBOARD_CLIENT_MAVROS_HPP_
