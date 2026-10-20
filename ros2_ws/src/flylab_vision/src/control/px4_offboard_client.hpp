// Copyright 2026 flylab
#ifndef PX4_OFFBOARD_CLIENT_HPP_
#define PX4_OFFBOARD_CLIENT_HPP_

/// PX4 offboard 执行层 (uXRCE-DDS) —— 可复用组件, 供 flylab_vision/src/control
/// 下所有"通过 PX4 控制飞机"的算法节点使用 (视觉降落 / 目标追踪控制 / ...)。
///
/// 封装内容:
///   - /fmu/in 三个发布者: offboard_control_mode / trajectory_setpoint / vehicle_command
///     (QoS 与 flylab/tracker_px4 一致: best_effort + transient_local)
///   - /fmu/out 三个订阅者: vehicle_status / vehicle_local_position / vehicle_command_ack
///   - 诊断日志: 链路建立、模式切换跟踪、failsafe 告警、切模式 ACK 结果
///   - 常用指令: 机体系 FRD 速度 setpoint (自动按 heading 旋到 NED)、悬停、
///     切 OFFBOARD / AUTO.LAND / AUTO.LOITER
///   - offboard 接管状态机 takeoverTick(): 心跳预热 → 周期请求 → 成功/超时,
///     任何上层算法接入前都要走这一步 (PX4 要求先收到 offboard 流才接受切模式)
///
/// 用法 (在 rclcpp::Node 子类中):
///   Px4OffboardClient px4_{*this};
///   px4_.takeoverReset(now());                        // 开始接管
///   if (px4_.takeoverTick(now(), 1.0, 10.0) != Takeover::ENGAGED) return;
///   px4_.sendVelocityFRD(0.5, 0.0, 0.1);              // 机体系速度 (前,右,下), 下正
///   px4_.requestAutoLand();
///
/// 坐标/符号约定 (全组件统一):
///   - 速度指令一律机体系 FRD: x 前 y 右 z 下, "下"为正 = 下降
///   - setpoint 最终发 NED (按当前 heading 旋转), yaw 保持当前机头
///
/// 注意: 不要与避障 tracker_px4/tracker_mavros 同时运行 (会争抢 /fmu/in setpoint)。

#include <chrono>
#include <cmath>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

namespace flylab_vision
{

/// 飞控导航模式名 (VehicleStatus.msg 常用枚举), 诊断日志用
inline const char * nav_state_name(uint8_t s)
{
  switch (s) {
    case 0: return "MANUAL";
    case 1: return "ALTCTL";
    case 2: return "POSCTL";
    case 3: return "AUTO_MISSION";
    case 4: return "AUTO_LOITER";
    case 5: return "AUTO_RTL";
    case 10: return "ACRO";
    case 12: return "DESCEND";
    case 13: return "TERMINATION";
    case 14: return "OFFBOARD";
    case 15: return "STAB";
    case 17: return "AUTO_TAKEOFF";
    case 18: return "AUTO_LAND";
    case 19: return "AUTO_FOLLOW";
    case 20: return "AUTO_PRECLAND";
    case 21: return "ORBIT";
    default: return "UNKNOWN";
  }
}

/// 指令回执结果名 (MAV_RESULT), 诊断日志用
inline const char * ack_result_name(uint8_t r)
{
  switch (r) {
    case 0: return "ACCEPTED(已接受)";
    case 1: return "TEMPORARILY_REJECTED(暂时拒绝)";
    case 2: return "DENIED(永久拒绝)";
    case 3: return "UNSUPPORTED(不支持)";
    case 4: return "FAILED(执行失败)";
    case 5: return "IN_PROGRESS(执行中)";
    case 6: return "CANCELLED(已取消)";
    default: return "UNKNOWN";
  }
}

class Px4OffboardClient
{
public:
  /// offboard 接管状态机输出
  enum class Takeover
  {
    ENGAGED,     // 已确认进入 OFFBOARD, 上层可以开始发控制 setpoint
    REQUESTING,  // 请求中 (内部已在发心跳+悬停, 继续每 tick 调用)
    FAILED       // 超过 warmup+retry 仍未进入 (上层决定放弃/交还)
  };

  // 模式码 (px4_custom_mode.h): 主模式 OFFBOARD=6 / AUTO=4; 子模式 LOITER=3 / LAND=6
  static constexpr float kMainOffboard = 6.0f;
  static constexpr float kMainAuto = 4.0f;
  static constexpr float kSubLoiter = 3.0f;
  static constexpr float kSubLand = 6.0f;

  explicit Px4OffboardClient(rclcpp::Node & node)
  : node_(node)
  {
    rclcpp::QoS fmu_qos(rclcpp::KeepLast(1));
    fmu_qos.best_effort().transient_local();

    offboard_control_mode_pub_ = node_.create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", fmu_qos);
    trajectory_setpoint_pub_ = node_.create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", fmu_qos);
    vehicle_command_pub_ = node_.create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", fmu_qos);

    vehicle_status_sub_ = node_.create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status", fmu_qos,
      [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        status_ = *msg;
        last_status_time_ = node_.now();
        if (!got_status_) {
          got_status_ = true;
          RCLCPP_INFO(
            node_.get_logger(), "\033[32m已接收到 PX4 飞控数据, MicroXRCEAgent 链路正常 "
            "(当前模式: %s, %s)\033[0m",
            nav_state_name(msg->nav_state),
            msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED ? "已解锁" : "未解锁");
          last_nav_state_ = msg->nav_state;
        }
        // 模式切换跟踪: 任何模式变化 (含 failsafe 接管) 都打印, 链路问题一眼可见
        if (msg->nav_state != last_nav_state_) {
          RCLCPP_INFO(
            node_.get_logger(), "\033[36m飞控模式切换: %s -> %s%s\033[0m",
            nav_state_name(last_nav_state_), nav_state_name(msg->nav_state),
            msg->failsafe ? " [failsafe!]" : "");
          last_nav_state_ = msg->nav_state;
        }
        // failsafe 触发/解除告警 (failsafe 会强制接管模式, 是 "抖动/不降落" 的常见元凶)
        if (msg->failsafe != last_failsafe_) {
          RCLCPP_WARN(
            node_.get_logger(), "\033[33m飞控 failsafe %s (user_took_over=%d)\033[0m",
            msg->failsafe ? "触发" : "解除", msg->failsafe_and_user_took_over);
          last_failsafe_ = msg->failsafe;
        }
      });
    position_sub_ = node_.create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", fmu_qos,
      [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {position_ = *msg;});

    // 切模式指令回执: 被拒绝时给出原因 (如 offboard 流未就绪 / 模式不可用)
    command_ack_sub_ = node_.create_subscription<px4_msgs::msg::VehicleCommandAck>(
      "/fmu/out/vehicle_command_ack", fmu_qos,
      [this](const px4_msgs::msg::VehicleCommandAck::SharedPtr ack) {
        if (ack->command != px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE) {
          return;
        }
        if (ack->result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
          RCLCPP_INFO(node_.get_logger(), "切模式指令已被飞控接受");
        } else {
          RCLCPP_WARN(
            node_.get_logger(),
            "\033[33m切模式指令被拒绝: result=%u (%s), param1=%u param2=%d "
            "(常见原因: offboard 数据流未就绪/被打断, 或当前模式不允许切换)\033[0m",
            ack->result, ack_result_name(ack->result), ack->result_param1, ack->result_param2);
        }
      });
  }

  // ---------------- 状态查询 ----------------

  bool gotStatus() const {return got_status_;}
  bool armed() const
  {
    return status_.arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;
  }
  bool isOffboard() const
  {
    return status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  }
  bool isAutoLand() const
  {
    return status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND;
  }
  const char * currentModeName() const {return nav_state_name(status_.nav_state);}

  /// 最近一次飞控状态距现在是否仍在 timeout_sec 内 (链路断链判定)
  bool linkFresh(double timeout_sec) const
  {
    return got_status_ && (node_.now() - last_status_time_).seconds() <= timeout_sec;
  }

  const px4_msgs::msg::VehicleStatus & status() const {return status_;}
  const px4_msgs::msg::VehicleLocalPosition & localPosition() const {return position_;}

  // ---------------- 控制指令 ----------------

  /// 机体系 FRD 速度 setpoint (x前 y右 z下, "下"为正) 并同时发 offboard 心跳。
  /// 内部按当前 heading 旋转到 NED; yaw 保持当前机头。
  /// offboard 模式下必须持续调用 (断流会触发 offboard failsafe)
  void sendVelocityFRD(double fwd, double right, double down)
  {
    heartbeatVelocity();
    const float heading = position_.heading_good_for_control ? position_.heading : 0.0f;
    const float c = std::cos(heading), s = std::sin(heading);
    px4_msgs::msg::TrajectorySetpoint sp;
    sp.position = {NAN, NAN, NAN};
    sp.velocity = {c * static_cast<float>(fwd) - s * static_cast<float>(right),
      s * static_cast<float>(fwd) + c * static_cast<float>(right),
      static_cast<float>(down)};
    sp.acceleration = {NAN, NAN, NAN};
    sp.yaw = heading;
    sp.yawspeed = NAN;
    sp.timestamp = timestamp_now();
    trajectory_setpoint_pub_->publish(sp);
  }

  /// 原地悬停 (零速度 setpoint)
  void sendHover() {sendVelocityFRD(0.0, 0.0, 0.0);}

  /// 只发 offboard 心跳 (声明速度控制模式), 不发 setpoint
  void heartbeatVelocity()
  {
    px4_msgs::msg::OffboardControlMode msg;
    msg.position = false;
    msg.velocity = true;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = timestamp_now();
    offboard_control_mode_pub_->publish(msg);
  }

  void requestOffboard() {requestMode(kMainOffboard, 0.0f);}
  void requestAutoLand() {requestMode(kMainAuto, kSubLand);}
  void requestLoiter() {requestMode(kMainAuto, kSubLoiter);}

  /// 切飞控模式: DO_SET_MODE (param2=主模式, param3=子模式)
  void requestMode(float main_mode, float sub_mode)
  {
    px4_msgs::msg::VehicleCommand msg;
    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
    msg.param1 = 1.0f;   // CUSTOM_MODE_ENABLED
    msg.param2 = main_mode;
    msg.param3 = sub_mode;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = timestamp_now();
    vehicle_command_pub_->publish(msg);
  }

  // ---------------- offboard 接管状态机 ----------------

  /// 开始一次接管流程 (复位内部计时与计数)
  void takeoverReset(const rclcpp::Time & now)
  {
    takeover_start_ = now;
    last_request_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    attempts_ = 0;
    engaged_logged_ = false;
  }

  /// 每 tick 调用直到返回 ENGAGED 或 FAILED:
  ///   未进入 offboard 期间自动发心跳+悬停 setpoint (PX4 要求先收到流才接受切模式),
  ///   预热 warmup_sec 后每 0.5s 重发请求, 超过 warmup+retry_sec 报 FAILED。
  /// 返回 ENGAGED 后上层接管 setpoint 发送 (心跳由 sendVelocityFRD 携带)。
  Takeover takeoverTick(const rclcpp::Time & now, double warmup_sec, double retry_sec)
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
    sendHover();   // 预热/等待期间持续发心跳+悬停 (既满足切换条件又保持姿态稳定)
    const double elapsed = (now - takeover_start_).seconds();
    if (elapsed > warmup_sec && (now - last_request_).seconds() > 0.5) {
      attempts_++;
      RCLCPP_INFO(
        node_.get_logger(), "请求进入 OFFBOARD (第 %u 次): 当前模式=%s, "
        "已预热 %.1fs, offboard 心跳+悬停 setpoint 持续发送中, 等待飞控确认/ACK ...",
        attempts_, currentModeName(), elapsed);
      requestOffboard();
      last_request_ = now;
    }
    if (elapsed > warmup_sec + retry_sec) {
      RCLCPP_WARN(
        node_.get_logger(), "无法进入 OFFBOARD (检查 MicroXRCEAgent / 是否有其他链路占用)");
      return Takeover::FAILED;
    }
    return Takeover::REQUESTING;
  }

private:
  static uint64_t timestamp_now()
  {
    return static_cast<uint64_t>(rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds() / 1000);
  }

  rclcpp::Node & node_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr command_ack_sub_;

  px4_msgs::msg::VehicleStatus status_;
  px4_msgs::msg::VehicleLocalPosition position_;
  bool got_status_ = false;
  uint8_t last_nav_state_ = 255;    // 模式切换日志用
  bool last_failsafe_ = false;      // failsafe 告警用
  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};

  // 接管状态机
  rclcpp::Time takeover_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_request_{0, 0, RCL_ROS_TIME};
  uint32_t attempts_ = 0;
  bool engaged_logged_ = false;
};

}  // namespace flylab_vision

#endif  // PX4_OFFBOARD_CLIENT_HPP_
