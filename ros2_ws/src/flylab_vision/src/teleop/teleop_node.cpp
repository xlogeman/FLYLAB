// Copyright 2026 flylab
/// 手动遥控节点 —— 地面站双摇杆 → offboard 速度/偏航角速率
///
/// 状态机: IDLE(等 teleop_start) → ENGAGING(接管 offboard) →
///         ENGAGED(按摇杆量下发速度+yaw速率, 回中=悬停, 流不能断) →
///         RELEASE(悬停+请求 LOITER, 确认退出 offboard 后回 IDLE)
///
/// 遥控量输入 (geometry_msgs/Twist, 归一化 -1..1, 地面站双摇杆实时下发):
///   linear.x 前+ linear.y 右+ linear.z 升+ angular.z 左转+
/// 各分量按速度上限换算 (力度 = 拨杆/摇杆偏离中位的距离), 死区 0.06;
/// 全部回中 = 零速度悬停 (流不能断)
///
/// 安全防御: 上锁 / AUTO.LAND 接管 / 飞控数据断流 → 立即交还控制权。
/// 使用前提: 已解锁升空 (地面 unarmed 状态 offboard 无法起飞)。
/// 运行: ros2 run flylab_vision teleop_node --ros-args -p link:=mavros|uxrce
///       -p v_xy:=5.0 -p v_z:=2.0 -p yaw_rate_deg:=30.0

#include <cmath>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

#include "control/px4_offboard_client.hpp"

namespace flylab_vision
{

namespace
{
constexpr const char kCmdStart[] = "teleop_start";
constexpr const char kCmdStop[] = "teleop_stop";
constexpr double kDeadzone = 0.06;     // 归一化量死区 (拨杆/摇杆回中抖动)
constexpr double kWarmupSec = 1.0;     // 接管: setpoint 预热时长
constexpr double kRetrySec = 5.0;      // 接管重试窗口 (超时判 FAILED)
constexpr double kReleaseSec = 2.0;    // 交还: 等 LOITER 确认的上限
}  // namespace

class TeleopNode : public rclcpp::Node
{
public:
  TeleopNode()
  : Node("teleop_node")
  {
    const auto cmd_topic = declare_parameter<std::string>("cmd_topic", "/flylab/vision_cmd");
    const auto twist_topic =
      declare_parameter<std::string>("teleop_cmd_topic", "/flylab/teleop_cmd");
    const auto status_topic =
      declare_parameter<std::string>("status_topic", "/flylab/teleop_status");
    link_ = declare_parameter<std::string>("link", "mavros");
    v_xy_ = declare_parameter<double>("v_xy", 5.0);                   // 平移速度上限 (m/s)
    v_z_ = declare_parameter<double>("v_z", 2.0);                     // 升降速度上限 (m/s)
    yaw_rate_rad_ = declare_parameter<double>("yaw_rate_deg", 30.0) * M_PI / 180.0;
    link_fresh_timeout_ = declare_parameter<double>("link_fresh_timeout", 1.0);

    if (link_ == "uxrce") {
      client_ = std::make_unique<Px4OffboardClientUxrce>(*this);
    } else if (link_ == "mavros") {
      client_ = std::make_unique<Px4OffboardClientMavros>(*this);
    } else {
      RCLCPP_ERROR(get_logger(), "未知链路 '%s' (可选 mavros|uxrce)", link_.c_str());
      throw std::runtime_error("unknown link: " + link_);
    }

    cmd_sub_ = create_subscription<std_msgs::msg::String>(
      cmd_topic, 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == kCmdStart) {
          requestStart();
        } else if (msg->data == kCmdStop) {
          requestStop();
        }
      });
    twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      twist_topic, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {twist_ = *msg;});
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic, 10);

    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] {tick();});
    RCLCPP_INFO(
      get_logger(), "手动遥控节点就绪 (link=%s, v_xy=%.1f v_z=%.1f yaw=%.0f°/s), 等待 teleop_start",
      link_.c_str(), v_xy_, v_z_, yaw_rate_rad_ * 180.0 / M_PI);
  }

private:
  enum class State {IDLE, ENGAGING, ENGAGED, RELEASE};

  void publishStatus(const std::string & s)
  {
    if (s == status_) {
      return;
    }
    status_ = s;
    std_msgs::msg::String m;
    m.data = s;
    status_pub_->publish(m);
  }

  void requestStart()
  {
    if (state_ != State::IDLE) {
      return;
    }
    if (!client_->gotStatus()) {
      RCLCPP_ERROR(get_logger(), "未收到飞控状态, 无法接管 (通信链路是否已开启?)");
      publishStatus("error: 通信链路未通");
      return;
    }
    if (!client_->armed()) {
      RCLCPP_ERROR(get_logger(), "飞机未解锁, offboard 无法起飞; 请先用飞行控制起飞");
      publishStatus("error: 未解锁起飞");
      return;
    }
    // 注: 不再拒绝 AUTO.LAND —— 飞行控制下发的自动降落可被开遥控打断
    // 接管 (防炸机出口); offboard 接管即切走 AUTO.LAND。接管成功后若
    // 又进入 AUTO.LAND (failsafe), ENGAGED 态的安全防御仍会退出遥控
    client_->takeoverReset(now());
    state_ = State::ENGAGING;
    publishStatus("engaging");
  }

  void requestStop()
  {
    if (state_ != State::IDLE) {
      beginRelease("stop");
    }
  }

  /// 交还控制权: 保持悬停流 + 请求 LOITER, 确认退出 offboard (或超时) 后回 IDLE
  void beginRelease(const char * reason)
  {
    RCLCPP_INFO(get_logger(), "退出手动遥控 (%s), 交还控制权 ...", reason);
    release_deadline_ = now() + rclcpp::Duration::from_seconds(kReleaseSec);
    last_loiter_req_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    state_ = State::RELEASE;
    publishStatus("releasing");
  }

  void tick()
  {
    const auto now = this->now();
    switch (state_) {
      case State::IDLE:
        break;
      case State::ENGAGING: {
        const auto rc = client_->takeoverTick(now, kWarmupSec, kRetrySec);
        if (rc == Px4OffboardClientBase::Takeover::ENGAGED) {
          RCLCPP_INFO(
            get_logger(), "\033[32m手动遥控接管成功, 摇杆可用 (回中 = 原地悬停)\033[0m");
          state_ = State::ENGAGED;
          publishStatus("engaged");
        } else if (rc == Px4OffboardClientBase::Takeover::FAILED) {
          RCLCPP_ERROR(get_logger(), "OFFBOARD 接管失败, 回到待命");
          state_ = State::IDLE;
          publishStatus("failed");
        }
        break;
      }
      case State::ENGAGED: {
        // 安全防御: 上锁 / 自动降落 / 断链 → 立即交还
        if (!client_->armed()) {
          RCLCPP_WARN(get_logger(), "检测到已上锁, 退出遥控");
          state_ = State::IDLE;
          publishStatus("idle");
          break;
        }
        if (client_->isAutoLand()) {
          RCLCPP_WARN(get_logger(), "AUTO.LAND 已接管, 停止遥控 setpoint");
          state_ = State::IDLE;
          publishStatus("idle");
          break;
        }
        if (!client_->linkFresh(link_fresh_timeout_)) {
          RCLCPP_WARN(get_logger(), "飞控数据断流, 交还控制权");
          beginRelease("link-lost");
          break;
        }
        applyTwist();
        break;
      }
      case State::RELEASE: {
        client_->sendHover();   // 退出 offboard 前保持流, 防止 offboard failsafe 抢模式
        if (!client_->isOffboard()) {
          RCLCPP_INFO(get_logger(), "\033[32m已退出 OFFBOARD, 控制权交还完成\033[0m");
          state_ = State::IDLE;
          publishStatus("idle");
          break;
        }
        if (now > release_deadline_) {
          RCLCPP_WARN(get_logger(), "等待退出 OFFBOARD 超时, 停止 setpoint 流");
          state_ = State::IDLE;
          publishStatus("idle");
          break;
        }
        if ((now - last_loiter_req_).seconds() > 0.5) {
          client_->requestLoiter();
          last_loiter_req_ = now;
        }
        break;
      }
    }
  }

  /// 归一化遥控量 → 机体系 FRD 速度 + 偏航角速率 (全部回中 = 原地悬停)
  void applyTwist()
  {
    // 死区: 摇杆回中附近的抖动不产生指令
    auto dz = [](double v) {return std::abs(v) < kDeadzone ? 0.0 : v;};
    const double fwd = dz(twist_.linear.x) * v_xy_;
    const double right = dz(twist_.linear.y) * v_xy_;
    const double down = -dz(twist_.linear.z) * v_z_;      // linear.z 升为正
    const double yaw_rate = -dz(twist_.angular.z) * yaw_rate_rad_;  // 左转+
    if (fwd == 0.0 && right == 0.0 && down == 0.0 && yaw_rate == 0.0) {
      client_->sendHover();
    } else {
      client_->sendVelocityFRDYawRate(fwd, right, down, yaw_rate);
    }
  }

  State state_ = State::IDLE;
  std::unique_ptr<Px4OffboardClientBase> client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string link_;
  std::string status_;
  geometry_msgs::msg::Twist twist_;
  double v_xy_ = 1.0;
  double v_z_ = 2.0;
  double yaw_rate_rad_ = 30.0 * M_PI / 180.0;
  double link_fresh_timeout_ = 1.0;
  rclcpp::Time release_deadline_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_loiter_req_{0, 0, RCL_ROS_TIME};
};

}  // namespace flylab_vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<flylab_vision::TeleopNode>());
  rclcpp::shutdown();
  return 0;
}
