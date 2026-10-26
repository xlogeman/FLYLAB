// tracker_base.hpp —— 避障规划轨迹跟踪节点公共基类
//
// tracker_uxrce 与 tracker_mavros 控制逻辑一致, 共用部分收敛到这里:
//   共用 (基类): 参数声明 / RViz 目标点拦截 / 规划指令缓存与事件驱动下发 /
//                到达检测 (距离+速度双条件) / OFFBOARD 状态机骨架
//   差异 (派生): setpoint 编码与坐标系转换 / 心跳形式 / 模式切换命令 / 状态来源
//
// 与规划算法解耦: 只订阅 quadrotor_msgs/PositionCommand (pos_cmd_topic),
// 换其他规划器时写桥接节点转成 PositionCommand 即可。
//
// 派生类需实现:
//   checkLink / isArmed / isOffboard / isAutoLand
//   publishOffboardHeartbeat / publishHover / fillPlannerSetpoint / publishPlannerSetpoint
//   enterOffboard / engageLoiter / currentPoseVelENU
#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>

class TrackerBase : public rclcpp::Node
{
public:
  TrackerBase(const std::string & node_name, double default_rate_hz, const std::string & banner)
  : Node(node_name)
  {
    // ---------------- 共用参数 ----------------
    declare_parameter<double>("goal_z_height", 2.0);    // RViz 目标点强制修改的高度 (米)
    declare_parameter<double>("goal_reach_dist", 0.5);  // 到达目标点的距离阈值 (米)
    declare_parameter<double>("goal_reach_speed", 0.5); // 到达目标点的速度阈值 (米/秒)
    declare_parameter<bool>("return_to_loiter", true);  // 到达后切回 LOITER (false 则保持 OFFBOARD 悬停)
    declare_parameter<double>("control_rate_hz", default_rate_hz); // 状态机/心跳频率 (PX4 要求 setpoint >= 2Hz)
    declare_parameter<std::string>("pos_cmd_topic", "/planning/pos_cmd"); // 规划指令话题 (桥接节点输出)

    // ---------------- 共用发布/订阅 ----------------
    goal_3d_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose_3d", 10);
    rviz_goal_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { rvizGoalCallback(*msg); });
    pos_cmd_subscriber_ = create_subscription<quadrotor_msgs::msg::PositionCommand>(
      get_parameter("pos_cmd_topic").as_string(), 10,
      [this](const quadrotor_msgs::msg::PositionCommand::SharedPtr msg) { plannerCmdCallback(*msg); });

    timer_ = create_wall_timer(
      std::chrono::milliseconds(
        static_cast<int>(1000.0 / get_parameter("control_rate_hz").as_double())),
      std::bind(&TrackerBase::tick, this));

    RCLCPP_INFO(get_logger(), "%s", banner.c_str());
  }

  virtual ~TrackerBase() = default;

protected:
  // ============ 传输层接口 (派生类实现) ============
  virtual bool checkLink() = 0;              // 链路自检, 返回 false 跳过本轮 (警告自行打印)
  virtual bool isArmed() const = 0;
  virtual bool isOffboard() const = 0;
  virtual bool isAutoLand() const = 0;
  virtual void publishOffboardHeartbeat() = 0;
  virtual void publishHover() = 0;           // 悬停保护 setpoint
  virtual void fillPlannerSetpoint(const quadrotor_msgs::msg::PositionCommand & msg) = 0;  // 规划指令 -> 各自 setpoint 格式
  virtual void publishPlannerSetpoint() = 0;
  virtual void enterOffboard() = 0;
  virtual void engageLoiter() = 0;
  // 当前位置 (ENU) 与合速度, 供到达检测使用
  virtual void currentPoseVelENU(double pos[3], double & speed) const = 0;

  // OFFBOARD 且已有规划指令时的动作; mavros 版覆写为"重发缓存轨迹点 + 到达检测"
  virtual void tickOffboardWithPlanner() { checkGoalReached(); }

  // ============ 共用状态机 ============
  void tick()
  {
    if (!checkLink()) {
      return;
    }

    // 1. 始终发布 Offboard 心跳 (形式由派生类决定)
    publishOffboardHeartbeat();

    // 2. 状态机逻辑
    if (!isArmed()) {
      // 未解锁: 等待 QGC 起飞解锁, 不干预
    } else if (isOffboard()) {
      // 已在 OFFBOARD 状态
      if (planner_active_) {
        // 【避障规划自动飞行模式】
        if (!has_planner_cmd_) {
          // 安全保护: 点击目标点后, 规划器还没算出轨迹时, 原地悬停
          publishHover();
        } else {
          tickOffboardWithPlanner();
        }
      } else {
        // 非规划模式下保持悬停, 等待下一次 RViz 点目标或 QGC 切走模式
        publishHover();
      }
    } else if (isAutoLand()) {
      // 降落由 QGC 控制, 不干预
    } else if (planner_active_) {
      // 规划器已接管但还没进 OFFBOARD: PX4 要求切入前必须有持续 setpoint 流,
      // 请求切换的同时补发悬停 setpoint 凑齐切换条件
      enterOffboard();
      publishHover();
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "规划器已接管，正在补发悬停 setpoint 并切入 OFFBOARD 模式 ...");
    }
    // 其他状态 (AUTO_LOITER 悬停 / 手动模式等): 不干预
  }

  // 规划指令回调: 缓存 -> 派生类编码 -> 事件驱动同步下发
  void plannerCmdCallback(const quadrotor_msgs::msg::PositionCommand & msg)
  {
    has_planner_cmd_ = true;
    fillPlannerSetpoint(msg);

    // 事件驱动: 只要处于避障规划模式下, 收到一帧就发一帧, 实现 PX4 和规划器同步
    if (isOffboard() && planner_active_) {
      publishPlannerSetpoint();
    }
  }

  // RViz 目标点回调: 强制高度 -> 记录目标(ENU) -> 接管控制权 -> 转发规划器
  void rvizGoalCallback(const geometry_msgs::msg::PoseStamped & msg)
  {
    const double target_z = get_parameter("goal_z_height").as_double();
    geometry_msgs::msg::PoseStamped goal = msg;
    goal.pose.position.z = target_z;
    RCLCPP_INFO(get_logger(), "拦截到RViz目标点，已将其高度修改为 Z = %.1f 米，并转发至规划器", target_z);

    // 点击目标点即接管控制权: 记录目标点(ENU)并进入避障规划自动飞行模式
    goal_enu_[0] = goal.pose.position.x;
    goal_enu_[1] = goal.pose.position.y;
    goal_enu_[2] = goal.pose.position.z;
    has_goal_ = true;
    has_planner_cmd_ = false;
    planner_active_ = true;
    RCLCPP_INFO(get_logger(), "控制权已接管，等待轨迹规划，将自动飞往目标点，到达后自动切回 AUTO_LOITER");

    goal_3d_publisher_->publish(goal);
  }

  // 到达检测: 位置距离与速度双条件判定, 到达后交还控制权
  void checkGoalReached()
  {
    if (!has_goal_) {
      return;
    }

    double pos[3] = {0.0, 0.0, 0.0};
    double speed = 0.0;
    currentPoseVelENU(pos, speed);

    const double dx = goal_enu_[0] - pos[0];
    const double dy = goal_enu_[1] - pos[1];
    const double dz = goal_enu_[2] - pos[2];
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (dist < get_parameter("goal_reach_dist").as_double() &&
        speed < get_parameter("goal_reach_speed").as_double())
    {
      planner_active_ = false;
      has_goal_ = false;
      if (get_parameter("return_to_loiter").as_bool()) {
        // 自动交还控制权: 切回 AUTO_LOITER, 由 PX4 定点悬停, QGC 可直接接管
        engageLoiter();
        RCLCPP_INFO(get_logger(), "\033[32m已到达目标点 (距离 %.2f 米)，已切回 AUTO_LOITER，控制权交还 QGC\033[0m", dist);
      } else {
        RCLCPP_INFO(get_logger(), "\033[32m已到达目标点 (距离 %.2f 米)，保持 OFFBOARD 悬停\033[0m", dist);
      }
    }
  }

  // ============ 共用状态 ============
  bool planner_active_ = false;   // 当前是否处于避障规划自动飞行模式
  bool has_goal_ = false;         // 是否有目标点
  bool has_planner_cmd_ = false;  // 是否收到过规划器的指令
  std::array<double, 3> goal_enu_{};  // 当前目标点 (ENU 坐标)

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_3d_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rviz_goal_subscriber_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
};
