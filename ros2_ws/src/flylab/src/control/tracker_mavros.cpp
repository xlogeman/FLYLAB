// tracker_mavros —— 避障规划链路轨迹跟踪 + MAVROS 混控节点
//
// 可执行名: tracker_mavros
// 链路: MAVROS (UDP 14540), 不依赖 MicroXRCEAgent / px4_msgs
// 与 tracker_px4 共用 tracker_base.hpp 的控制逻辑,
// 本文件只实现 MAVROS 传输层: ENU 轨迹直接透传 / set_mode 服务 / mavros 状态源
// 注: MAVROS setpoint_raw 插件输入为 ENU 语义, 内部负责 ENU->NED 转换 (含 yaw/yaw_rate)
#include <array>
#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>

#include "tracker_base.hpp"

class TrackerMavros : public TrackerBase
{
public:
    TrackerMavros()
    : TrackerBase("tracker_mavros_node", 20.0,
                  "MAVROS + 轨迹跟踪混合控制节点已启动！(QGC 起飞后，在 RViz 中点击 2D Goal Pose 即接管，到达目标点后自动切回 AUTO.LOITER 交还 QGC)")
    {
        setpoint_publisher_ = create_publisher<mavros_msgs::msg::PositionTarget>("/mavros/setpoint_raw/local", 10);

        state_subscriber_ = create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10,
            [this](const mavros_msgs::msg::State::SharedPtr msg) {
                if (msg->connected && !state_.connected) {
                    RCLCPP_INFO(get_logger(), "\033[32mMAVROS 已连接 PX4, 等待目标点\033[0m");
                }
                state_ = *msg;
            });
        pose_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { current_pose_ = *msg; });
        velocity_subscriber_ = create_subscription<geometry_msgs::msg::TwistStamped>(
            "/mavros/local_position/velocity_local", rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) { current_velocity_ = *msg; });

        set_mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    }

protected:
    // ---------------- 链路自检 ----------------
    bool checkLink() override
    {
        if (!state_.connected) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                                 "\033[33m尚未连接到飞控！请确认 PX4 SITL 与 MAVROS 已启动\033[0m");
            return false;
        }
        return true;
    }

    bool isArmed() const override { return state_.armed; }
    bool isOffboard() const override { return state_.mode == "OFFBOARD"; }
    bool isAutoLand() const override { return state_.mode == "AUTO.LAND"; }

    // ---------------- Offboard 心跳: 由持续 setpoint 流充当, 此处无需额外消息 ----------------
    void publishOffboardHeartbeat() override {}

    // ---------------- 悬停保护: 当前位置定点悬停 (ENU) ----------------
    void publishHover() override
    {
        mavros_msgs::msg::PositionTarget hover;
        hover.header.stamp = now();
        hover.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
        // 只用位置 + yaw: 忽略速度/加速度/偏航角速度
        hover.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_VX | mavros_msgs::msg::PositionTarget::IGNORE_VY |
                          mavros_msgs::msg::PositionTarget::IGNORE_VZ | mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                          mavros_msgs::msg::PositionTarget::IGNORE_AFY | mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                          mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
        hover.position = current_pose_.pose.position;
        hover.yaw = extractYaw(current_pose_.pose.orientation);
        setpoint_publisher_->publish(hover);
    }

    // ---------------- 规划指令编码: ENU 直接透传 ----------------
    void fillPlannerSetpoint(const quadrotor_msgs::msg::PositionCommand & msg) override
    {
        planner_setpoint_.header.stamp = now();
        planner_setpoint_.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
        // type_mask = 0: 所有字段(位置+速度+加速度+yaw+yaw_rate)全部生效, 不带任何 IGNORE 位;
        // 切勿置 FORCE 位(512)——PX4 不支持力设定点, 见到即丢弃整条 setpoint
        planner_setpoint_.type_mask = 0;
        planner_setpoint_.position.x = msg.position.x;
        planner_setpoint_.position.y = msg.position.y;
        planner_setpoint_.position.z = msg.position.z;
        planner_setpoint_.velocity.x = msg.velocity.x;
        planner_setpoint_.velocity.y = msg.velocity.y;
        planner_setpoint_.velocity.z = msg.velocity.z;
        planner_setpoint_.acceleration_or_force.x = msg.acceleration.x;
        planner_setpoint_.acceleration_or_force.y = msg.acceleration.y;
        planner_setpoint_.acceleration_or_force.z = msg.acceleration.z;
        planner_setpoint_.yaw = msg.yaw;
        planner_setpoint_.yaw_rate = msg.yaw_dot;
    }

    void publishPlannerSetpoint() override
    {
        setpoint_publisher_->publish(planner_setpoint_);
    }

    // ---------------- OFFBOARD 且已有规划指令: 重发缓存轨迹点(心跳兜底) + 到达检测 ----------------
    void tickOffboardWithPlanner() override
    {
        publishPlannerSetpoint();
        checkGoalReached();
    }

    // ---------------- 模式切换命令 ----------------
    void enterOffboard() override { requestMode("OFFBOARD"); }
    void engageLoiter() override { requestMode("AUTO.LOITER"); }

    // ---------------- 到达检测输入: 当前位置(ENU)与合速度 ----------------
    void currentPoseVelENU(double pos[3], double & speed) const override
    {
        pos[0] = current_pose_.pose.position.x;
        pos[1] = current_pose_.pose.position.y;
        pos[2] = current_pose_.pose.position.z;
        const auto & v = current_velocity_.twist.linear;
        speed = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    }

private:
    void requestMode(const std::string & mode)
    {
        if (!set_mode_client_->service_is_ready()) {
            RCLCPP_WARN(get_logger(), "/mavros/set_mode 服务未就绪");
            return;
        }
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = mode;
        set_mode_client_->async_send_request(request,
            [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
                if (!future.get()->mode_sent) {
                    RCLCPP_WARN(get_logger(), "切换到 %s 模式请求被拒", mode.c_str());
                }
            });
    }

    static double extractYaw(const geometry_msgs::msg::Quaternion & q)
    {
        return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    // ---------------- 发布者 ----------------
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_publisher_;

    // ---------------- 订阅者 ----------------
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_subscriber_;

    // ---------------- 服务客户端 ----------------
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;

    // ---------------- 状态变量 ----------------
    mavros_msgs::msg::State state_;                     // MAVROS 飞控状态 (connected/armed/mode)
    geometry_msgs::msg::PoseStamped current_pose_;      // 当前位置 (ENU)
    geometry_msgs::msg::TwistStamped current_velocity_; // 当前速度 (ENU)
    mavros_msgs::msg::PositionTarget planner_setpoint_; // 缓存的最新规划轨迹点 (ENU, 待透传)
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrackerMavros>());
    rclcpp::shutdown();
    return 0;
}
