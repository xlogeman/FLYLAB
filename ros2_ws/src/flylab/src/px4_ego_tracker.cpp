// px4_ego_tracker: EGO 轨迹跟踪 + PX4 混控节点
// QGC 起飞后，在 RViz 中点击 2D Goal Pose 即接管，到达目标点后自动切回 AUTO_LOITER 交还 QGC
#include <array>
#include <cmath>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>

class Px4EgoTrackerNode : public rclcpp::Node
{
public:
    Px4EgoTrackerNode() : Node("px4_ego_tracker_node")
    {
        // 和PX4通讯的QOS配置文件
        rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
        qos_profile.best_effort().transient_local();

        // ---------------- 参数 ----------------
        declare_parameter<double>("goal_z_height", 2.0);    // RViz 目标点强制修改的高度 (米)
        declare_parameter<double>("goal_reach_dist", 0.5);  // 到达目标点的距离阈值 (米)
        declare_parameter<double>("goal_reach_speed", 0.5); // 到达目标点的速度阈值 (米/秒)
        declare_parameter<bool>("return_to_loiter", true);  // 到达后切回 AUTO_LOITER (false 则保持 OFFBOARD 悬停)

        // ---------------- 发布者 ----------------
        vehicle_command_publisher_ = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos_profile);
        offboard_control_mode_publisher_ = create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", qos_profile);
        trajectory_setpoint_publisher_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos_profile);
        goal_3d_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose_3d", 10);

        // ---------------- 订阅者 ----------------
        vehicle_status_subscriber_ = create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", qos_profile,
            [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
                vehicle_status_ = *msg;
                if (!received_status_) {
                    received_status_ = true;
                    RCLCPP_INFO(get_logger(), "已接收到 PX4 飞控数据，MicroXRCEAgent 链路正常");
                }
            });
        sub_vehicle_pos_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos_profile,
            [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) { vehicle_position_ = *msg; });
        rviz_goal_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { rviz_goal_callback(*msg); });

        // 订阅 EGO-Planner 的轨迹输出
        ego_pos_cmd_subscriber_ = create_subscription<quadrotor_msgs::msg::PositionCommand>(
            "/drone_0_planning/pos_cmd", 10,
            [this](const quadrotor_msgs::msg::PositionCommand::SharedPtr msg) { ego_pos_cmd_callback(*msg); });

        // 定时器处理函数 (50Hz，仅用于维护 PX4 Offboard 心跳和状态机逻辑)
        timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&Px4EgoTrackerNode::timer_callback, this));

        RCLCPP_INFO(get_logger(), "PX4 + EGO 混合控制节点已启动！(QGC 起飞后，在 RViz 中点击 2D Goal Pose 即接管，到达目标点后自动切回 AUTO_LOITER 交还 QGC)");
    }

private:
    // =============================================================================================================================
    // ================= 各种订阅的回调函数 ========================================================================================
    // =============================================================================================================================
    // 接收 EGO-Planner 传来的位置指令，转换成PX4的NED坐标系，然后同步发送给PX4
    void ego_pos_cmd_callback(const quadrotor_msgs::msg::PositionCommand & msg)
    {
        has_ego_cmd_ = true;
        // 坐标系转换：ROS(ENU) -> PX4(NED)
        // 1. 位置转换
        ego_setpoint_.position = {static_cast<float>(msg.position.y), static_cast<float>(msg.position.x), static_cast<float>(-msg.position.z)};
        // 2. 速度转换
        ego_setpoint_.velocity = {static_cast<float>(msg.velocity.y), static_cast<float>(msg.velocity.x), static_cast<float>(-msg.velocity.z)};
        // 3. 加速度转换
        ego_setpoint_.acceleration = {static_cast<float>(msg.acceleration.y), static_cast<float>(msg.acceleration.x), static_cast<float>(-msg.acceleration.z)};
        // 4. 偏航角 (Yaw) 转换
        ego_setpoint_.yaw = static_cast<float>(M_PI / 2.0) - msg.yaw;
        // 5. 偏航角速度 (Yaw Rate) 转换
        ego_setpoint_.yawspeed = -msg.yaw_dot;

        // 事件驱动：只要处于 EGO 避障模式下，收到一帧就发一帧，实现PX4和EGO同步
        if (vehicle_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD && ego_active_) {
            ego_setpoint_.timestamp = timestamp_now();
            trajectory_setpoint_publisher_->publish(ego_setpoint_);
        }
    }

    // =============================================================================================================================
    // ================= 目标高度更换 ==============================================================================================
    // =============================================================================================================================
    void rviz_goal_callback(geometry_msgs::msg::PoseStamped msg)
    {
        double target_z = get_parameter("goal_z_height").as_double();
        msg.pose.position.z = target_z;
        RCLCPP_INFO(get_logger(), "拦截到RViz目标点，已将其高度修改为 Z = %.1f 米，并转发至 EGO-Planner", target_z);

        // 点击目标点即接管控制权：记录目标点(NED)并进入 EGO 自动避障模式
        ego_goal_ned_[0] = msg.pose.position.y;
        ego_goal_ned_[1] = msg.pose.position.x;
        ego_goal_ned_[2] = -target_z;
        has_goal_ = true;
        has_ego_cmd_ = false;
        ego_active_ = true;
        RCLCPP_INFO(get_logger(), "EGO 控制权已接管，等待轨迹规划，将自动飞往目标点，到达后自动切回 AUTO_LOITER");

        goal_3d_publisher_->publish(msg);
    }

    // 检测是否已到达目标点：位置距离与速度双条件判定，到达后交还控制权
    void check_goal_reached()
    {
        if (!has_goal_) {
            return;
        }

        double dx = ego_goal_ned_[0] - vehicle_position_.x;
        double dy = ego_goal_ned_[1] - vehicle_position_.y;
        double dz = ego_goal_ned_[2] - vehicle_position_.z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        double speed = std::sqrt(vehicle_position_.vx * vehicle_position_.vx +
                                 vehicle_position_.vy * vehicle_position_.vy +
                                 vehicle_position_.vz * vehicle_position_.vz);

        if (dist < get_parameter("goal_reach_dist").as_double() && speed < get_parameter("goal_reach_speed").as_double()) {
            ego_active_ = false;
            has_goal_ = false;
            if (get_parameter("return_to_loiter").as_bool()) {
                // 自动交还控制权：切回 AUTO_LOITER，由 PX4 定点悬停，QGC 可直接接管
                engage_loiter_mode();
                RCLCPP_INFO(get_logger(), "已到达目标点 (距离 %.2f 米)，已切回 AUTO_LOITER，控制权交还 QGC", dist);
            } else {
                RCLCPP_INFO(get_logger(), "已到达目标点 (距离 %.2f 米)，保持 OFFBOARD 悬停", dist);
            }
        }
    }

    // =============================================================================================================================
    // ================= 无人机状态切换的命令 ======================================================================================
    // =============================================================================================================================
    void engage_offboard_mode()
    {
        // param2=6 → 主模式 OFFBOARD
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
    }

    void engage_loiter_mode()
    {
        // 切回 AUTO_LOITER (PX4_CUSTOM_MAIN_MODE_AUTO=4, PX4_CUSTOM_SUB_MODE_AUTO_LOITER=3)
        // DO_SET_MODE 参数约定: param1=1(CUSTOM_MODE_ENABLED), param2=主模式, param3=子模式
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 4.0f, 3.0f);
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f,
                                 float param3 = 0.0f, float param4 = 0.0f,
                                 float param5 = 0.0f, float param6 = 0.0f, float param7 = 0.0f)
    {
        px4_msgs::msg::VehicleCommand msg;
        msg.command = command;
        msg.param1 = param1;
        msg.param2 = param2;
        msg.param3 = param3;
        msg.param4 = param4;
        msg.param5 = param5;
        msg.param6 = param6;
        msg.param7 = param7;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = timestamp_now();
        vehicle_command_publisher_->publish(msg);
    }

    // =============================================================================================================================
    // ================= 控制命令发布 ==============================================================================================
    // =============================================================================================================================
    // 发布 offboard 心跳，解锁 PX4 的速度和加速度前馈控制权
    void publish_offboard_control_heartbeat_signal()
    {
        px4_msgs::msg::OffboardControlMode msg;

        if (ego_active_) {
            // EGO 自动避障模式
            msg.position = true;
            msg.velocity = true;
            msg.acceleration = true;
        } else {
            // 悬停等待模式
            msg.position = false;
            msg.velocity = true;
            msg.acceleration = false;
        }

        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = timestamp_now();
        offboard_control_mode_publisher_->publish(msg);
    }

    // 悬停保护：发布当前位置为期望位置
    void publish_hover_setpoint()
    {
        px4_msgs::msg::TrajectorySetpoint msg;
        msg.position = {NAN, NAN, NAN};
        msg.velocity = {0.0f, 0.0f, 0.0f};
        msg.acceleration = {NAN, NAN, NAN};
        msg.yaw = vehicle_position_.heading;
        msg.yawspeed = NAN;
        msg.timestamp = timestamp_now();
        trajectory_setpoint_publisher_->publish(msg);
    }

    // =============================================================================================================================
    // ================= 定时处理函数，主循环 ======================================================================================
    // =============================================================================================================================
    void timer_callback()
    {
        // 0. 数据链路自检：收不到飞控数据时提醒启动 MicroXRCEAgent
        if (!received_status_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                                 "尚未收到 PX4 飞控数据！请先启动 MicroXRCEAgent：  MicroXRCEAgent udp4 -p 8888");
        }

        // 1. 始终高频发布 Offboard 心跳
        publish_offboard_control_heartbeat_signal();

        // 2. 状态机逻辑
        if (vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED) {
            // 未解锁：等待 QGC 起飞解锁，不干预
        } else if (vehicle_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
            // 已在 OFFBOARD 状态
            if (ego_active_) {
                // 【EGO 自动避障模式】
                if (!has_ego_cmd_) {
                    // 安全保护：点击目标点后，EGO 还没算出轨迹时，原地悬停
                    publish_hover_setpoint();
                } else {
                    // 到达检测：位置与速度双条件满足后交还控制权
                    check_goal_reached();
                }
            } else {
                // 非 EGO 模式下保持悬停，等待下一次 RViz 点目标或 QGC 切走模式
                publish_hover_setpoint();
            }
        } else if (vehicle_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND) {
            // 降落由 QGC 控制，不干预
        } else if (ego_active_) {
            // EGO 已接管但还没进 OFFBOARD（QGC / commander takeoff 起飞后悬停的场景）：
            // PX4 要求切入 OFFBOARD 前必须有持续的有效 setpoint 流，否则切换会被拒绝。
            // 因此这里在请求切换模式的同时，以 50Hz 补发悬停 setpoint 凑齐切换条件。
            engage_offboard_mode();
            publish_hover_setpoint();
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "EGO 已接管，正在补发悬停 setpoint 并切入 OFFBOARD 模式 ...");
        }
        // 其他状态（AUTO_LOITER 悬停 / 手动模式等）：不干预
    }

    static uint64_t timestamp_now()
    {
        return static_cast<uint64_t>(rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds() / 1000);
    }

    // ---------------- 发布者 ----------------
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_3d_publisher_;

    // ---------------- 订阅者 ----------------
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_vehicle_pos_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rviz_goal_subscriber_;
    rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr ego_pos_cmd_subscriber_;

    rclcpp::TimerBase::SharedPtr timer_;

    // ---------------- 状态变量 ----------------
    px4_msgs::msg::VehicleStatus vehicle_status_;           // 无人机状态信息
    px4_msgs::msg::VehicleLocalPosition vehicle_position_;  // 无人机当前位置
    px4_msgs::msg::TrajectorySetpoint ego_setpoint_;        // 转换后的EGO轨迹点

    bool has_ego_cmd_ = false;                // 是否收到过EGO的指令
    bool ego_active_ = false;                 // 当前是否处于 EGO 自动避障模式
    bool has_goal_ = false;                   // 是否有目标点
    bool received_status_ = false;            // 是否收到过 PX4 数据 (用于检测 MicroXRCEAgent)
    std::array<double, 3> ego_goal_ned_{};    // 当前目标点 (NED 坐标)
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Px4EgoTrackerNode>());
    rclcpp::shutdown();
    return 0;
}
