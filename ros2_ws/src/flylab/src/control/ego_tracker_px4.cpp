// ego_tracker_px4 —— EGO 链路轨迹跟踪 + PX4 混控节点 (uXRCE-DDS 版)
//
// 可执行名: ego_tracker_px4
// 链路: px4_msgs 直发 /fmu/in/*, 依赖 MicroXRCEAgent (udp4 -p 8888)
// 与 ego_tracker_mavros 共用 ego_tracker_base.hpp 的控制逻辑,
// 本文件只实现 uXRCE-DDS 传输层: setpoint 编码 / ENU->NED 转换 / 模式切换命令
#include <array>
#include <cmath>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>

#include "ego_tracker_base.hpp"

class EgoTrackerPx4 : public EgoTrackerBase
{
public:
    EgoTrackerPx4()
    : EgoTrackerBase("ego_tracker_px4_node", 50.0,
                     "PX4 + EGO 混合控制节点已启动！(QGC 起飞后，在 RViz 中点击 2D Goal Pose 即接管，到达目标点后自动切回 AUTO_LOITER 交还 QGC)")
    {
        // 和PX4通讯的QOS配置文件
        rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
        qos_profile.best_effort().transient_local();

        declare_parameter<double>("link_timeout_sec", 3.0); // 飞控数据断链判定超时 (秒)

        vehicle_command_publisher_ = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos_profile);
        offboard_control_mode_publisher_ = create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", qos_profile);
        trajectory_setpoint_publisher_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos_profile);

        vehicle_status_subscriber_ = create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", qos_profile,
            [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
                vehicle_status_ = *msg;
                last_status_time_ = now();
                if (!received_status_) {
                    received_status_ = true;
                    RCLCPP_INFO(get_logger(), "\033[32m已接收到 PX4 飞控数据，MicroXRCEAgent 链路正常\033[0m");
                }
            });
        vehicle_pos_subscriber_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos_profile,
            [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) { vehicle_position_ = *msg; });

        // 链路超时计时起点 = 节点启动时间 (启动后 link_timeout_sec 秒内不报"尚未收到")
        last_status_time_ = now();
    }

protected:
    // ---------------- 链路自检 ----------------
    bool checkLink() override
    {
        // link_timeout_sec 秒内没收到 vehicle_status 才报警 (只打一次断链警告,
        // 复位后重新收到数据时会再次打印绿色"链路正常"提示)
        const double link_timeout = get_parameter("link_timeout_sec").as_double();
        if ((now() - last_status_time_).seconds() > link_timeout) {
            if (received_status_) {
                received_status_ = false;
                RCLCPP_WARN(get_logger(),
                            "\033[33mPX4 飞控数据已中断 (超过 %.1f 秒未收到 vehicle_status)，MicroXRCEAgent 链路可能已断开！\033[0m",
                            link_timeout);
            } else {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                                     "\033[33m尚未收到 PX4 飞控数据！请先启动 MicroXRCEAgent：  MicroXRCEAgent udp4 -p 8888\033[0m");
            }
        }
        return true;  // 断链只报警不中断, 与原实现一致
    }

    bool isArmed() const override
    {
        return vehicle_status_.arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;
    }
    bool isOffboard() const override
    {
        return vehicle_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    }
    bool isAutoLand() const override
    {
        return vehicle_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND;
    }

    // ---------------- Offboard 心跳: 解锁 PX4 的速度和加速度前馈控制权 ----------------
    void publishOffboardHeartbeat() override
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

    // ---------------- 悬停保护: 零速度 setpoint ----------------
    void publishHover() override
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

    // ---------------- EGO 指令编码: ENU -> NED ----------------
    void fillEgoSetpoint(const quadrotor_msgs::msg::PositionCommand & msg) override
    {
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
    }

    void publishEgoSetpoint() override
    {
        ego_setpoint_.timestamp = timestamp_now();
        trajectory_setpoint_publisher_->publish(ego_setpoint_);
    }

    // ---------------- 模式切换命令 ----------------
    void enterOffboard() override
    {
        // param2=6 → 主模式 OFFBOARD
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
    }

    void engageLoiter() override
    {
        // 切回 AUTO_LOITER (PX4_CUSTOM_MAIN_MODE_AUTO=4, PX4_CUSTOM_SUB_MODE_AUTO_LOITER=3)
        // DO_SET_MODE 参数约定: param1=1(CUSTOM_MODE_ENABLED), param2=主模式, param3=子模式
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 4.0f, 3.0f);
    }

    // ---------------- 到达检测输入: 当前位置(ENU)与合速度 ----------------
    void currentPoseVelENU(double pos[3], double & speed) const override
    {
        pos[0] = vehicle_position_.y;
        pos[1] = vehicle_position_.x;
        pos[2] = -vehicle_position_.z;
        speed = std::sqrt(vehicle_position_.vx * vehicle_position_.vx +
                          vehicle_position_.vy * vehicle_position_.vy +
                          vehicle_position_.vz * vehicle_position_.vz);
    }

private:
    void publishVehicleCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f,
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

    static uint64_t timestamp_now()
    {
        return static_cast<uint64_t>(rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds() / 1000);
    }

    // ---------------- 发布者 ----------------
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;

    // ---------------- 订阅者 ----------------
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_pos_subscriber_;

    // ---------------- 状态变量 ----------------
    px4_msgs::msg::VehicleStatus vehicle_status_;           // 无人机状态信息
    px4_msgs::msg::VehicleLocalPosition vehicle_position_;  // 无人机当前位置
    px4_msgs::msg::TrajectorySetpoint ego_setpoint_;        // 转换后的EGO轨迹点
    bool received_status_ = false;                          // 当前链路是否正常 (断链超时后会被复位)
    rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};     // 最近一次收到 vehicle_status 的时间
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EgoTrackerPx4>());
    rclcpp::shutdown();
    return 0;
}
