// ============================================================================
// move_velocity —— 通用 PX4 offboard 速度控制器 (C++)
//
// 功能:
//   1. 去掉键盘依赖 (HWSimpleKeyboardInfo), 改订阅标准 geometry_msgs/Twist
//   2. Twist 按 REP-103 惯例解释 (FLU: x前 y左 z上, angular.z 逆时针为正),
//      内部转换为 PX4 NED 系速度
//   3. 非 offboard 状态 (QGC 手动控制等) 不干预, 避免抢控制权
//
// 控制约定:
//   解锁+起飞: disarmed 时收到 linear.z > 5  → 切 OFFBOARD + 解锁 + 爬升
//   降落:      OFFBOARD 中 linear.z < -5 且离地 < 5m → 发 NAV_LAND
//   降落取消:  AUTO_LAND 中收到 linear.z > 5 → 切回 OFFBOARD 重新飞行
//
// 手动测试 (-r 10 表示 10Hz):
//   ros2 topic pub -r 10 /flylab/cmd_vel geometry_msgs/msg/Twist "{linear: {z: 6.0}}"   // 起飞
//   ros2 topic pub -r 10 /flylab/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"   // 前飞 1 m/s
//   ros2 topic pub -r 10 /flylab/cmd_vel geometry_msgs/msg/Twist "{linear: {z: -6.0}}"  // 下降触发自动降落
// ============================================================================
#include <cmath>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <geometry_msgs/msg/twist.hpp>

using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleStatus;
using sensor_msgs::msg::Range;
using geometry_msgs::msg::Twist;

// PX4 uXRCE-DDS 标配 QoS: BEST_EFFORT + TRANSIENT_LOCAL, 必须与 px4_msgs 一致
static rclcpp::QoS make_px4_qos()
{
    return rclcpp::QoS(1).best_effort().durability(rclcpp::DurabilityPolicy::TransientLocal);
}

class MoveVelocity : public rclcpp::Node
{
public:
    MoveVelocity() : Node("move_velocity")
    {
        rclcpp::QoS qos = make_px4_qos();

        // 切换模式用，包括起飞，着陆，返回，offboard等
        vehicle_command_pub_ =
            create_publisher<VehicleCommand>("/fmu/in/vehicle_command", qos);
        // 发布offboard控制模式，设置为速度控制模式
        offboard_control_mode_pub_ =
            create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", qos);
        // 发布速度值
        trajectory_setpoint_pub_ =
            create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos);

        // 订阅无人机状态信息
        vehicle_status_sub_ = create_subscription<VehicleStatus>(
            "/fmu/out/vehicle_status", qos,
            [this](const VehicleStatus::SharedPtr msg) { vehicle_status_ = *msg; });
        // 订阅向下的距离传感器，用来降落
        distance_sensor_sub_ = create_subscription<Range>(
            "/airsim_node/PX4/distance/DistanceDown", 1,
            [this](const Range::SharedPtr msg) { distance_ = msg->range; });
        // 订阅速度指令 (Twist, FLU 惯例)
        cmd_vel_sub_ = create_subscription<Twist>(
            "/flylab/cmd_vel", 1,
            [this](const Twist::SharedPtr msg) { cmd_vel_ = *msg; });

        // 10Hz 主循环
        timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                   std::bind(&MoveVelocity::timerCallback, this));
    }

private:
    // PX4 时间戳: 微秒
    uint64_t nowUs() const { return static_cast<uint64_t>(now().nanoseconds() / 1000); }

    // 解锁/加锁命令
    void arm()    { publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f); }
    void disarm() { publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f); }

    // 切换到offboard模式
    void engageOffboardMode()
    {
        publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
    }

    // 降落命令
    void land() { publishVehicleCommand(VehicleCommand::VEHICLE_CMD_NAV_LAND); }

    // 返回命令
    void returnHome() { publishVehicleCommand(VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH); }

    // 设置为offboard速度控制模式
    void publishOffboardControlHeartbeat()
    {
        OffboardControlMode msg;
        msg.position = false;
        msg.velocity = true;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = nowUs();
        offboard_control_mode_pub_->publish(msg);
    }

    // 设置各个方向速度 (输入为 NED 系)
    void publishVelocitySetpoint(float x, float y, float z, float yawspeed)
    {
        TrajectorySetpoint msg;
        msg.position = {NAN, NAN, NAN};
        msg.velocity = {x, y, z};
        msg.acceleration = {NAN, NAN, NAN};
        msg.yaw = NAN;
        msg.yawspeed = yawspeed;
        msg.timestamp = nowUs();
        trajectory_setpoint_pub_->publish(msg);
    }

    // Twist(FLU) → NED 并发布
    void publishCmdVel()
    {
        const float ned_x = cmd_vel_.linear.x;
        const float ned_y = -cmd_vel_.linear.y;        // FLU 左(y+) → NED 右(y+)
        const float ned_z = -cmd_vel_.linear.z;        // FLU 上(z+) → NED 下(z+)
        const float ned_yawrate = -cmd_vel_.angular.z; // FLU 逆时针(+) → NED 顺时针(+)
        publishVelocitySetpoint(ned_x, ned_y, ned_z, ned_yawrate);
    }

    // 发布命令
    void publishVehicleCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
    {
        VehicleCommand msg;
        msg.command = command;
        msg.param1 = param1;
        msg.param2 = param2;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = nowUs();
        vehicle_command_pub_->publish(msg);
    }

    // 主逻辑: 根据当前飞行状态和速度指令控制无人机
    void timerCallback()
    {
        // offboard心跳
        publishOffboardControlHeartbeat();

        if (vehicle_status_.arming_state == VehicleStatus::ARMING_STATE_DISARMED) {
            // 未解锁状态, 收到上升命令则解锁并起飞, 切换到OFFBOARD状态
            if (cmd_vel_.linear.z > 5.0f) {
                engageOffboardMode();
                arm();
                publishCmdVel();
            }
        } else if (vehicle_status_.nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
            // OFFBOARD 状态: 下降指令 + 低空 → 触发降落; 否则按指令飞行
            if (cmd_vel_.linear.z < -5.0f && distance_ < 5.0f) {
                land();
            } else {
                publishCmdVel();
            }
        } else if (vehicle_status_.nav_state == VehicleStatus::NAVIGATION_STATE_AUTO_LAND) {
            // 降落状态下收到上升指令, 取消降落切回OFFBOARD
            if (cmd_vel_.linear.z > 5.0f) {
                engageOffboardMode();
                publishCmdVel();
            }
        } else {
            // 其他状态 (QGC 手动控制的 POSCTL / AUTO_TAKEOFF 等) 属于
            // 地面站合法操作, 不干预, 避免与 QGC 抢控制权
        }
    }

    // 发布器
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;

    // 订阅器
    rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<Range>::SharedPtr distance_sensor_sub_;
    rclcpp::Subscription<Twist>::SharedPtr cmd_vel_sub_;

    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;

    // 状态缓存
    VehicleStatus vehicle_status_;
    Twist cmd_vel_;
    float distance_{0.0f};
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveVelocity>());
    rclcpp::shutdown();
    return 0;
}
