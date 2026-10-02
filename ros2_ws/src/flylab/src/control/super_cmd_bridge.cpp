// super_cmd_bridge —— SUPER 输出 -> tracker 输入 的消息桥接节点
//
// SUPER (hku-mars fsm_node) 输出的是 mars_quadrotor_msgs/PositionCommand
// (自定义消息, 比 flylab tracker 用的标准 quadrotor_msgs/PositionCommand
// 多了 angular_velocity/attitude/thrust/jerk 等字段), 而 tracker 链路
// (tracker_px4 / tracker_mavros) 统一订阅标准 quadrotor_msgs/PositionCommand。
// 本节点把两者对接: 订阅 SUPER 控制指令, 转成标准消息发给 tracker,
// 使 SUPER 与 EGO-Planner 共用同一套轨迹跟踪链路。
//
// 订阅: super_cmd_topic (默认 /super/pos_cmd, mars_quadrotor_msgs/PositionCommand,
//        与 SUPER 的 config/flylab_super.yaml 中 fsm/cmd_topic 对应)
// 发布: pos_cmd_topic  (默认 /planning/pos_cmd, quadrotor_msgs/PositionCommand,
//        与 tracker 的 pos_cmd_topic 参数对应)
//
// 字段映射: position/velocity/acceleration/yaw/yaw_dot 一一对应;
// mars 版的 attitude 是 RPY 欧拉角、thrust 是归一化推力, 标准 tracker 不消费,
// 不做转换直接丢弃; trajectory_flag 两版消息常量值一致
// (EMPTY=0/READY=1/COMPLETED=3), 直接透传。
#include <rclcpp/rclcpp.hpp>
#include <mars_quadrotor_msgs/msg/position_command.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>

class SuperCmdBridgeNode : public rclcpp::Node {
public:
    SuperCmdBridgeNode() : Node("super_cmd_bridge_node") {
        super_cmd_topic_ = this->declare_parameter<std::string>("super_cmd_topic", "/super/pos_cmd");
        pos_cmd_topic_ = this->declare_parameter<std::string>("pos_cmd_topic", "/planning/pos_cmd");

        pub_cmd_ = this->create_publisher<quadrotor_msgs::msg::PositionCommand>(pos_cmd_topic_, 10);
        sub_cmd_ = this->create_subscription<mars_quadrotor_msgs::msg::PositionCommand>(
            super_cmd_topic_, rclcpp::SensorDataQoS(),
            [this](const mars_quadrotor_msgs::msg::PositionCommand::SharedPtr msg) { cmdCallback(msg); });

        RCLCPP_INFO(this->get_logger(),
                    "SUPER 指令桥接已启动: %s (mars) -> %s (standard)",
                    super_cmd_topic_.c_str(), pos_cmd_topic_.c_str());
    }

private:
    void cmdCallback(const mars_quadrotor_msgs::msg::PositionCommand::SharedPtr msg) {
        // 空指令不转发 (SUPER 心跳/空闲期输出 EMPTY)
        if (msg->trajectory_flag == mars_quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_EMPTY) {
            return;
        }

        quadrotor_msgs::msg::PositionCommand out;
        out.header = msg->header;
        out.header.frame_id = "world";
        out.position = msg->position;
        out.velocity = msg->velocity;
        out.acceleration = msg->acceleration;
        out.yaw = msg->yaw;
        out.yaw_dot = msg->yaw_dot;
        out.trajectory_id = msg->trajectory_id;
        out.trajectory_flag = msg->trajectory_flag;

        pub_cmd_->publish(out);
    }

    std::string super_cmd_topic_, pos_cmd_topic_;
    rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pub_cmd_;
    rclcpp::Subscription<mars_quadrotor_msgs::msg::PositionCommand>::SharedPtr sub_cmd_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SuperCmdBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
