// vins_arming_switch —— VINS 起飞联动开关
//
// 功能: 监听 PX4 arming 状态跳变, 通过 VINS-Fusion 移植版的控制话题
//       开关其传感器输入并触发重置, 使 VINS 只在飞行期间运行:
//   起飞 (disarmed -> armed):   开 IMU + 开相机 + 重置 VINS
//   降落 (armed -> disarmed):   关 IMU + 关相机 + 重置 VINS
// 订阅: /fmu/out/vehicle_status (px4_msgs/VehicleStatus, BEST_EFFORT + TRANSIENT_LOCAL)
// 发布: /vins_imu_switch  /vins_cam_switch  /vins_restart (std_msgs/Bool)
// 参数: use_stereo (默认 true; VINS 端相机开关语义为 true=双目 / false=单目左相机,
//       mono 配置 num_of_cam: 1 若被切到双目会取不到 cam1 数据, 故需按链路配置)
// 初始: 按 ARMED 处理, 保证节点晚启动时首次收到的 DISARMED 也能触发"降落关闭"分支
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

using px4_msgs::msg::VehicleStatus;
using std_msgs::msg::Bool;

class VinsSwitch : public rclcpp::Node
{
public:
    VinsSwitch() : Node("vins_arming_switch")
    {
        // 相机模式: true=双目(vins_stereo 用), false=单目左相机(vins_mono 用)
        // VINS 端 /vins_cam_switch 的语义是 true=切双目 / false=切单目,
        // mono 配置 (num_of_cam: 1) 若被切到双目会取不到 cam1 数据, 故用参数区分
        use_stereo_ = declare_parameter<bool>("use_stereo", true);

        // 用来发布 VINS FUSION 控制话题
        cam_pub_ = create_publisher<Bool>("/vins_cam_switch", 10);     // 摄像头开关
        imu_pub_ = create_publisher<Bool>("/vins_imu_switch", 10);     // IMU开关
        restart_pub_ = create_publisher<Bool>("/vins_restart", 10);    // 重置动作

        // 和PX4通讯的QOS配置 (BEST_EFFORT + TRANSIENT_LOCAL, 与 px4_msgs 匹配)
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.best_effort();
        qos.transient_local();

        // 订阅无人机状态信息
        vehicle_status_sub_ = create_subscription<VehicleStatus>(
            "/fmu/out/vehicle_status", qos,
            std::bind(&VinsSwitch::vehicle_status_callback, this, std::placeholders::_1));

        // 初始化为 ARMED, 保证首次收到 DISARMED 状态时触发"降落关闭"分支
        vehicle_status_.arming_state = VehicleStatus::ARMING_STATE_ARMED;

        RCLCPP_INFO(get_logger(), "Start VINS FUSION switch logic...");
    }

private:
    // 辅助发话题函数
    void publish_bool(const rclcpp::Publisher<Bool>::SharedPtr &publisher, bool value)
    {
        Bool msg;
        msg.data = value;
        publisher->publish(msg);
    }

    // 获取无人机状态, 检测 arming 状态跳变
    void vehicle_status_callback(const VehicleStatus::SharedPtr msg)
    {
        if (vehicle_status_.arming_state == VehicleStatus::ARMING_STATE_ARMED &&
            msg->arming_state == VehicleStatus::ARMING_STATE_DISARMED)
        {
            // 检测到降落，关闭VINS FUSION
            publish_bool(imu_pub_, false);
            publish_bool(cam_pub_, false);
            publish_bool(restart_pub_, true);
            RCLCPP_INFO(get_logger(), "[SWITCH]: Vehicle disarmed, close VINS Fusion");
        }
        else if (vehicle_status_.arming_state == VehicleStatus::ARMING_STATE_DISARMED &&
                 msg->arming_state == VehicleStatus::ARMING_STATE_ARMED)
        {
            // 检测到起飞，重启VINS FUSION
            // 相机开关按配置模式发: 双目=true, 单目=false(切到单目左相机)
            publish_bool(imu_pub_, true);
            publish_bool(cam_pub_, use_stereo_);
            publish_bool(restart_pub_, true);
            RCLCPP_INFO(get_logger(), "[SWITCH]: Vehicle armed, open and reset VINS Fusion");
        }
        vehicle_status_ = *msg;
    }

    rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Publisher<Bool>::SharedPtr cam_pub_;
    rclcpp::Publisher<Bool>::SharedPtr imu_pub_;
    rclcpp::Publisher<Bool>::SharedPtr restart_pub_;
    VehicleStatus vehicle_status_;
    bool use_stereo_ = true;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VinsSwitch>());
    rclcpp::shutdown();
    return 0;
}
