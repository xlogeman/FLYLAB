// bridge_vio —— VIO 断桨门控桥接节点 (纯 ROS, 不依赖 AirSim)
//
// 功能: OpenVINS 没有类似 VINS-Fusion /vins_imu_switch 的控制话题, 因此在
//       数据源端掐断: 订阅 PX4 /fmu/out/vehicle_status, 断桨(降落)时停发
//       图像+IMU 使 VIO 等效关闭, 重新解锁后恢复 (OpenVINS 从原状态继续,
//       不做重置)。数据原样转发, 不做任何变换。
//       灰度转换在 airsim_camera (grayscale 参数) 完成, 本节点只做门控。
// 订阅: /airsim/cam/left_cam/image_raw, /airsim/cam/right_cam/image_raw,
//       /airsim/imu/data_raw  (均可在 launch 中重映射)
// 发布: /vio/left/image_raw, /vio/right/image_raw, /vio/imu/data
// 参数: gate_by_arming (默认 true, 关闭则直通)
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <atomic>
#include <memory>

class BridgeVioNode : public rclcpp::Node {
public:
    BridgeVioNode() : Node("bridge_vio_node") {
        gate_by_arming_ = this->declare_parameter<bool>("gate_by_arming", true);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
        pub_left_ = this->create_publisher<sensor_msgs::msg::Image>("/vio/left/image_raw", qos);
        pub_right_ = this->create_publisher<sensor_msgs::msg::Image>("/vio/right/image_raw", qos);
        pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("/vio/imu/data", qos);

        sub_left_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/airsim/cam/left_cam/image_raw", qos,
            [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { pass_image(pub_left_, msg); });
        sub_right_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/airsim/cam/right_cam/image_raw", qos,
            [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { pass_image(pub_right_, msg); });
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/airsim/imu/data_raw", qos,
            [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { pass_imu(msg); });

        if (gate_by_arming_) {
            // QoS 与 PX4 uXRCE-DDS 发布端匹配: best_effort + transient_local
            rclcpp::QoS px4_qos(rclcpp::KeepLast(1));
            px4_qos.best_effort().transient_local();
            vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
                "/fmu/out/vehicle_status", px4_qos,
                [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
                    const bool was_armed =
                        (vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
                    const bool now_armed =
                        (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
                    if (was_armed && !now_armed) {
                        publishing_.store(false);
                        RCLCPP_INFO(this->get_logger(),
                                    "\033[33m[ARMING] 检测到降落断桨, 已暂停发布 VIO 数据 (图像/IMU)\033[0m");
                    } else if (!was_armed && now_armed) {
                        publishing_.store(true);
                        RCLCPP_INFO(this->get_logger(),
                                    "\033[32m[ARMING] 检测到解锁起飞, 恢复发布 VIO 数据\033[0m");
                    }
                    vehicle_status_ = *msg;
                });
            // 初始按"已解锁"处理: 未接入 PX4 链路时照常发布; 首次收到 DISARMED 即暂停
            vehicle_status_.arming_state = px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
            RCLCPP_INFO(this->get_logger(), "VIO 门控桥接节点启动 (断桨联动: 开)");
        } else {
            RCLCPP_INFO(this->get_logger(), "VIO 门控桥接节点启动 (断桨联动: 关, 直通)");
        }
    }

private:
    void pass_image(const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& pub,
                    sensor_msgs::msg::Image::ConstSharedPtr msg) {
        if (gate_by_arming_ && !publishing_.load()) return;  // 断桨期间停发
        pub->publish(*msg);
    }

    void pass_imu(sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        if (gate_by_arming_ && !publishing_.load()) return;
        pub_imu_->publish(*msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_, pub_right_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_left_, sub_right_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    px4_msgs::msg::VehicleStatus vehicle_status_;
    bool gate_by_arming_ = true;
    std::atomic<bool> publishing_{true};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BridgeVioNode>());
    rclcpp::shutdown();
    return 0;
}
