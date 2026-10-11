// airsim_nav —— 低频导航传感器采集节点 (GPS/气压计/磁力计, 算法无关)
//
// 功能: 通过 RPC 直连 Cosys-AirSim 采集 GNSS/气压计/磁力计, 50Hz 单通道发布。
// 发布: /airsim/gps   (sensor_msgs/NavSatFix: 经纬度/海拔)
//       /airsim/baro  (sensor_msgs/FluidPressure: Pa)
//       /airsim/mag   (sensor_msgs/MagneticField: FLU 机体系, y/z 取反)
// 参数:
//   period_ms     采集周期 (默认 20ms = 50Hz)
//   vehicle_name  默认 PX4
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include "airsim_common.hpp"

using namespace std::chrono_literals;

class AirsimNavNode : public rclcpp::Node {
public:
    AirsimNavNode() : Node("airsim_nav_node") {
        vehicle_name_ = this->declare_parameter<std::string>("vehicle_name", "PX4");
        const int period_ms = this->declare_parameter<int>("period_ms", 20);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
        pub_gps_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/airsim/gps", qos);
        pub_baro_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("/airsim/baro", qos);
        pub_mag_ = this->create_publisher<sensor_msgs::msg::MagneticField>("/airsim/mag", qos);

        const std::string ip = flylab::airsim_host();
        RCLCPP_INFO(this->get_logger(), "低频导航传感器节点启动 (IP: %s, GPS/气压/磁力计 50Hz)",
                    ip.c_str());
        try {
            client_ = flylab::connect_airsim(ip);
        } catch (const rpc::rpc_error& e) {
            RCLCPP_ERROR(this->get_logger(), "连接失败: %s", e.what());
            throw std::runtime_error("AirSim Connection Failed");
        }

        timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms),
                                         std::bind(&AirsimNavNode::grab_low_freq, this));
    }

private:
    void grab_low_freq() {
        try {
            const auto stamp = this->now();

            // GNSS
            auto gps_data = client_->getGpsData("Gps", vehicle_name_);
            sensor_msgs::msg::NavSatFix gps;
            gps.header.stamp = stamp;
            gps.header.frame_id = "gps_link";
            gps.latitude = gps_data.gnss.geo_point.latitude;
            gps.longitude = gps_data.gnss.geo_point.longitude;
            gps.altitude = gps_data.gnss.geo_point.altitude;
            pub_gps_->publish(gps);

            // 气压计
            auto baro_data = client_->getBarometerData("Barometer", vehicle_name_);
            sensor_msgs::msg::FluidPressure baro;
            baro.header.stamp = stamp;
            baro.header.frame_id = "baro_link";
            baro.fluid_pressure = baro_data.pressure;
            pub_baro_->publish(baro);

            // 磁力计 (FRD -> FLU: x 不变, y/z 取反)
            auto mag_data = client_->getMagnetometerData("Magnetometer", vehicle_name_);
            sensor_msgs::msg::MagneticField mag;
            mag.header.stamp = stamp;
            mag.header.frame_id = "mag_link_flu";
            mag.magnetic_field.x = mag_data.magnetic_field_body.x();
            mag.magnetic_field.y = -mag_data.magnetic_field_body.y();
            mag.magnetic_field.z = -mag_data.magnetic_field_body.z();
            pub_mag_->publish(mag);
        } catch (const rpc::rpc_error& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "抓取低频传感器失败: RPC 通信异常! (%s)", e.what());
        }
    }

    std::unique_ptr<flylab::AirSimClient> client_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_gps_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pub_baro_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string vehicle_name_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AirsimNavNode>());
    rclcpp::shutdown();
    return 0;
}
