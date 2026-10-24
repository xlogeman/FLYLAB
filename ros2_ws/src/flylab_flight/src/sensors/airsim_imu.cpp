// airsim_imu —— 通用 IMU 采集节点 (算法无关, 可插拔)
//
// 功能: 通过 RPC 直连 Cosys-AirSim 采集 IMU, 同时发布两个坐标系的版本:
// 发布: /airsim/imu/data_raw  (FRD 原生机体系, 姿态四元数为 NED->FRD 原样透传,
//                             供 OpenVINS 等按原生约定配置的算法使用)
//       /airsim/imu/data      (FLU/ENU 标准机体系, 完整姿态四元数换算,
//                             供 FAST-LIO/ORB-SLAM3 等标准 ROS 约定算法使用)
// 时间戳: 统一经 SimClockSync 映射 (ros = offset + 仿真时间), 与
//         airsim_lidar / airsim_odom 落在同一条时间轴上, FAST-LIO 等
//         多源融合算法可直接混用 (仿真慢于实时时不会系统性过积分)。
// 去重: 仿真慢于实时 (UE 过载) 时墙钟轮询会重复抓到同一仿真采样, 按
//       time_stamp 去重, 避免下游误积分。
// 参数:
//   imu_name     AirSim 端 IMU 传感器名 (默认 "Imu")
//   period_ms    采集周期 (默认 5ms = 200Hz)
//   inject_bias  注入模拟校准残余偏置 (依据 settings.json 的 ARW/VRW 自动定级,
//                默认关闭; 原 ego/orb 链路为开启)
//   vehicle_name 默认 PX4
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "airsim_common.hpp"

using namespace std::chrono_literals;

class AirsimImuNode : public rclcpp::Node {
public:
    AirsimImuNode() : Node("airsim_imu_node") {
        // 1. 参数
        vehicle_name_ = this->declare_parameter<std::string>("vehicle_name", "PX4");
        imu_name_ = this->declare_parameter<std::string>("imu_name", "Imu");
        const int period_ms = this->declare_parameter<int>("period_ms", 5);
        inject_bias_ = this->declare_parameter<bool>("inject_bias", false);

        // 2. 发布者
        auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
        pub_raw_ = this->create_publisher<sensor_msgs::msg::Imu>("/airsim/imu/data_raw", qos);
        pub_flu_ = this->create_publisher<sensor_msgs::msg::Imu>("/airsim/imu/data", qos);

        // 3. 连接 AirSim
        const std::string ip = flylab::airsim_host();
        RCLCPP_INFO(this->get_logger(), "IMU 采集节点启动 (IP: %s, 传感器: %s, 偏置注入: %s)",
                    ip.c_str(), imu_name_.c_str(), inject_bias_ ? "开" : "关");
        try {
            client_ = flylab::connect_airsim(ip);
        } catch (const rpc::rpc_error& e) {
            RCLCPP_ERROR(this->get_logger(), "连接失败: %s", e.what());
            throw std::runtime_error("AirSim Connection Failed");
        }

        // 4. 采集定时器
        timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms),
                                         std::bind(&AirsimImuNode::grab_imu, this));
    }

private:
    // 依据 settings.json 的噪声参数自动定级并推算校准残余偏置 (模拟 2 秒均值校准)
    void initialize_imu_bias() {
        try {
            const std::string settings_json = client_->getSettingsString();
            auto get_val = [&](const std::string& key) -> double {
                const size_t pos = settings_json.find("\"" + key + "\":");
                if (pos == std::string::npos) return -1.0;
                const size_t start =
                    settings_json.find_first_of("0123456789.", pos + key.length() + 1);
                const size_t end = settings_json.find_first_of(",}\n", start);
                return std::stod(settings_json.substr(start, end - start));
            };

            double arw = get_val("AngularRandomWalk");
            double vrw = get_val("VelocityRandomWalk");
            if (arw < 0) arw = 0.3;
            if (vrw < 0) vrw = 0.24;

            std::string grade_str;
            if (arw == 0.0 && vrw == 0.0) {
                grade_str = "理想级 (Ideal)";
                gyro_bias_[0] = gyro_bias_[1] = gyro_bias_[2] = 0.0;
                acc_bias_[0] = acc_bias_[1] = acc_bias_[2] = 0.0;
            } else {
                const double base_gyro_b = arw * 0.016;
                const double base_accel_b = vrw * 0.04;
                gyro_bias_[0] = base_gyro_b * 1.0;
                gyro_bias_[1] = base_gyro_b * -0.8;
                gyro_bias_[2] = base_gyro_b * 1.2;
                acc_bias_[0] = base_accel_b * 1.1;
                acc_bias_[1] = base_accel_b * 0.7;
                acc_bias_[2] = base_accel_b * -1.3;
                grade_str = (arw <= 0.01) ? "航空级" : (arw <= 0.1 ? "工业级" : "消费级");
            }
            RCLCPP_INFO(this->get_logger(), "IMU 自动定级: [%s] | ARW=%.3f, VRW=%.3f",
                        grade_str.c_str(), arw, vrw);
            is_bias_initialized_ = true;
        } catch (...) {
            gyro_bias_[0] = 0.005;
            gyro_bias_[1] = -0.003;
            gyro_bias_[2] = 0.004;
            acc_bias_[0] = 0.05;
            acc_bias_[1] = -0.04;
            acc_bias_[2] = 0.06;
            is_bias_initialized_ = true;
        }
    }

    void grab_imu() {
        try {
            if (inject_bias_ && !is_bias_initialized_) {
                initialize_imu_bias();
            }

            auto imu = client_->getImuData(imu_name_, vehicle_name_);

            // 重复采样去重 (见文件头注释)
            if (imu.time_stamp > 0 && imu.time_stamp == last_sim_stamp_) {
                return;
            }
            last_sim_stamp_ = imu.time_stamp;

            // 时间戳: 仿真 time_stamp 经统一映射; 缺失时兜底用本机时钟
            rclcpp::Time stamp;
            if (imu.time_stamp > 0) {
                const double mapped = clock_sync_.to_ros(
                    static_cast<double>(imu.time_stamp) * 1e-9, this->now().seconds());
                stamp = rclcpp::Time(static_cast<int64_t>(mapped * 1e9), RCL_ROS_TIME);
            } else {
                stamp = this->now();
            }

            // 偏置注入 (可选)
            const double ax = imu.linear_acceleration.x() + acc_bias_[0];
            const double ay = imu.linear_acceleration.y() + acc_bias_[1];
            const double az = imu.linear_acceleration.z() + acc_bias_[2];
            const double gx = imu.angular_velocity.x() + gyro_bias_[0];
            const double gy = imu.angular_velocity.y() + gyro_bias_[1];
            const double gz = imu.angular_velocity.z() + gyro_bias_[2];

            // 1. FRD 原生 (姿态四元数 NED->FRD 原样透传)
            sensor_msgs::msg::Imu raw;
            raw.header.stamp = stamp;
            raw.header.frame_id = "imu_link_frd";
            raw.angular_velocity.x = gx;
            raw.angular_velocity.y = gy;
            raw.angular_velocity.z = gz;
            raw.linear_acceleration.x = ax;
            raw.linear_acceleration.y = ay;
            raw.linear_acceleration.z = az;
            raw.orientation.x = imu.orientation.x();
            raw.orientation.y = imu.orientation.y();
            raw.orientation.z = imu.orientation.z();
            raw.orientation.w = imu.orientation.w();
            pub_raw_->publish(raw);

            // 2. FLU/ENU 标准 (相对量 y/z 取反, 姿态四元数完整换算)
            sensor_msgs::msg::Imu flu;
            flu.header.stamp = stamp;
            flu.header.frame_id = "imu_link_flu";
            flu.angular_velocity.x = gx;
            flu.angular_velocity.y = -gy;
            flu.angular_velocity.z = -gz;
            flu.linear_acceleration.x = ax;
            flu.linear_acceleration.y = -ay;
            flu.linear_acceleration.z = -az;
            const tf2::Quaternion q_raw(imu.orientation.x(), imu.orientation.y(),
                                        imu.orientation.z(), imu.orientation.w());
            tf2::Quaternion q_final = flylab::quat_nedfrd_to_enuflu(q_raw);
            q_final.normalize();
            flu.orientation.x = q_final.x();
            flu.orientation.y = q_final.y();
            flu.orientation.z = q_final.z();
            flu.orientation.w = q_final.w();
            pub_flu_->publish(flu);
        } catch (const rpc::rpc_error&) {
            // 高频轮询下的网络抖动, 静默重试
        }
    }

    std::unique_ptr<flylab::AirSimClient> client_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_raw_, pub_flu_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string vehicle_name_, imu_name_;
    bool inject_bias_ = false;
    bool is_bias_initialized_ = false;
    flylab::SimClockSync clock_sync_;
    uint64_t last_sim_stamp_ = 0;
    double gyro_bias_[3] = {0, 0, 0};
    double acc_bias_[3] = {0, 0, 0};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AirsimImuNode>());
    rclcpp::shutdown();
    return 0;
}
