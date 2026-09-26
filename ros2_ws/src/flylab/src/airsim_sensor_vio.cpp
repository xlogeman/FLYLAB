#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <thread>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // 处理 ROS 消息转换
#include <cmath> // 提供 M_PI

// AirSim C++ 客户端核心头文件
#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif
#include "rpc/rpc_error.h"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
STRICT_MODE_ON

using namespace std::chrono_literals;

// =========================================================================
// 【QoS 切换宏】
// 0: BEST_EFFORT 
// 1: RELIABLE (可靠传输)
#define USE_RELIABLE_QOS 1
// =========================================================================

class AirSimSensorVioNode : public rclcpp::Node {
public:
    AirSimSensorVioNode() : Node("airsim_sensor_vio_node"), vehicle_name_("PX4"), frame_count_(0){
        
        // 1. 设置 QoS 策略
#if USE_RELIABLE_QOS
        auto qos_fast = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
#else
        auto qos_fast = rclcpp::SensorDataQoS(); // Best Effort
#endif
        auto qos_slow = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();

        // 2. 创建发布者
        pub_img_left_ = this->create_publisher<sensor_msgs::msg::Image>("/airsim/left_cam/image_raw", qos_fast);
        pub_img_right_ = this->create_publisher<sensor_msgs::msg::Image>("/airsim/right_cam/image_raw", qos_fast);
        pub_imu_px4_ = this->create_publisher<sensor_msgs::msg::Imu>("/airsim/imu_px4/data", qos_fast);
        pub_imu_ros2_ = this->create_publisher<sensor_msgs::msg::Imu>("/airsim/imu_ros2/data", qos_fast);
        pub_gps_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/airsim/gps/data", qos_slow);
        pub_baro_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("/airsim/baro/data", qos_slow);
        pub_mag_ = this->create_publisher<sensor_msgs::msg::MagneticField>("/airsim/mag/data", qos_slow);

        // 3. 获取 IP
        const char* ip_env = std::getenv("PX4_SIM_HOST_ADDR");
        std::string airsim_ip = ip_env ? ip_env : "172.28.144.1";
        RCLCPP_INFO(this->get_logger(), "正在建立多通道 C++ 直连 (IP: %s) ...", airsim_ip.c_str());
        
        // 4. 创建 3 个独立的客户端，防止多线程 RPC 冲突
        try {
            client_img_ = std::make_unique<msr::airlib::MultirotorRpcLibClient>(airsim_ip);
            client_img_->confirmConnection();
            
            client_imu_ = std::make_unique<msr::airlib::MultirotorRpcLibClient>(airsim_ip);
            client_imu_->confirmConnection();
            
            client_low_ = std::make_unique<msr::airlib::MultirotorRpcLibClient>(airsim_ip);
            client_low_->confirmConnection();
            RCLCPP_INFO(this->get_logger(), "3条独立 RPC 数据通道连接成功！");
        } catch (rpc::rpc_error& e) {
            RCLCPP_ERROR(this->get_logger(), "连接失败: %s", e.what());
            throw std::runtime_error("AirSim Connection Failed");
        }

        // 5. 准备图像请求参数
        img_request_.push_back(msr::airlib::ImageCaptureBase::ImageRequest(
            "left_cam", msr::airlib::ImageCaptureBase::ImageType::Scene, false, false));
        img_request_.push_back(msr::airlib::ImageCaptureBase::ImageRequest(
            "right_cam", msr::airlib::ImageCaptureBase::ImageType::Scene, false, false));

        // 6. 【核心】创建独立的互斥回调组
        cb_group_img_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_imu_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_low_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // 7. 启动多线程定时器
        start_time_ = this->now();
        // 图像: 33ms (30Hz，和电脑性能相关)
        timer_img_ = this->create_wall_timer(33ms, std::bind(&AirSimSensorVioNode::grab_image, this), cb_group_img_);
        // IMU: 5ms (200Hz)
        timer_imu_ = this->create_wall_timer(5ms, std::bind(&AirSimSensorVioNode::grab_imu, this), cb_group_imu_);
        // 低频 (GPS/Baro/Mag): 20ms (50Hz)
        timer_low_ = this->create_wall_timer(20ms, std::bind(&AirSimSensorVioNode::grab_low_freq, this), cb_group_low_);

        RCLCPP_INFO(this->get_logger(), "全传感器极速采集节点启动成功！");
    }

private:
    // ================= 线程 1：双目图像发布 (极速 0拷贝) =================
    void grab_image() {
        try {
            const std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses = client_img_->simGetImages(img_request_, vehicle_name_);

            if (responses.size() == 2) {
                auto common_stamp = this->now();

                // 左目
                if (responses[0].image_data_uint8.size() > 0) {
                    const auto& resp_l = responses[0];
                    cv::Mat img_bgr_l(resp_l.height, resp_l.width, CV_8UC3, (void*)resp_l.image_data_uint8.data());
                    cv::Mat img_gray_l;
                    cv::cvtColor(img_bgr_l, img_gray_l, cv::COLOR_BGR2GRAY);

                    std_msgs::msg::Header header_l;
                    header_l.stamp = common_stamp;
                    header_l.frame_id = "left_cam_optical_frame";
                    auto msg_l = cv_bridge::CvImage(header_l, "mono8", img_gray_l).toImageMsg();
                    pub_img_left_->publish(*msg_l);
                }

                // 右目
                if (responses[1].image_data_uint8.size() > 0) {
                    const auto& resp_r = responses[1];
                    cv::Mat img_bgr_r(resp_r.height, resp_r.width, CV_8UC3, (void*)resp_r.image_data_uint8.data());
                    cv::Mat img_gray_r;
                    cv::cvtColor(img_bgr_r, img_gray_r, cv::COLOR_BGR2GRAY);

                    std_msgs::msg::Header header_r;
                    header_r.stamp = common_stamp;
                    header_r.frame_id = "right_cam_optical_frame";
                    auto msg_r = cv_bridge::CvImage(header_r, "mono8", img_gray_r).toImageMsg();
                    pub_img_right_->publish(*msg_r);
                }
                /*
                frame_count_++;
                if (frame_count_ % 100 == 0) {
                    auto current_time = this->now();
                    double dt = (current_time - start_time_).seconds();
                    RCLCPP_INFO(this->get_logger(), "已发布 %d 帧 | 当前图像 FPS: %.2f", frame_count_, 100.0 / dt);
                    start_time_ = current_time;
                }
                */
            }
        } catch (rpc::rpc_error& e) {
            RCLCPP_DEBUG(this->get_logger(), "抓图异常 (网络抖动): %s", e.what());
        }
    }

    // ================= 线程 2：极高频 IMU 发布 (200Hz) =================
    void grab_imu() {
        try {
            // 1. 从 AirSim 获取单次原始快照 (基于 NED 世界系 和 FRD 机体系)
            auto imu_data = client_imu_->getImuData("Imu", vehicle_name_);
            auto current_time = this->now();
    
            // ==========================================
            // 第一部分：发布 IMU_PX4 (原生 NED/FRD 格式)
            // ==========================================
            sensor_msgs::msg::Imu msg_px4;
            msg_px4.header.stamp = current_time;
            msg_px4.header.frame_id = "imu_link_frd"; // 标注为 FRD 系
    
            // 相对数据直接透传
            msg_px4.angular_velocity.x = imu_data.angular_velocity.x();
            msg_px4.angular_velocity.y = imu_data.angular_velocity.y();
            msg_px4.angular_velocity.z = imu_data.angular_velocity.z();
            msg_px4.linear_acceleration.x = imu_data.linear_acceleration.x();
            msg_px4.linear_acceleration.y = imu_data.linear_acceleration.y();
            msg_px4.linear_acceleration.z = imu_data.linear_acceleration.z();
    
            // 绝对姿态直接透传 (此时四元数表示的是 NED -> FRD)
            msg_px4.orientation.x = imu_data.orientation.x();
            msg_px4.orientation.y = imu_data.orientation.y();
            msg_px4.orientation.z = imu_data.orientation.z();
            msg_px4.orientation.w = imu_data.orientation.w();
    
            pub_imu_px4_->publish(msg_px4);
    
    
            // ==========================================
            // 第二部分：发布 IMU_ROS2 (转换后的 ENU/FLU 格式)
            // ==========================================
            sensor_msgs::msg::Imu msg_ros2;
            msg_ros2.header.stamp = current_time;
            msg_ros2.header.frame_id = "imu_link_flu"; // 标注为 FLU 系
    
            // A. 相对数据变换 (FRD -> FLU)
            // 逻辑：X向前不变, Y(右->左)取反, Z(下->上)取反
            msg_ros2.linear_acceleration.x =  imu_data.linear_acceleration.x();
            msg_ros2.linear_acceleration.y = -imu_data.linear_acceleration.y();
            msg_ros2.linear_acceleration.z = -imu_data.linear_acceleration.z();
    
            msg_ros2.angular_velocity.x =  imu_data.angular_velocity.x();
            msg_ros2.angular_velocity.y = -imu_data.angular_velocity.y();
            msg_ros2.angular_velocity.z = -imu_data.angular_velocity.z();
    
            // B. 绝对姿态变换 (同时改变参考系和机体系)
            
            // 1. 原始四元数 (NED -> FRD)
            tf2::Quaternion q_raw(
                imu_data.orientation.x(),
                imu_data.orientation.y(),
                imu_data.orientation.z(),
                imu_data.orientation.w()
            );
    
            // 2. 左乘：参考系从 NED 变 ENU (将世界基准对齐 ROS)
            // 绕 Z 轴旋转 90度，再绕 X 轴旋转 180 度
            tf2::Quaternion q_ned_to_enu;
            q_ned_to_enu.setRPY(M_PI, 0, M_PI / 2.0);
    
            // 3. 右乘：机体系从 FRD 变 FLU (将飞机本身定义对齐 ROS)
            // 绕机身 X 轴旋转 180 度
            tf2::Quaternion q_frd_to_flu;
            q_frd_to_flu.setRPY(M_PI, 0, 0);
    
            // 4. 合成最终四元数 (ENU -> FLU)
            // 公式：q_final = 世界变换 * 原始四元数 * 机体变换
            tf2::Quaternion q_final = q_ned_to_enu * q_raw * q_frd_to_flu;
            q_final.normalize();
    
            msg_ros2.orientation.x = q_final.x();
            msg_ros2.orientation.y = q_final.y();
            msg_ros2.orientation.z = q_final.z();
            msg_ros2.orientation.w = q_final.w();
    
            pub_imu_ros2_->publish(msg_ros2);
    
        } catch (rpc::rpc_error&) {
            // 异常处理
        }
    }
  
    // ================= 线程 3：低频传感器发布 (50Hz) =================
    void grab_low_freq() {
        try {
            auto now_stamp = this->now();
            
            // 1. GPS
            auto gps_data = client_low_->getGpsData("Gps", vehicle_name_);
            sensor_msgs::msg::NavSatFix msg_gps;
            msg_gps.header.stamp = now_stamp;
            msg_gps.header.frame_id = "gps_link";
            msg_gps.latitude = gps_data.gnss.geo_point.latitude;
            msg_gps.longitude = gps_data.gnss.geo_point.longitude;
            msg_gps.altitude = gps_data.gnss.geo_point.altitude;
            pub_gps_->publish(msg_gps);
            
            // 2. Barometer
            auto baro_data = client_low_->getBarometerData("Barometer", vehicle_name_);
            sensor_msgs::msg::FluidPressure msg_baro;
            msg_baro.header.stamp = now_stamp;
            msg_baro.header.frame_id = "baro_link";
            msg_baro.fluid_pressure = baro_data.pressure;
            pub_baro_->publish(msg_baro);
            
            // 3. Magnetometer
            auto mag_data = client_low_->getMagnetometerData("Magnetometer", vehicle_name_);
            sensor_msgs::msg::MagneticField msg_mag;
            msg_mag.header.stamp = now_stamp;
            msg_mag.header.frame_id = "mag_link";
            // 【修改点】：同样使用 .x() 函数
            msg_mag.magnetic_field.x = mag_data.magnetic_field_body.x();
            msg_mag.magnetic_field.y = mag_data.magnetic_field_body.y();
            msg_mag.magnetic_field.z = mag_data.magnetic_field_body.z();
            pub_mag_->publish(msg_mag);

        } catch (rpc::rpc_error& e) {
            RCLCPP_DEBUG(this->get_logger(), "低频传感器异常: %s", e.what());
        }
    }

    std::unique_ptr<msr::airlib::MultirotorRpcLibClient> client_img_;
    std::unique_ptr<msr::airlib::MultirotorRpcLibClient> client_imu_;
    std::unique_ptr<msr::airlib::MultirotorRpcLibClient> client_low_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_left_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_right_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_px4_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_ros2_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_gps_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pub_baro_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag_;

    rclcpp::CallbackGroup::SharedPtr cb_group_img_;
    rclcpp::CallbackGroup::SharedPtr cb_group_imu_;
    rclcpp::CallbackGroup::SharedPtr cb_group_low_;

    rclcpp::TimerBase::SharedPtr timer_img_;
    rclcpp::TimerBase::SharedPtr timer_imu_;
    rclcpp::TimerBase::SharedPtr timer_low_;

    std::vector<msr::airlib::ImageCaptureBase::ImageRequest> img_request_;
    std::string vehicle_name_;
    int frame_count_;
    rclcpp::Time start_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AirSimSensorVioNode>();
    
    // 使用 4 个线程的执行器，保证 3 个独立回调组绝对并行，互不阻塞！
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
