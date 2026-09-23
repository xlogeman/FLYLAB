#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <thread>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <cmath> 
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

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

class AirSimFullSensorNode : public rclcpp::Node {
public:
    AirSimFullSensorNode() : Node("airsim_full_sensor_node"), vehicle_name_("PX4"), frame_count_(0){
        
        // 1. 设置 QoS 策略
#if USE_RELIABLE_QOS
        auto qos_fast = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
#else
        auto qos_fast = rclcpp::SensorDataQoS(); // Best Effort
#endif
        auto qos_slow = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();

        // 2. 创建发布者 
        pub_img_depth_ = this->create_publisher<sensor_msgs::msg::Image>("/airsim/depth_cam/image_raw", qos_fast);
        pub_imu_px4_ = this->create_publisher<sensor_msgs::msg::Imu>("/airsim/imu_px4/data", qos_fast);
        pub_imu_ros2_ = this->create_publisher<sensor_msgs::msg::Imu>("/airsim/imu_ros2/data", qos_fast);
        pub_true_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/airsim/true_odom/data", qos_fast);
        pub_gps_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/airsim/gps/data", qos_slow);
        pub_baro_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("/airsim/baro/data", qos_slow);
        pub_mag_ = this->create_publisher<sensor_msgs::msg::MagneticField>("/airsim/mag/data", qos_slow);

        // 3. 获取 IP
        const char* ip_env = std::getenv("PX4_SIM_HOST_ADDR");
        std::string airsim_ip = ip_env ? ip_env : "172.28.144.1";
        RCLCPP_INFO(this->get_logger(), "正在建立多通道 C++ 直连 (IP: %s) ...", airsim_ip.c_str());
        
        // 4. 创建 3 个独立的客户端
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

        // 5. 准备图像请求参数 (修改为单目 DepthPlanar，且要求返回 float 数据)
        img_request_.push_back(msr::airlib::ImageCaptureBase::ImageRequest(
            "depth_cam", msr::airlib::ImageCaptureBase::ImageType::DepthPlanar, true, false));

        // 6. 创建互斥回调组
        cb_group_img_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_imu_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_low_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // 7. 启动多线程定时器
        start_time_ = this->now();
        // 图像: 33ms (默认30Hz)
        timer_img_ = this->create_wall_timer(33ms, std::bind(&AirSimFullSensorNode::grab_image, this), cb_group_img_);
        // IMU: 5ms (200Hz)
        timer_imu_ = this->create_wall_timer(5ms, std::bind(&AirSimFullSensorNode::grab_imu, this), cb_group_imu_);
        // 低频 (GPS/Baro/Mag): 20ms (50Hz)
        timer_low_ = this->create_wall_timer(20ms, std::bind(&AirSimFullSensorNode::grab_low_freq, this), cb_group_low_);

        // 初始化偏置标记
        is_imu_initialized_ = false;
    }

private:
    // =========================================================================
    // 【新增函数】 自动读取配置并计算残余偏置 (模拟校准后的温漂残余)
    // =========================================================================
    void initialize_imu_bias() {
        try {
            std::string settings_json = client_imu_->getSettingsString();
            
            auto get_val = [&](const std::string& key) -> double {
                size_t pos = settings_json.find("\"" + key + "\":");
                if (pos == std::string::npos) {
                    return -1.0;
                }
                size_t start = settings_json.find_first_of("0123456789.", pos + key.length() + 1);
                size_t end = settings_json.find_first_of(",}\n", start);
                return std::stod(settings_json.substr(start, end - start));
            };

            double arw = get_val("AngularRandomWalk");
            double vrw = get_val("VelocityRandomWalk");

            // 兜底处理
            if (arw < 0) {
                arw = 0.3; 
            }
            if (vrw < 0) {
                vrw = 0.24;
            }

            std::string grade_str;

            // 理想级判定
            if (arw == 0.0 && vrw == 0.0) {
                grade_str = "理想级 (Ideal)";
                gyro_bias_x_ = 0.0;
                gyro_bias_y_ = 0.0;
                gyro_bias_z_ = 0.0;
                acc_bias_x_ = 0.0;
                acc_bias_y_ = 0.0;
                acc_bias_z_ = 0.0;
            } else {
                // 基于白噪声水平推算校准后的残余偏置 (模拟 2秒均值校准残差)
                double base_gyro_b = arw * 0.016; 
                double base_accel_b = vrw * 0.04;

                gyro_bias_x_ = base_gyro_b * 1.0;  
                gyro_bias_y_ = base_gyro_b * -0.8; 
                gyro_bias_z_ = base_gyro_b * 1.2;
                
                acc_bias_x_  = base_accel_b * 1.1; 
                acc_bias_y_  = base_accel_b * 0.7;  
                acc_bias_z_  = base_accel_b * -1.3;
                
                grade_str = (arw <= 0.01) ? "航空级" : (arw <= 0.1 ? "工业级" : "消费级");
            }

            RCLCPP_INFO(this->get_logger(), "IMU 自动定级: [%s] | ARW=%.3f, VRW=%.3f", grade_str.c_str(), arw, vrw);
            is_imu_initialized_ = true;
        } catch (...) {
            gyro_bias_x_ = 0.005; 
            gyro_bias_y_ = -0.003; 
            gyro_bias_z_ = 0.004;
            
            acc_bias_x_  = 0.05;  
            acc_bias_y_  = -0.04;  
            acc_bias_z_  = 0.06; 
            
            is_imu_initialized_ = true;
        }
    }

    // ================= 线程 1：深度图像发布 =================
    void grab_image()
    {
        try
        {
            const std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses = client_img_->simGetImages(img_request_, vehicle_name_);

            if (responses.size() == 1) {
                auto common_stamp = this->now();

                // 深度图数据在 image_data_float 中
                if (responses[0].image_data_float.size() > 0) {
                    
                    // 构建 32位浮点型 OpenCV 矩阵
                    cv::Mat img_depth(responses[0].height, responses[0].width, CV_32FC1, (void*)responses[0].image_data_float.data());
                    
                    std_msgs::msg::Header header; 
                    header.stamp = common_stamp; 
                    header.frame_id = "camera_optical_frame"; // 相机光学坐标系
                    
                    // 使用 cv_bridge 将浮点矩阵转为 sensor_msgs/Image，编码指定为 "32FC1"
                    pub_img_depth_->publish(*(cv_bridge::CvImage(header, "32FC1", img_depth).toImageMsg()));
                } else {
                    RCLCPP_WARN(this->get_logger(), "深度图为空！请检查 ImageRequest 是否设置了 pixels_as_float = true");
                }
            }
        }
        catch (rpc::rpc_error& e)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "抓取 Camera Image 失败: RPC 通信异常! (%s)", e.what());
        }
    }

    // ================= 线程 2：IMU 与 真值里程计 (注入偏置, 四元数清零) =================
    void grab_imu() {
        try {
            if (!is_imu_initialized_) {
                initialize_imu_bias();
            }

            auto imu_data = client_imu_->getImuData("Imu", vehicle_name_);
            auto odom_data = client_imu_->simGetGroundTruthKinematics(vehicle_name_);
            auto current_time = this->now();

            // 注入计算出的物理偏置
            double f_acc_x = imu_data.linear_acceleration.x() + acc_bias_x_;
            double f_acc_y = imu_data.linear_acceleration.y() + acc_bias_y_;
            double f_acc_z = imu_data.linear_acceleration.z() + acc_bias_z_;
            double f_gyr_x = imu_data.angular_velocity.x() + gyro_bias_x_;
            double f_gyr_y = imu_data.angular_velocity.y() + gyro_bias_y_;
            double f_gyr_z = imu_data.angular_velocity.z() + gyro_bias_z_;
    
            // 1. 发布PX4坐标体系下的IMU_PX4，就是原始数据即可 (带偏置, 四元数清零)
            sensor_msgs::msg::Imu msg_px4;
            msg_px4.header.stamp = current_time;
            msg_px4.header.frame_id = "imu_link_frd";  // 标注为FRD坐标系
            msg_px4.linear_acceleration.x = f_acc_x; 
            msg_px4.linear_acceleration.y = f_acc_y; 
            msg_px4.linear_acceleration.z = f_acc_z;
            msg_px4.angular_velocity.x = f_gyr_x; 
            msg_px4.angular_velocity.y = f_gyr_y; 
            msg_px4.angular_velocity.z = f_gyr_z;
            msg_px4.orientation.w = 1.0; 
            msg_px4.orientation.x = 0.0; 
            msg_px4.orientation.y = 0.0; 
            msg_px4.orientation.z = 0.0;
            msg_px4.orientation_covariance[0] = -1.0;
            pub_imu_px4_->publish(msg_px4);
    
            // 2. 发布ROS2坐标体系下的IMU_ROS2，需要进行FRD到FLU的转换(带偏置, 机体坐标系转换, 四元数清零) 
            sensor_msgs::msg::Imu msg_ros2;
            msg_ros2.header.stamp = current_time;
            msg_ros2.header.frame_id = "imu_link_flu"; // 标注为FLU坐标系

            // 坐标系变换 (FRD -> FLU)
            // 逻辑：F-X到F-X(前->前)不变, R-Y到L-Y(右->左)取反, D-Z到U-Z(下->上)取反
            msg_ros2.linear_acceleration.x =  f_acc_x;
            msg_ros2.linear_acceleration.y = -f_acc_y; 
            msg_ros2.linear_acceleration.z = -f_acc_z;
            msg_ros2.angular_velocity.x =  f_gyr_x;
            msg_ros2.angular_velocity.y = -f_gyr_y;
            msg_ros2.angular_velocity.z = -f_gyr_z;
            msg_ros2.orientation.w = 1.0; 
            msg_ros2.orientation.x = 0.0; 
            msg_ros2.orientation.y = 0.0; 
            msg_ros2.orientation.z = 0.0;
            msg_ros2.orientation_covariance[0] = -1.0;
            
            pub_imu_ros2_->publish(msg_ros2);
    
            // 3. 发布ROS2坐标体系下的真值里程计，需要进行NED到ENU的转换(世界坐标系转换，保留四元数用于可视化) 
            nav_msgs::msg::Odometry msg_true_odom;
            msg_true_odom.header.stamp = current_time;
            msg_true_odom.header.frame_id = "world";
            msg_true_odom.child_frame_id = "base_link";
            // 坐标系变换 (NED -> ENU)
            // 逻辑：N-X到N-Y(X->Y), E-Y到N-X(Y->X), D-Z到U-Z(下->上)取反
            msg_true_odom.pose.pose.position.x = odom_data.pose.position.y(); 
            msg_true_odom.pose.pose.position.y = odom_data.pose.position.x(); 
            msg_true_odom.pose.pose.position.z = -odom_data.pose.position.z();
            
            tf2::Quaternion q_raw(odom_data.pose.orientation.x(), odom_data.pose.orientation.y(), odom_data.pose.orientation.z(), odom_data.pose.orientation.w());
            tf2::Quaternion q_n2e; 
            q_n2e.setRPY(M_PI, 0, M_PI / 2.0); 
            tf2::Quaternion q_f2f; 
            q_f2f.setRPY(M_PI, 0, 0);
            tf2::Quaternion q_final = q_n2e * q_raw * q_f2f;
            
            msg_true_odom.pose.pose.orientation.x = q_final.x(); 
            msg_true_odom.pose.pose.orientation.y = q_final.y(); 
            msg_true_odom.pose.pose.orientation.z = q_final.z(); 
            msg_true_odom.pose.pose.orientation.w = q_final.w();
            
            pub_true_odom_->publish(msg_true_odom);

            // =======================================================
            // 【必须新增】：同时发布 TF 树 (world -> base_link)
            // =======================================================
            if (!tf_broadcaster_) {
                tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            }

            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = current_time;
            t.header.frame_id = "world";
            t.child_frame_id = "base_link";

            // 使用和 Odom 完全一样的平移和旋转数据
            t.transform.translation.x = msg_true_odom.pose.pose.position.x;
            t.transform.translation.y = msg_true_odom.pose.pose.position.y;
            t.transform.translation.z = msg_true_odom.pose.pose.position.z;
            
            t.transform.rotation.x = msg_true_odom.pose.pose.orientation.x;
            t.transform.rotation.y = msg_true_odom.pose.pose.orientation.y;
            t.transform.rotation.z = msg_true_odom.pose.pose.orientation.z;
            t.transform.rotation.w = msg_true_odom.pose.pose.orientation.w;

            tf_broadcaster_->sendTransform(t);
            // =======================================================
        }
        catch (rpc::rpc_error& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "抓取 IMU 失败: RPC 通信异常! (%s)", e.what());
        }
    }
  
    // ================= 线程 3：低频传感器发布 =================
    void grab_low_freq() {
        try {
            auto now_stamp = this->now();
            auto gps_data = client_low_->getGpsData("Gps", vehicle_name_);

            sensor_msgs::msg::NavSatFix msg_gps;
            msg_gps.header.stamp = now_stamp; 
            msg_gps.header.frame_id = "gps_link";
            msg_gps.latitude = gps_data.gnss.geo_point.latitude; 
            msg_gps.longitude = gps_data.gnss.geo_point.longitude; 
            msg_gps.altitude = gps_data.gnss.geo_point.altitude;
            pub_gps_->publish(msg_gps);

            auto baro_data = client_low_->getBarometerData("Barometer", vehicle_name_);
            sensor_msgs::msg::FluidPressure msg_baro;
            msg_baro.header.stamp = now_stamp; 
            msg_baro.header.frame_id = "baro_link";
            msg_baro.fluid_pressure = baro_data.pressure;
            pub_baro_->publish(msg_baro);

            // 发布ROS2坐标体系下的磁力计，需要进行FRD到FLU的转换
            auto mag_data = client_low_->getMagnetometerData("Magnetometer", vehicle_name_);
            sensor_msgs::msg::MagneticField msg_mag;
            msg_mag.header.stamp = now_stamp; 
            msg_mag.header.frame_id = "mag_link_flu";  // 标注为FLU坐标系
            // 坐标系变换 (FRD -> FLU)
            // 逻辑：F-X到F-X(前->前)不变, R-Y到L-Y(右->左)取反, D-Z到U-Z(下->上)取反
            msg_mag.magnetic_field.x =  mag_data.magnetic_field_body.x();
            msg_mag.magnetic_field.y = -mag_data.magnetic_field_body.y();
            msg_mag.magnetic_field.z = -mag_data.magnetic_field_body.z();

            pub_mag_->publish(msg_mag);
        }
        catch (rpc::rpc_error& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "抓取 GPS 失败: RPC 通信异常! (%s)", e.what());
        }
    }

    std::unique_ptr<msr::airlib::MultirotorRpcLibClient> client_img_, client_imu_, client_low_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_depth_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_px4_, pub_imu_ros2_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_true_odom_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_gps_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pub_baro_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag_;
    rclcpp::CallbackGroup::SharedPtr cb_group_img_, cb_group_imu_, cb_group_low_;
    rclcpp::TimerBase::SharedPtr timer_img_, timer_imu_, timer_low_;
    std::vector<msr::airlib::ImageCaptureBase::ImageRequest> img_request_;
    std::string vehicle_name_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    int frame_count_;
    rclcpp::Time start_time_;
    bool is_imu_initialized_;
    double gyro_bias_x_, gyro_bias_y_, gyro_bias_z_;
    double acc_bias_x_, acc_bias_y_, acc_bias_z_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AirSimFullSensorNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}