// airsim_sensor_lidar —— FAST-LIO 链路 AirSim 直连传感器节点
//
// 功能: 通过 RPC 直连 Cosys-AirSim 采集 3D LiDAR + IMU 数据并发布 ROS2 话题,
//       供 FAST-LIO (fast_lio.launch.py) 使用, 不依赖 airsim_ros_pkgs。
//       Cosys-AirSim LiDAR 输出无逐点时间戳、无 intensity (整帧只有一个时间戳),
//       本节点按旋转方位角合成 velodyne 风格 per-point time, 供 FAST-LIO 运动畸变校正;
//       未命中的 (0,0,0) 点被过滤, intensity/ring 以参数填充。
// 发布: /airsim/lidar/points  (PointCloud2: x,y,z,intensity,ring,time; FLU; frame=lidar_link)
//       /airsim/imu/data      (IMU, FRD->FLU 转换后, 200Hz)
// 采集: 2 个独立 RPC 客户端 (LiDAR/IMU) 多线程并行, 互不阻塞。
// 连接: IP 读环境变量 PX4_SIM_HOST_ADDR, 未设置则默认 172.28.144.1。
// 外参: 发布静态 TF base_link -> lidar_link (挂载参数与 settings.json 一致, NED->FLU 换算)。
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <chrono>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

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

class AirSimSensorLidarNode : public rclcpp::Node {
public:
    AirSimSensorLidarNode() : Node("airsim_sensor_lidar_node") {
        // 1. 参数 (与 configs/fast_lio/settings.json 保持一致)
        vehicle_name_ = this->declare_parameter<std::string>("vehicle_name", "PX4");
        lidar_name_ = this->declare_parameter<std::string>("lidar_sensor_name", "Lidar1");
        imu_name_ = this->declare_parameter<std::string>("imu_sensor_name", "Imu");
        num_channels_ = static_cast<uint16_t>(this->declare_parameter<int>("num_channels", 16));
        meas_per_cycle_ = this->declare_parameter<int>("measurements_per_cycle", 1800);
        rotation_hz_ = this->declare_parameter<double>("rotation_frequency", 10.0);
        intensity_ = this->declare_parameter<double>("intensity", 100.0);
        cloud_frame_ = this->declare_parameter<std::string>("cloud_frame_id", "lidar_link");
        // 挂载位置/姿态: 与 settings.json 中 LiDAR 的 X/Y/Z/Roll/Pitch/Yaw 一致 (NED 机体系, 米/度)
        mount_x_ = this->declare_parameter<double>("mount_x", 0.0);
        mount_y_ = this->declare_parameter<double>("mount_y", 0.0);
        mount_z_ = this->declare_parameter<double>("mount_z", 0.0);
        mount_roll_ = this->declare_parameter<double>("mount_roll_deg", 0.0);
        mount_pitch_ = this->declare_parameter<double>("mount_pitch_deg", 0.0);
        mount_yaw_ = this->declare_parameter<double>("mount_yaw_deg", 0.0);
        int lidar_ms = this->declare_parameter<int>("lidar_period_ms", 100);  // 10Hz
        int imu_ms = this->declare_parameter<int>("imu_period_ms", 5);        // 200Hz

        // 2. 发布者 (QoS 与其他 flylab 传感器节点一致: reliable)
        auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
        pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/airsim/lidar/points", qos);
        pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("/airsim/imu/data", qos);

        // 3. 静态 TF: base_link -> lidar_link (settings 挂载值 NED -> FLU)
        publish_mount_tf();

        // 4. 连接 AirSim (LiDAR/IMU 各一条独立 RPC 通道)
        const char* ip_env = std::getenv("PX4_SIM_HOST_ADDR");
        std::string airsim_ip = ip_env ? ip_env : "172.28.144.1";
        RCLCPP_INFO(this->get_logger(), "正在建立 LiDAR/IMU 双通道直连 (IP: %s) ...", airsim_ip.c_str());
        try {
            client_lidar_ = std::make_unique<msr::airlib::MultirotorRpcLibClient>(airsim_ip);
            client_lidar_->confirmConnection();
            client_imu_ = std::make_unique<msr::airlib::MultirotorRpcLibClient>(airsim_ip);
            client_imu_->confirmConnection();
            RCLCPP_INFO(this->get_logger(), "2条独立 RPC 数据通道连接成功！");
        } catch (rpc::rpc_error& e) {
            RCLCPP_ERROR(this->get_logger(), "连接失败: %s", e.what());
            throw std::runtime_error("AirSim Connection Failed");
        }

        // 5. 多线程定时器 (互不阻塞)
        cb_group_lidar_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_imu_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_lidar_ = this->create_wall_timer(std::chrono::milliseconds(lidar_ms),
            std::bind(&AirSimSensorLidarNode::grab_lidar, this), cb_group_lidar_);
        timer_imu_ = this->create_wall_timer(std::chrono::milliseconds(imu_ms),
            std::bind(&AirSimSensorLidarNode::grab_imu, this), cb_group_imu_);
    }

private:
    // ================= 静态 TF: 挂载外参 NED -> FLU =================
    void publish_mount_tf() {
        // 平移: NED 机体系 (x前,y右,z下) -> FLU (x前,y左,z上)
        geometry_msgs::msg::TransformStamped t;
        t.header.frame_id = "base_link";
        t.child_frame_id = cloud_frame_;
        t.transform.translation.x = mount_x_;
        t.transform.translation.y = -mount_y_;
        t.transform.translation.z = -mount_z_;

        // 旋转: R_flu = M * R_ned * M^T, M = diag(1,-1,-1)
        tf2::Quaternion q_ned;
        q_ned.setRPY(
            mount_roll_ * M_PI / 180.0,
            mount_pitch_ * M_PI / 180.0,
            mount_yaw_ * M_PI / 180.0);
        tf2::Matrix3x3 r_ned(q_ned);
        tf2::Matrix3x3 m(1, 0, 0, 0, -1, 0, 0, 0, -1);
        tf2::Matrix3x3 r_flu = m * r_ned * m.transpose();
        double roll, pitch, yaw;
        r_flu.getRPY(roll, pitch, yaw);
        tf2::Quaternion q_flu;
        q_flu.setRPY(roll, pitch, yaw);
        t.transform.rotation.x = q_flu.x();
        t.transform.rotation.y = q_flu.y();
        t.transform.rotation.z = q_flu.z();
        t.transform.rotation.w = q_flu.w();

        static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
        static_tf_->sendTransform(t);
    }

    // ================= 线程 1: LiDAR 点云发布 =================
    void grab_lidar() {
        try {
            auto lidar_data = client_lidar_->getLidarData(lidar_name_, vehicle_name_);
            const auto stamp = this->now();
            const size_t total = lidar_data.point_cloud.size() / 3;
            if (total == 0) {
                return;
            }

            // 每列方位角对应的时间步长 (秒): 一转 360° 均分 measurements_per_cycle 列
            const double col_period =
                (rotation_hz_ > 0.0) ? 1.0 / (rotation_hz_ * meas_per_cycle_) : 0.0;

            // velodyne 风格字段: x,y,z,intensity (float32), ring (uint16), time (float32)
            struct PointXYZIRT { float x, y, z, intensity; uint16_t ring; float time; };
            std::vector<PointXYZIRT> pts;
            pts.reserve(total);

            size_t hits = 0;
            for (size_t i = 0; i < total; ++i) {
                const float px = lidar_data.point_cloud[i * 3];
                const float py = lidar_data.point_cloud[i * 3 + 1];
                const float pz = lidar_data.point_cloud[i * 3 + 2];
                // 未命中点为 (0,0,0), 过滤
                if (px == 0.0f && py == 0.0f && pz == 0.0f) {
                    continue;
                }
                const size_t col = i / num_channels_;   // 点云按列 (方位角) 主序排列
                const uint16_t ring = static_cast<uint16_t>(i % num_channels_);
                PointXYZIRT p;
                // 雷达局部 NED -> 雷达局部 FLU
                p.x = px;
                p.y = -py;
                p.z = -pz;
                p.intensity = static_cast<float>(intensity_);
                p.ring = ring;
                p.time = static_cast<float>(col * col_period);
                pts.push_back(p);
                ++hits;
            }
            if (pts.empty()) {
                return;
            }

            sensor_msgs::msg::PointCloud2 msg;
            msg.header.stamp = stamp;               // ≈ 扫描结束时刻
            msg.header.frame_id = cloud_frame_;     // FLU 语义
            msg.height = 1;
            msg.width = static_cast<uint32_t>(pts.size());
            msg.is_dense = true;
            msg.is_bigendian = false;
            msg.fields.resize(6);
            msg.fields[0] = make_field("x", 0, sensor_msgs::msg::PointField::FLOAT32);
            msg.fields[1] = make_field("y", 4, sensor_msgs::msg::PointField::FLOAT32);
            msg.fields[2] = make_field("z", 8, sensor_msgs::msg::PointField::FLOAT32);
            msg.fields[3] = make_field("intensity", 12, sensor_msgs::msg::PointField::FLOAT32);
            msg.fields[4] = make_field("ring", 16, sensor_msgs::msg::PointField::UINT16);
            msg.fields[5] = make_field("time", 20, sensor_msgs::msg::PointField::FLOAT32);
            msg.point_step = sizeof(PointXYZIRT);   // 24 字节 (ring 后 2 字节对齐填充)
            msg.row_step = msg.point_step * msg.width;
            msg.data.resize(msg.row_step);
            std::memcpy(msg.data.data(), pts.data(), msg.row_step);

            pub_cloud_->publish(msg);
            RCLCPP_DEBUG(this->get_logger(), "LiDAR: %zu/%zu 点有效", hits, total);
        }
        catch (rpc::rpc_error& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "抓取 LiDAR 失败: RPC 通信异常! (%s)", e.what());
        }
    }

    static sensor_msgs::msg::PointField make_field(
        const std::string& name, uint32_t offset, uint8_t datatype) {
        sensor_msgs::msg::PointField f;
        f.name = name;
        f.offset = offset;
        f.datatype = datatype;
        f.count = 1;
        return f;
    }

    // ================= 线程 2: IMU 发布 (FRD -> FLU, 原始数据) =================
    void grab_imu() {
        try {
            auto imu_data = client_imu_->getImuData(imu_name_, vehicle_name_);
            const auto stamp = this->now();

            sensor_msgs::msg::Imu msg;
            msg.header.stamp = stamp;
            msg.header.frame_id = "imu_link_flu";
            // 坐标系变换 (FRD -> FLU): x 不变, y/z 取反
            msg.linear_acceleration.x = imu_data.linear_acceleration.x();
            msg.linear_acceleration.y = -imu_data.linear_acceleration.y();
            msg.linear_acceleration.z = -imu_data.linear_acceleration.z();
            msg.angular_velocity.x = imu_data.angular_velocity.x();
            msg.angular_velocity.y = -imu_data.angular_velocity.y();
            msg.angular_velocity.z = -imu_data.angular_velocity.z();
            // FAST-LIO 不使用姿态四元数, 置 identity 并标记无效
            msg.orientation.w = 1.0;
            msg.orientation.x = 0.0;
            msg.orientation.y = 0.0;
            msg.orientation.z = 0.0;
            msg.orientation_covariance[0] = -1.0;
            pub_imu_->publish(msg);
        }
        catch (rpc::rpc_error& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "抓取 IMU 失败: RPC 通信异常! (%s)", e.what());
        }
    }

    std::unique_ptr<msr::airlib::MultirotorRpcLibClient> client_lidar_, client_imu_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    rclcpp::CallbackGroup::SharedPtr cb_group_lidar_, cb_group_imu_;
    rclcpp::TimerBase::SharedPtr timer_lidar_, timer_imu_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
    std::string vehicle_name_, lidar_name_, imu_name_, cloud_frame_;
    uint16_t num_channels_;
    int meas_per_cycle_;
    double rotation_hz_, intensity_;
    double mount_x_, mount_y_, mount_z_, mount_roll_, mount_pitch_, mount_yaw_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AirSimSensorLidarNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
