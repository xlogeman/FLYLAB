// airsim_sensor_super —— SUPER 链路 AirSim 直连传感器节点
//
// 功能: 通过 RPC 直连 Cosys-AirSim 采集 3D LiDAR + 真值里程计, 转换成
//       SUPER (hku-mars, rog_map) 需要的输入格式, 不依赖 airsim_ros_pkgs。
//       与 airsim_sensor_lidar (FAST-LIO 链路) 的区别: SUPER 不做里程计融合,
//       要求点云是 world 系的已配准点云。
//       注意: AirSim 的 LidarData.pose 只是传感器相对机体的挂载外参
//       (LidarSimple.hpp: output.pose = params_.relative_pose), 不是世界位姿!
//       点云本体是传感器系 (UnrealLidarSensor.cpp: transformToBodyFrame)。
//       因此本节点在抓取每帧点云时同步读取真值里程计位姿, 按挂载外参 +
//       机体位姿手动把点云变换到 world 系后再发布。
// 发布: /cloud_registered  (PointCloud2: x,y,z,intensity; world 系 ENU;
//                           best_effort QoS, 与 SUPER 订阅端匹配)
//       /lidar_slam/odom   (Odometry: world 系 ENU, SUPER 唯一的位姿来源,
//                           100Hz, best_effort QoS)
// 过滤: 未命中点 (0,0,0) 与机身半径 body_filter_radius 内的点 (螺旋桨/机身
//       回波会污染占据地图), 半径内点直接丢弃。
// 连接: IP 读环境变量 PX4_SIM_HOST_ADDR, 未设置则默认 172.28.144.1。
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
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

class AirSimSensorSuperNode : public rclcpp::Node {
public:
    AirSimSensorSuperNode() : Node("airsim_sensor_super_node") {
        // 1. 参数 (与 configs/settings.json 中 Lidar1 配置一致)
        vehicle_name_ = this->declare_parameter<std::string>("vehicle_name", "PX4");
        lidar_name_ = this->declare_parameter<std::string>("lidar_sensor_name", "Lidar1");
        // UE 端传感器类型: true=GPU 雷达 (SensorType 8, getGPULidarData, 每点 5 float),
        // false=CPU 雷达 (SensorType 6, getLidarData, 每点 3 float)。点云均为传感器局部 NED 系
        use_gpu_lidar_ = this->declare_parameter<bool>("use_gpu_lidar", true);
        cloud_topic_ = this->declare_parameter<std::string>("cloud_topic", "/cloud_registered");
        odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/lidar_slam/odom");
        cloud_frame_ = this->declare_parameter<std::string>("cloud_frame_id", "world");
        // 机身安全半径: 半径内的点视为机身/螺旋桨回波, 直接丢弃 (米)
        body_filter_radius_ = this->declare_parameter<double>("body_filter_radius", 0.8);
        intensity_ = this->declare_parameter<double>("intensity", 100.0);
        // 挂载位置/姿态: 与 settings.json 中 LiDAR 的 X/Y/Z/Roll/Pitch/Yaw 一致 (NED 机体系, 米/度)
        mount_x_ = this->declare_parameter<double>("mount_x", 0.0);
        mount_y_ = this->declare_parameter<double>("mount_y", 0.0);
        mount_z_ = this->declare_parameter<double>("mount_z", 0.0);
        mount_roll_ = this->declare_parameter<double>("mount_roll_deg", 0.0);
        mount_pitch_ = this->declare_parameter<double>("mount_pitch_deg", 0.0);
        mount_yaw_ = this->declare_parameter<double>("mount_yaw_deg", 0.0);
        int lidar_ms = this->declare_parameter<int>("lidar_period_ms", 100);  // 10Hz, 与 UpdateFrequency 一致
        int odom_ms = this->declare_parameter<int>("odom_period_ms", 10);     // 100Hz, SUPER 断 odom 会停止规划

        // 2. 发布者 (SUPER 订阅端为 best_effort, 发布端用同款 QoS 保持匹配)
        auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
        pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, qos);
        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, qos);

        // 3. 连接 AirSim (LiDAR / 里程计 各一条独立 RPC 通道, 互不阻塞)
        const char* ip_env = std::getenv("PX4_SIM_HOST_ADDR");
        std::string airsim_ip = ip_env ? ip_env : "172.28.144.1";
        RCLCPP_INFO(this->get_logger(), "正在建立 SUPER 链路 LiDAR/odom 双通道直连 (IP: %s) ...", airsim_ip.c_str());
        try {
            client_lidar_ = std::make_unique<msr::airlib::MultirotorRpcLibClient>(airsim_ip);
            client_lidar_->confirmConnection();
            client_odom_ = std::make_unique<msr::airlib::MultirotorRpcLibClient>(airsim_ip);
            client_odom_->confirmConnection();
            RCLCPP_INFO(this->get_logger(), "2条独立 RPC 数据通道连接成功！");
        } catch (rpc::rpc_error& e) {
            RCLCPP_ERROR(this->get_logger(), "连接失败: %s", e.what());
            throw std::runtime_error("AirSim Connection Failed");
        }

        // 4. 多线程定时器 (LiDAR 10Hz + odom 100Hz 互不阻塞)
        cb_group_lidar_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_odom_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_lidar_ = this->create_wall_timer(std::chrono::milliseconds(lidar_ms),
            std::bind(&AirSimSensorSuperNode::grab_lidar, this), cb_group_lidar_);
        timer_odom_ = this->create_wall_timer(std::chrono::milliseconds(odom_ms),
            std::bind(&AirSimSensorSuperNode::grab_odom, this), cb_group_odom_);
    }

private:
    // ================= 线程 1: LiDAR -> world 系点云 =================
    void grab_lidar() {
        try {
            // 读取点云: GPU 雷达 (getGPULidarData, 每点 5 float: x,y,z,分割rgb,intensity)
            // 或 CPU 雷达 (getLidarData, 每点 3 float)。两者点云均为传感器局部 NED 系,
            // 未命中点同为 (0,0,0) 占位
            uint64_t sim_stamp = 0;
            std::vector<float> points;
            if (use_gpu_lidar_) {
                auto d = client_lidar_->getGPULidarData(lidar_name_, vehicle_name_);
                sim_stamp = d.time_stamp;
                points = std::move(d.point_cloud);
            } else {
                auto d = client_lidar_->getLidarData(lidar_name_, vehicle_name_);
                sim_stamp = d.time_stamp;
                points = std::move(d.point_cloud);
            }

            // 重复帧去重: RPC 轮询相位与传感器输出相位未锁定, 同一帧可能被重复发布,
            // 会导致占据地图重复更新与位姿-点云错位累积
            if (sim_stamp > 0 && sim_stamp == last_sim_stamp_) {
                return;
            }
            last_sim_stamp_ = sim_stamp;

            const auto stamp = this->now();
            const size_t point_stride = use_gpu_lidar_ ? 5 : 3;
            const size_t total = points.size() / point_stride;
            if (total == 0) {
                return;
            }

            // LidarData.pose 只是挂载外参 (不是世界位姿!), 点云在传感器系。
            // 抓取点云的同一时刻同步读取真值里程计, 得到机体在 world 系 (NED) 位姿:
            //   传感器系 --挂载外参--> 机体系 --机体位姿--> world NED
            const auto kinematics = client_lidar_->simGetGroundTruthKinematics(vehicle_name_);
            const auto& vp = kinematics.pose;

            // 机体位姿 (world NED): 旋转 R_v + 平移 t_v
            tf2::Quaternion q_v(vp.orientation.x(), vp.orientation.y(),
                                vp.orientation.z(), vp.orientation.w());
            q_v.normalize();
            const tf2::Matrix3x3 r_v(q_v);
            const tf2::Vector3 t_v(vp.position.x(), vp.position.y(), vp.position.z());

            // 挂载外参 (NED, 机身 -> 传感器): 与 settings.json 的 X/Y/Z/Roll/Pitch/Yaw 一致
            tf2::Quaternion q_mount;
            q_mount.setRPY(mount_roll_ * M_PI / 180.0,
                           mount_pitch_ * M_PI / 180.0,
                           mount_yaw_ * M_PI / 180.0);
            const tf2::Matrix3x3 r_mount(q_mount);
            const tf2::Vector3 t_mount(mount_x_, mount_y_, mount_z_);

            // XYZI 字段 (SUPER 按 pcl::PointXYZI 解析, intensity < intensity_thresh 的点被过滤)
            struct PointXYZI { float x, y, z, intensity; };
            std::vector<PointXYZI> pts;
            pts.reserve(total);

            size_t hits = 0;
            for (size_t i = 0; i < total; ++i) {
                // 雷达传感器系 NED
                const tf2::Vector3 p_sensor(points[i * point_stride],
                                            points[i * point_stride + 1],
                                            points[i * point_stride + 2]);
                // 未命中点为 (0,0,0)
                if (p_sensor.isZero()) {
                    continue;
                }
                // 机身半径过滤: 距雷达原点 (=机体原点) 过近的点视为机身/螺旋桨回波
                if (p_sensor.length() < body_filter_radius_) {
                    continue;
                }
                // 传感器系 -> world NED
                const tf2::Vector3 p_body = r_mount * p_sensor + t_mount;
                const tf2::Vector3 p_w_ned = r_v * p_body + t_v;
                PointXYZI p;
                // world NED -> ENU: (X,Y,Z)_enu = (Y,X,-Z)_ned
                p.x = static_cast<float>(p_w_ned.y());
                p.y = static_cast<float>(p_w_ned.x());
                p.z = static_cast<float>(-p_w_ned.z());
                p.intensity = static_cast<float>(intensity_);
                pts.push_back(p);
                ++hits;
            }
            if (pts.empty()) {
                return;
            }

            sensor_msgs::msg::PointCloud2 msg;
            msg.header.stamp = stamp;
            msg.header.frame_id = cloud_frame_;     // world (ENU 语义)
            msg.height = 1;
            msg.width = static_cast<uint32_t>(pts.size());
            msg.is_dense = true;
            msg.is_bigendian = false;
            msg.fields.resize(4);
            msg.fields[0] = make_field("x", 0, sensor_msgs::msg::PointField::FLOAT32);
            msg.fields[1] = make_field("y", 4, sensor_msgs::msg::PointField::FLOAT32);
            msg.fields[2] = make_field("z", 8, sensor_msgs::msg::PointField::FLOAT32);
            msg.fields[3] = make_field("intensity", 12, sensor_msgs::msg::PointField::FLOAT32);
            msg.point_step = sizeof(PointXYZI);     // 16 字节
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

    // ================= 线程 2: 真值里程计 (NED -> ENU) =================
    // 换算逻辑与 airsim_sensor_ego 保持一致
    void grab_odom() {
        try {
            auto odom_data = client_odom_->simGetGroundTruthKinematics(vehicle_name_);
            const auto current_time = this->now();

            nav_msgs::msg::Odometry msg;
            msg.header.stamp = current_time;
            msg.header.frame_id = "world";
            msg.child_frame_id = "base_link";
            // 位置: NED -> ENU: (X,Y,Z)_enu = (Y,X,-Z)_ned
            msg.pose.pose.position.x = odom_data.pose.position.y();
            msg.pose.pose.position.y = odom_data.pose.position.x();
            msg.pose.pose.position.z = -odom_data.pose.position.z();

            // 姿态: NED -> ENU (SUPER 只用 pose, 速度仅填充供 RViz 使用)
            tf2::Quaternion q_raw(odom_data.pose.orientation.x(), odom_data.pose.orientation.y(),
                                  odom_data.pose.orientation.z(), odom_data.pose.orientation.w());
            tf2::Quaternion q_n2e;
            q_n2e.setRPY(M_PI, 0, M_PI / 2.0);
            tf2::Quaternion q_f2f;
            q_f2f.setRPY(M_PI, 0, 0);
            tf2::Quaternion q_final = q_n2e * q_raw * q_f2f;
            msg.pose.pose.orientation.x = q_final.x();
            msg.pose.pose.orientation.y = q_final.y();
            msg.pose.pose.orientation.z = q_final.z();
            msg.pose.pose.orientation.w = q_final.w();

            // 线速度: NED -> ENU
            msg.twist.twist.linear.x = odom_data.twist.linear.y();
            msg.twist.twist.linear.y = odom_data.twist.linear.x();
            msg.twist.twist.linear.z = -odom_data.twist.linear.z();

            pub_odom_->publish(msg);
        }
        catch (rpc::rpc_error& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "抓取 odom 失败: RPC 通信异常! (%s)", e.what());
        }
    }

    std::unique_ptr<msr::airlib::MultirotorRpcLibClient> client_lidar_, client_odom_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::CallbackGroup::SharedPtr cb_group_lidar_, cb_group_odom_;
    rclcpp::TimerBase::SharedPtr timer_lidar_, timer_odom_;
    std::string vehicle_name_, lidar_name_, cloud_topic_, odom_topic_, cloud_frame_;
    bool use_gpu_lidar_ = true;
    uint64_t last_sim_stamp_ = 0;       // 上次发布的雷达仿真 time_stamp (重复帧去重)
    double body_filter_radius_, intensity_;
    double mount_x_, mount_y_, mount_z_, mount_roll_, mount_pitch_, mount_yaw_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AirSimSensorSuperNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
