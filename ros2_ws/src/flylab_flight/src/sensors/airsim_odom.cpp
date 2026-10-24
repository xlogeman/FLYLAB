// airsim_odom —— 通用真值里程计采集节点 (算法无关, 可插拔)
//
// 功能: 通过 RPC 直连 Cosys-AirSim 采集真值运动学 (simGetGroundTruthKinematics),
//       发布 NED->ENU 换算后的标准里程计, 并可选广播 world -> base_link TF。
// 发布: /airsim/odom  (nav_msgs/Odometry, world 系 ENU)
//       TF: world -> base_link (enable_tf, 时间戳用本机时钟, 供 RViz 使用)
// 时间戳: 关键设计 —— header.stamp 落在与 airsim_lidar/airsim_imu 相同的
//         映射时间轴上 (ros = offset + 仿真时间), bridge_super 据此对点云
//         帧做位姿插值 (点云是 UE 端缓存快照, 必须用生成时刻的位姿)。
//         仿真时间基准: getMultirotorState 返回的服务器时钟 (与
//         LidarData.time_stamp 同一时钟), 每 10 次采样同步一次; 各样本的
//         仿真时间戳按本地时钟流逝以估计速率外推, 不依赖 WSL/Windows 时钟对齐。
// 参数:
//   odom_period_ms  采集周期 (默认 10ms = 100Hz)
//   enable_tf       是否广播 world -> base_link TF (默认 true)
//   vehicle_name    默认 PX4
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include "airsim_common.hpp"

using namespace std::chrono_literals;

class AirsimOdomNode : public rclcpp::Node {
public:
    AirsimOdomNode() : Node("airsim_odom_node") {
        vehicle_name_ = this->declare_parameter<std::string>("vehicle_name", "PX4");
        const int odom_ms = this->declare_parameter<int>("odom_period_ms", 10);  // 100Hz
        enable_tf_ = this->declare_parameter<bool>("enable_tf", true);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/airsim/odom", qos);

        const std::string ip = flylab::airsim_host();
        RCLCPP_INFO(this->get_logger(), "真值里程计节点启动 (IP: %s, TF: %s)",
                    ip.c_str(), enable_tf_ ? "开" : "关");
        try {
            client_ = flylab::connect_airsim(ip);
        } catch (const rpc::rpc_error& e) {
            RCLCPP_ERROR(this->get_logger(), "连接失败: %s", e.what());
            throw std::runtime_error("AirSim Connection Failed");
        }

        if (enable_tf_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

        timer_ = this->create_wall_timer(std::chrono::milliseconds(odom_ms),
                                         std::bind(&AirsimOdomNode::grab_odom, this));
    }

private:
    void grab_odom() {
        try {
            auto odom_data = client_->simGetGroundTruthKinematics(vehicle_name_);

            // ---- 仿真时间戳: getMultirotorState 服务器时钟为基准, 本地外推 ----
            ++cnt_;
            if (cnt_ % 10 == 1) {
                const uint64_t t_ref = client_->getMultirotorState().timestamp;
                const uint64_t l_ref = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                // 估计仿真时钟相对本地时钟的速率: UE 卡顿时仿真时间流逝变慢,
                // 按实际速率外推可避免外推值超前于真实仿真时间
                if (sim_ref_sim_t_ > 0 && t_ref > sim_ref_sim_t_ && l_ref > sim_ref_local_ns_) {
                    const double rate = static_cast<double>(t_ref - sim_ref_sim_t_) /
                                        static_cast<double>(l_ref - sim_ref_local_ns_);
                    sim_rate_ = std::min(std::max(rate, 0.25), 4.0);
                }
                sim_ref_sim_t_ = t_ref;
                sim_ref_local_ns_ = l_ref;
            }
            const uint64_t local_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            const double sample_sim_s =
                static_cast<double>(sim_ref_sim_t_) * 1e-9 +
                (static_cast<double>(local_ns - sim_ref_local_ns_) * 1e-9) * sim_rate_;

            // 映射到与 LiDAR/IMU 相同的 ROS 时间轴
            const auto stamp = rclcpp::Time(static_cast<int64_t>(
                clock_sync_.to_ros(sample_sim_s, this->now().seconds()) * 1e9), RCL_ROS_TIME);

            // ---- 里程计消息 (NED -> ENU) ----
            nav_msgs::msg::Odometry msg;
            msg.header.stamp = stamp;
            msg.header.frame_id = "world";
            msg.child_frame_id = "base_link";
            // 位置: (X,Y,Z)_enu = (Y,X,-Z)_ned
            msg.pose.pose.position.x = odom_data.pose.position.y();
            msg.pose.pose.position.y = odom_data.pose.position.x();
            msg.pose.pose.position.z = -odom_data.pose.position.z();
            // 姿态: q_enu_flu = q_ned_to_enu * q_raw * q_frd_to_flu
            const tf2::Quaternion q_raw(odom_data.pose.orientation.x(),
                                        odom_data.pose.orientation.y(),
                                        odom_data.pose.orientation.z(),
                                        odom_data.pose.orientation.w());
            tf2::Quaternion q_final = flylab::quat_nedfrd_to_enuflu(q_raw);
            q_final.normalize();
            msg.pose.pose.orientation.x = q_final.x();
            msg.pose.pose.orientation.y = q_final.y();
            msg.pose.pose.orientation.z = q_final.z();
            msg.pose.pose.orientation.w = q_final.w();
            // 线速度: NED -> ENU (供 RViz / 下游参考)
            msg.twist.twist.linear.x = odom_data.twist.linear.y();
            msg.twist.twist.linear.y = odom_data.twist.linear.x();
            msg.twist.twist.linear.z = -odom_data.twist.linear.z();
            pub_odom_->publish(msg);

            // ---- TF: world -> base_link (本机时钟戳, 供 RViz) ----
            if (tf_broadcaster_) {
                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = this->now();
                t.header.frame_id = "world";
                t.child_frame_id = "base_link";
                t.transform.translation.x = msg.pose.pose.position.x;
                t.transform.translation.y = msg.pose.pose.position.y;
                t.transform.translation.z = msg.pose.pose.position.z;
                t.transform.rotation.x = msg.pose.pose.orientation.x;
                t.transform.rotation.y = msg.pose.pose.orientation.y;
                t.transform.rotation.z = msg.pose.pose.orientation.z;
                t.transform.rotation.w = msg.pose.pose.orientation.w;
                tf_broadcaster_->sendTransform(t);
            }
        } catch (const rpc::rpc_error& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "抓取 odom 失败: RPC 通信异常! (%s)", e.what());
        }
    }

    std::unique_ptr<flylab::AirSimClient> client_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string vehicle_name_;
    bool enable_tf_ = true;
    flylab::SimClockSync clock_sync_;

    // 仿真时钟基准 (仅 odom 线程访问): getMultirotorState 的服务器时间戳
    // 及其对应的本地 steady_clock 时刻, 用于按速率外推各样本的仿真时间戳
    uint64_t cnt_ = 0;
    uint64_t sim_ref_sim_t_ = 0;
    uint64_t sim_ref_local_ns_ = 0;
    double sim_rate_ = 1.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AirsimOdomNode>());
    rclcpp::shutdown();
    return 0;
}
