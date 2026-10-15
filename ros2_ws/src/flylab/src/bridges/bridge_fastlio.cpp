// bridge_fastlio —— FAST-LIO 输入适配桥接节点 (纯 ROS, 不依赖 AirSim)
//
// 功能: 把 airsim_lidar 的原始传感器系点云 (NED) 转成 FAST-LIO 期望的
//       ROS 标准机体系 (FLU), 并发布雷达挂载外参静态 TF。
// 订阅: /airsim/lidar/points_raw  (PointXYZIRT, 传感器局部 NED, header=帧首)
// 发布: /airsim/lidar/points      (同字段, y/z 取反转 FLU, frame=lidar_link)
//       静态 TF: body -> base_link (恒等, fast_lio.yaml 零位外参 => body 即机体系,
//                                  把 FAST-LIO 的 camera_init->body 与挂载链接通,
//                                  消除 RViz "No transform from base_link to camera_init")
//       静态 TF: base_link -> lidar_link (挂载参数 NED -> FLU 换算)
// 说明: IMU 由 airsim_imu 直接以 /airsim/imu/data (FLU) 供 fast_lio.yaml 使用,
//       本节点不处理 IMU; per-point time 字段在 NED->FLU 旋转下不变, 原样透传。
// 参数 (挂载位置/姿态, 与 settings.json 中 LiDAR 的 X/Y/Z/Roll/Pitch/Yaw 一致,
//       NED 机体系, 米/度):
//   mount_x, mount_y, mount_z, mount_roll_deg, mount_pitch_deg, mount_yaw_deg
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cstring>
#include <memory>

class BridgeFastlioNode : public rclcpp::Node {
public:
    BridgeFastlioNode() : Node("bridge_fastlio_node") {
        // 挂载外参 (NED 机体系, 米/度)
        mount_x_ = this->declare_parameter<double>("mount_x", 0.0);
        mount_y_ = this->declare_parameter<double>("mount_y", 0.0);
        mount_z_ = this->declare_parameter<double>("mount_z", 0.0);
        mount_roll_ = this->declare_parameter<double>("mount_roll_deg", 0.0);
        mount_pitch_ = this->declare_parameter<double>("mount_pitch_deg", 0.0);
        mount_yaw_ = this->declare_parameter<double>("mount_yaw_deg", 0.0);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
        pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/airsim/lidar/points", qos);
        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/airsim/lidar/points_raw", qos,
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { convert(msg); });

        publish_mount_tf();
        RCLCPP_INFO(this->get_logger(), "FAST-LIO 桥接节点启动 (NED->FLU + 挂载外参 TF)");
    }

private:
    // 静态 TF: body -> base_link (恒等, 接通 FAST-LIO 的 body 子树) + 挂载外参 NED -> FLU
    void publish_mount_tf() {
        std::vector<geometry_msgs::msg::TransformStamped> transforms;

        geometry_msgs::msg::TransformStamped body_to_base;
        body_to_base.header.frame_id = "body";
        body_to_base.child_frame_id = "base_link";
        body_to_base.transform.rotation.w = 1.0;
        transforms.push_back(body_to_base);

        // 平移: NED 机体系 (x前,y右,z下) -> FLU (x前,y左,z上)
        geometry_msgs::msg::TransformStamped t;
        t.header.frame_id = "base_link";
        t.child_frame_id = "lidar_link";
        t.transform.translation.x = mount_x_;
        t.transform.translation.y = -mount_y_;
        t.transform.translation.z = -mount_z_;

        // 旋转: R_flu = M * R_ned * M^T, M = diag(1,-1,-1)
        tf2::Quaternion q_ned;
        q_ned.setRPY(mount_roll_ * M_PI / 180.0,
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
        static_tf_->sendTransform(transforms);
    }

    // NED -> FLU: 只需 y/z 取反 (逐点 3 个 float, 直接改写缓冲, 零拷贝结构)
    void convert(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        sensor_msgs::msg::PointCloud2 out = *msg;
        if (out.point_step >= 12) {
            for (uint32_t i = 0; i < out.width * out.height; ++i) {
                float* p = reinterpret_cast<float*>(out.data.data() + i * out.point_step);
                p[1] = -p[1];  // y: 右 -> 左
                p[2] = -p[2];  // z: 下 -> 上
            }
        }
        out.header.frame_id = "lidar_link";
        pub_cloud_->publish(out);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
    double mount_x_, mount_y_, mount_z_, mount_roll_, mount_pitch_, mount_yaw_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BridgeFastlioNode>());
    rclcpp::shutdown();
    return 0;
}
