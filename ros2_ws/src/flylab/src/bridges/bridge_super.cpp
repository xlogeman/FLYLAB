// bridge_super —— SUPER 规划器输入适配桥接节点 (纯 ROS, 不依赖 AirSim)
//
// 功能: 把 airsim_lidar 的原始传感器系点云 (NED, 含逐点捕获时间) 变换到
//       world 系 ENU 已配准点云, 供 SUPER (hku-mars, rog_map) 直接使用;
//       并把 airsim_odom 的真值里程计转发为 SUPER 订阅的话题。
// 订阅: /airsim/lidar/points_raw  (PointXYZIRT: 传感器局部 NED,
//                                  header=帧首, time=逐点捕获时刻-帧首)
//       /airsim/odom              (world 系 ENU, 时间戳与点云同一时间轴)
// 发布: /cloud_registered  (PointXYZI: world 系 ENU, best_effort, 与 SUPER 订阅端匹配)
//       /lidar_slam/odom   (Odometry: 原样转发, SUPER 唯一的位姿来源, 100Hz)
// 位姿对齐 (核心): 点云帧是 UE 端传感器更新 tick 的缓存快照 (age 可达 1 个
//       更新周期 + RPC 延迟), 不能用最新位姿去变换, 否则旋转/平移时点云
//       整体错位 (拖影), 幽灵障碍被写进 ROG-Map 导致规划失败/目标被挪。
//       本节点用 odom 环形缓冲按 ROS 时间戳插值 (两路数据源在
//       airsim_lidar/airsim_odom 内已统一映射到同一时间轴):
//         - 帧位姿 = 帧结束时刻 (header + max(time)) 的插值位姿;
//         - 去畸变: GPU 雷达一帧是旋转周期内多次渲染累积, 各点捕获时刻
//           不同, 按每点 time 字段插值该时刻的位姿逐点放置 (deskew_en 控制);
//         - 点云时刻早于缓冲范围 (过旧, UE 卡顿) 时丢弃该帧。
// 过滤: 未命中点 (0,0,0) 已在采集端过滤; 本节点再过滤机身半径
//       body_filter_radius 内的点 (螺旋桨/机身回波会污染占据地图)。
// 参数:
//   deskew_en, body_filter_radius, intensity,
//   mount_x/y/z, mount_roll_deg/pitch_deg/yaw_deg (挂载外参, NED 机体系)
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include <cmath>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

class BridgeSuperNode : public rclcpp::Node {
public:
    BridgeSuperNode() : Node("bridge_super_node") {
        // 1. 参数
        deskew_en_ = this->declare_parameter<bool>("deskew_en", true);
        body_filter_radius_ = this->declare_parameter<double>("body_filter_radius", 0.8);
        intensity_ = this->declare_parameter<double>("intensity", 100.0);
        mount_x_ = this->declare_parameter<double>("mount_x", 0.0);
        mount_y_ = this->declare_parameter<double>("mount_y", 0.0);
        mount_z_ = this->declare_parameter<double>("mount_z", 0.0);
        mount_roll_ = this->declare_parameter<double>("mount_roll_deg", 0.0);
        mount_pitch_ = this->declare_parameter<double>("mount_pitch_deg", 0.0);
        mount_yaw_ = this->declare_parameter<double>("mount_yaw_deg", 0.0);

        // 挂载外参旋转 (NED 机体系 机身->传感器)
        q_mount_.setRPY(mount_roll_ * M_PI / 180.0,
                        mount_pitch_ * M_PI / 180.0,
                        mount_yaw_ * M_PI / 180.0);

        // 2. 发布/订阅 (SUPER 订阅端为 best_effort, 发布端用同款 QoS 匹配;
        //    内部订阅传感器流用 reliable, 不丢帧)
        auto qos_in = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
        auto qos_out = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
        pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", qos_out);
        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/lidar_slam/odom", qos_out);
        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/airsim/lidar/points_raw", qos_in,
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { process_cloud(msg); });
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/airsim/odom", qos_in,
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                on_odom(msg);
                pub_odom_->publish(*msg);  // 原样转发给 SUPER
            });

        RCLCPP_INFO(this->get_logger(), "SUPER 桥接节点启动 (去畸变: %s, 机身过滤半径: %.2f m)",
                    deskew_en_ ? "开" : "关", body_filter_radius_);
    }
private:
    // ---- 位姿环形缓冲: odom 订阅写入, 点云按时刻插值读取 ----
    struct PoseSample {
        rclcpp::Time t;   // 映射时间轴上的时间戳 (与点云 header 同轴)
        tf2::Vector3 p;   // world ENU 位置
        tf2::Quaternion q;  // world ENU 姿态
    };
    static constexpr double kPoseHistSec = 2.0;  // 缓冲保留 2s

    void on_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const rclcpp::Time stamp(msg->header.stamp);
        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                          msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        q.normalize();
        tf2::Vector3 p(msg->pose.pose.position.x, msg->pose.pose.position.y,
                       msg->pose.pose.position.z);
        std::lock_guard<std::mutex> lk(hist_mutex_);
        // 时间轴必须单调: 小回跳 (时钟估计抖动) 钳位处理不清缓冲,
        // 大回跳 (>1s, 仿真重置) 才清空重建
        if (!hist_.empty() && stamp < hist_.back().t) {
            if ((hist_.back().t - stamp).seconds() > 1.0) {
                hist_.clear();
            }
            return;
        }
        hist_.push_back(PoseSample{stamp, p, q});
        while (hist_.size() > 2 &&
               (stamp - hist_.front().t).seconds() > kPoseHistSec) {
            hist_.pop_front();
        }
    }

    // 按时间戳插值位姿; 返回 false 表示超出缓冲范围 (过旧/过新且无法覆盖)
    bool get_pose_at(const rclcpp::Time& t, tf2::Quaternion& q_out, tf2::Vector3& p_out) const {
        if (hist_.empty()) return false;
        if (t <= hist_.front().t) return false;         // 过旧, 调用方决定丢弃
        if (t >= hist_.back().t) {                       // 略超前: 用最新位姿
            q_out = hist_.back().q;
            p_out = hist_.back().p;
            return true;
        }
        for (size_t i = hist_.size() - 1; i > 0; --i) {
            const auto& s0 = hist_[i - 1];
            const auto& s1 = hist_[i];
            if (s0.t <= t && t <= s1.t) {
                const double alpha = (s1.t == s0.t)
                    ? 0.0
                    : (t - s0.t).seconds() / (s1.t - s0.t).seconds();
                p_out = s0.p.lerp(s1.p, alpha);
                q_out = s0.q.slerp(s1.q, alpha);
                return true;
            }
        }
        q_out = hist_.back().q;
        p_out = hist_.back().p;
        return true;
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
    // ---- 点云处理: 传感器系 NED -> world 系 ENU ----
    // 输入: x,y,z (传感器局部 NED), intensity, ring, time (捕获时刻-帧首, 秒)
    void process_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        if (msg->point_step < 24 || msg->width == 0) {
            return;
        }
        const auto* data = msg->data.data();

        // 帧结束时刻 = 帧首 header + max(time) (点云生成时刻 ≈ 扫描结束)
        float max_time = 0.0f;
        for (uint32_t i = 0; i < msg->width; ++i) {
            float t;
            std::memcpy(&t, data + i * msg->point_step + 20, sizeof(float));
            if (t > max_time) max_time = t;
        }
        const rclcpp::Time end_stamp = rclcpp::Time(msg->header.stamp) +
            rclcpp::Duration::from_seconds(max_time);

        // 帧位姿: 取点云生成时刻的插值位姿 (时间对齐, 见文件头注释)
        tf2::Quaternion q_end;
        tf2::Vector3 t_end;
        {
            std::lock_guard<std::mutex> lk(hist_mutex_);
            if (hist_.empty()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "位姿缓冲为空 (启动初期), 丢弃该帧点云");
                return;
            }
            if (end_stamp < hist_.front().t) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "点云落后位姿缓冲 %.0f ms, 已过旧, 丢弃该帧 (检查 UE 帧率/Lidar UpdateFrequency)",
                                     (hist_.front().t - end_stamp).seconds() * 1e3);
                return;
            }
            get_pose_at(end_stamp, q_end, t_end);
        }
        const tf2::Matrix3x3 r_end(q_end);

        // 位姿快照拷贝一次, 循环内免锁插值
        std::vector<PoseSample> snap;
        if (deskew_en_) {
            std::lock_guard<std::mutex> lk(hist_mutex_);
            snap.assign(hist_.begin(), hist_.end());
        }
        std::deque<PoseSample> hist_snap(snap.begin(), snap.end());
        auto interp_pose = [&](const rclcpp::Time& t,
                               tf2::Quaternion& q_o, tf2::Vector3& p_o) -> bool {
            if (hist_snap.size() < 2 || t <= hist_snap.front().t || t >= hist_snap.back().t) {
                return false;
            }
            for (size_t i = hist_snap.size() - 1; i > 0; --i) {
                const auto& s0 = hist_snap[i - 1];
                const auto& s1 = hist_snap[i];
                if (s0.t <= t && t <= s1.t) {
                    const double alpha = (s1.t == s0.t)
                        ? 0.0
                        : (t - s0.t).seconds() / (s1.t - s0.t).seconds();
                    p_o = s0.p.lerp(s1.p, alpha);
                    q_o = s0.q.slerp(s1.q, alpha);
                    return true;
                }
            }
            return false;
        };

        // XYZI 字段 (SUPER 按 pcl::PointXYZI 解析, intensity < intensity_thresh 的点被过滤)
        struct PointXYZI { float x, y, z, intensity; };
        std::vector<PointXYZI> pts;
        pts.reserve(msg->width);

        for (uint32_t i = 0; i < msg->width; ++i) {
            const auto* base = data + i * msg->point_step;
            float px, py, pz, pt;
            std::memcpy(&px, base + 0, sizeof(float));
            std::memcpy(&py, base + 4, sizeof(float));
            std::memcpy(&pz, base + 8, sizeof(float));
            std::memcpy(&pt, base + 20, sizeof(float));

            // 机身半径过滤: 距雷达原点过近的点视为机身/螺旋桨回波
            const tf2::Vector3 p_sensor(px, py, pz);
            if (p_sensor.isZero() || p_sensor.length() < body_filter_radius_) {
                continue;
            }

            // 传感器系 (NED) -> 机体系 (NED): 挂载外参
            const tf2::Vector3 p_body_ned = tf2::Matrix3x3(q_mount_) * p_sensor +
                tf2::Vector3(mount_x_, mount_y_, mount_z_);
            // 机体系 NED -> FLU: (X,Y,Z)_flu = (X_ned, -Y_ned, -Z_ned)
            const tf2::Vector3 p_body_flu(p_body_ned.x(), -p_body_ned.y(), -p_body_ned.z());

            // 机体 -> world: 默认帧位姿; 开启去畸变时按每点捕获时刻插值位姿
            tf2::Vector3 p_w;
            bool placed = false;
            if (deskew_en_ && pt > 0.0f) {
                const rclcpp::Time t_cap = rclcpp::Time(msg->header.stamp) +
                    rclcpp::Duration::from_seconds(pt);
                tf2::Quaternion q_i;
                tf2::Vector3 t_i;
                if (interp_pose(t_cap, q_i, t_i)) {
                    p_w = tf2::Matrix3x3(q_i) * p_body_flu + t_i;
                    placed = true;
                }
            }
            if (!placed) {
                p_w = r_end * p_body_flu + t_end;
            }

            pts.push_back({static_cast<float>(p_w.x()), static_cast<float>(p_w.y()),
                           static_cast<float>(p_w.z()), static_cast<float>(intensity_)});
        }
        if (pts.empty()) {
            return;
        }

        sensor_msgs::msg::PointCloud2 out;
        out.header.stamp = end_stamp;
        out.header.frame_id = "world";  // ENU 语义
        out.height = 1;
        out.width = static_cast<uint32_t>(pts.size());
        out.is_dense = true;
        out.is_bigendian = false;
        out.fields.resize(4);
        out.fields[0] = make_field("x", 0, sensor_msgs::msg::PointField::FLOAT32);
        out.fields[1] = make_field("y", 4, sensor_msgs::msg::PointField::FLOAT32);
        out.fields[2] = make_field("z", 8, sensor_msgs::msg::PointField::FLOAT32);
        out.fields[3] = make_field("intensity", 12, sensor_msgs::msg::PointField::FLOAT32);
        out.point_step = sizeof(PointXYZI);  // 16 字节
        out.row_step = out.point_step * out.width;
        out.data.resize(out.row_step);
        std::memcpy(out.data.data(), pts.data(), out.row_step);

        pub_cloud_->publish(out);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    bool deskew_en_ = true;
    double body_filter_radius_ = 0.8, intensity_ = 100.0;
    double mount_x_, mount_y_, mount_z_, mount_roll_, mount_pitch_, mount_yaw_;
    tf2::Quaternion q_mount_;
    std::deque<PoseSample> hist_;
    mutable std::mutex hist_mutex_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BridgeSuperNode>());
    rclcpp::shutdown();
    return 0;
}
