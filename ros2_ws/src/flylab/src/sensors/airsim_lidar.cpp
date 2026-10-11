// airsim_lidar —— 通用 3D LiDAR 采集节点 (算法无关, 可插拔)
//
// 功能: 通过 RPC 直连 Cosys-AirSim 采集原始点云 (传感器局部 NED 系), 填充
//       时间模型后发布。坐标变换 (FLU/world 系) 与外参属于算法适配,
//       由 bridge_fastlio / bridge_super 各自完成。
// 发布: /airsim/lidar/points_raw
//       (PointCloud2: x,y,z 传感器局部 NED; intensity,ring,time;
//        frame_id=lidar_ned, reliable QoS)
// 时间模型 (Cosys-AirSim LiDAR 无逐点时间戳, 整帧只有一个 time_stamp):
//   - CPU 雷达 (SensorType 6): 单 tick 瞬时快照, 整帧同一位姿, 统一填
//     sweep_dur, header = 快照时刻 - sweep_dur (窗口末 = 真实成像时刻);
//   - GPU 雷达 (SensorType 8): 旋转扫描式, 一帧 = 一个旋转周期内多段渲染
//     累积, 各点真实捕获时刻不同。使用插件逐段上报的真实渲染时刻
//     (GPULidarData.segment_stamps) 生成真实逐点时间, 下游据此做逐点
//     运动畸变校正; 无段数据 (旧插件) 回退 CPU 雷达同款快照模型。
//   未命中的 (0,0,0) 占位点被过滤。
// 时间戳: header = 帧首 (仿真 time_stamp 经 SimClockSync 统一映射), 与
//         airsim_imu/airsim_odom 落在同一条时间轴 (仿真慢于实时时不会
//         系统性过积分, 详见 airsim_common.hpp)。
// 参数 (必须与 UE 端 settings.json 中 Lidar1 配置一致):
//   lidar_name, use_gpu_lidar, num_channels, rotation_frequency,
//   intensity, lidar_period_ms, vehicle_name
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "airsim_common.hpp"

class AirsimLidarNode : public rclcpp::Node {
public:
    AirsimLidarNode() : Node("airsim_lidar_node") {
        vehicle_name_ = this->declare_parameter<std::string>("vehicle_name", "PX4");
        lidar_name_ = this->declare_parameter<std::string>("lidar_sensor_name", "Lidar1");
        use_gpu_lidar_ = this->declare_parameter<bool>("use_gpu_lidar", true);
        num_channels_ = static_cast<uint16_t>(this->declare_parameter<int>("num_channels", 16));
        rotation_hz_ = this->declare_parameter<double>("rotation_frequency", 10.0);
        intensity_ = this->declare_parameter<double>("intensity", 100.0);
        cloud_frame_ = this->declare_parameter<std::string>("cloud_frame_id", "lidar_ned");
        const int lidar_ms = this->declare_parameter<int>("lidar_period_ms", 100);  // 10Hz

        auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
        pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/airsim/lidar/points_raw", qos);

        const std::string ip = flylab::airsim_host();
        RCLCPP_INFO(this->get_logger(), "LiDAR 采集节点启动 (IP: %s, 传感器: %s, %s)",
                    ip.c_str(), lidar_name_.c_str(), use_gpu_lidar_ ? "GPU 雷达" : "CPU 雷达");
        try {
            client_ = flylab::connect_airsim(ip);
        } catch (const rpc::rpc_error& e) {
            RCLCPP_ERROR(this->get_logger(), "连接失败: %s", e.what());
            throw std::runtime_error("AirSim Connection Failed");
        }

        timer_ = this->create_wall_timer(std::chrono::milliseconds(lidar_ms),
                                         std::bind(&AirsimLidarNode::grab_lidar, this));
    }

private:
    struct SegStamp { double start_deg, sweep_deg; uint64_t t_ns; };

    static sensor_msgs::msg::PointField make_field(
        const std::string& name, uint32_t offset, uint8_t datatype) {
        sensor_msgs::msg::PointField f;
        f.name = name;
        f.offset = offset;
        f.datatype = datatype;
        f.count = 1;
        return f;
    }

    // 按点方位角 (度, [0,360)) 查所属捕获段的真实渲染时刻; 找不到
    // (方位角与段边界舍入误差) 回退到扫角中心最近的段
    static uint64_t seg_time_for_azimuth(const std::vector<SegStamp>& segs, double theta_deg) {
        uint64_t t_best = 0;
        double dist_best = 1e9;
        for (const auto& s : segs) {
            const double sweep = std::max(s.sweep_deg, 1e-3);
            const double fwd = std::fmod(theta_deg - s.start_deg + 720.0, 360.0);
            if (fwd < sweep) {
                return s.t_ns;
            }
            const double center = std::fmod(s.start_deg + sweep / 2.0, 360.0);
            const double dist = std::fabs(std::fmod(theta_deg - center + 540.0, 360.0) - 180.0);
            if (dist < dist_best) {
                dist_best = dist;
                t_best = s.t_ns;
            }
        }
        return t_best;
    }

    void grab_lidar() {
        try {
            // 读取点云: GPU 雷达 (SensorType 8, getGPULidarData) 或 CPU 雷达
            // (SensorType 6), 点云均为传感器局部 NED 系; GPU 雷达附带段级
            // 真实渲染时刻列表
            uint64_t sim_stamp = 0;
            std::vector<float> points;
            std::vector<SegStamp> seg_stamps;
            if (use_gpu_lidar_) {
                auto d = client_->getGPULidarData(lidar_name_, vehicle_name_);
                sim_stamp = d.time_stamp;
                points = std::move(d.point_cloud);
                seg_stamps.reserve(d.segment_stamps.size());
                for (const auto& s : d.segment_stamps) {
                    seg_stamps.push_back({s.start_angle, s.sweep_angle, s.time_stamp});
                }
            } else {
                auto d = client_->getLidarData(lidar_name_, vehicle_name_);
                sim_stamp = d.time_stamp;
                points = std::move(d.point_cloud);
            }
            const double wall_s = this->now().seconds();
            const double sweep_dur = (rotation_hz_ > 0.0) ? 1.0 / rotation_hz_ : 0.1;

            // 重复帧去重: RPC 轮询相位与雷达输出相位未锁定, 同一帧可能被连续
            // 抓取两次; 下游 (FAST-LIO) 对相同时间戳的帧不去重, 会导致空 IMU
            // 窗口的退化配准
            if (sim_stamp > 0 && sim_stamp == last_sim_stamp_) {
                return;
            }
            last_sim_stamp_ = sim_stamp;

            // FAST-LIO 约定 header.stamp = 扫描"起始"时刻 (lidar_end = beg + max(time)).
            // AirSim time_stamp (纳秒, 仿真时钟) = 数据生成时刻 ≈ 扫描"结束"时刻。
            // GPU 雷达有段级时刻时: header = 最早段渲染时刻 (真实帧首), 各点 time =
            // 所属段渲染时刻 - header (纯仿真轴差值, 经时间轴线性映射不变)。
            bool use_seg_times = !seg_stamps.empty() && sim_stamp > 0;
            uint64_t t_header_ns = 0;
            double header_s;
            if (use_seg_times) {
                t_header_ns = UINT64_MAX;
                for (const auto& s : seg_stamps) {
                    t_header_ns = std::min(t_header_ns, s.t_ns);
                }
                header_s = clock_sync_.to_ros(t_header_ns * 1e-9, wall_s);
            } else if (sim_stamp > 0) {
                const double sim_s = static_cast<double>(sim_stamp) * 1e-9;
                header_s = clock_sync_.to_ros(sim_s, wall_s) - sweep_dur;
            } else {
                // 兜底: 仿真端未填 time_stamp, 退化为按"抓取 ≈ 扫描结束"回推
                header_s = wall_s - sweep_dur;
            }
            const auto stamp = rclcpp::Time(static_cast<int64_t>(header_s * 1e9), RCL_ROS_TIME);
            // 每点 float 数: CPU 雷达 3 (x,y,z); GPU 雷达 5 (x,y,z,分割rgb,intensity)
            const size_t point_stride = use_gpu_lidar_ ? 5 : 3;
            const size_t total = points.size() / point_stride;
            if (total == 0) {
                return;
            }

            // per-point time 回退语义 (无段级时刻时): AirSim 整帧点云按单时刻
            // 快照处理, 统一填 sweep_dur (而非 0~sweep_dur 递增):
            //   - header.stamp = 成像时刻 - sweep_dur (扫描"起始"), FAST-LIO 窗口
            //     末点 = header + max(time) = 成像时刻, 恰为快照真实位姿 -> 零剪切;
            //   - 不能填 0: 会触发 FAST-LIO "按方位角反推时间"兜底, 重新注入伪畸变;
            //   - 不能填递增值: 伪逐点时间会反向注入最大 sweep_dur*速度 的剪切。
            // 有段级时刻时: time = 真实捕获时刻 - header, FAST-LIO 按真实时刻做
            // 逐点畸变校正 (GPU 帧本就是一个旋转周期的多次渲染累积, 非快照)。
            const float kPointTime =
                static_cast<float>((rotation_hz_ > 0.0) ? 1.0 / rotation_hz_ : 0.1);

            // velodyne 风格字段: x,y,z,intensity (float32), ring (uint16), time (float32)
            // 点坐标保持 AirSim 原生传感器局部 NED 系, 坐标变换由桥接节点完成
            struct PointXYZIRT { float x, y, z, intensity; uint16_t ring; float time; };
            std::vector<PointXYZIRT> pts;
            pts.reserve(total);

            for (size_t i = 0; i < total; ++i) {
                const float px = points[i * point_stride];
                const float py = points[i * point_stride + 1];
                const float pz = points[i * point_stride + 2];
                // 未命中点为 (0,0,0), 过滤
                if (px == 0.0f && py == 0.0f && pz == 0.0f) {
                    continue;
                }
                const uint16_t ring = static_cast<uint16_t>(i % num_channels_);
                float time_s = kPointTime;
                if (use_seg_times) {
                    double theta = std::atan2(py, px);  // [-pi, pi], 与激光角同坐标系
                    if (theta < 0.0) theta += 2.0 * M_PI;
                    const uint64_t t_seg =
                        seg_time_for_azimuth(seg_stamps, theta * 180.0 / M_PI);
                    if (t_seg > 0) {
                        time_s = static_cast<float>((t_seg - t_header_ns) * 1e-9);
                        // 避免恰为 0 触发 FAST-LIO 的方位角反推时间特判
                        if (time_s < 1e-3f) time_s = 1e-3f;
                    }
                }
                pts.push_back({px, py, pz, static_cast<float>(intensity_), ring, time_s});
            }
            if (pts.empty()) {
                return;
            }

            sensor_msgs::msg::PointCloud2 msg;
            msg.header.stamp = stamp;             // 帧首段渲染时刻 (快照模型: 成像-周期)
            msg.header.frame_id = cloud_frame_;   // 传感器局部 NED 语义
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
            msg.point_step = sizeof(PointXYZIRT);  // 24 字节 (ring 后 2 字节对齐填充)
            msg.row_step = msg.point_step * msg.width;
            msg.data.resize(msg.row_step);
            std::memcpy(msg.data.data(), pts.data(), msg.row_step);

            pub_cloud_->publish(msg);
        } catch (const rpc::rpc_error& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "抓取 LiDAR 失败: RPC 通信异常! (%s)", e.what());
        }
    }

    std::unique_ptr<flylab::AirSimClient> client_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string vehicle_name_, lidar_name_, cloud_frame_;
    bool use_gpu_lidar_ = false;
    uint16_t num_channels_ = 16;
    double rotation_hz_ = 10.0, intensity_ = 100.0;
    flylab::SimClockSync clock_sync_;
    uint64_t last_sim_stamp_ = 0;  // 上次发布的雷达仿真 time_stamp (重复帧去重)
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AirsimLidarNode>());
    rclcpp::shutdown();
    return 0;
}
