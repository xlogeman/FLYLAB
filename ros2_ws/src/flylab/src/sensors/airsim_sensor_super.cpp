// airsim_sensor_super —— SUPER 链路 AirSim 直连传感器节点
//
// 功能: 通过 RPC 直连 Cosys-AirSim 采集 3D LiDAR + 真值里程计, 转换成
//       SUPER (hku-mars, rog_map) 需要的输入格式, 不依赖 airsim_ros_pkgs。
//       与 airsim_sensor_lidar (FAST-LIO 链路) 的区别: SUPER 不做里程计融合,
//       要求点云是 world 系的已配准点云。
//       注意: AirSim 的 LidarData.pose 只是传感器相对机体的挂载外参
//       (LidarSimple.hpp: output.pose = params_.relative_pose), 不是世界位姿!
//       点云本体是传感器系 (UnrealLidarSensor.cpp: transformToBodyFrame)。
//       因此本节点按挂载外参 + 机体位姿手动把点云变换到 world 系后再发布。
//       位姿对齐: 点云帧是 UE 端传感器更新 tick 的缓存快照 (age 可达 1 个更新
//       周期 + RPC 延迟), 不能用 RPC 当下的位姿去变换, 否则旋转/平移时点云
//       整体错位 (拖影), 幽灵障碍被写进 ROG-Map 导致规划失败/目标被挪。
//       本节点 100Hz odom 线程把 (仿真时间戳, 位姿) 存入环形缓冲, 点云用
//       LidarData.time_stamp 从缓冲插值出该时刻的位姿与 ROS 时间戳, 实现
//       点云-位姿时间对齐; 点云时间戳早于缓冲范围 (过旧) 时丢弃该帧。
//       仿真时间戳以 getMultirotorState 的服务器时钟 (每 10 次采样同步一次)
//       为基准, 按本地时钟流逝外推, 不依赖 WSL/Windows 时钟对齐。
//       去畸变: GPU 雷达 (SensorType 8) 是旋转扫描式, 一帧 = 一个旋转周期内
//       多次渲染累积 (LidarCamera.cpp 转满一圈才发布), 非单时刻快照, 运动
//       时帧内存在剪切。点由激光角 LUT 重建, 每个点的传感器系方位角 = 捕获
//       时的扫描角度。捕获时刻优先取插件逐段上报的真实渲染时刻
//       (GPULidarData.segment_stamps: 段起始扫角/跨度 + 渲染瞬间, 修复旋转
//       时点云整体随转偏移的问题 —— 根因是异步渲染管线里深度图比扫描角记账
//       晚 1~2 个 UE 帧); 无段数据 (旧插件/CPU 雷达) 回退按方位角均匀旋转
//       反推。逐点插值位姿 (deskew_en 控制)。
//       CPU 雷达 (SensorType 6) 单 tick 一次出全帧, 无此问题。
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
#include <deque>
#include <mutex>
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
        // 去畸变 (GPU 雷达专用): 一帧点云是 UE 端一个旋转周期内多次渲染累积的,
        // 帧内各点捕获时刻不同。按每个点的激光方位角反推捕获时刻并插值位姿,
        // 消除旋转/平移时的帧内剪切。三个参数必须与 UE 端 settings.json 的
        // Lidar1 配置一致: 旋转周期 = 1/RotationsPerSecond, 起始角 = HorizontalFOVStart
        deskew_en_ = this->declare_parameter<bool>("deskew_en", true);
        rot_period_s_ = this->declare_parameter<double>("rot_period_s", 0.1);
        rot_start_deg_ = this->declare_parameter<double>("rot_start_deg", 0.0);
        int lidar_ms = this->declare_parameter<int>("lidar_period_ms", 100);  // 10Hz, 与 UpdateFrequency 一致
        int odom_ms = this->declare_parameter<int>("odom_period_ms", 10);     // 100Hz, SUPER 断 odom 会停止规划

        // 2. 发布者 (SUPER 订阅端为 best_effort, 发布端用同款 QoS 保持匹配)
        auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
        pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, qos);
        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, qos);

        // 3. 连接 AirSim (LiDAR / 里程计 各一条独立 RPC 通道, 互不阻塞)
        const char* ip_env = std::getenv("PX4_SIM_HOST_ADDR");
        std::string airsim_ip = ip_env ? ip_env : "172.28.144.1";
        RCLCPP_INFO(this->get_logger(),
            "正在建立 SUPER 链路 LiDAR/odom 双通道直连 (IP: %s) ... "
            "[v4: 位姿按点云时刻对齐 + 段级真实渲染时刻去畸变]",
            airsim_ip.c_str());
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
            // 段级真实渲染时刻 (插件端 LidarCamera 异步捕获段上报): 每段记录
            // 起始扫角/扫角跨度 (度, 与激光角同坐标系) + 该段渲染瞬间 (AirSim 时钟)
            struct SegStamp { double start_deg, sweep_deg; uint64_t t_ns; };
            std::vector<SegStamp> seg_stamps;
            if (use_gpu_lidar_) {
                auto d = client_lidar_->getGPULidarData(lidar_name_, vehicle_name_);
                sim_stamp = d.time_stamp;
                points = std::move(d.point_cloud);
                seg_stamps.reserve(d.segment_stamps.size());
                for (const auto& s : d.segment_stamps) {
                    seg_stamps.push_back({s.start_angle, s.sweep_angle, s.time_stamp});
                }
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

            const size_t point_stride = use_gpu_lidar_ ? 5 : 3;
            const size_t total = points.size() / point_stride;
            if (total == 0) {
                return;
            }

            // 点云在传感器系, 需要机体在 world 系 (NED) 的位姿:
            //   传感器系 --挂载外参--> 机体系 --机体位姿--> world NED
            // 位姿取点云生成时刻 (LidarData.time_stamp) 而非 RPC 当下,
            // 从 100Hz odom 位姿缓冲插值, 保证点云-位姿时间对齐
            tf2::Quaternion q_v;
            tf2::Vector3 t_v;
            rclcpp::Time stamp = this->now();
            double cloud_age_ms = 0.0;
            if (!getPoseAtSimTime(sim_stamp, q_v, t_v, stamp, cloud_age_ms)) {
                return;  // 点云时间戳无法对齐 (数据过旧), 丢弃该帧
            }
            const tf2::Matrix3x3 r_v(q_v);

            // 挂载外参 (NED, 机身 -> 传感器): 与 settings.json 的 X/Y/Z/Roll/Pitch/Yaw 一致
            tf2::Quaternion q_mount;
            q_mount.setRPY(mount_roll_ * M_PI / 180.0,
                           mount_pitch_ * M_PI / 180.0,
                           mount_yaw_ * M_PI / 180.0);
            const tf2::Matrix3x3 r_mount(q_mount);
            const tf2::Vector3 t_mount(mount_x_, mount_y_, mount_z_);

            // 去畸变准备: GPU 雷达一帧是 UE 端一个旋转周期 (rot_period_s_) 内
            // 多次渲染累积的 (LidarCamera.cpp: 转满一圈才把 point_cloud 拷贝到
            // point_cloud_final 并发布), 帧内各点捕获时刻不同, 单一位姿无法正确
            // 放置整帧。点由激光角 LUT 重建 (polar_to_cartesian_lut_), 故每个点
            // 的传感器系方位角 = 它被捕获时扫描的角度。捕获时刻确定方式:
            //   1. 优先用插件上报的段级真实渲染时刻: 点方位角落在某段的
            //      [start_angle, start_angle+sweep) 扫角区间内, 则该点渲染于
            //      该段 time_stamp 瞬间 (消除 UE 异步渲染一帧延迟的系统性偏差);
            //   2. 回退按方位角均匀旋转反推: t_cap = 帧时间戳
            //      - (1 - (theta-start)/360) * 旋转周期 (CPU 雷达/旧插件版本)。
            // 位姿快照拷贝一次, 循环内免锁二分插值
            std::vector<PoseSample> pose_snap;
            if (deskew_en_) {
                std::lock_guard<std::mutex> lk(pose_hist_mutex_);
                pose_snap.assign(pose_hist_.begin(), pose_hist_.end());
            }
            const double rot_period_ns = rot_period_s_ * 1e9;
            const double start_frac = rot_start_deg_ / 360.0;
            // 按点方位角 (度, [0,360)) 查所属捕获段的真实渲染时刻; 找不到
            // (方位角与段边界舍入误差) 回退到扫角中心最近的段
            auto segTimeForAzimuth = [&](double theta_deg) -> uint64_t {
                if (seg_stamps.empty()) {
                    return 0;
                }
                uint64_t t_best = 0;
                double dist_best = 1e9;
                for (const auto& s : seg_stamps) {
                    const double sweep = std::max(s.sweep_deg, 1e-3);
                    // 扫描方向上相对段起始角的前向距离 (0..360)
                    const double fwd = std::fmod(theta_deg - s.start_deg + 720.0, 360.0);
                    if (fwd < sweep) {
                        return s.t_ns;
                    }
                    // 兜底: 与段扫角中心的圆周距离
                    const double center = std::fmod(s.start_deg + sweep / 2.0, 360.0);
                    const double dist = std::fabs(std::fmod(theta_deg - center + 540.0, 360.0) - 180.0);
                    if (dist < dist_best) {
                        dist_best = dist;
                        t_best = s.t_ns;
                    }
                }
                return t_best;
            };
            auto poseFromSnap = [&](uint64_t t_cap,
                                    tf2::Quaternion &q_o, tf2::Vector3 &p_o) -> bool {
                if (pose_snap.size() < 2 ||
                    t_cap < pose_snap.front().sim_t || t_cap > pose_snap.back().sim_t) {
                    return false;
                }
                size_t lo = 0, hi = pose_snap.size() - 1;
                while (hi - lo > 1) {
                    const size_t mid = (lo + hi) / 2;
                    if (pose_snap[mid].sim_t <= t_cap) lo = mid; else hi = mid;
                }
                const auto &s0 = pose_snap[lo];
                const auto &s1 = pose_snap[hi];
                const double alpha = (s1.sim_t == s0.sim_t)
                    ? 0.0
                    : static_cast<double>(t_cap - s0.sim_t) /
                      static_cast<double>(s1.sim_t - s0.sim_t);
                p_o = s0.p.lerp(s1.p, alpha);
                q_o = s0.q.slerp(s1.q, alpha);
                return true;
            };

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
                const tf2::Vector3 p_body = r_mount * p_sensor + t_mount;

                // 传感器系 -> world NED: 默认用帧时间戳位姿, 开启去畸变时按
                // 每个点的方位角确定捕获时刻 (优先段级真实渲染时刻, 回退均匀
                // 旋转反推) 并插值该时刻的位姿
                tf2::Vector3 p_w_ned;
                bool deskewed = false;
                if (deskew_en_) {
                    double theta = std::atan2(p_sensor.y(), p_sensor.x());  // [-pi, pi]
                    if (theta < 0.0) theta += 2.0 * M_PI;
                    uint64_t t_cap = 0;
                    if (!seg_stamps.empty()) {
                        // 优先: 插件上报的段级真实渲染时刻
                        t_cap = segTimeForAzimuth(theta * 180.0 / M_PI);
                    }
                    if (t_cap == 0) {
                        // 回退: 按旋转周期均匀假设由方位角反推
                        const double frac =
                            std::fmod(theta / (2.0 * M_PI) - start_frac + 1.0, 1.0);
                        t_cap = sim_stamp -
                            static_cast<uint64_t>((1.0 - frac) * rot_period_ns);
                    }
                    tf2::Quaternion q_i;
                    tf2::Vector3 t_i;
                    if (poseFromSnap(t_cap, q_i, t_i)) {
                        p_w_ned = tf2::Matrix3x3(q_i) * p_body + t_i;
                        deskewed = true;
                    }
                }
                if (!deskewed) {
                    p_w_ned = r_v * p_body + t_v;
                }

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

            // 仿真时钟基准: getMultirotorState 返回服务器仿真时间戳 (与
            // LidarData.time_stamp 同一时钟), 每 10 次调用同步一次; 各样本的
            // 仿真时间戳用本地时钟流逝从基准外推 (sim 倍速偏离 1 时误差仅为
            // 同步间隔 x 倍速差, ~ms 级), 不依赖 WSL/Windows 时钟对齐
            ++odom_cnt_;
            if (odom_cnt_ % 10 == 1) {
                const uint64_t t_ref = client_odom_->getMultirotorState().timestamp;
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
            uint64_t sample_sim_t = sim_ref_sim_t_ +
                static_cast<uint64_t>(static_cast<double>(local_ns - sim_ref_local_ns_) * sim_rate_);

            // 记录 (仿真时间戳, ROS 时刻, NED 位姿) 样本, 供 lidar 线程按点云
            // 生成时刻插值位姿; 时间轴必须单调: 小回跳 (仿真时钟速率抖动) 钳位
            // 处理不清缓冲, 大回跳 (>1s, UE 重启/世界重置) 才清空重建
            {
                tf2::Quaternion q_s(odom_data.pose.orientation.x(), odom_data.pose.orientation.y(),
                                    odom_data.pose.orientation.z(), odom_data.pose.orientation.w());
                q_s.normalize();
                std::lock_guard<std::mutex> lk(pose_hist_mutex_);
                if (!pose_hist_.empty() && sample_sim_t < pose_hist_.back().sim_t) {
                    if (pose_hist_.back().sim_t - sample_sim_t > 1000000000ULL) {
                        pose_hist_.clear();
                    } else {
                        sample_sim_t = pose_hist_.back().sim_t;
                    }
                }
                pose_hist_.push_back(PoseSample{sample_sim_t, current_time,
                                                tf2::Vector3(odom_data.pose.position.x(),
                                                             odom_data.pose.position.y(),
                                                             odom_data.pose.position.z()),
                                                q_s});
                while (pose_hist_.size() > 2 &&
                       sample_sim_t - pose_hist_.front().sim_t > kPoseHistNs) {
                    pose_hist_.pop_front();
                }
            }

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
    bool deskew_en_ = true;
    double rot_period_s_ = 0.1, rot_start_deg_ = 0.0;
    uint64_t last_sim_stamp_ = 0;       // 上次发布的雷达仿真 time_stamp (重复帧去重)
    double body_filter_radius_, intensity_;
    double mount_x_, mount_y_, mount_z_, mount_roll_, mount_pitch_, mount_yaw_;

    // 位姿环形缓冲: odom 线程 (100Hz) 写入, lidar 线程按点云生成时刻插值读取。
    // 解决 "当前位姿 × 旧点云快照" 在旋转/平移时的整体错位 (拖影) 问题
    struct PoseSample {
        uint64_t sim_t;       // AirSim TTimePoint (ns), 该位姿的仿真时间戳
        rclcpp::Time ros_t;   // 本端抓取该位姿的时刻 (用于对齐 ROS 消息时间戳)
        tf2::Vector3 p;       // world NED 位置
        tf2::Quaternion q;    // world NED 姿态
    };
    std::deque<PoseSample> pose_hist_;
    std::mutex pose_hist_mutex_;
    static constexpr uint64_t kPoseHistNs = 2ULL * 1000 * 1000 * 1000;  // 缓冲保留 2s

    // 仿真时钟基准 (仅 odom 线程访问): getMultirotorState 的服务器时间戳
    // 及其对应的本地 steady_clock 时刻, 用于外推各样本的仿真时间戳
    uint64_t odom_cnt_ = 0;
    uint64_t sim_ref_sim_t_ = 0;
    uint64_t sim_ref_local_ns_ = 0;
    double sim_rate_ = 1.0;  // 仿真时钟/本地时钟 速率估计 (滑动更新)

    // 按仿真时间戳插值出该时刻的机体位姿 (world NED) 与对应 ROS 时间戳。
    // 返回 false 且调用方应丢弃该帧: 点云时间戳早于缓冲最老样本 (点云过旧,
    // UE 端可能卡顿/暂停)。缓冲为空或时间戳无效时退回抓取当前位姿 (旧行为)
    bool getPoseAtSimTime(uint64_t sim_stamp, tf2::Quaternion &q_out,
                          tf2::Vector3 &p_out, rclcpp::Time &ros_t_out, double &age_ms) {
        std::lock_guard<std::mutex> lk(pose_hist_mutex_);
        if (sim_stamp == 0 || pose_hist_.empty()) {
            // 回退路径: 无缓冲可用 (启动初期/时间戳无效), 抓当前位姿
            try {
                const auto kinematics = client_lidar_->simGetGroundTruthKinematics(vehicle_name_);
                p_out = tf2::Vector3(kinematics.pose.position.x(), kinematics.pose.position.y(),
                                     kinematics.pose.position.z());
                q_out = tf2::Quaternion(kinematics.pose.orientation.x(), kinematics.pose.orientation.y(),
                                        kinematics.pose.orientation.z(), kinematics.pose.orientation.w());
                q_out.normalize();
                ros_t_out = this->now();
                age_ms = -1.0;  // 年龄未知
                return true;
            } catch (const rpc::rpc_error& e) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "抓取位姿失败: RPC 通信异常! (%s)", e.what());
                return false;
            }
        }
        const auto &front = pose_hist_.front();
        const auto &back = pose_hist_.back();
        if (sim_stamp < front.sim_t) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "点云落后位姿缓冲 %.0f ms, 已过旧, 丢弃该帧 (检查 UE 端帧率/Lidar UpdateFrequency)",
                (front.sim_t - sim_stamp) / 1e6);
            return false;
        }
        if (sim_stamp >= back.sim_t) {
            p_out = back.p;
            q_out = back.q;
            ros_t_out = back.ros_t;
            age_ms = 0.0;
            return true;
        }
        // sim_stamp 落在缓冲范围内: 找相邻两样本做插值
        for (size_t i = pose_hist_.size() - 1; i > 0; --i) {
            const auto &s0 = pose_hist_[i - 1];
            const auto &s1 = pose_hist_[i];
            if (s0.sim_t <= sim_stamp && sim_stamp <= s1.sim_t) {
                const double alpha = (s1.sim_t == s0.sim_t)
                    ? 0.0
                    : static_cast<double>(sim_stamp - s0.sim_t) /
                      static_cast<double>(s1.sim_t - s0.sim_t);
                p_out = s0.p.lerp(s1.p, alpha);
                q_out = s0.q.slerp(s1.q, alpha);
                ros_t_out = s0.ros_t + rclcpp::Duration::from_seconds(
                    alpha * (s1.ros_t - s0.ros_t).seconds());
                age_ms = (back.sim_t - sim_stamp) / 1e6;  // 点云生成至今的年龄
                return true;
            }
        }
        p_out = back.p;
        q_out = back.q;
        ros_t_out = back.ros_t;
        age_ms = 0.0;
        return true;
    }
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
