// airsim_camera —— 通用相机采集节点 (算法无关, 可插拔)
//
// 功能: 通过 RPC 直连 Cosys-AirSim, 按参数采集一路或多路相机并发布 ROS2 话题。
//       与算法解耦: 深度图/双目/单目均由参数配置, 哪个算法需要就由其 launch
//       启动一个实例并按需配置; 多相机在同一次 RPC 请求中采集 (硬件同步),
//       共用同一时间戳。
// 参数:
//   camera_names    相机名列表, 逗号分隔 (默认 "left_cam,front_cam,right_cam",
//                   需与 UE 端 settings.json 中的相机名一致)
//   image_type      AirSim ImageType 枚举值 (0=Scene 1=DepthPlanar
//                   2=DepthPerspective 3=DepthVis ...), 对所有相机生效
//   pixels_as_float -1=自动 (Scene 用 uint8, 深度类用 float32), 或显式 0/1
//   grayscale       Scene 图转灰度 (mono8) 输出, 供 ORB-SLAM3/OpenVINS 等使用
//   period_ms       采集周期 (默认 33ms = 30Hz)
//   vehicle_name    默认 PX4
// 发布: /airsim/cam/<相机名>/image_raw
//       编码: 深度类 32FC1 (单位米); Scene 类 bgr8, grayscale=true 时 mono8
//       frame_id: <相机名>_optical_frame (相机光学坐标系)
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "airsim_common.hpp"

using namespace std::chrono_literals;

class AirsimCameraNode : public rclcpp::Node {
public:
    AirsimCameraNode() : Node("airsim_camera_node") {
        // 1. 参数
        vehicle_name_ = this->declare_parameter<std::string>("vehicle_name", "PX4");
        const std::string names_str =
            this->declare_parameter<std::string>("camera_names", "left_cam,front_cam,right_cam");
        const int image_type = this->declare_parameter<int>("image_type", 0);  // 0 = Scene
        const int pixels_as_float = this->declare_parameter<int>("pixels_as_float", -1);
        grayscale_ = this->declare_parameter<bool>("grayscale", false);
        const int period_ms = this->declare_parameter<int>("period_ms", 33);

        // 解析相机名列表 (去空白)
        for (size_t pos = 0; pos <= names_str.size(); ++pos) {
            const size_t begin = names_str.find_first_not_of(" ,", pos);
            if (begin == std::string::npos) break;
            const size_t end = names_str.find(',', begin);
            const size_t stop = (end == std::string::npos) ? names_str.size() : end;
            camera_names_.push_back(names_str.substr(begin, stop - begin));
            pos = stop;
        }
        if (camera_names_.empty()) {
            throw std::runtime_error("camera_names 参数为空, 至少需要一个相机名");
        }

        // pixels_as_float 自动判定: 深度类需要 float 返回
        const bool is_depth = (image_type != 0);  // 非 Scene 视为深度/测距类
        const bool as_float = (pixels_as_float < 0) ? is_depth : (pixels_as_float != 0);

        // 2. 发布者 (QoS reliable, 与各算法订阅端匹配)
        auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
        for (const auto& name : camera_names_) {
            pubs_.push_back(this->create_publisher<sensor_msgs::msg::Image>(
                "/airsim/cam/" + name + "/image_raw", qos));
        }

        // 3. 连接 AirSim
        const std::string ip = flylab::airsim_host();
        RCLCPP_INFO(this->get_logger(), "相机采集节点启动 (IP: %s, 相机: [%s], image_type=%d%s)",
                    ip.c_str(), names_str.c_str(), image_type,
                    grayscale_ ? ", 灰度输出" : "");
        try {
            client_ = flylab::connect_airsim(ip);
        } catch (const rpc::rpc_error& e) {
            RCLCPP_ERROR(this->get_logger(), "连接失败: %s", e.what());
            throw std::runtime_error("AirSim Connection Failed");
        }

        // 4. 图像请求: 所有相机打包进同一次 simGetImages 调用 (硬件同步)
        for (const auto& name : camera_names_) {
            requests_.push_back(msr::airlib::ImageCaptureBase::ImageRequest(
                name,
                static_cast<msr::airlib::ImageCaptureBase::ImageType>(image_type),
                as_float, false));
        }

        // 5. 采集定时器
        timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms),
                                         std::bind(&AirsimCameraNode::grab_images, this));
    }

private:
    void grab_images() {
        try {
            const auto& responses = client_->simGetImages(requests_, vehicle_name_);
            if (responses.size() != requests_.size()) {
                return;
            }
            // 多相机共用同一时间戳 (同一次请求, 硬件同步)。
            // 时间戳必须与 airsim_imu 等节点落在同一时间轴 (SimClockSync 映射
            // 仿真 time_stamp), 否则视觉算法 (OpenVINS 等) 按时间戳对齐 IMU/图像
            // 时会全部错位, 永远无法初始化; 仿真端未填 time_stamp 时退回墙钟。
            const double wall_s = this->now().seconds();
            rclcpp::Time stamp = this->now();
            if (responses[0].time_stamp > 0) {
                const double mapped =
                    clock_sync_.to_ros(responses[0].time_stamp * 1e-9, wall_s);
                stamp = rclcpp::Time(static_cast<int64_t>(mapped * 1e9), RCL_ROS_TIME);
            }
            for (size_t i = 0; i < responses.size(); ++i) {
                const auto& resp = responses[i];
                std_msgs::msg::Header header;
                header.stamp = stamp;
                header.frame_id = camera_names_[i] + "_optical_frame";

                if (!resp.image_data_float.empty()) {
                    // 深度图: 32 位浮点 (米)
                    cv::Mat img(resp.height, resp.width, CV_32FC1,
                                (void*)resp.image_data_float.data());
                    pubs_[i]->publish(*cv_bridge::CvImage(header, "32FC1", img).toImageMsg());
                } else if (!resp.image_data_uint8.empty()) {
                    // Scene 图: BGR, 可选灰度
                    cv::Mat img(resp.height, resp.width, CV_8UC3,
                                (void*)resp.image_data_uint8.data());
                    if (grayscale_) {
                        cv::Mat gray;
                        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
                        pubs_[i]->publish(*cv_bridge::CvImage(header, "mono8", gray).toImageMsg());
                    } else {
                        pubs_[i]->publish(*cv_bridge::CvImage(header, "bgr8", img).toImageMsg());
                    }
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                         "相机 %s 返回空图像 (深度类请确认 pixels_as_float 已开启)",
                                         camera_names_[i].c_str());
                }
            }
        } catch (const rpc::rpc_error& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "抓取图像失败: RPC 通信异常! (%s)", e.what());
        }
    }

    std::unique_ptr<flylab::AirSimClient> client_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> pubs_;
    std::vector<msr::airlib::ImageCaptureBase::ImageRequest> requests_;
    std::vector<std::string> camera_names_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string vehicle_name_;
    bool grayscale_ = false;
    flylab::SimClockSync clock_sync_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AirsimCameraNode>());
    rclcpp::shutdown();
    return 0;
}
