// Copyright 2026 flylab
/// ORTrack 单目标追踪 ROS2 节点 —— 接入地面站视觉工具体系
/// (基类 common/display_vision_node.hpp, 链路约定同 yolo_track_node)。
///
/// ORTrack: CVPR 2025 "Learning Occlusion-Robust Vision Transformers for
/// Real-Time UAV Tracking", 模板匹配式 Siamese-ViT 追踪器 (无检测候选,
/// 与 yolo_track_node 的 检测+ReID 方案互为补充, 对遮挡更鲁棒)。
///
/// 链路 (话题名与 flylab_gcs/config.py 保持一致, 均可用参数覆盖):
///   1. 图像输入: 订阅地面站"当前显示的帧" /flylab/display/image_raw
///   2. 工具按钮: /flylab/vision_cmd 收 track_start / track_stop 启停
///   3. 框选目标: /flylab/track_roi (latched) 收 {"roi":[x,y,w,h]}
///   4. 开始/取消: /flylab/vision_cmd 收 track_begin / track_end
///   5. 画面叠加: 目标框转 JSON 标注发 /flylab/annotations (组名 "track")
///   6. 面板数据: {"src":"track", ...} 发 /flylab/aruco_info (10Hz 限频)
///
/// 流程: track_start 启动 → 右栏"框选目标"框住目标 → track_begin 锁定
/// (当前帧裁 128x128 模板, 全程固定不更新) → 逐帧以上帧结果为中心裁
/// 256x256 搜索区推理, Hann 窗加权解码目标框。track_end 取消后需重新框选。
///
/// 模型: models/track/ortrack_deit_tiny.onnx (CMakeLists 安装, 可用
/// model 参数覆盖); device:=cuda 时 CUDA EP 注册失败自动回退 CPU。
///
/// 运行:
///   ros2 run flylab_vision ortrack_node
///   ros2 run flylab_vision ortrack_node --ros-args -p device:=cuda
#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "common/display_vision_node.hpp"
#include "track/ortrack.hpp"

namespace flylab_vision
{

class ORTrackNode final : public DisplayVisionNode
{
public:
  ORTrackNode()
  : DisplayVisionNode("ortrack_node",
      {"track_start", "track_stop", "track", "ORTrack 追踪"})
  {
    // ---------- 参数 ----------
    const auto model = declare_parameter<std::string>(
      "model", default_model_path("track", "ortrack_deit_tiny.onnx"));
    const auto device = declare_parameter<std::string>("device", "cpu");
    const auto roi_topic =
      declare_parameter<std::string>("roi_topic", "/flylab/track_roi");
    const auto ctl_topic =
      declare_parameter<std::string>("ctl_topic", "/flylab/vision_cmd");
    const auto info_topic =
      declare_parameter<std::string>("info_topic", "/flylab/aruco_info");

    // ---------- 模型 (启动即加载, 失败抛异常直接终止暴露配置问题) ----------
    ORTrackConfig cfg;
    cfg.model_path = model;
    cfg.use_cuda = device == "cuda";
    tracker_ = std::make_unique<ORTracker>(cfg);
    RCLCPP_INFO(
      get_logger(), "模型加载完成: %s [%s]",
      model.c_str(), tracker_->cuda_active() ? "cuda" : "cpu");
    if (cfg.use_cuda && !tracker_->cuda_active()) {
      RCLCPP_WARN(
        get_logger(), "CUDA EP 注册失败, 已回退 CPU: %s",
        tracker_->cuda_error().c_str());
    }

    // ---------- 通信 (基类之外: ROI 框选 / 开始取消指令 / 面板数据) ----------
    // latched: 地面站先框选后启动节点也能拿到当前框选区域
    roi_sub_ = create_subscription<std_msgs::msg::String>(
      roi_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {on_roi(*msg);});
    ctl_sub_ = create_subscription<std_msgs::msg::String>(
      ctl_topic, 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {on_ctl(*msg);});
    info_pub_ = create_publisher<std_msgs::msg::String>(info_topic, 10);

    log_ready("ORTrack 追踪");
  }

protected:
  std::string process(const cv::Mat & bgr, int /*frame_width*/) override
  {
    // begin 请求在帧上下文里处理: 锁定需要当前帧来裁模板
    if (begin_req_) {
      begin_req_ = false;
      if (!tracker_->tracking()) {
        begin_tracking(bgr);
      }
    }
    std::string json;
    if (tracker_->tracking()) {
      const cv::Rect2f box = tracker_->track(bgr);
      track_box_ = box;
      track_state_ = "TRACK";
      json = track_annotations(box, tracker_->last_score());
      publish_info();
    } else {
      // 未追踪: 无检测候选可画, 只回显框选区域供用户核对
      track_state_ = has_roi_ ? "SELECTED" : "IDLE";
      json = has_roi_ ? browse_annotations() : "";
      publish_info();
    }
    return json;
  }

  std::string clear_json() const override
  {
    return "{\"algo\":\"" + std::string(algo_group()) + "\",\"items\":[]}";
  }

private:
  void on_roi(const std_msgs::msg::String & msg)
  {
    // {"reset":true} 清除框选 (追踪算法停止时地面站发, latched 话题上的
    // 旧 ROI 不再投递给重连的节点)
    if (msg.data.find("\"reset\"") != std::string::npos) {
      if (has_roi_) {
        has_roi_ = false;
        roi_ = cv::Rect2f();
        if (!tracker_->tracking()) {
          track_state_ = "IDLE";
          publish_info(true);
        }
        RCLCPP_INFO(get_logger(), "框选区域已清除");
      }
      return;
    }
    // {"roi":[x,y,w,h]} (显示帧像素, 与转发帧同一坐标系); 手写提取数字
    // 数组, 不引 JSON 依赖 (schema 由地面站保证), 同 yolo_track_node
    const auto pos = msg.data.find("\"roi\"");
    if (pos == std::string::npos) {
      return;
    }
    size_t i = msg.data.find('[', pos);
    if (i == std::string::npos) {
      return;
    }
    ++i;   // 进入数组
    double v[4] = {0, 0, 0, 0};
    int n = 0;
    while (n < 4 && i < msg.data.size()) {
      const size_t end = msg.data.find_first_of(",]", i);
      if (end == std::string::npos) {
        return;
      }
      try {
        v[n++] = std::stod(msg.data.substr(i, end - i));
      } catch (const std::exception &) {
        return;
      }
      if (msg.data[end] == ']') {
        break;
      }
      i = end + 1;
    }
    if (n != 4 || v[2] < 2.0 || v[3] < 2.0) {
      RCLCPP_WARN(get_logger(), "框选区域无效: %s", msg.data.c_str());
      return;
    }
    roi_ = cv::Rect2f(
      static_cast<float>(v[0]), static_cast<float>(v[1]),
      static_cast<float>(v[2]), static_cast<float>(v[3]));
    has_roi_ = true;
    RCLCPP_INFO(
      get_logger(), "收到目标框选区域: (%.0f, %.0f, %.0fx%.0f)",
      roi_.x, roi_.y, roi_.width, roi_.height);
    publish_info(true);
  }

  void on_ctl(const std_msgs::msg::String & msg)
  {
    if (msg.data == "track_begin" && !tracker_->tracking()) {
      begin_req_ = true;   // 帧上下文里锁定 (需要裁模板)
    } else if (msg.data == "track_end" && tracker_->tracking()) {
      tracker_->reset();
      track_state_ = "IDLE";   // ROI 已随开始追踪消费, 需重新框选
      track_box_ = cv::Rect2f();
      last_score_ = 0.f;
      RCLCPP_INFO(get_logger(), "追踪已取消 (重新追踪请重新框选目标)");
      publish_info(true);
    }
  }

  /// 开始追踪: 框选区域即目标, 当前帧裁模板锁定
  void begin_tracking(const cv::Mat & bgr)
  {
    if (!has_roi_) {
      RCLCPP_WARN(get_logger(), "尚未框选目标, 无法开始追踪");
      return;
    }
    if (tracker_->initialize(bgr, roi_)) {
      tracking_started();
      RCLCPP_INFO(
        get_logger(), "目标已锁定: (%.0f, %.0f, %.0fx%.0f), 逐帧模板匹配追踪中",
        roi_.x, roi_.y, roi_.width, roi_.height);
    } else {
      RCLCPP_WARN(get_logger(), "开始追踪失败: 框选区域无效或过小");
    }
  }

  /// 锁定成功: ROI 已消费 (画面黄框随之消失), 取消追踪后需重新框选
  void tracking_started()
  {
    has_roi_ = false;
    roi_ = cv::Rect2f();
    track_state_ = "TRACK";
    publish_info(true);
  }

  /// 追踪中标注: 只画目标框 (青色, 峰值响应作置信度参考)
  std::string track_annotations(const cv::Rect2f & box, float score)
  {
    last_score_ = score;
    std::ostringstream os;
    os << "{\"algo\":\"" << algo_group() << "\",\"items\":["
       << "{\"type\":\"bbox\",\"box\":[" << num(box.x) << ","
       << num(box.y) << "," << num(box.width) << ","
       << num(box.height) << "],"
       << "\"label\":\"目标 " << num(score, 2) << "\","
       << "\"color\":[0,255,255]}]}";
    return os.str();
  }

  /// 未追踪标注: 框选区域 (黄色折线框), 与 yolo_track_node 浏览态一致
  std::string browse_annotations() const
  {
    std::ostringstream os;
    os << "{\"algo\":\"" << algo_group() << "\",\"items\":["
       << "{\"type\":\"polyline\",\"close\":true,\"color\":[255,255,0],"
       << "\"points\":[[" << num(roi_.x) << "," << num(roi_.y) << "],"
       << "[" << num(roi_.x + roi_.width) << "," << num(roi_.y) << "],"
       << "[" << num(roi_.x + roi_.width) << "," << num(roi_.y + roi_.height) << "],"
       << "[" << num(roi_.x) << "," << num(roi_.y + roi_.height) << "]]}]}";
    return os.str();
  }

  /// 面板数据 {"src":"track", ...}: 10Hz 限频, force=True 状态切换立即发
  /// (字段与 yolo_track_node 同 schema, 右栏共用目标追踪面板)
  void publish_info(bool force = false)
  {
    const auto now = std::chrono::steady_clock::now();
    if (!force && now - info_last_ < std::chrono::milliseconds(100)) {
      return;
    }
    info_last_ = now;
    std::ostringstream os;
    os << "{\"src\":\"" << algo_group() << "\","
       << "\"state\":\"" << track_state_ << "\","
       << "\"cls\":\"ORTrack\","
       << "\"conf\":" << num(last_score_, 2) << ","
       << "\"sim\":" << num(last_score_, 2) << ","
       << "\"w\":" << num(track_box_.width, 0) << ","
       << "\"h\":" << num(track_box_.height, 0) << ","
       << "\"count\":0}";
    std_msgs::msg::String out;
    out.data = os.str();
    info_pub_->publish(out);
  }

  static std::string num(double v, int prec = 1)
  {
    std::ostringstream os;
    os << std::fixed << std::setprecision(prec) << v;
    return os.str();
  }

  std::unique_ptr<ORTracker> tracker_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr roi_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ctl_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr info_pub_;
  // 追踪状态 (proc_group_ 单线程访问; on_roi/on_ctl 在 ROS 回调线程处理,
  // begin 经 begin_req_ 落到帧上下文, 与 yolo_track_node 同一约定)
  cv::Rect2f roi_;
  bool has_roi_ = false;
  bool begin_req_ = false;
  std::string track_state_ = "IDLE";
  // 面板数据快照
  cv::Rect2f track_box_;
  float last_score_ = 0.f;
  std::chrono::steady_clock::time_point info_last_;
};

}  // namespace flylab_vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<flylab_vision::ORTrackNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
