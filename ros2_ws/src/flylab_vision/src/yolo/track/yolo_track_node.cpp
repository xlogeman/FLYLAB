// Copyright 2026 flylab
/// YOLO 单目标追踪 ROS2 节点 —— 接入地面站视觉工具体系 (基类 common/display_vision_node.hpp)。
///
/// 链路 (话题名与 flylab_gcs/config.py 保持一致, 均可用参数覆盖):
///   1. 图像输入: 订阅地面站"当前显示的帧" /flylab/display/image_raw
///   2. 工具按钮: /flylab/vision_cmd 收 track_start / track_stop 启停
///   3. 框选目标: /flylab/track_roi (latched) 收地面站右栏"框选目标"按钮
///      框出的矩形 {"roi":[x,y,w,h]} (显示帧像素)
///   4. 开始/取消: /flylab/vision_cmd 收 track_begin / track_end
///   5. 画面叠加: 检测/目标框转 JSON 标注发 /flylab/annotations (组名 "track")
///   6. 面板数据: {"src":"track", ...} 发 /flylab/aruco_info (10Hz 限频)
///
/// 流程: 启动后逐帧检测 (未追踪时显示全部检测 + 框选区域) → 地面站框选
/// 目标 → track_begin 锁定: 框选区内 IoU 最大的检测裁块出 ReID 外观向量
/// (区内无检测时直接用框选区域兜底, 检测器漏检的目标也能跟) → 之后逐帧
/// 在检测候选里按余弦相似度续跟, 低于阈值报丢失 (保持最后已知位置)。
///
/// 模型: 检测 models/detect/yolo26s.onnx + 外观 models/track/yolo26s-reid.onnx
/// (CMakeLists 安装, 可用 model / reid_model 参数覆盖)。
///
/// 运行:
///   ros2 run flylab_vision yolo_track_node
///   ros2 run flylab_vision yolo_track_node --ros-args
///       -p model:=/path/to/xx.onnx -p device:=cuda -p match_thres:=0.35
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "common/display_vision_node.hpp"
#include "yolo/yolo_detector.hpp"
#include "yolo/yolo_tracker.hpp"

namespace flylab_vision
{

class YoloTrackNode final : public DisplayVisionNode
{
public:
  YoloTrackNode()
  : DisplayVisionNode("yolo_track_node",
      {"track_start", "track_stop", "track", "追踪"})
  {
    // ---------- 参数 ----------
    const auto model = declare_parameter<std::string>(
      "model", default_model_path("detect", "yolo26s.onnx"));
    const auto reid_model = declare_parameter<std::string>(
      "reid_model", default_model_path("track", "yolo26s-reid.onnx"));
    const auto input_size = declare_parameter<int>("input_size", 640);
    const auto conf = declare_parameter<double>("conf", 0.25);
    const auto iou = declare_parameter<double>("iou", 0.45);
    const auto device = declare_parameter<std::string>("device", "cpu");
    const auto match_thres = declare_parameter<double>("match_thres", 0.30);
    const auto iou_gate = declare_parameter<double>("iou_gate", 0.10);
    const auto max_scan = declare_parameter<int>("max_scan", 15);
    const auto roi_topic =
      declare_parameter<std::string>("roi_topic", "/flylab/track_roi");
    const auto ctl_topic =
      declare_parameter<std::string>("ctl_topic", "/flylab/vision_cmd");
    const auto info_topic =
      declare_parameter<std::string>("info_topic", "/flylab/aruco_info");

    // ---------- 模型 (启动即加载, 失败抛异常直接终止暴露配置问题) ----------
    YoloConfig cfg;
    cfg.model_path = model;
    cfg.input_size = input_size;
    cfg.conf_thres = static_cast<float>(conf);
    cfg.iou_thres = static_cast<float>(iou);
    cfg.use_cuda = device == "cuda";
    detector_ = std::make_unique<YoloDetector>(cfg);
    ReidConfig rcfg;
    rcfg.model_path = reid_model;
    rcfg.use_cuda = cfg.use_cuda;
    TrackParams tparams;
    tparams.match_thres = static_cast<float>(match_thres);
    tparams.iou_gate = static_cast<float>(iou_gate);
    tparams.max_scan = max_scan;
    tracker_ = std::make_unique<YoloTracker>(rcfg, tparams);
    RCLCPP_INFO(
      get_logger(),
      "模型加载完成: 检测 %s [%s], 外观 %s [%s], 匹配阈值 %.2f",
      model.c_str(), detector_->cuda_active() ? "cuda" : "cpu",
      reid_model.c_str(), tracker_->cuda_active() ? "cuda" : "cpu",
      tparams.match_thres);
    if (cfg.use_cuda && !detector_->cuda_active()) {
      RCLCPP_WARN(
        get_logger(), "检测 CUDA EP 注册失败, 已回退 CPU: %s",
        detector_->cuda_error().c_str());
    }
    if (cfg.use_cuda && !tracker_->cuda_active()) {
      RCLCPP_WARN(
        get_logger(), "外观 CUDA EP 注册失败, 已回退 CPU: %s",
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

    log_ready("目标追踪");
  }

protected:
  std::string process(const cv::Mat & bgr, int /*frame_width*/) override
  {
    // begin 请求在帧上下文里处理: 锁定需要当前帧来裁外观块
    if (begin_req_) {
      begin_req_ = false;
      if (!tracking_) {
        begin_tracking(bgr);
      }
    }
    const auto dets = detector_->detect(bgr);
    last_count_ = dets.size();
    std::string json;
    if (tracking_) {
      const auto st = tracker_->update(bgr, dets);
      track_state_ = st.has ? "TRACK" : "LOST";
      last_sim_ = st.sim;
      if (st.has) {
        track_box_ = st.box;
        last_name_ = st.name;
        last_conf_ = st.conf;
      }
      json = track_annotations(st);
    } else {
      // 未追踪: 显示全部检测 + 框选区域, 供用户核对框选
      track_state_ = has_roi_ ? "SELECTED" : "IDLE";
      json = browse_annotations(dets);
    }
    publish_info(static_cast<int>(dets.size()));
    return json;
  }

  std::string clear_json() const override
  {
    return "{\"algo\":\"" + std::string(algo_group()) + "\",\"items\":[]}";
  }

  void log_first_infer(long ms) override
  {
    RCLCPP_INFO(
      get_logger(), "首帧推理耗时 %ldms, 检出 %zu 个目标", ms, last_count_);
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
        if (!tracking_) {   // 追踪中不降状态 (状态由 process 按帧刷新)
          track_state_ = "IDLE";
          publish_info(0, true);
        }
        RCLCPP_INFO(get_logger(), "框选区域已清除");
      }
      return;
    }
    // {"roi":[x,y,w,h]} (显示帧像素, 与转发帧同一坐标系); 手写提取数字
    // 数组, 不引 JSON 依赖 (schema 由地面站保证)
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
    if (n != 4 || v[2] < 1.0 || v[3] < 1.0) {
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
    publish_info(0, true);
  }

  void on_ctl(const std_msgs::msg::String & msg)
  {
    if (msg.data == "track_begin" && !tracking_) {
      begin_req_ = true;   // 帧上下文里锁定 (需要裁外观块)
    } else if (msg.data == "track_end" && tracking_) {
      tracking_ = false;
      tracker_->reset();
      track_state_ = "IDLE";   // ROI 已随开始追踪消费, 需重新框选
      // 清面板快照: 否则 IDLE 态残留上次目标的类别/尺寸/相似度
      track_box_ = cv::Rect2f();
      last_name_.clear();
      last_conf_ = 0.f;
      last_sim_ = 0.f;
      RCLCPP_INFO(get_logger(), "追踪已取消 (重新追踪请重新框选目标)");
      publish_info(0, true);
    }
  }

  /// 开始追踪: 框选区内 IoU 最大的检测作目标裁块出外观向量; 区内无
  /// 检测时直接用框选区域兜底 (检测器漏检的目标也能跟)
  void begin_tracking(const cv::Mat & bgr)
  {
    if (!has_roi_) {
      RCLCPP_WARN(get_logger(), "尚未框选目标, 无法开始追踪");
      return;
    }
    const auto dets = detector_->detect(bgr);
    const YoloDetection * best = nullptr;
    float best_iou = 0.01f;   // 低于此 IoU 视为框内无检测
    for (const auto & d : dets) {
      const float v = box_iou(roi_, d.box);
      if (v > best_iou) {
        best_iou = v;
        best = &d;
      }
    }
    bool ok = false;
    if (best != nullptr) {
      ok = tracker_->set_target(bgr, best->box);
      if (ok) {
        RCLCPP_INFO(
          get_logger(), "目标已锁定: %s (%.2f), IoU %.2f",
          best->name.c_str(), best->conf, best_iou);
      }
    } else {
      ok = tracker_->set_target(bgr, roi_);
      if (ok) {
        RCLCPP_INFO(
          get_logger(), "框选区内无检测, 以框选区域外观直接锁定");
      }
    }
    if (ok) {
      tracking_ = true;
      // ROI 已消费: 进入追踪即销毁框选区域 (画面黄框随之消失, 未追踪
      // 状态才画黄框); 取消追踪后需重新框选才能再开始
      has_roi_ = false;
      roi_ = cv::Rect2f();
      track_state_ = "TRACK";
      publish_info(0, true);
    } else {
      RCLCPP_WARN(get_logger(), "开始追踪失败: 框选区域无效或过小");
    }
  }

  /// 追踪中标注: 只画目标框 (锁定青色 / 丢失红色), 画面干净聚焦目标
  std::string track_annotations(const TrackState & st) const
  {
    std::ostringstream os;
    os << "{\"algo\":\"" << algo_group() << "\",\"items\":[";
    os << "{\"type\":\"bbox\",\"box\":[" << num(st.box.x) << ","
       << num(st.box.y) << "," << num(st.box.width) << ","
       << num(st.box.height) << "],";
    if (st.has) {
      os << "\"label\":\"目标 " << num(st.sim, 2) << "\","
         << "\"color\":[0,255,255]}";
    } else {
      os << "\"label\":\"目标丢失\","
         << "\"color\":[255,0,0]}";
    }
    os << "]}";
    return os.str();
  }

  /// 未追踪标注: 全部检测 (类别色) + 框选区域 (黄色折线框)
  std::string browse_annotations(const std::vector<YoloDetection> & dets) const
  {
    std::string base = to_annotations_json(dets, algo_group());
    if (!has_roi_) {
      return base;
    }
    std::ostringstream os;
    os << "{\"type\":\"polyline\",\"close\":true,\"color\":[255,255,0],"
       << "\"points\":[[" << num(roi_.x) << "," << num(roi_.y) << "],"
       << "[" << num(roi_.x + roi_.width) << "," << num(roi_.y) << "],"
       << "[" << num(roi_.x + roi_.width) << "," << num(roi_.y + roi_.height) << "],"
       << "[" << num(roi_.x) << "," << num(roi_.y + roi_.height) << "]]}";
    // items 空时无逗号 (to_annotations_json 输出 "items":[] 或 "...item]")
    const std::string item = os.str();
    const std::string tail = base.substr(base.size() - 3);
    const bool empty_items = tail == "[]}";
    base.insert(base.size() - 2, (empty_items ? "" : ",") + item);
    return base;
  }

  /// 面板数据 {"src":"track", ...}: 10Hz 限频, force=True 状态切换立即发
  void publish_info(int count, bool force = false)
  {
    const auto now = std::chrono::steady_clock::now();
    if (!force && now - info_last_ < std::chrono::milliseconds(100)) {
      return;
    }
    info_last_ = now;
    std::ostringstream os;
    os << "{\"src\":\"" << algo_group() << "\","
       << "\"state\":\"" << track_state_ << "\","
       << "\"cls\":\"" << last_name_ << "\","
       << "\"conf\":" << num(last_conf_, 2) << ","
       << "\"sim\":" << num(last_sim_, 2) << ","
       << "\"w\":" << num(track_box_.width, 0) << ","
       << "\"h\":" << num(track_box_.height, 0) << ","
       << "\"count\":" << count << "}";
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

  std::unique_ptr<YoloDetector> detector_;
  std::unique_ptr<YoloTracker> tracker_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr roi_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ctl_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr info_pub_;
  // 追踪状态 (proc_group_ 单线程访问; track_end 在 ROS 回调线程处理)
  cv::Rect2f roi_;
  bool has_roi_ = false;
  bool tracking_ = false;
  bool begin_req_ = false;
  std::string track_state_ = "IDLE";
  // 面板数据快照 (追踪标注路径里随 TrackState 更新)
  cv::Rect2f track_box_;
  std::string last_name_;
  float last_conf_ = 0.f;
  float last_sim_ = 0.f;
  size_t last_count_ = 0;
  std::chrono::steady_clock::time_point info_last_;
};

}  // namespace flylab_vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<flylab_vision::YoloTrackNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
