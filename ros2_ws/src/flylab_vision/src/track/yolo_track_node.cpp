// Copyright 2026 flylab
/// YOLO 单目标追踪 ROS2 节点 —— 接入地面站视觉工具体系。
///
/// 公共脚手架 (ROI 框选 / 启停指令 / 反控接管 / 面板数据 / 清屏) 见
/// common/track_node_base.hpp, 本文件只保留 检测+ReID 方案的差异部分。
///
/// 方案: yolo26s 逐帧检测出候选, 框选区内 IoU 最大的检测裁块提 512 维
/// ReID 外观向量 (区内无检测时直接用框选区域兜底, 漏检目标也能跟),
/// 之后逐帧在候选里按余弦相似度续跟, 低于阈值报丢失 (保持最后已知
/// 位置); 未追踪时显示全部检测 + 框选区域供用户核对。
///
/// 模型: 检测 models/detect/yolo26s.onnx + 外观 models/track/yolo26s-reid.onnx
/// (CMakeLists 安装, 可用 model / reid_model 参数覆盖);
/// device:=cuda 时 CUDA EP 注册失败自动回退 CPU。
///
/// 运行:
///   ros2 run flylab_vision yolo_track_node
///   ros2 run flylab_vision yolo_track_node --ros-args
///       -p model:=/path/to/xx.onnx -p device:=cuda -p match_thres:=0.35
#include <memory>
#include <string>
#include <vector>

#include "common/track_node_base.hpp"
#include "detect/yolo_detector.hpp"
#include "track/yolo_tracker.hpp"

namespace flylab_vision
{

class YoloTrackNode final : public TrackNodeBase
{
public:
  YoloTrackNode()
  : TrackNodeBase("yolo_track_node",
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

    log_ready("目标追踪");
  }

protected:
  bool is_tracking() const override {return tracking_;}

  void stop_tracking() override
  {
    tracking_ = false;
    tracker_->reset();
  }

  /// 框选区内 IoU 最大的检测作目标裁块出外观向量; 区内无检测时直接
  /// 用框选区域兜底 (漏检目标也能跟)
  void begin_tracking(const cv::Mat & bgr) override
  {
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
        RCLCPP_INFO(get_logger(), "框选区内无检测, 以框选区域外观直接锁定");
      }
    }
    if (ok) {
      tracking_ = true;
      clear_roi();
      track_state_ = "TRACK";
      publish_info(true);
    } else {
      RCLCPP_WARN(get_logger(), "开始追踪失败: 框选区域无效或过小");
    }
  }

  void append_info_fields(std::ostringstream & os, bool force) override
  {
    os << "\"cls\":\"" << last_name_ << "\","
       << "\"conf\":" << num(last_conf_, 2) << ","
       << "\"sim\":" << num(last_sim_, 2) << ","
       // force 上屏 = 状态切换 (非逐帧), 检出行归零
       << "\"count\":" << (force ? 0 : static_cast<int>(last_count_));
  }

  void log_first_infer(long ms) override
  {
    RCLCPP_INFO(
      get_logger(), "首帧推理耗时 %ldms, 检出 %zu 个目标", ms, last_count_);
  }

  std::string process(const cv::Mat & bgr, int /*frame_width*/) override
  {
    takeover_->notify_frame();   // 反控图像断流看门狗喂狗
    consume_begin(bgr);          // begin 帧上下文处理: 锁定需当前帧裁外观块
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
      // 云台伺服 (丢失清零速率) + 反控测量
      servo_follow(st.box, bgr, st.has);
      // 框上标签 = 类别名 + 检测置信度 (无前缀, 相似度只在右下面板看)
      json = st.has ?
        target_box_json(
        st.box, std::string(st.name) + " " + num(st.conf, 2), false) :
        target_box_json(st.box, "目标丢失", true);
      publish_info();
    } else {
      // 未追踪: 显示全部检测 + 框选区域, 供用户核对框选
      track_state_ = has_roi_ ? "SELECTED" : "IDLE";
      servo_follow(cv::Rect2f(), bgr, false);
      json = browse_annotations(dets);
      publish_info();
    }
    return json;
  }

private:
  /// 未追踪标注: 全部检测 (类别色) + 框选区域 (黄色折线框)
  std::string browse_annotations(const std::vector<YoloDetection> & dets) const
  {
    std::string base = to_annotations_json(dets, algo_group());
    if (!has_roi_) {
      return base;
    }
    // items 空时无逗号 (to_annotations_json 输出 "items":[] 或 "...item]")
    const std::string item = roi_polyline_item();
    const bool empty_items = base.substr(base.size() - 3) == "[]}";
    base.insert(base.size() - 2, (empty_items ? "" : ",") + item);
    return base;
  }

  std::unique_ptr<YoloDetector> detector_;
  std::unique_ptr<YoloTracker> tracker_;
  bool tracking_ = false;
  // 面板数据快照 (追踪标注路径里随 TrackState 更新)
  std::string last_name_;
  float last_conf_ = 0.f;
  float last_sim_ = 0.f;
  size_t last_count_ = 0;
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
