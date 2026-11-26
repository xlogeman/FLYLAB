// Copyright 2026 flylab
/// ORTrack 单目标追踪 ROS2 节点 —— 接入地面站视觉工具体系。
///
/// 公共脚手架 (ROI 框选 / 启停指令 / 反控接管 / 面板数据 / 清屏) 见
/// common/track_node_base.hpp, 本文件只保留 模板匹配方案的差异部分。
///
/// ORTrack: CVPR 2025 "Learning Occlusion-Robust Vision Transformers for
/// Real-Time UAV Tracking", 模板匹配式 Siamese-ViT 追踪器 (无检测候选,
/// 与 yolo_track_node 的 检测+ReID 方案互为补充, 对遮挡更鲁棒)。
///
/// 流程: track_start 启动 → 画面拖框框住目标 (松开即自动锁定) → 当前帧
/// 裁 128x128 模板, 全程固定不更新 → 逐帧以上帧结果为中心裁
/// 256x256 搜索区推理, Hann 窗加权解码目标框; 追踪中重新框选即换目标。
///
/// 模型: models/track/ortrack_deit_tiny.onnx (CMakeLists 安装, 可用
/// model 参数覆盖) + 类别 models/classify/yolo26s-cls.onnx (模板匹配无
/// 类别输出, 追踪框裁块按 cls_interval 限频分类, 结果上面板"目标"栏);
/// device:=cuda 时 CUDA EP 注册失败自动回退 CPU。
///
/// 运行:
///   ros2 run flylab_vision ortrack_node
///   ros2 run flylab_vision ortrack_node --ros-args -p device:=cuda
#include <chrono>
#include <memory>
#include <sstream>
#include <string>

#include "classify/yolo_classifier.hpp"
#include "common/track_node_base.hpp"
#include "track/ortrack.hpp"

namespace flylab_vision
{

class ORTrackNode final : public TrackNodeBase
{
public:
  ORTrackNode()
  : TrackNodeBase("ortrack_node",
      {"track_start", "track_stop", "track", "ORTrack 追踪"})
  {
    min_roi_px_ = 2.0;   // 模板裁块最小边长 (公共层默认 1.0)

    // ---------- 参数 ----------
    const auto model = declare_parameter<std::string>(
      "model", default_model_path("track", "ortrack_deit_tiny.onnx"));
    const auto cls_model = declare_parameter<std::string>(
      "cls_model", default_model_path("classify", "yolo26s-cls.onnx"));
    // 类别识别限频 (秒): 追踪框裁块跑 cls 模型的最小间隔
    const auto cls_interval = declare_parameter<double>("cls_interval", 0.5);
    const auto device = declare_parameter<std::string>("device", "cpu");

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
    // 类别模型 (ORTrack 无类别输出): 追踪框裁块跑 YOLO cls, 结果上面板
    ClsConfig ccfg;
    ccfg.model_path = cls_model;
    ccfg.topk = 1;
    ccfg.use_cuda = cfg.use_cuda;
    classifier_ = std::make_unique<YoloClassifier>(ccfg);
    RCLCPP_INFO(
      get_logger(), "类别模型加载完成: %s [%s, classes %d], 分类间隔 %.1fs",
      cls_model.c_str(), classifier_->cuda_active() ? "cuda" : "cpu",
      classifier_->num_classes(), cls_interval);
    if (cfg.use_cuda && !classifier_->cuda_active()) {
      RCLCPP_WARN(
        get_logger(), "类别 CUDA EP 注册失败, 已回退 CPU: %s",
        classifier_->cuda_error().c_str());
    }
    cls_interval_ = cls_interval;

    log_ready("ORTrack 追踪");
  }

protected:
  bool is_tracking() const override {return tracker_->tracking();}

  void stop_tracking() override
  {
    if (tracker_->tracking()) {
      tracker_->reset();   // 清旧模板, 随后 initialize 重锁新目标
    }
  }

  /// 框选区域即目标, 当前帧裁模板锁定
  void begin_tracking(const cv::Mat & bgr) override
  {
    if (tracker_->initialize(bgr, roi_)) {
      // 换目标清类别快照, 立即重分类新目标
      cls_name_.clear();
      cls_conf_ = 0.f;
      cls_last_ = {};
      const cv::Rect2f locked = roi_;
      clear_roi();
      track_state_ = "TRACK";
      publish_info(true);
      RCLCPP_INFO(
        get_logger(), "目标已锁定: (%.0f, %.0f, %.0fx%.0f), 逐帧模板匹配追踪中",
        locked.x, locked.y, locked.width, locked.height);
    } else {
      RCLCPP_WARN(get_logger(), "开始追踪失败: 框选区域无效或过小");
    }
  }

  void append_info_fields(std::ostringstream & os, bool /*force*/) override
  {
    // cls/conf: 类别模型快照 (未出首帧分类前退化为 ORTrack + 追踪峰值分);
    // 无 count 字段 —— 模板匹配无检测候选, 地面站据此隐藏"检出"行
    const bool has_cls = !cls_name_.empty();
    os << "\"cls\":\"" << (has_cls ? cls_name_ : "ORTrack") << "\","
       << "\"conf\":" << num(has_cls ? cls_conf_ : last_score_, 2) << ","
       << "\"sim\":" << num(last_score_, 2);
  }

  std::string process(const cv::Mat & bgr, int /*frame_width*/) override
  {
    takeover_->notify_frame();   // 反控图像断流看门狗喂狗
    consume_begin(bgr);          // begin 帧上下文处理: 锁定需当前帧裁模板
    std::string json;
    if (tracker_->tracking()) {
      const cv::Rect2f box = tracker_->track(bgr);
      track_box_ = box;
      track_state_ = "TRACK";
      last_score_ = tracker_->last_score();
      classify_target(bgr, box);
      // 云台伺服 + 反控测量 (模板匹配恒有目标框)
      servo_follow(box, bgr, true);
      // 框上标签 = 类别名 + 概率 (首帧分类未出前不显示文字)
      json = target_box_json(box, cls_label(), false);
      publish_info();
    } else {
      // 未追踪: 无检测候选可画, 只回显框选区域供用户核对
      track_state_ = has_roi_ ? "SELECTED" : "IDLE";
      servo_follow(cv::Rect2f(), bgr, false);
      json = browse_annotations();
      publish_info();
    }
    return json;
  }

private:
  /// 目标类别识别: 追踪框裁块跑 YOLO cls 模型, 按 cls_interval 限频
  /// 更新快照 (面板"目标"栏显示; ORTrack 本身无类别输出)
  void classify_target(const cv::Mat & bgr, const cv::Rect2f & box)
  {
    const auto now = std::chrono::steady_clock::now();
    if (now - cls_last_ < std::chrono::duration<double>(cls_interval_)) {
      return;
    }
    // 追踪框夹回画面内 (输出可能贴边越界), 退化裁块跳过本轮
    const cv::Rect roi = cv::Rect(box) & cv::Rect(0, 0, bgr.cols, bgr.rows);
    if (roi.width < 2 || roi.height < 2) {
      return;
    }
    const auto preds = classifier_->classify(bgr(roi));
    cls_last_ = now;
    if (!preds.empty()) {
      cls_name_ = preds.front().name;
      cls_conf_ = preds.front().conf;
      publish_info(true);   // 类别快照刷新即上屏
    }
  }

  std::string cls_label() const
  {
    return cls_name_.empty() ? std::string() :
      cls_name_ + " " + num(cls_conf_, 2);
  }

  /// 未追踪标注: 框选区域 (黄色折线框), 与 yolo_track_node 浏览态一致
  std::string browse_annotations() const
  {
    return "{\"algo\":\"" + std::string(algo_group()) + "\",\"items\":[" +
      roi_polyline_item() + "]}";
  }

  std::unique_ptr<ORTracker> tracker_;
  std::unique_ptr<YoloClassifier> classifier_;   // 类别模型 (面板"目标"栏)
  double cls_interval_ = 0.5;
  float last_score_ = 0.f;   // 追踪响应峰值分 (面板 sim/conf)
  // 类别识别快照 (cls_last_ 为上次分类时刻, epoch = 尚未分类)
  std::string cls_name_;
  float cls_conf_ = 0.f;
  std::chrono::steady_clock::time_point cls_last_;
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
