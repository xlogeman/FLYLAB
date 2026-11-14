// Copyright 2026 flylab
#ifndef FLYLAB_VISION__TRACK__ORTRACK_HPP_
#define FLYLAB_VISION__TRACK__ORTRACK_HPP_

#include <memory>
#include <string>

#include <opencv2/core.hpp>

namespace flylab_vision
{

/// ORTrack 推理配置 (models/track/ortrack_deit_tiny.onnx, CVPR 2025
/// "Learning Occlusion-Robust Vision Transformers for Real-Time UAV Tracking",
/// OSTrack 体系 Siamese-ViT 单目标追踪, 仓库自带 ONNX 导出)
struct ORTrackConfig
{
  std::string model_path;      // ONNX 模型路径
  bool use_cuda = false;       // onnxruntime 无 CUDA EP 时自动回退 CPU
};

/// ORTrack 单目标追踪器 —— 固定模板 + 逐帧搜索区前向。
/// 与 YoloTracker (检测+ReID 身份关联) 不同, 本类是"模板匹配"式追踪:
/// 首帧框选裁出 128x128 模板, 之后逐帧以上帧结果为中心裁 256x256 搜索区,
/// 网络输出响应图经 Hann 窗加权取峰解码出目标框。纯算法, 不含 ROS;
/// 单实例单线程使用 (推理在 DisplayVisionNode 的 proc 回调组串行驱动)。
class ORTracker
{
public:
  /// 模型加载失败抛 std::runtime_error
  explicit ORTracker(const ORTrackConfig & cfg);
  ~ORTracker();

  /// 锁定目标: 从图内 box 裁模板 (固定整个追踪过程不更新), 开始跟踪
  /// (区域退化裁不出有效块时返回 false)
  bool initialize(const cv::Mat & bgr, const cv::Rect2f & box);

  /// 取消跟踪 (清模板与状态, 回到未跟踪态)
  void reset();

  /// 逐帧更新: 以上帧结果为中心裁搜索区推理, 返回目标框 (原图像素)。
  /// 无内建丢失判据 (模板匹配总有输出), 峰值响应分数可作为置信度参考
  cv::Rect2f track(const cv::Mat & bgr);

  /// 本帧峰值响应分数 (Hann 加权后, 0~1; track() 后有效)
  float last_score() const;

  /// 是否已锁定目标 (initialize 成功且未 reset)
  bool tracking() const;

  /// CUDA EP 是否真正生效 (use_cuda=true 但注册失败时为 false, 已回退 CPU)
  bool cuda_active() const;
  /// CUDA EP 注册失败原因 (成功/未启用为空)
  std::string cuda_error() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;   // pimpl: 隔离 onnxruntime 依赖
  ORTrackConfig cfg_;
};

}  // namespace flylab_vision

#endif  // FLYLAB_VISION__TRACK__ORTRACK_HPP_
