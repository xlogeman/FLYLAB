// Copyright 2026 flylab
#ifndef FLYLAB_VISION__YOLO_TRACKER_HPP_
#define FLYLAB_VISION__YOLO_TRACKER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "yolo/yolo_detector.hpp"

namespace flylab_vision
{

/// ReID 嵌入模型配置 (models/track/yolo26s-reid.onnx, Ultralytics 导出)
struct ReidConfig
{
  std::string model_path;        // ONNX 模型路径
  int input_size = 448;          // 裁块缩放边长 (模型 metadata imgsz 448x448)
  bool use_cuda = false;         // onnxruntime 无 CUDA EP 时自动回退 CPU
};

/// 跟踪匹配参数
struct TrackParams
{
  float match_thres = 0.30f;     // 余弦相似度匹配阈值, 低于视为丢失
  float iou_gate = 0.10f;        // 上一框邻域候选门槛 (邻域外不裁块省算力)
  int max_scan = 15;             // 全量兜底扫描时按置信度截断的候选数
};

/// 两框 IoU (float 域) —— 跟踪候选筛选与节点框选匹配共用
float box_iou(const cv::Rect2f & a, const cv::Rect2f & b);

/// 单帧跟踪状态
struct TrackState
{
  bool has = false;              // 本帧锁定到目标 (box/sim 有效)
  bool lost = false;             // 追踪中但本帧无匹配 (box = 最后已知位置)
  cv::Rect2f box;                // 目标框 (原图像素)
  float sim = 0.f;               // 与目标外观的余弦相似度 0~1
  std::string name;              // 匹配检测的类别名
  float conf = 0.f;              // 匹配检测的置信度
};

/// 单目标跟踪器 —— 检测框 + ReID 外观匹配 (检测由 YoloDetector 负责,
/// 本类只做身份关联): 目标框裁块出 512 维嵌入向量, 逐帧在检测候选里找
/// 余弦相似度最高者续跟。纯算法, 不含 ROS / 窗口显示; 单实例单线程使用。
class YoloTracker
{
public:
  /// 模型加载/空跑校验失败抛 std::runtime_error
  YoloTracker(const ReidConfig & reid_cfg, const TrackParams & params = {});
  ~YoloTracker();

  /// 锁定目标: 从图内 box 裁块提取外观向量, 开始跟踪
  /// (区域越界/退化裁不出有效块时返回 false)
  bool set_target(const cv::Mat & bgr, const cv::Rect2f & box);

  /// 取消跟踪 (清外观向量, 回到未跟踪态)
  void reset();

  /// 逐帧更新: 在检测结果里续跟目标。
  /// 候选优先取上一框邻域 (IoU >= iou_gate, 通常 1~3 个, 裁块省算力),
  /// 邻域无候选 (目标快速移动) 回退全量并按置信度截断 max_scan 个;
  /// 最佳余弦相似度 >= match_thres 续跟, 否则本帧丢失 (box 保持末位置)
  TrackState update(const cv::Mat & bgr, const std::vector<YoloDetection> & dets);

  /// CUDA EP 是否真正生效 (use_cuda=true 但注册失败时为 false, 已回退 CPU)
  bool cuda_active() const;
  /// CUDA EP 注册失败原因 (成功/未启用为空)
  std::string cuda_error() const;

private:
  struct Impl;
  /// 裁块 -> ReID 推理 -> L2 归一化嵌入向量 (越界/退化裁块返回空)
  std::vector<float> embed_crop(
    const cv::Mat & bgr, const cv::Rect2f & box) const;
  std::unique_ptr<Impl> impl_;   // pimpl: 隔离 onnxruntime 依赖
  TrackParams params_;
};

}  // namespace flylab_vision

#endif  // FLYLAB_VISION__YOLO_TRACKER_HPP_
