// Copyright 2026 flylab
#ifndef FLYLAB_VISION__YOLO_POSE_HPP_
#define FLYLAB_VISION__YOLO_POSE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace flylab_vision
{

/// Pose 估计器配置
struct YoloPoseConfig
{
  std::string model_path;          // ONNX 模型路径
  int input_size = 640;            // 网络输入边长 (letterbox 到正方形, pose 模型为 640)
  float conf_thres = 0.25f;        // 置信度阈值
  float iou_thres = 0.45f;         // 水平框 NMS IoU 阈值
  float kpt_conf = 0.5f;           // 关键点可见性阈值 (低于则不绘制/不输出坐标)
  bool use_cuda = false;           // onnxruntime 无 CUDA EP 时自动回退 CPU
};

/// 单个人体姿态检测结果 (一帧可含多个)
struct YoloPoseDetection
{
  cv::Rect2f box;                        // 水平框, 原图像素坐标
  std::string name;                      // 类别名 (COCO-pose 为 "person")
  float conf = 0.f;                      // 置信度 0~1
  std::vector<cv::Point2f> keypoints;    // 17 个关键点, 原图像素坐标
  std::vector<float> kpt_conf;           // 各关键点可见性 0~1 (与 keypoints 等长)
};

/// YOLO Pose (人体关键点) 估计器 —— 纯算法, 不含 ROS / 窗口显示。
/// 推理后端为 onnxruntime (选型原因见 yolo_detector.hpp)。
/// 模型: Ultralytics Pose ONNX 标准导出 (models/pose/yolo26s-pose.onnx)
///   输入  1x3xSxS (RGB, /255, letterbox)
///   输出  1x(5+3*nk)xA: 前 4 通道 = cx/cy/w/h (letterbox 输入坐标系, 像素),
///         第 5 通道 = 类别分数 (COCO-pose 单类 person),
///         后 3*nk 通道 = 关键点 x/y/可见性 (letterbox 坐标系);
///         兼容 anchor-major 布局。nk 与关键点维数在首次空跑时校验。
/// NMS 为水平框贪心抑制 (与 yolo_detector 同思路)。
/// 单实例单线程使用。
class YoloPoseDetector
{
public:
  /// 模型加载/空跑校验失败抛 std::runtime_error
  explicit YoloPoseDetector(const YoloPoseConfig & config);
  ~YoloPoseDetector();

  /// 单帧推理 (BGR), 返回 NMS 后的结果; 无目标返回空
  std::vector<YoloPoseDetection> detect(const cv::Mat & bgr);

  /// 关键点个数 (由模型输出通道数推出, COCO-pose 为 17)
  int kpt_count() const;

  /// CUDA EP 是否真正生效 (use_cuda=true 但注册失败时为 false, 已回退 CPU)
  bool cuda_active() const;
  /// CUDA EP 注册失败原因 (成功/未启用为空)
  std::string cuda_error() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;   // pimpl: 隔离 onnxruntime 头文件依赖
  YoloPoseConfig config_;
};

/// 检测结果 -> 地面站叠加标注 JSON (schema 见 flylab_gcs/overlay.py)。
/// 输出为按算法分组的消息 {"algo": algo, "items": [...]}, 每个人体:
///   骨架连线 polyline (close=false) + 可见关键点 point
/// (box/conf 只用于内部解码与 NMS, 不输出到画面 —— 单人类别下骨架
/// 本身即完整定位信息, 再画框属于冗余)
/// 坐标为原图像素, 图像源须与地面站显示的相机/分辨率一致。
std::string to_annotations_json(
  const std::vector<YoloPoseDetection> & results, const std::string & algo = "pose");

}  // namespace flylab_vision

#endif  // FLYLAB_VISION__YOLO_POSE_HPP_
