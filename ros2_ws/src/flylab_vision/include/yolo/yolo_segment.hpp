// Copyright 2026 flylab
#ifndef FLYLAB_VISION__YOLO_SEGMENT_HPP_
#define FLYLAB_VISION__YOLO_SEGMENT_HPP_

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace flylab_vision
{

/// 实例分割器配置
struct YoloSegmentConfig
{
  std::string model_path;            // ONNX 模型路径
  int input_size = 640;              // 网络输入边长 (letterbox 到正方形)
  float conf_thres = 0.25f;          // 置信度阈值
  float iou_thres = 0.45f;           // NMS IoU 阈值
  float mask_thres = 0.5f;           // 掩码概率二值化阈值
  std::vector<int> class_filter;     // 只保留这些类别 id; 空 = 全部
  bool use_cuda = false;             // onnxruntime 无 CUDA EP 时自动回退 CPU
};

/// 单个实例的分割结果 (一帧可含多个)
struct YoloSegmentDetection
{
  cv::Rect2f box;        // 原图像素坐标 (x, y 为左上角)
  cv::Mat mask;          // 原图尺寸单通道 8U 掩码, 实例区域 255 其余 0
  int class_id = -1;     // 类别下标 (COCO 80 类)
  std::string name;      // 类别名
  float conf = 0.f;      // 置信度 0~1
};

/// YOLO 实例分割器 —— 纯算法, 不含 ROS / 窗口显示。
/// 推理后端为 onnxruntime (同 yolo_detector, 系统 OpenCV DNN 解析不了
/// yolo26 的 attention Split 节点)。
/// 模型: Ultralytics 分割 ONNX 标准导出 (models/segment/yolo26s-seg.onnx)
///   输入   1x3xSxS (RGB, /255, letterbox)
///   输出0  1x(4+nc+32)xA: 前 4 通道 = cx/cy/w/h (letterbox 坐标系, 像素),
///          中间 nc 通道 = 各类分数, 末尾 32 通道 = 掩码系数;
///          兼容 anchor-major 布局
///   输出1  原型掩码 1x32x(H/4)x(W/4)
/// 掩码 = sigmoid(系数 · 原型), 按检测框裁剪后上采样回原图并二值化。
/// 单实例单线程使用。
class YoloSegmentDetector
{
public:
  /// 模型加载/空跑校验失败抛 std::runtime_error
  explicit YoloSegmentDetector(const YoloSegmentConfig & config);
  ~YoloSegmentDetector();

  /// 单帧分割 (BGR), 返回按类 NMS 后的结果; 无目标返回空
  std::vector<YoloSegmentDetection> segment(const cv::Mat & bgr);


  /// CUDA EP 是否真正生效 (use_cuda=true 但注册失败时为 false, 已回退 CPU)
  bool cuda_active() const;
  /// CUDA EP 注册失败原因 (成功/未启用为空)
  std::string cuda_error() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;   // pimpl: 隔离 onnxruntime 头文件依赖
  YoloSegmentConfig config_;
};

/// 分割结果 -> 地面站叠加标注 JSON (schema 见 flylab_gcs/overlay.py)。
/// 输出为按算法分组的消息 {"algo": algo, "items": [...]}, 每个实例:
///   掩码轮廓 (polyline 闭合多边形 + fill 半透明填充, 颜色按类别 id
///   稳定生成, 每实例最多取 3 个最大轮廓) + 类别置信度文字 (text)
/// 坐标为原图像素, 图像源须与地面站显示的相机/分辨率一致。
std::string to_annotations_json(
  const std::vector<YoloSegmentDetection> & results, const std::string & algo = "seg");

}  // namespace flylab_vision

#endif  // FLYLAB_VISION__YOLO_SEGMENT_HPP_
