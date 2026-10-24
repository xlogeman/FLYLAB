// Copyright 2026 flylab
#ifndef FLYLAB_VISION__YOLO_DETECTOR_HPP_
#define FLYLAB_VISION__YOLO_DETECTOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace flylab_vision
{

/// COCO 80 类类别名 (yolo26s 默认训练集), 下标即类别 id
const std::vector<std::string> & coco_names();

/// 检测器配置
struct YoloConfig
{
  std::string model_path;            // ONNX 模型路径
  int input_size = 640;              // 网络输入边长 (letterbox 到正方形)
  float conf_thres = 0.25f;          // 置信度阈值
  float iou_thres = 0.45f;           // NMS IoU 阈值
  std::vector<int> class_filter;     // 只保留这些类别 id; 空 = 全部
  bool use_cuda = false;             // onnxruntime 无 CUDA EP 时自动回退 CPU
};

/// 单个目标的检测结果 (一帧可含多个)
struct YoloDetection
{
  cv::Rect2f box;        // 原图像素坐标 (x, y 为左上角)
  int class_id = -1;     // 类别下标 (COCO 80 类)
  std::string name;      // 类别名
  float conf = 0.f;      // 置信度 0~1
};

/// YOLO 目标检测器 —— 纯算法, 不含 ROS / 窗口显示。
/// 推理后端为 onnxruntime (第三方预编译包 thirdparty/onnxruntime/,
/// 系统 OpenCV 4.5.x 的 DNN 解析不了 yolo26 的 attention Split 节点)。
/// 模型: Ultralytics ONNX 标准导出 (输入 1x3xSxS, 输出 1x84xA:
/// 4 框坐标 + 80 类分数; 兼容 anchor-major 布局)。
/// 单实例单线程使用。
class YoloDetector
{
public:
  /// 模型加载/空跑校验失败抛 std::runtime_error
  explicit YoloDetector(const YoloConfig & config);
  ~YoloDetector();

  /// 单帧检测 (BGR), 返回按类 NMS 后的结果; 无目标返回空
  std::vector<YoloDetection> detect(const cv::Mat & bgr);


  /// CUDA EP 是否真正生效 (use_cuda=true 但注册失败时为 false, 已回退 CPU)
  bool cuda_active() const;
  /// CUDA EP 注册失败原因 (成功/未启用为空)
  std::string cuda_error() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;   // pimpl: 隔离 onnxruntime 头文件依赖
  YoloConfig config_;
};

/// 检测结果 -> 地面站叠加标注 JSON (schema 见 flylab_tools/overlay.py)。
/// 输出为按算法分组的消息 {"algo": algo, "items": [...]}, 每个目标:
///   bbox (box=[x,y,w,h], label="类别 置信度", 颜色按类别 id 稳定生成)
/// 坐标为原图像素, 图像源须与地面站显示的相机/分辨率一致。
std::string to_annotations_json(
  const std::vector<YoloDetection> & results, const std::string & algo = "yolo");

/// 按类别 id 生成稳定的 BGR 颜色 (HSV 色环均匀取色, 同类同色)
cv::Scalar class_color(int class_id);

}  // namespace flylab_vision

#endif  // FLYLAB_VISION__YOLO_DETECTOR_HPP_
