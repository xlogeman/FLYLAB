// Copyright 2026 flylab
#ifndef FLYLAB_VISION__YOLO_OBB_HPP_
#define FLYLAB_VISION__YOLO_OBB_HPP_

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace flylab_vision
{

/// DOTA v1 15 类类别名 (yolo26s-obb 默认训练集), 下标即类别 id
const std::vector<std::string> & dota_names();

/// OBB 检测器配置
struct YoloObbConfig
{
  std::string model_path;            // ONNX 模型路径
  int input_size = 1024;             // 网络输入边长 (letterbox 到正方形, obb 模型为 1024)
  float conf_thres = 0.25f;          // 置信度阈值
  float iou_thres = 0.45f;           // 旋转框 NMS IoU 阈值
  std::vector<int> class_filter;     // 只保留这些类别 id; 空 = 全部
  bool use_cuda = false;             // onnxruntime 无 CUDA EP 时自动回退 CPU
};

/// 单个旋转框检测结果 (一帧可含多个)
struct YoloObbDetection
{
  cv::RotatedRect box;   // 原图像素坐标 (center + size + angle, angle 单位: 度)
  int class_id = -1;     // 类别下标 (DOTA 15 类)
  std::string name;      // 类别名
  float conf = 0.f;      // 置信度 0~1
};

/// YOLO OBB (旋转框) 检测器 —— 纯算法, 不含 ROS / 窗口显示。
/// 推理后端为 onnxruntime (同 yolo_detector, 系统 OpenCV DNN 解析不了
/// yolo26 的 attention Split 节点)。
/// 模型: Ultralytics OBB ONNX 标准导出 (models/obb/yolo26s-obb.onnx)
///   输入  1x3xSxS (RGB, /255, letterbox)
///   输出  1x(5+nc)xA: 前 4 通道 = cx/cy/w/h (letterbox 输入坐标系, 像素),
///         第 5 通道 = 角度 (弧度), 后 nc 通道 = 各类分数;
///         兼容 anchor-major 布局。
/// NMS 为自定义旋转框 IoU 贪心抑制 (cv::dnn::NMSBoxes 只支持水平框)。
/// 单实例单线程使用。
class YoloObbDetector
{
public:
  /// 模型加载/空跑校验失败抛 std::runtime_error
  explicit YoloObbDetector(const YoloObbConfig & config);
  ~YoloObbDetector();

  /// 单帧检测 (BGR), 返回按类旋转 NMS 后的结果; 无目标返回空
  std::vector<YoloObbDetection> detect(const cv::Mat & bgr);


  /// CUDA EP 是否真正生效 (use_cuda=true 但注册失败时为 false, 已回退 CPU)
  bool cuda_active() const;
  /// CUDA EP 注册失败原因 (成功/未启用为空)
  std::string cuda_error() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;   // pimpl: 隔离 onnxruntime 头文件依赖
  YoloObbConfig config_;
};

/// 检测结果 -> 地面站叠加标注 JSON (schema 见 flylab_tools/overlay.py)。
/// 输出为按算法分组的消息 {"algo": algo, "items": [...]}, 每个目标两个图元:
///   polyline (4 角点闭合多边形, 颜色按类别 id 稳定生成) + text (类别 置信度)
/// 坐标为原图像素, 图像源须与地面站显示的相机/分辨率一致。
std::string to_annotations_json(
  const std::vector<YoloObbDetection> & results, const std::string & algo = "obb");

}  // namespace flylab_vision

#endif  // FLYLAB_VISION__YOLO_OBB_HPP_
