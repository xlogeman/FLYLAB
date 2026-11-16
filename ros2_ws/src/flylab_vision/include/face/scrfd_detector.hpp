// Copyright 2026 flylab
#ifndef FLYLAB_VISION__FACE__SCRFD_DETECTOR_HPP_
#define FLYLAB_VISION__FACE__SCRFD_DETECTOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace flylab_vision
{

/// 检测器配置
struct ScrfdConfig
{
  std::string model_path;      // ONNX 模型路径 (insightface scrfd_10g_bnkps 导出)
  int input_size = 640;        // 网络输入边长 (letterbox 到正方形)
  float conf_thres = 0.5f;     // 置信度阈值
  float iou_thres = 0.4f;      // NMS IoU 阈值
  bool use_cuda = false;       // onnxruntime 无 CUDA EP 时自动回退 CPU
};

/// 单张人脸的检测结果 (一帧可含多张)
struct ScrfdFace
{
  cv::Rect2f box;              // 原图像素坐标 (x, y 为左上角)
  float conf = 0.f;            // 置信度 0~1
  cv::Point2f kps[5];          // 5 关键点: 左眼/右眼/鼻尖/左嘴角/右嘴角
};

/// SCRFD-10G 人脸检测器 —— 纯算法, 不含 ROS / 窗口显示。
/// 推理后端为 onnxruntime (thirdparty/onnxruntime/), insightface 模型
/// 格式: 9 个输出 = 3 尺度 (stride 8/16/32) 的 scores [N,1] / 框距离
/// [N,4] / 5 关键点距离 [N,10], 每网格点 2 anchor, distance2bbox/kps
/// 解码 + letterbox 逆变换 + NMS。
/// 单实例单线程使用。
class ScrfdDetector
{
public:
  /// 模型加载/空跑校验失败抛 std::runtime_error
  explicit ScrfdDetector(const ScrfdConfig & config);
  ~ScrfdDetector();

  /// 单帧检测 (BGR), 返回按置信度 NMS 后的人脸; 无目标返回空
  std::vector<ScrfdFace> detect(const cv::Mat & bgr);

  /// CUDA EP 是否真正生效 (use_cuda=true 但注册失败时为 false, 已回退 CPU)
  bool cuda_active() const;
  /// CUDA EP 注册失败原因 (成功/未启用为空)
  std::string cuda_error() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;   // pimpl: 隔离 onnxruntime 头文件依赖
  ScrfdConfig config_;
};

/// 检测结果 -> 地面站叠加标注 JSON (schema 见 flylab_gcs/overlay.py)。
/// 输出为按算法分组的消息 {"algo": algo, "items": [...]}, 每张脸:
///   bbox (box=[x,y,w,h], label="face 置信度", 绿色) +
///   5 个关键点 point (黄色, 无 label)
/// 坐标为原图像素, 图像源须与地面站显示的相机/分辨率一致。
std::string to_annotations_json(
  const std::vector<ScrfdFace> & faces, const std::string & algo = "face");

}  // namespace flylab_vision

#endif  // FLYLAB_VISION__FACE__SCRFD_DETECTOR_HPP_
