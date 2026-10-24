// Copyright 2026 flylab
#ifndef FLYLAB_VISION__YOLO_DEPTH_HPP_
#define FLYLAB_VISION__YOLO_DEPTH_HPP_

#include <memory>
#include <string>

#include <opencv2/core.hpp>

namespace flylab_vision
{

/// 深度估计器配置
struct DepthConfig
{
  std::string model_path;      // ONNX 模型路径 (Ultralytics depth 标准导出)
  int input_size = 0;          // 网络输入边长; 0 = 自动取模型固定输入尺寸,
                               // 模型为动态尺寸 (-1) 时必须显式指定
  bool log_depth = false;      // 模型输出是否为裸 log 深度 (需 exp 还原)。
                               // Ultralytics depth 任务标准 ONNX 导出已在图内
                               // 完成 exp(logit) + 米制校准 (cal_a/cal_b),
                               // 输出直接是米, 保持 false; 仅自定义裸 log
                               // 导出才需要 true
  bool use_cuda = false;       // onnxruntime 无 CUDA EP 时自动回退 CPU
};

/// 单帧深度估计结果
struct DepthResult
{
  cv::Mat depth;               // CV_32F 深度图, 尺寸 = 输入帧 (米/相对单位)
  float dmin = 0.f;            // 本帧最小深度 (排除 letterbox 无效区)
  float dmax = 0.f;            // 本帧最大深度
};

/// YOLO 单目深度估计器 —— 纯算法, 不含 ROS / 窗口显示。
/// 推理后端为 onnxruntime (同 YoloDetector, 系统 OpenCV 4.5.x DNN 解析不了
/// yolo26 的 attention Split 节点)。
/// 模型: Ultralytics depth 任务 ONNX 导出 (yolo26s-depth.onnx)
///   输入  1x3xSxS (RGB, /255, letterbox 正方形)  —— S 由模型固定输入尺寸决定
///   输出  1x1xSxS 逐像素米制深度, 与输入画布同分辨率
/// 内部结构: 网络预测相对 log 深度场, 导出图内已含 Clip[-4,5] -> exp ->
/// 两参数米制校准 (d^cal_a * exp(cal_b), 训练验证集上拟合的全局尺度),
/// 故输出直接是米制绝对深度, 无需再 exp。
/// 预处理与 Ultralytics 一致: letterbox 正方形 (114 灰边) + RGB + /255;
/// 后处理: 裁掉 letterbox 无效区 -> 缩放回原帧尺寸, 与原图逐像素对齐。
/// 注: 绝对尺度按预训练验证集标定, 相机内参/场景分布差异会带来全局偏差;
/// 需要更高精度时用官方 model.calibrate() 重标定后重导出。
/// 单实例单线程使用。
class YoloDepthEstimator
{
public:
  /// 模型加载/空跑校验失败抛 std::runtime_error
  explicit YoloDepthEstimator(const DepthConfig & config);
  ~YoloDepthEstimator();

  /// 单帧推理 (BGR), 返回与原帧同尺寸的深度图; 输入为空返回空结果
  DepthResult estimate(const cv::Mat & bgr);


  /// 实际使用的输入边长 (config.input_size=0 时为模型固定尺寸)
  int input_size() const;

  /// CUDA EP 是否真正生效 (use_cuda=true 但注册失败时为 false, 已回退 CPU)
  bool cuda_active() const;
  /// CUDA EP 注册失败原因 (成功/未启用为空)
  std::string cuda_error() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;   // pimpl: 隔离 onnxruntime 头文件依赖
  DepthConfig config_;
};

/// 深度图 -> 地面站叠加标注 JSON (schema 见 flylab_tools/overlay.py)。
/// 输出为按算法分组的消息 {"algo": algo, "items": [...]}:
///   image 图元: turbo 伪彩深度图 (min-max 归一化) 经 JPEG+base64 编码,
///     以不透明小窗贴在画面右下角 (pip_scale 控制窗宽占原帧比例),
///     相机画面完全不被遮挡;
///   text 图元: 本帧深度范围 (min~max), 画面左下角。
/// jpeg_quality 为 JPEG 压缩质量 (1~100)。
std::string to_depth_annotations_json(
  const DepthResult & result, const std::string & algo = "yolo_depth",
  float pip_scale = 0.33f, int jpeg_quality = 70);

}  // namespace flylab_vision

#endif  // FLYLAB_VISION__YOLO_DEPTH_HPP_
