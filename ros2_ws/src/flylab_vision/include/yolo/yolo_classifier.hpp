// Copyright 2026 flylab
#ifndef FLYLAB_VISION__YOLO_CLASSIFIER_HPP_
#define FLYLAB_VISION__YOLO_CLASSIFIER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace flylab_vision
{

/// 分类器配置
struct ClsConfig
{
  std::string model_path;      // ONNX 模型路径 (Ultralytics cls 标准导出)
  int input_size = 0;          // 网络输入边长; 0 = 自动取模型固定输入尺寸,
                               // 模型为动态尺寸 (-1) 时必须显式指定
  int topk = 5;                // 每帧返回置信度最高的 k 类
  std::string labels_file;     // 外部标签文件 (每行一个); 空 = 用模型内嵌标签,
                               // 两者都取不到时退化为 "class_N" 占位名
  bool use_cuda = false;       // onnxruntime 无 CUDA EP 时自动回退 CPU
};

/// 单个分类预测
struct ClsPrediction
{
  int class_id = -1;           // 类别下标
  std::string name;            // 类别名 (优先取模型内嵌标签)
  float conf = 0.f;            // softmax 概率 0~1
};

/// YOLO 图像分类器 —— 纯算法, 不含 ROS / 窗口显示。
/// 推理后端为 onnxruntime (同 YoloDetector, 系统 OpenCV 4.5.x DNN 解析不了
/// yolo26 的 attention Split 节点)。
/// 模型: Ultralytics ONNX 标准导出 (输入 1x3xSxS, 输出 1xC 类别分数)。
/// 预处理与 Ultralytics 一致: letterbox 正方形 (114 灰边) + RGB + /255;
/// 后处理: top-k (yolo26-cls 导出已内建 softmax, 输出即概率; 纯线性导出
/// 则按数值特征自动补 softmax)。
/// 类别标签来源优先级: 模型内嵌 metadata "names" (Ultralytics 导出自带) >
/// 外部标签文件 (每行一个) > "class_N" 占位名。
/// 单实例单线程使用。
class YoloClassifier
{
public:
  /// 模型加载/空跑校验失败抛 std::runtime_error
  explicit YoloClassifier(const ClsConfig & config);
  ~YoloClassifier();

  /// 单帧分类 (BGR), 返回按置信度降序的 top-k 结果
  std::vector<ClsPrediction> classify(const cv::Mat & bgr);

  const ClsConfig & config() const {return config_;}

  /// 类别总数
  int num_classes() const;
  /// 实际使用的输入边长 (config.input_size=0 时为模型固定尺寸)
  int input_size() const;
  /// 实际使用的类别名表 (下标即类别 id)
  const std::vector<std::string> & labels() const;

  /// CUDA EP 是否真正生效 (use_cuda=true 但注册失败时为 false, 已回退 CPU)
  bool cuda_active() const;
  /// CUDA EP 注册失败原因 (成功/未启用为空)
  std::string cuda_error() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;   // pimpl: 隔离 onnxruntime 头文件依赖
  ClsConfig config_;
};

/// 分类结果 -> 地面站叠加标注 JSON (schema 见 flylab_tools/overlay.py)。
/// 输出为按算法分组的消息 {"algo": algo, "items": [...]}, top-k 逐行渲染为
/// 右上角自由文字 (type=text, 右对齐, 右边距 12px): 第 1 行 top-1 用类别
/// 稳定色, 其余灰白色。frame_width 为当前帧像素宽 (右对齐锚点需要);
/// <=0 时退化为左上角显示。
std::string to_cls_annotations_json(
  const std::vector<ClsPrediction> & preds,
  const std::string & algo = "yolo_cls",
  int frame_width = 0);

}  // namespace flylab_vision

#endif  // FLYLAB_VISION__YOLO_CLASSIFIER_HPP_
