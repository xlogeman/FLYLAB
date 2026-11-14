// Copyright 2026 flylab
#ifndef FLYLAB_VISION__YOLO_SEMANTIC_HPP_
#define FLYLAB_VISION__YOLO_SEMANTIC_HPP_

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace flylab_vision
{

/// 语义分割器配置
struct YoloSemanticConfig
{
  std::string model_path;          // ONNX 模型路径
  int input_size = 0;              // 网络输入边长; 0 = 自动取模型固定输入尺寸
                                   // (yolo26s-sem 导出为 1024), 动态尺寸 (-1) 时必须显式指定
  int smooth_kernel = 5;           // 类别图中值滤波核 (奇数), 去单像素椒盐噪声;
                                   // 0/1 = 关闭。中值近似多数投票, 类别边界更整
  std::vector<int> class_filter;   // 只保留这些类别 id; 空 = 全部 19 类。
                                   // 被滤掉的像素在 class_map 中置 kVoid
  bool use_cuda = false;           // onnxruntime 无 CUDA EP 时自动回退 CPU
};

/// class_map 中的无效像素值 (无类别归属: 被过滤/输入为空)
constexpr uint8_t kSemanticVoid = 255;

/// 单帧语义分割结果
struct YoloSemanticResult
{
  cv::Mat class_map;          // CV_8UC1, 尺寸 = 输入帧, 像素值 = 类别 id
                              // (cityscapes 19 类), 无效像素 = kSemanticVoid
  std::vector<int> present;   // 本帧出现的类别 id, 按像素占比降序
  std::vector<float> present_share;   // 与 present 一一对应的像素占比 (%)
};

/// YOLO 语义分割器 —— 纯算法, 不含 ROS / 窗口显示。
/// 推理后端为 onnxruntime (选型原因见 yolo_detector.hpp)。
/// 模型: Ultralytics 语义分割 ONNX 标准导出 (models/semantic/yolo26s-sem.onnx,
/// Cityscapes 19 类, SemanticSegment 头)
///   输入   1x3xSxS (RGB, /255, letterbox), yolo26s-sem 固定 S=1024
///   输出   1xSxS uint8 逐像素类别图 (argmax 已烘焙进图内, 与输入画布同分辨率)
/// 预处理与 Ultralytics 一致: letterbox 正方形 (114 灰边) + RGB + /255;
/// 后处理: 裁掉 letterbox 无效区 -> 最近邻缩放回原帧尺寸, 与原图逐像素对齐。
/// 单实例单线程使用。
class YoloSemanticSegmenter
{
public:
  /// 模型加载/空跑校验失败抛 std::runtime_error
  explicit YoloSemanticSegmenter(const YoloSemanticConfig & config);
  ~YoloSemanticSegmenter();

  /// 单帧推理 (BGR), 返回与原帧同尺寸的类别图; 输入为空返回空结果
  YoloSemanticResult segment(const cv::Mat & bgr);


  /// 实际使用的输入边长 (config.input_size=0 时为模型固定尺寸)
  int input_size() const;

  /// CUDA EP 是否真正生效 (use_cuda=true 但注册失败时为 false, 已回退 CPU)
  bool cuda_active() const;
  /// CUDA EP 注册失败原因 (成功/未启用为空)
  std::string cuda_error() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;   // pimpl: 隔离 onnxruntime 头文件依赖
  YoloSemanticConfig config_;
};

/// cityscapes 19 类类别名 (下标 = 类别 id, 与 ONNX 元数据 names 一致)
const std::vector<std::string> & cityscapes_names();

/// 19 类配色表 (RGB, 下标 = 类别 id): 自定义清新风格, 类别图缩略图与
/// 图例色块共用 (原为 Cityscapes 标准配色, 因整体偏沉闷替换)
const std::vector<cv::Vec3b> & cityscapes_palette_rgb();

/// 语义分割结果 -> 地面站右栏"算法信息"面板数据 JSON (发 /flylab/aruco_info,
/// 与 ArUco 检测/深度估计同一话题, 面板按 src 分组):
///   {"src":"semantic","nclass":..,"w":..,"h":..,
///    "present":[["car",12.3],...],"img":"<base64 PNG>"|省略}
/// present 为本帧出现的类别 [名, 像素占比%, [R,G,B] 配色] (按占比降序,
/// 全量; 图例截断在面板侧), img 为类别图按 cityscapes 配色着色
/// (无效像素深灰) 缩到 img_width 宽后的 PNG base64 (平坦色块体积小且无
/// 伪影; img_width<=0 或输入为空时省略, 面板只显示文字字段)。
std::string to_semantic_info_json(
  const YoloSemanticResult & result, int frame_w, int frame_h, int img_width);

}  // namespace flylab_vision

#endif  // FLYLAB_VISION__YOLO_SEMANTIC_HPP_
