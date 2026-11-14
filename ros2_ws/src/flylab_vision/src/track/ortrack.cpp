// Copyright 2026 flylab
/// ORTrack 单目标追踪器实现 (移植 /root/ORTrack/tracking/webcam_demo.py
/// 的 OnnxTracker, 数值逻辑逐行对齐原库):
///   - 裁块: lib/train/data/processing_utils.py sample_target()
///     (crop_sz = ceil(sqrt(w*h)*factor), 中心对齐裁剪, 常量边补, resize)
///   - 解码: hann 窗加权 score map 取 argmax + size/offset 通道回归
///     (webcam_demo.py _cal_bbox)
///   - 映回: _map_box_back (相对搜索区 cxcywh -> 原图 xywh) + clip_box
///     (lib/utils/box_ops.py, margin=10)
/// 与 PyTorch 参考实现的唯一差异: 归一化前显式 BGR->RGB (webcam_demo
/// 直接喂 BGR 是上游的通道序疏漏, ImageNet mean/std 按 RGB 语义定义)。
#include "track/ortrack.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

namespace flylab_vision
{

// CUDA EP 的专用注册接口 (GPU 构建的 libonnxruntime 导出, 同
// yolo_detector.cpp; 字符串版 AppendExecutionProvider 不认 CUDA)
extern "C" OrtStatus * OrtSessionOptionsAppendExecutionProvider_CUDA(
  OrtSessionOptions * options, int device_id);

struct ORTracker::Impl
{
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "ortrack"};
  std::unique_ptr<Ort::Session> session;
  std::string in_template;       // 输入名按图序取 (template, search)
  std::string in_search;
  std::vector<std::string> out_names;
  std::string cuda_err;          // CUDA EP 注册失败原因 (成功/未启用为空)
  bool cuda_ok = false;

  // ---------- 模型几何常量 (experiments/ortrack/deit_tiny_patch16_224.yaml) ----------
  static constexpr int kTemplateSize = 128;   // 模板边长
  static constexpr int kSearchSize = 256;     // 搜索区边长
  static constexpr float kTemplateFactor = 2.0f;
  static constexpr float kSearchFactor = 4.0f;
  static constexpr int kStride = 16;          // backbone 下采样率 (patch 16)
  static constexpr int kFeatSz = kSearchSize / kStride;   // 16x16 响应图

  // 预生成的 2D Hann 窗 (centered, lib/test/utils/hann.py hann2d),
  // 加权压低响应图边缘的虚假峰
  std::vector<float> hann;

  // 追踪状态
  std::vector<float> z_blob;     // 模板张量 (固定整个追踪过程, 1x3x128x128)
  cv::Rect2f state;              // 上帧目标框 xywh (原图像素)
  bool has_state = false;
  float score = 0.f;             // 上帧峰值响应

  Impl() {hann = make_hann(kFeatSz);}

  static std::vector<float> make_hann(int sz)
  {
    // hann1d centered=True: w[i] = 0.5*(1-cos(2*pi*i/(sz+1))), i = 1..sz
    std::vector<float> w(sz);
    for (int i = 0; i < sz; ++i) {
      const double x = 2.0 * M_PI * (i + 1) / (sz + 1);
      w[i] = static_cast<float>(0.5 * (1.0 - std::cos(x)));
    }
    std::vector<float> out(static_cast<size_t>(sz) * sz);
    for (int y = 0; y < sz; ++y) {
      for (int x = 0; x < sz; ++x) {
        out[static_cast<size_t>(y) * sz + x] = w[y] * w[x];
      }
    }
    return out;
  }

  /// sample_target: 以 bb 为中心裁 factor 倍面积的方形块并缩放到 output_sz,
  /// 返回缩放系数 (crop_sz -> output_sz, 映回原图时用)
  static float sample_target_into(
    const cv::Mat & bgr, const cv::Rect2f & bb, float factor,
    int output_sz, cv::Mat & patch)
  {
    const float x = bb.x, y = bb.y, w = bb.width, h = bb.height;
    const int crop_sz = static_cast<int>(std::ceil(std::sqrt(w * h) * factor));
    const int x1 = static_cast<int>(std::lround(x + 0.5f * w - 0.5f * crop_sz));
    const int x2 = x1 + crop_sz;
    const int y1 = static_cast<int>(std::lround(y + 0.5f * h - 0.5f * crop_sz));
    const int y2 = y1 + crop_sz;

    // 越界部分用常量边补 (与原库一致, 黑边)
    const int x1_pad = std::max(0, -x1);
    const int x2_pad = std::max(x2 - bgr.cols + 1, 0);
    const int y1_pad = std::max(0, -y1);
    const int y2_pad = std::max(y2 - bgr.rows + 1, 0);

    const cv::Mat crop =
      cv::Mat(bgr, cv::Range(y1 + y1_pad, y2 - y2_pad),
      cv::Range(x1 + x1_pad, x2 - x2_pad)).clone();
    cv::Mat padded;
    cv::copyMakeBorder(
      crop, padded, y1_pad, y2_pad, x1_pad, x2_pad, cv::BORDER_CONSTANT);
    cv::resize(padded, patch, cv::Size(output_sz, output_sz));
    return static_cast<float>(output_sz) / crop_sz;
  }

  /// BGR -> RGB -> CHW -> ImageNet 归一化张量 (1x3xHxW)
  static void normalize_into(const cv::Mat & patch, std::vector<float> & blob)
  {
    cv::Mat rgb;
    cv::cvtColor(patch, rgb, cv::COLOR_BGR2RGB);
    rgb.convertTo(rgb, CV_32F, 1.0 / 255.0);
    std::vector<cv::Mat> ch;
    cv::split(rgb, ch);
    const float mean[3] = {0.485f, 0.456f, 0.406f};
    const float stdv[3] = {0.229f, 0.224f, 0.225f};
    const size_t plane = static_cast<size_t>(rgb.rows) * rgb.cols;
    blob.resize(3 * plane);
    for (int c = 0; c < 3; ++c) {
      const float * src = ch[c].ptr<float>();
      float * dst = blob.data() + c * plane;
      for (size_t i = 0; i < plane; ++i) {
        dst[i] = (src[i] - mean[c]) / stdv[c];
      }
    }
  }

  /// clip_box (lib/utils/box_ops.py): 框过小时强制最小边长, 防状态退化
  static cv::Rect2f clip_box(const cv::Rect2f & b, int h, int w, int margin)
  {
    float x1 = b.x, y1 = b.y;
    float x2 = b.x + b.width, y2 = b.y + b.height;
    x1 = std::min(std::max(0.f, x1), static_cast<float>(w - margin));
    x2 = std::min(std::max(static_cast<float>(margin), x2), static_cast<float>(w));
    y1 = std::min(std::max(0.f, y1), static_cast<float>(h - margin));
    y2 = std::min(std::max(static_cast<float>(margin), y2), static_cast<float>(h));
    return cv::Rect2f(
      x1, y1,
      std::max(static_cast<float>(margin), x2 - x1),
      std::max(static_cast<float>(margin), y2 - y1));
  }
};

ORTracker::ORTracker(const ORTrackConfig & cfg)
: impl_(std::make_unique<Impl>()), cfg_(cfg)
{
  Ort::SessionOptions opts;
  opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  if (cfg.use_cuda) {
    // 失败时保留原因串由上层日志暴露, 自动回退 CPU
    OrtStatus * st = OrtSessionOptionsAppendExecutionProvider_CUDA(opts, 0);
    if (st != nullptr) {
      impl_->cuda_err = Ort::GetApi().GetErrorMessage(st);
      Ort::GetApi().ReleaseStatus(st);
    } else {
      impl_->cuda_ok = true;
    }
  }
  impl_->session = std::make_unique<Ort::Session>(
    impl_->env, cfg.model_path.c_str(), opts);

  // 输入名/数量校验 (template + search 双输入)
  if (impl_->session->GetInputCount() != 2) {
    throw std::runtime_error(
            "ORTrack 模型输入数异常: " + std::to_string(impl_->session->GetInputCount()));
  }
  {
    Ort::AllocatorWithDefaultOptions alloc;
    impl_->in_template = impl_->session->GetInputNameAllocated(0, alloc).get();
    impl_->in_search = impl_->session->GetInputNameAllocated(1, alloc).get();
    const auto count = impl_->session->GetOutputCount();
    impl_->out_names.reserve(count);
    for (size_t i = 0; i < count; ++i) {
      impl_->out_names.emplace_back(impl_->session->GetOutputNameAllocated(i, alloc).get());
    }
  }
}

ORTracker::~ORTracker() = default;

bool ORTracker::initialize(const cv::Mat & bgr, const cv::Rect2f & box)
{
  if (bgr.empty() || box.width < 2.f || box.height < 2.f) {
    return false;
  }
  // 模板裁块: 输出尺寸 128, factor 2.0 (目标面积 4 倍上下文)
  cv::Mat patch;
  const float rf = Impl::sample_target_into(
    bgr, box, Impl::kTemplateFactor, Impl::kTemplateSize, patch);
  (void)rf;   // 模板无映回需求
  Impl::normalize_into(patch, impl_->z_blob);
  impl_->state = box;
  impl_->has_state = true;
  impl_->score = 0.f;
  return true;
}

void ORTracker::reset()
{
  impl_->z_blob.clear();
  impl_->z_blob.shrink_to_fit();
  impl_->has_state = false;
  impl_->score = 0.f;
}

cv::Rect2f ORTracker::track(const cv::Mat & bgr)
{
  if (!impl_->has_state || bgr.empty()) {
    return impl_->state;
  }
  const int feat_sz = Impl::kFeatSz;

  // 搜索区裁块 (以上帧状态为中心, factor 4.0 -> 256x256)
  cv::Mat patch;
  const float rf =
    Impl::sample_target_into(bgr, impl_->state, Impl::kSearchFactor, Impl::kSearchSize, patch);
  std::vector<float> x_blob;
  Impl::normalize_into(patch, x_blob);

  // 前向: 输出序 = 导出图序 (pred_boxes, score_map, size_map, offset_map)
  const std::int64_t z_shape[4] = {1, 3, Impl::kTemplateSize, Impl::kTemplateSize};
  const std::int64_t x_shape[4] = {1, 3, Impl::kSearchSize, Impl::kSearchSize};
  std::vector<Ort::Value> outputs;
  {
    const char * ins[2] = {impl_->in_template.c_str(), impl_->in_search.c_str()};
    const size_t n_out = impl_->out_names.size();
    std::vector<const char *> outs;
    outs.reserve(n_out);
    for (const auto & n : impl_->out_names) {
      outs.push_back(n.c_str());
    }
    Ort::Value z = Ort::Value::CreateTensor<float>(
      Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault),
      impl_->z_blob.data(), impl_->z_blob.size(), z_shape, 4);
    Ort::Value x = Ort::Value::CreateTensor<float>(
      Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault),
      x_blob.data(), x_blob.size(), x_shape, 4);
    Ort::Value inputs[] = {std::move(z), std::move(x)};
    outputs = impl_->session->Run(
      Ort::RunOptions{nullptr}, ins, inputs, 2, outs.data(), n_out);
  }
  if (outputs.size() != 4) {
    throw std::runtime_error("ORTrack 输出数异常");
  }
  const float * pb = outputs[0].GetTensorData<float>();      // (1,1,4) cxcywh 相对搜索块
  const float * sm = outputs[1].GetTensorData<float>();      // (1,1,16,16)
  const float * sz = outputs[2].GetTensorData<float>();      // (1,2,16,16) wh 回归
  const float * of = outputs[3].GetTensorData<float>();      // (1,2,16,16) 亚像素偏移
  (void)pb;   // pred_boxes 与下述解码等效, 统一走 hann 峰值路径

  // 解码: hann 加权取峰 + 该位置的 size/offset 回归 (_cal_bbox)
  float best = -1e30f;
  int best_idx = 0;
  for (int i = 0; i < feat_sz * feat_sz; ++i) {
    const float v = sm[i] * impl_->hann[static_cast<size_t>(i)];
    if (v > best) {
      best = v;
      best_idx = i;
    }
  }
  const int iy = best_idx / feat_sz;
  const int ix = best_idx % feat_sz;
  const size_t plane = static_cast<size_t>(feat_sz) * feat_sz;
  const float w = sz[0 * plane + iy * feat_sz + ix];
  const float h = sz[1 * plane + iy * feat_sz + ix];
  const float off_x = of[0 * plane + iy * feat_sz + ix];
  const float off_y = of[1 * plane + iy * feat_sz + ix];
  const float cx = (ix + off_x) / feat_sz;
  const float cy = (iy + off_y) / feat_sz;
  impl_->score = best;

  // 映回原图: 相对搜索块坐标 / resize_factor + 上帧中心平移 (_map_box_back)
  const float scale = Impl::kSearchSize / rf;
  const float cx_prev = impl_->state.x + 0.5f * impl_->state.width;
  const float cy_prev = impl_->state.y + 0.5f * impl_->state.height;
  const float half = 0.5f * Impl::kSearchSize / rf;
  const float cx_real = cx * scale + (cx_prev - half);
  const float cy_real = cy * scale + (cy_prev - half);
  impl_->state = Impl::clip_box(
    cv::Rect2f(cx_real - 0.5f * w * scale, cy_real - 0.5f * h * scale,
    w * scale, h * scale), bgr.rows, bgr.cols, 10);
  return impl_->state;
}

float ORTracker::last_score() const {return impl_->score;}
bool ORTracker::tracking() const {return impl_->has_state;}
bool ORTracker::cuda_active() const {return impl_->cuda_ok;}
std::string ORTracker::cuda_error() const {return impl_->cuda_err;}

}  // namespace flylab_vision
