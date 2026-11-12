// Copyright 2026 flylab
/// 单目标跟踪器实现 —— ReID 裁块嵌入 + 余弦相似度身份匹配。
///
/// 外观模型: models/track/yolo26s-reid.onnx (Ultralytics YOLO26s-reid,
/// MSMT17 行人重识别训练, 通用外观嵌入)
///   输入  1x3x448x448 (RGB, /255, 裁块直接缩放 —— ReID/分类预处理,
///         不做 letterbox)
///   输出  1x512, L2 归一化后即外观向量, 余弦相似度 = 点积
///
/// 为什么用 onnxruntime: 同 YoloDetector (系统 OpenCV 4.5.x DNN 解析不了
/// yolo26 attention 的动态 Split 节点), 库随包发布在 thirdparty/onnxruntime/。
#include "yolo/yolo_tracker.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

#include <onnxruntime_cxx_api.h>
#include <opencv2/imgproc.hpp>

namespace flylab_vision
{

// CUDA EP 的专用注册接口 (同 yolo_detector.cpp, 字符串版接口不认 CUDA)
extern "C" OrtStatus * OrtSessionOptionsAppendExecutionProvider_CUDA(
  OrtSessionOptions * options, int device_id);

float box_iou(const cv::Rect2f & a, const cv::Rect2f & b)
{
  const float inter_w =
    std::min(a.x + a.width, b.x + b.width) - std::max(a.x, b.x);
  const float inter_h =
    std::min(a.y + a.height, b.y + b.height) - std::max(a.y, b.y);
  if (inter_w <= 0.f || inter_h <= 0.f) {
    return 0.f;
  }
  const float inter = inter_w * inter_h;
  return inter / (a.area() + b.area() - inter);
}

/// pimpl: onnxruntime 对象与跟踪状态全部藏在实现里
struct YoloTracker::Impl
{
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "yolo_tracker"};
  std::unique_ptr<Ort::Session> session;
  Ort::MemoryInfo mem{Ort::MemoryInfo::CreateCpu(
      OrtArenaAllocator, OrtMemTypeDefault)};
  std::string input_name, output_name;
  int size = 448;         // 裁块输入边长 (构造时按 ReidConfig 定)
  cv::Mat canvas;         // 裁块缩放画布 (embed 每次复用)
  cv::Mat blob;           // NCHW float 输入 (448 下 ~2.4MB, 首次分配后重写)
  bool cuda_ok = false;
  std::string cuda_err;

  // 跟踪状态
  std::vector<float> target;   // 目标外观向量 (L2 归一化), 空 = 未跟踪
  cv::Rect2f box;              // 目标框 (原图像素, 最后已知位置)
  bool tracking = false;
};

YoloTracker::YoloTracker(const ReidConfig & reid_cfg, const TrackParams & params)
: impl_(std::make_unique<Impl>()), params_(params)
{
  if (reid_cfg.model_path.empty()) {
    throw std::runtime_error("YoloTracker: reid model_path 为空");
  }
  impl_->size = reid_cfg.input_size;
  Ort::SessionOptions opts;
  opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  if (reid_cfg.use_cuda) {
    OrtStatus * st = OrtSessionOptionsAppendExecutionProvider_CUDA(opts, 0);
    if (st != nullptr) {
      impl_->cuda_err = Ort::GetApi().GetErrorMessage(st);
      Ort::GetApi().ReleaseStatus(st);
    } else {
      impl_->cuda_ok = true;
    }
  }
  impl_->session = std::make_unique<Ort::Session>(
    impl_->env, reid_cfg.model_path.c_str(), opts);
  {
    Ort::AllocatorWithDefaultOptions alloc;
    impl_->input_name = impl_->session->GetInputNameAllocated(0, alloc).get();
    impl_->output_name =
      impl_->session->GetOutputNameAllocated(0, alloc).get();
  }
  // 空跑确认模型可用并确定嵌入维度 (失败在此抛, 不拖到首帧)
  cv::Mat probe(impl_->size, impl_->size, CV_8UC3, cv::Scalar(114, 114, 114));
  const std::vector<float> emb = embed_crop(probe, cv::Rect2f(0, 0, static_cast<float>(impl_->size), static_cast<float>(impl_->size)));
  if (emb.empty()) {
    throw std::runtime_error("YoloTracker: ReID 模型空跑失败");
  }
}

YoloTracker::~YoloTracker() = default;

bool YoloTracker::cuda_active() const
{
  return impl_->cuda_ok;
}

std::string YoloTracker::cuda_error() const
{
  return impl_->cuda_err;
}

/// 裁块 -> 画布 (直接缩放, ReID 预处理不做 letterbox) -> NCHW RGB /255
/// blob -> 推理 -> L2 归一化嵌入; 裁块越界/退化返回空
std::vector<float> YoloTracker::embed_crop(
  const cv::Mat & bgr, const cv::Rect2f & boxf) const
{
  cv::Rect2f clipped =
    boxf & cv::Rect2f(0, 0, static_cast<float>(bgr.cols), static_cast<float>(bgr.rows));
  cv::Rect box(
    cvRound(clipped.x), cvRound(clipped.y),
    cvRound(clipped.width), cvRound(clipped.height));
  box &= cv::Rect(0, 0, bgr.cols, bgr.rows);
  if (box.width < 2 || box.height < 2) {
    return {};
  }
  const int s = impl_->size;
  if (impl_->canvas.size() != cv::Size(s, s) || impl_->canvas.type() != CV_8UC3) {
    impl_->canvas.create(s, s, CV_8UC3);
  }
  cv::resize(bgr(box), impl_->canvas, cv::Size(s, s), 0, 0, cv::INTER_LINEAR);
  const size_t plane = static_cast<size_t>(s) * s;
  if (impl_->blob.total() != 3 * plane || impl_->blob.type() != CV_32F) {
    impl_->blob.create(1, static_cast<int>(3 * plane), CV_32F);
  }
  // 行级单次遍历手写平面化 (同 yolo_detector make_blob_into)
  float * base = impl_->blob.ptr<float>();
  for (int y = 0; y < s; ++y) {
    const uchar * row = impl_->canvas.ptr<uchar>(y);
    float * dr = base + static_cast<size_t>(y) * s;
    float * dg = dr + plane;
    float * db = dg + plane;
    for (int x = 0; x < s; ++x) {
      dr[x] = row[3 * x + 2] * (1.f / 255.f);   // R (BGR -> RGB)
      dg[x] = row[3 * x + 1] * (1.f / 255.f);   // G
      db[x] = row[3 * x + 0] * (1.f / 255.f);   // B
    }
  }
  const std::array<int64_t, 4> in_shape{1, 3, s, s};
  auto in = Ort::Value::CreateTensor<float>(
    impl_->mem, impl_->blob.ptr<float>(), impl_->blob.total(),
    in_shape.data(), in_shape.size());
  const char * in_names[] = {impl_->input_name.c_str()};
  const char * out_names[] = {impl_->output_name.c_str()};
  auto outs = impl_->session->Run(
    Ort::RunOptions{nullptr}, in_names, &in, 1, out_names, 1);
  const float * data = outs[0].GetTensorData<float>();
  const size_t dim = outs[0].GetTensorTypeAndShapeInfo().GetElementCount();
  std::vector<float> emb(data, data + dim);
  float norm = 0.f;
  for (const float v : emb) {
    norm += v * v;
  }
  norm = std::sqrt(norm);
  if (norm > 1e-6f) {
    for (float & v : emb) {
      v /= norm;
    }
  }
  return emb;
}

bool YoloTracker::set_target(const cv::Mat & bgr, const cv::Rect2f & box)
{
  const std::vector<float> emb = embed_crop(bgr, box);
  if (emb.empty()) {
    return false;
  }
  impl_->target = emb;
  impl_->box = box;
  impl_->tracking = true;
  return true;
}

void YoloTracker::reset()
{
  impl_->target.clear();
  impl_->tracking = false;
}

TrackState YoloTracker::update(
  const cv::Mat & bgr, const std::vector<YoloDetection> & dets)
{
  TrackState st;
  if (!impl_->tracking || bgr.empty()) {
    return st;
  }
  st.box = impl_->box;
  if (dets.empty()) {
    st.lost = true;
    return st;
  }
  // 候选筛选: 优先上一框邻域 (IoU 达门槛, 通常 1~3 个, 省裁块算力);
  // 邻域无候选 (目标快速移动/检测抖动) 回退全量, 按置信度截断
  std::vector<const YoloDetection *> cands;
  for (const auto & d : dets) {
    if (box_iou(impl_->box, d.box) >= params_.iou_gate) {
      cands.push_back(&d);
    }
  }
  if (cands.empty()) {
    std::vector<const YoloDetection *> all;
    for (const auto & d : dets) {
      all.push_back(&d);
    }
    std::sort(all.begin(), all.end(),
      [](const YoloDetection * a, const YoloDetection * b) {
        return a->conf > b->conf;
      });
    const size_t n = std::min(all.size(),
      static_cast<size_t>(std::max(1, params_.max_scan)));
    cands.assign(all.begin(), all.begin() + static_cast<long>(n));
  }
  // 余弦相似度最高者胜出 (向量已归一化, 相似度 = 点积)
  const YoloDetection * best = nullptr;
  float best_sim = -1.f;
  for (const auto * d : cands) {
    const std::vector<float> emb = embed_crop(bgr, d->box);
    if (emb.empty() || emb.size() != impl_->target.size()) {
      continue;
    }
    float sim = 0.f;
    for (size_t i = 0; i < emb.size(); ++i) {
      sim += emb[i] * impl_->target[i];
    }
    if (sim > best_sim) {
      best_sim = sim;
      best = d;
    }
  }
  if (best == nullptr || best_sim < params_.match_thres) {
    st.lost = true;
    st.sim = std::max(0.f, best_sim);
    return st;
  }
  impl_->box = best->box;
  st.has = true;
  st.lost = false;
  st.box = best->box;
  st.sim = best_sim;
  st.name = best->name;
  st.conf = best->conf;
  return st;
}

}  // namespace flylab_vision
