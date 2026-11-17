// Copyright 2026 flylab
/// SCRFD 人脸检测器实现 —— onnxruntime 推理 + 解码 + NMS + 地面站标注 JSON。
///
/// 模型: insightface scrfd_10g_bnkps ONNX (models/face/scrfd_10g_bnkps.onnx)
///   输入  1x3x640x640 (RGB, (x-127.5)/128, letterbox)
///   输出  9 个张量, 3 尺度 stride 8/16/32 各 3 个:
///     [0..2] scores [N,1] / [3..5] 框距离 [N,4] / [6..8] 关键点距离 [N,10]
///   每个网格点 2 个 anchor, 距离解码: center + distance * stride。
/// 解码算法对齐 insightface 官方 scrfd python 实现。
#include "face/scrfd_detector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <iomanip>
#include <onnxruntime_cxx_api.h>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <opencv2/dnn.hpp>       // cv::dnn::NMSBoxes (仅 NMS 工具)
#include <opencv2/imgproc.hpp>

namespace flylab_vision
{

// CUDA EP 的专用注册接口 (与 yolo_detector.cpp 同因: 字符串版
// AppendExecutionProvider("CUDAExecutionProvider") 不认 CUDA, 会抛
// "Unknown provider name", 必须走这个 C 接口)
extern "C" OrtStatus * OrtSessionOptionsAppendExecutionProvider_CUDA(
  OrtSessionOptions * options, int device_id);

namespace
{

constexpr int kStrides[3] = {8, 16, 32};
constexpr int kNumAnchors = 2;   // 每个网格点 2 个 anchor (10G 系列约定)

/// 等比缩放 + 黑边填充到 size x size (SCRFD 训练约定), 带回缩放比 / pad
void letterbox_into(
  const cv::Mat & bgr, cv::Mat & canvas, int size,
  float & scale, int & pad_x, int & pad_y)
{
  const float r = std::min(size / (float)bgr.rows, size / (float)bgr.cols);
  const int nh = std::round(bgr.rows * r), nw = std::round(bgr.cols * r);
  pad_x = (size - nw) / 2;
  pad_y = (size - nh) / 2;
  scale = r;
  if (canvas.size() != cv::Size(size, size) || canvas.type() != CV_8UC3) {
    canvas.create(size, size, CV_8UC3);
  }
  canvas.setTo(cv::Scalar(0, 0, 0));
  cv::resize(bgr, canvas(cv::Rect(pad_x, pad_y, nw, nh)),
             cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);
}

/// 画布 -> NCHW float blob (RGB, (x - 127.5) / 128, SCRFD 归一化约定)。
/// 复用缓冲版: blob 由调用方持有 (640 下 ~4.9MB, 首次分配后每帧只重写)
void make_blob_into(const cv::Mat & canvas, cv::Mat & blob)
{
  const int size = canvas.rows;
  const int plane = size * size;
  if (blob.total() != static_cast<size_t>(3) * plane || blob.type() != CV_32F) {
    blob.create(1, 3 * plane, CV_32F);
  }
  constexpr float kNorm = 1.f / 128.f;
  float * base = blob.ptr<float>();
  for (int y = 0; y < size; ++y) {
    const uchar * row = canvas.ptr<uchar>(y);
    float * dr = base + static_cast<size_t>(y) * size;
    float * dg = dr + plane;
    float * db = dg + plane;
    for (int x = 0; x < size; ++x) {
      dr[x] = (row[3 * x + 2] - 127.5f) * kNorm;   // R (BGR -> RGB)
      dg[x] = (row[3 * x + 1] - 127.5f) * kNorm;   // G
      db[x] = (row[3 * x + 0] - 127.5f) * kNorm;   // B
    }
  }
}

/// anchor 中心 + 距离预测 -> 边界框 (x0, y0, x1, y1)。
/// 距离预测以"格数"为单位, 须乘 stride 换算成像素 (对齐 scrfd.py 的
/// distance2bbox(centers, dist * stride))
void distance2bbox(
  const cv::Point2f & center, const float * d, float stride,
  float & x1, float & y1, float & x2, float & y2)
{
  x1 = center.x - d[0] * stride;
  y1 = center.y - d[1] * stride;
  x2 = center.x + d[2] * stride;
  y2 = center.y + d[3] * stride;
}

/// anchor 中心 + 距离预测 -> 5 关键点 (交错 dx, dy 对), 距离同样乘 stride
void distance2kps(
  const cv::Point2f & center, const float * d, float stride,
  cv::Point2f * kps)
{
  for (int i = 0; i < 5; ++i) {
    kps[i] = {center.x + d[2 * i] * stride, center.y + d[2 * i + 1] * stride};
  }
}

std::string num(double v, int prec = 1)
{
  std::ostringstream os;
  os << std::fixed << std::setprecision(prec) << v;
  return os.str();
}

}  // namespace

/// pimpl: onnxruntime 对象全部藏在实现里, 头文件不暴露其依赖
struct ScrfdDetector::Impl
{
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "scrfd_detector"};
  std::unique_ptr<Ort::Session> session;
  Ort::MemoryInfo mem{Ort::MemoryInfo::CreateCpu(
      OrtArenaAllocator, OrtMemTypeDefault)};
  std::string input_name;
  std::vector<std::string> output_names;
  std::vector<const char *> output_name_ptrs;   // Run 用, 构造时与上面同步填好
  cv::Mat canvas;         // letterbox 画布 (detect 每帧复用, 免重复分配)
  cv::Mat blob;           // NCHW float 输入 (640 下 ~4.9MB, 同上)
  bool cuda_ok = false;
  std::string cuda_err;   // CUDA EP 注册失败原因 (成功则为空)

  /// 指定尺度的网格中心 (带 kNumAnchors 份拷贝), 首次计算后缓存
  const std::vector<cv::Point2f> & centers(int idx)
  {
    auto & cache = scale_centers[idx];
    if (!cache.empty()) {
      return cache;
    }
    const int stride = kStrides[idx];
    const int h = canvas.rows / stride;
    const int w = canvas.cols / stride;
    cache.reserve(static_cast<size_t>(h) * w * kNumAnchors);
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        for (int a = 0; a < kNumAnchors; ++a) {
          cache.push_back(
            {static_cast<float>(x * stride), static_cast<float>(y * stride)});
        }
      }
    }
    return cache;
  }

private:
  // 各尺度网格中心缓存 (下标 = kStrides 下标; 首帧按画布尺寸生成)
  std::vector<cv::Point2f> scale_centers[3];
};

ScrfdDetector::ScrfdDetector(const ScrfdConfig & config)
: impl_(std::make_unique<Impl>()), config_(config)   // 顺序须与声明一致 (impl_ 在前)
{
  if (config_.model_path.empty()) {
    throw std::runtime_error("ScrfdDetector: model_path 为空");
  }
  Ort::SessionOptions opts;
  opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  if (config_.use_cuda) {
    // 失败 (无显卡 / providers 插件加载不了 / 驱动缺失) 时保留状态串,
    // 由上层日志暴露并自动回退 CPU; CUDA 不支持的节点 ORT 自动落回 CPU
    OrtStatus * st = OrtSessionOptionsAppendExecutionProvider_CUDA(opts, 0);
    if (st != nullptr) {
      impl_->cuda_err = Ort::GetApi().GetErrorMessage(st);
      Ort::GetApi().ReleaseStatus(st);
    } else {
      impl_->cuda_ok = true;
    }
  }
  impl_->session = std::make_unique<Ort::Session>(
    impl_->env, config_.model_path.c_str(), opts);
  {
    Ort::AllocatorWithDefaultOptions alloc;
    impl_->input_name = impl_->session->GetInputNameAllocated(0, alloc).get();
    const size_t n_out = impl_->session->GetOutputCount();
    if (n_out != 9) {
      throw std::runtime_error(
        "ScrfdDetector: 输出数不是 9 (须为 scrfd_10g_bnkps 导出)");
    }
    impl_->output_names.reserve(n_out);
    impl_->output_name_ptrs.reserve(n_out);
    for (size_t i = 0; i < n_out; ++i) {
      impl_->output_names.push_back(
        impl_->session->GetOutputNameAllocated(i, alloc).get());
      impl_->output_name_ptrs.push_back(impl_->output_names.back().c_str());
    }
  }
}

ScrfdDetector::~ScrfdDetector() = default;

bool ScrfdDetector::cuda_active() const
{
  return impl_->cuda_ok;
}

std::string ScrfdDetector::cuda_error() const
{
  return impl_->cuda_err;
}

std::vector<ScrfdFace> ScrfdDetector::detect(const cv::Mat & bgr)
{
  if (bgr.empty()) {
    return {};
  }
  float scale = 1.f;
  int pad_x = 0, pad_y = 0;
  letterbox_into(bgr, impl_->canvas, config_.input_size, scale, pad_x, pad_y);
  make_blob_into(impl_->canvas, impl_->blob);
  const std::array<int64_t, 4> in_shape{
    1, 3, config_.input_size, config_.input_size};
  auto in = Ort::Value::CreateTensor<float>(
    impl_->mem, impl_->blob.ptr<float>(), impl_->blob.total(),
    in_shape.data(), in_shape.size());
  const char * in_names[] = {impl_->input_name.c_str()};
  auto outs = impl_->session->Run(
    Ort::RunOptions{nullptr}, in_names, &in, 1,
    impl_->output_name_ptrs.data(), impl_->output_name_ptrs.size());

  // 逐尺度解码: score / 框距离 / 关键点距离, 过阈值后逆 letterbox 变换
  std::vector<cv::Rect2f> boxes;
  std::vector<float> confs;
  std::vector<std::array<cv::Point2f, 5>> all_kps;
  for (int s = 0; s < 3; ++s) {
    const float * scores = outs[s].GetTensorData<float>();
    const float * dists = outs[3 + s].GetTensorData<float>();
    const float * kp_dists = outs[6 + s].GetTensorData<float>();
    const auto score_shape = outs[s].GetTensorTypeAndShapeInfo().GetShape();
    const int n = static_cast<int>(
      std::accumulate(score_shape.begin(), score_shape.end(), 1,
        std::multiplies<int64_t>()));
    const auto & centers = impl_->centers(s);
    const float stride = static_cast<float>(kStrides[s]);
    for (int i = 0; i < n; ++i) {
      const float score = scores[i];
      if (score < config_.conf_thres) {
        continue;
      }
      const cv::Point2f & c = centers[static_cast<size_t>(i)];
      const float * d = dists + static_cast<size_t>(i) * 4;
      float x1, y1, x2, y2;
      distance2bbox(c, d, stride, x1, y1, x2, y2);
      // letterbox 逆变换 -> 原图像素, 裁回画幅
      cv::Rect2f box(
        (x1 - pad_x) / scale, (y1 - pad_y) / scale,
        (x2 - x1) / scale, (y2 - y1) / scale);
      box.x = std::max(0.f, box.x);
      box.y = std::max(0.f, box.y);
      box.width = std::min(box.width, bgr.cols - box.x);
      box.height = std::min(box.height, bgr.rows - box.y);
      if (box.width <= 1.f || box.height <= 1.f) {
        continue;
      }
      std::array<cv::Point2f, 5> kps;
      distance2kps(
        c, kp_dists + static_cast<size_t>(i) * 10, stride, kps.data());
      for (auto & p : kps) {
        p.x = (p.x - pad_x) / scale;
        p.y = (p.y - pad_y) / scale;
      }
      boxes.push_back(box);
      confs.push_back(score);
      all_kps.push_back(kps);
    }
  }
  if (boxes.empty()) {
    return {};
  }

  // NMS 后取回关键点
  std::vector<cv::Rect> rects;
  rects.reserve(boxes.size());
  for (const auto & b : boxes) {
    rects.emplace_back(
      cvRound(b.x), cvRound(b.y), cvRound(b.width), cvRound(b.height));
  }
  std::vector<int> keep;
  cv::dnn::NMSBoxes(rects, confs, config_.conf_thres, config_.iou_thres, keep);

  std::vector<ScrfdFace> results;
  results.reserve(keep.size());
  for (const int k : keep) {
    ScrfdFace f;
    f.box = boxes[static_cast<size_t>(k)];
    f.conf = confs[static_cast<size_t>(k)];
    const auto & kps = all_kps[static_cast<size_t>(k)];
    std::copy(kps.begin(), kps.end(), f.kps);
    results.push_back(f);
  }
  return results;
}

void face_items_json(std::ostream & os, const std::vector<ScrfdFace> & faces)
{
  for (size_t i = 0; i < faces.size(); ++i) {
    const auto & f = faces[i];
    if (i > 0) {
      os << ",";
    }
    // 框 = 绿色 bbox, 关键点 = 黄色 point (标注 schema 见 flylab_gcs/overlay.py)
    os << "{\"type\":\"bbox\",\"box\":[" << num(f.box.x) << ","
       << num(f.box.y) << "," << num(f.box.width) << ","
       << num(f.box.height) << "],"
       << "\"label\":\"face " << num(f.conf, 2) << "\","
       << "\"color\":[0,255,0]}";
    for (const auto & p : f.kps) {
      os << ",{\"type\":\"point\",\"point\":[" << num(p.x) << ","
         << num(p.y) << "],"
         << "\"color\":[0,255,255]}";
    }
  }
}

std::string to_annotations_json(
  const std::vector<ScrfdFace> & faces, const std::string & algo)
{
  std::ostringstream os;
  os << "{\"algo\":\"" << algo << "\",\"items\":[";
  face_items_json(os, faces);
  os << "]}";
  return os.str();
}

}  // namespace flylab_vision
