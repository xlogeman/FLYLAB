// Copyright 2026 flylab
/// WiLoR 手部姿态管线实现 —— 手部检测 + 逐手 MANO 回归 + 关键点回原图。
///
/// 模型 (models/hand/):
///   detector.onnx   YOLO 手部检测, 输入 1x3x512x512 (RGB /255, letterbox),
///                   输出 1x(6+3nk)xN pose 布局: 前 4 通道 cx/cy/w/h
///                   (letterbox 坐标), 行 4:6 = 左/右手分数, 其余为关键点
///                   (本管线不用, 检测框即够)
///   wilor_fp16.onnx MANO 回归, 输入 1x3x256x256 (RGB, ImageNet 归一化),
///                   三路输出 kpts_2d (1x21x2) / pred_cam_t (1x3) /
///                   verts (1x778x3), 见下方 MANO 网格一段
///
/// patch 预处理 (wilor/datasets/utils.py 的 torch-free 移植):
///   检测框中心 + RESIZE_FACTOR 外扩 → 补 192:256 比例 → 仿射裁 256x256
///   (过大降采样抗锯齿) → 左手镜像成右手 → RGB / ImageNet 归一化
///
/// 关键点回原图: warpAffine 用的是"原图 → patch"前向仿射 M (函数内部
/// 求逆采样), 左手镜像在 warp 之后; 故 patch 坐标先做 x → 255-x 镜像
/// 复原, 再过 invertAffineTransform(M) 即得原图像素
///
/// MANO 网格 (verts 1x778x3) 拓扑内嵌于 include/wilor/mano_faces.hpp
/// (原 mano_faces.npy 已烘焙进源码): verts + pred_cam_t → 相机系 3D,
/// 过官方 perspective_projection (f' = 5000/256, 光心 0) 投影 patch
/// 归一化坐标, 与 kpts_2d 同基准, 同样经镜像/逆仿射回原图像素
#include "wilor/wilor.hpp"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cstring>
#include <iomanip>
#include <onnxruntime_cxx_api.h>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include "wilor/mano_faces.hpp"
#include <opencv2/dnn.hpp>       // cv::dnn::NMSBoxes (仅 NMS 工具)
#include <opencv2/imgcodecs.hpp> // 网格渲染 JPEG 编码
#include <opencv2/imgproc.hpp>

namespace flylab_vision
{

// CUDA EP 的专用注册接口 (同 yolo_detector.cpp: 字符串版注册不认 CUDA)
extern "C" OrtStatus * OrtSessionOptionsAppendExecutionProvider_CUDA(
  OrtSessionOptions * options, int device_id);

namespace
{

constexpr int kImageSize = 256;              // wilor 输入边长
constexpr float kResizeFactor = 2.0f;        // 检测框外扩系数 (官方默认)
constexpr int kDetSize = 512;                // 检测器输入边长 (模型元数据为准)
// 官方透视投影焦距 (相机系米 → patch 归一化坐标, 光心 0):
// perspective_projection(pred_keypoints_3d, pred_cam_t, 5000/IMAGE_SIZE),
// kpts_2d/verts 同一公式同一基准 (wilor/models/wilor.py:155)
constexpr float kFocalNorm = 5000.f / kImageSize;
// ImageNet 归一化常数 (RGB, 0~255 域)
constexpr float kMean255[3] = {123.675f, 116.28f, 103.53f};
constexpr float kStd255[3] = {58.395f, 57.12f, 57.375f};

// openpose-21 骨架链 (每链一条 polyline, 覆盖全部 19 条边):
// 拇指 0-1-2-3-4 / 食指 0-5-6-7-8 / 中指 5-9-10-11 /
// 无名指 9-13-14-15 / 小指 13-17-18-19-20 / 腕-小指 0-17
constexpr int kChains[][5] = {
  {0, 1, 2, 3, 4},
  {0, 5, 6, 7, 8},
  {5, 9, 10, 11, -1},
  {9, 13, 14, 15, -1},
  {13, 17, 18, 19, 20},
  {0, 17, -1, -1, -1}};

/// 检测器输出的原始手框 (det_stride 隔帧复用)
struct RawHandDet
{
  cv::Rect2f box;
  bool is_right;
  float conf;
};

std::string num(double v, int prec = 1)
{
  std::ostringstream os;
  os << std::fixed << std::setprecision(prec) << v;
  return os.str();
}

/// 补齐到目标宽高比 (192:256): 短边外扩, 返回 [w, h]
cv::Size2f expand_to_aspect_ratio(float w, float h, float tw, float th)
{
  if (h / w < th / tw) {
    return {w, std::max(w * th / tw, h)};
  }
  return {std::max(h * tw / th, w), h};
}

/// 仿射 patch 变换: 原图 (c_x, c_y) 为中心的 bbox_size 方块 → 256x256。
/// 返回前向矩阵 M (原图坐标 → patch 坐标), 与 cv2.warpAffine 语义一致
/// (函数内部自动求逆采样), 关键点回原图用其逆矩阵
cv::Mat gen_patch_transform(
  float c_x, float c_y, float bbox_size, float dst_w, float dst_h)
{
  const cv::Point2f src[3] = {
    {c_x, c_y},
    {c_x, c_y + bbox_size * 0.5f},
    {c_x + bbox_size * 0.5f, c_y}};
  const cv::Point2f dst[3] = {
    {dst_w * 0.5f, dst_h * 0.5f},
    {dst_w * 0.5f, dst_h * 0.5f + dst_h * 0.5f},
    {dst_w * 0.5f + dst_w * 0.5f, dst_h * 0.5f}};
  // getAffineTransform 返回 CV_64F, 统一转 CV_32F (下游按 float 读,
  // 且 invertAffineTransform 输出类型跟随输入)
  cv::Mat m = cv::getAffineTransform(src, dst);
  m.convertTo(m, CV_32F);
  return m;
}

}  // namespace

/// pimpl: onnxruntime 对象全部藏在实现里, 头文件不暴露其依赖
struct WilorPipeline::Impl
{
  // ERROR 级别: 屏蔽 fp16 混合精度图加载期的无害警告 (常量折叠缺 CPU
  // kernel / Memcpy 插入 / ScatterND 索引重复——MANO 蒙皮本就是覆盖写)
  Ort::Env env{ORT_LOGGING_LEVEL_ERROR, "wilor"};
  Ort::SessionOptions opts;   // 两会话共用 (同一份 CUDA 注册)
  std::unique_ptr<Ort::Session> det_session, wilor_session;
  Ort::MemoryInfo mem{Ort::MemoryInfo::CreateCpu(
      OrtArenaAllocator, OrtMemTypeDefault)};
  std::string det_in, det_out, wilor_in, wilor_out;
  std::string wilor_out_cam, wilor_out_verts;   // pred_cam_t / verts 输出名
  int det_size = kDetSize;    // 检测器输入边长 (空跑时按模型元数据修正)
  int reg_batch = 1;          // wilor 输入 batch 维: 1 = 静态 (逐手回退),
                              // -1 = 动态 (检出手拼批一次回归, 上限 8)

  // 每帧复用缓冲 (免重复分配)
  cv::Mat det_canvas;         // letterbox 画布 512x512
  cv::Mat det_blob;           // NCHW float (512 下 ~3.1MB)
  cv::Mat patch;              // 256x256 仿射裁块 (BGR)
  cv::Mat patch_blob;         // 1x3x256x256 float (~0.75MB)
  cv::Mat transform;          // patch 仿射 (原图 → patch)
  cv::Mat transform_inv;      // patch → 原图 (关键点回映射)

  bool cuda_ok = false;
  std::string cuda_err;

  // det_stride 隔帧复用: 最近一次检测框 + 帧计数
  std::vector<RawHandDet> cached_dets;
  uint64_t frame_idx = 0;

  // 检测器 letterbox 画布 → NCHW float blob (RGB, /255)
  void make_det_blob(const cv::Mat & canvas, cv::Mat & blob);
  // wilor patch (BGR) → NCHW float (RGB, ImageNet 归一化) 写入 dst
  // (dst 为 [n,3,256,256] 连续 blob 中第 k 个样本起点, 支持批量拼接)
  void fill_wilor_blob(const cv::Mat & patch, float * dst);
  // 两次推理封装 (输入/输出名取自构造时缓存, 只取需要的输出)
  Ort::Value run_det(cv::Mat & blob, const std::array<int64_t, 4> & shape);
  // wilor 一次取三路输出: [kpts_2d (1x21x2), pred_cam_t (1x3), verts (1x778x3)]
  std::vector<Ort::Value> run_wilor(
    cv::Mat & blob, const std::array<int64_t, 4> & shape);
};

void WilorPipeline::Impl::make_det_blob(const cv::Mat & canvas, cv::Mat & blob)
{
  const int size = canvas.rows;
  const int plane = size * size;
  if (blob.total() != static_cast<size_t>(3) * plane || blob.type() != CV_32F) {
    blob.create(1, static_cast<int>(3 * plane), CV_32F);
  }
  float * base = blob.ptr<float>();
  for (int y = 0; y < size; ++y) {
    const uchar * row = canvas.ptr<uchar>(y);
    float * dr = base + static_cast<size_t>(y) * size;
    float * dg = dr + plane;
    float * db = dg + plane;
    for (int x = 0; x < size; ++x) {
      dr[x] = row[3 * x + 2] * (1.f / 255.f);   // R (BGR -> RGB)
      dg[x] = row[3 * x + 1] * (1.f / 255.f);   // G
      db[x] = row[3 * x + 0] * (1.f / 255.f);   // B
    }
  }
}

void WilorPipeline::Impl::fill_wilor_blob(const cv::Mat & patch, float * base)
{
  const int plane = kImageSize * kImageSize;
  for (int y = 0; y < kImageSize; ++y) {
    const uchar * row = patch.ptr<uchar>(y);
    float * dr = base + static_cast<size_t>(y) * kImageSize;
    float * dg = dr + plane;
    float * db = dg + plane;
    for (int x = 0; x < kImageSize; ++x) {
      dr[x] = (row[3 * x + 2] - kMean255[0]) / kStd255[0];   // R
      dg[x] = (row[3 * x + 1] - kMean255[1]) / kStd255[1];   // G
      db[x] = (row[3 * x + 0] - kMean255[2]) / kStd255[2];   // B
    }
  }
}

Ort::Value WilorPipeline::Impl::run_det(
  cv::Mat & blob, const std::array<int64_t, 4> & shape)
{
  auto in = Ort::Value::CreateTensor<float>(
    mem, blob.ptr<float>(), blob.total(), shape.data(), shape.size());
  const char * in_names[] = {det_in.c_str()};
  const char * out_names[] = {det_out.c_str()};
  auto outs = det_session->Run(
    Ort::RunOptions{nullptr}, in_names, &in, 1, out_names, 1);
  return std::move(outs[0]);
}

std::vector<Ort::Value> WilorPipeline::Impl::run_wilor(
  cv::Mat & blob, const std::array<int64_t, 4> & shape)
{
  auto in = Ort::Value::CreateTensor<float>(
    mem, blob.ptr<float>(), blob.total(), shape.data(), shape.size());
  const char * in_names[] = {wilor_in.c_str()};
  const char * out_names[] = {
    wilor_out.c_str(), wilor_out_cam.c_str(), wilor_out_verts.c_str()};
  auto outs = wilor_session->Run(
    Ort::RunOptions{nullptr}, in_names, &in, 1, out_names, 3);
  return outs;
}

WilorPipeline::WilorPipeline(const WilorConfig & config)
: impl_(std::make_unique<Impl>()), config_(config)
{
  if (config_.detector_path.empty() || config_.wilor_path.empty()) {
    throw std::runtime_error("WilorPipeline: 模型路径为空");
  }
  impl_->opts.SetGraphOptimizationLevel(
    GraphOptimizationLevel::ORT_ENABLE_ALL);
  if (config_.use_cuda) {
    // 失败时保留原因串并自动回退 CPU (同 yolo 系列约定)
    OrtStatus * st = OrtSessionOptionsAppendExecutionProvider_CUDA(
      impl_->opts, 0);
    if (st != nullptr) {
      impl_->cuda_err = Ort::GetApi().GetErrorMessage(st);
      Ort::GetApi().ReleaseStatus(st);
    } else {
      impl_->cuda_ok = true;
    }
  }
  impl_->det_session = std::make_unique<Ort::Session>(
    impl_->env, config_.detector_path.c_str(), impl_->opts);
  impl_->wilor_session = std::make_unique<Ort::Session>(
    impl_->env, config_.wilor_path.c_str(), impl_->opts);
  {
    Ort::AllocatorWithDefaultOptions alloc;
    impl_->det_in = impl_->det_session->GetInputNameAllocated(0, alloc).get();
    impl_->det_out = impl_->det_session->GetOutputNameAllocated(0, alloc).get();
    impl_->wilor_in =
      impl_->wilor_session->GetInputNameAllocated(0, alloc).get();
    // 三路输出必须按名字取 (输出下标 0 是 pred_cam, 不用)
    bool found_kpts = false, found_cam = false, found_verts = false;
    for (size_t i = 0; i < impl_->wilor_session->GetOutputCount(); ++i) {
      auto name = impl_->wilor_session->GetOutputNameAllocated(i, alloc);
      if (std::strcmp(name.get(), "kpts_2d") == 0) {
        impl_->wilor_out = name.get();
        found_kpts = true;
      } else if (std::strcmp(name.get(), "pred_cam_t") == 0) {
        impl_->wilor_out_cam = name.get();
        found_cam = true;
      } else if (std::strcmp(name.get(), "verts") == 0) {
        impl_->wilor_out_verts = name.get();
        found_verts = true;
      }
    }
    if (!found_kpts || !found_cam || !found_verts) {
      throw std::runtime_error(
        "WilorPipeline: wilor 模型缺少输出 (需 kpts_2d/pred_cam_t/verts)");
    }
    // 检测器输入边长以模型元数据为准 (symbolic 维回退默认 512)
    const auto in_shape = impl_->det_session->GetInputTypeInfo(0)
      .GetTensorTypeAndShapeInfo().GetShape();
    if (in_shape.size() == 4 && in_shape[2] > 0 && in_shape[3] > 0) {
      impl_->det_size = static_cast<int>(in_shape[3]);
    }
    // wilor 输入 batch 维: symbolic (≤0) = 动态批量, 固定 1 = 静态逐手
    const auto win_shape = impl_->wilor_session->GetInputTypeInfo(0)
      .GetTensorTypeAndShapeInfo().GetShape();
    if (win_shape.size() == 4 && win_shape[0] <= 0) {
      impl_->reg_batch = -1;
    }
  }

  // 空跑校验 (模型坏在这里抛, 不拖到首帧)
  impl_->det_canvas.create(impl_->det_size, impl_->det_size, CV_8UC3);
  impl_->det_canvas.setTo(cv::Scalar(114, 114, 114));
  impl_->make_det_blob(impl_->det_canvas, impl_->det_blob);
  const std::array<int64_t, 4> det_shape{1, 3, impl_->det_size, impl_->det_size};
  impl_->run_det(impl_->det_blob, det_shape);

  impl_->patch_blob.create(1, 3 * kImageSize * kImageSize, CV_32F);
  impl_->patch_blob.setTo(0.f);
  const std::array<int64_t, 4> wilor_shape{1, 3, kImageSize, kImageSize};
  impl_->run_wilor(impl_->patch_blob, wilor_shape);
}

WilorPipeline::~WilorPipeline() = default;

bool WilorPipeline::cuda_active() const
{
  return impl_->cuda_ok;
}

std::string WilorPipeline::cuda_error() const
{
  return impl_->cuda_err;
}

bool WilorPipeline::batch_dynamic() const
{
  return impl_->reg_batch < 0;
}

std::vector<WilorHand> WilorPipeline::detect(const cv::Mat & bgr)
{
  if (bgr.empty()) {
    return {};
  }

  // ---------- 第一级: 手部检测 (det_stride>1 时隔帧复用手框) ----------
  ++impl_->frame_idx;
  std::vector<RawHandDet> raws;
  const bool det_now = config_.det_stride <= 1 ||
                       impl_->cached_dets.empty() ||
                       impl_->frame_idx % config_.det_stride == 1;
  if (det_now) {
    float scale = 1.f;
    int pad_x = 0, pad_y = 0;
    {
      const int size = impl_->det_size;
      const float r = std::min(size / (float)bgr.rows, size / (float)bgr.cols);
      const int nh = std::round(bgr.rows * r), nw = std::round(bgr.cols * r);
      pad_x = (size - nw) / 2;
      pad_y = (size - nh) / 2;
      scale = r;
      if (impl_->det_canvas.size() != cv::Size(size, size)) {
        impl_->det_canvas.create(size, size, CV_8UC3);
      }
      impl_->det_canvas.setTo(cv::Scalar(114, 114, 114));
      cv::resize(bgr, impl_->det_canvas(cv::Rect(pad_x, pad_y, nw, nh)),
                 cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);
      impl_->make_det_blob(impl_->det_canvas, impl_->det_blob);
    }
    const std::array<int64_t, 4> det_shape{1, 3, impl_->det_size, impl_->det_size};
    auto det_out = impl_->run_det(impl_->det_blob, det_shape);

    // 解码 pose 布局: [1, 6+3nk, N] (channel-major) 或转置 (anchor-major)。
    // 只用前 6 行: box(4) + 左右手分数(2)
    auto det_shape_out = det_out.GetTensorTypeAndShapeInfo().GetShape();
    std::vector<int64_t> s = det_shape_out;
    while (s.size() > 2 && s.front() == 1) {
      s.erase(s.begin());
    }
    while (s.size() > 2 && s.back() == 1) {
      s.pop_back();
    }
    if (s.size() != 2 || std::min(s[0], s[1]) < 6) {
      throw std::runtime_error("WilorPipeline: 检测器输出布局不支持");
    }
    const bool channel_major = s[0] < s[1];
    const int rows = static_cast<int>(channel_major ? s[0] : s[1]);
    const int anchors = static_cast<int>(channel_major ? s[1] : s[0]);
    const float * data = det_out.GetTensorData<float>();
    auto at = [&](int c, int a) {
        return channel_major ?
               data[static_cast<size_t>(c) * anchors + a] :
               data[static_cast<size_t>(a) * rows + c];
      };

    // 过阈值 + letterbox 坐标 → 原图; 类别 0 = 左手, 1 = 右手
    for (int cls = 0; cls < 2; ++cls) {
      std::vector<cv::Rect> rects;
      std::vector<float> confs;
      for (int a = 0; a < anchors; ++a) {
        const float score = at(4 + cls, a);
        if (score < config_.conf_thres) {
          continue;
        }
        const float cx = at(0, a), cy = at(1, a);
        const float w = at(2, a), h = at(3, a);
        cv::Rect2f box(
          (cx - w * 0.5f - pad_x) / scale, (cy - h * 0.5f - pad_y) / scale,
          w / scale, h / scale);
        box.x = std::max(0.f, box.x);
        box.y = std::max(0.f, box.y);
        box.width = std::min(box.width, bgr.cols - box.x);
        box.height = std::min(box.height, bgr.rows - box.y);
        if (box.width <= 1.f || box.height <= 1.f) {
          continue;
        }
        rects.emplace_back(
          cvRound(box.x), cvRound(box.y), cvRound(box.width), cvRound(box.height));
        confs.push_back(score);
      }
      if (rects.empty()) {
        continue;
      }
      std::vector<int> keep;
      cv::dnn::NMSBoxes(rects, confs, config_.conf_thres, config_.iou_thres, keep);
      for (const int k : keep) {
        raws.push_back({rects[k], cls == 1, confs[k]});
      }
    }
    impl_->cached_dets = raws;
  } else {
    raws = impl_->cached_dets;
  }
  if (raws.empty()) {
    return {};
  }

  // ---------- 第二级: 逐手 patch 预处理 + 分批 MANO 回归 ----------
  struct PatchData { cv::Mat patch; cv::Mat inv; };
  std::vector<PatchData> patches;
  patches.reserve(raws.size());
  std::vector<WilorHand> hands(raws.size());
  for (size_t i = 0; i < raws.size(); ++i) {
    const auto & raw = raws[i];
    const float x1 = raw.box.x, y1 = raw.box.y;
    const float x2 = raw.box.x + raw.box.width;
    const float y2 = raw.box.y + raw.box.height;
    const float c_x = (x1 + x2) / 2.f, c_y = (y1 + y2) / 2.f;
    // 官方预处理: 框外扩 RESIZE_FACTOR → 补 192:256 比例 → 取方形 bbox 尺寸
    const cv::Size2f wh = expand_to_aspect_ratio(
      kResizeFactor * (x2 - x1), kResizeFactor * (y2 - y1), 192.f, 256.f);
    const float bbox_size = std::max(wh.width, wh.height);

    // 过大降采样抗锯齿 (dsf>1.1), 原版用 skimage 高斯, cv 高斯等价替代
    cv::Mat src_img;
    const float dsf = bbox_size / kImageSize / 2.f;
    if (dsf > 1.1f) {
      const float sigma = (dsf - 1.f) / 2.f;
      const int ks = static_cast<int>(sigma * 4 + 1) | 1;
      cv::GaussianBlur(bgr, src_img, cv::Size(ks, ks), sigma);
    } else {
      src_img = bgr;
    }

    // 仿射裁 patch + 左手镜像 (MANO 只建模右手)
    impl_->transform = gen_patch_transform(
      c_x, c_y, bbox_size, kImageSize, kImageSize);
    cv::warpAffine(src_img, impl_->patch, impl_->transform,
                   cv::Size(kImageSize, kImageSize), cv::INTER_LINEAR,
                   cv::BORDER_CONSTANT, cv::Scalar(0));
    if (!raw.is_right) {
      cv::flip(impl_->patch, impl_->patch, 1);
    }
    cv::invertAffineTransform(impl_->transform, impl_->transform_inv);
    patches.push_back({impl_->patch.clone(), impl_->transform_inv.clone()});
  }

  // 分批回归: 动态 batch 模型一次推理处理 ≤8 手 (摊薄算子调度开销,
  // 实测 batch=4 时 19ms/手 vs 静态逐手 28ms/手); 静态 batch=1 模型
  // 自动退化为逐手推理, 行为与旧版完全一致
  constexpr size_t kBatchCap = 8;   // 单次批量上限 (延迟/显存有界)
  const size_t batch_max = impl_->reg_batch < 0 ? kBatchCap : 1;
  const size_t plane = 3 * kImageSize * kImageSize;
  for (size_t base = 0; base < raws.size(); base += batch_max) {
    const size_t n = std::min(batch_max, raws.size() - base);
    if (impl_->patch_blob.total() != n * plane ||
        impl_->patch_blob.type() != CV_32F) {
      impl_->patch_blob.create(1, static_cast<int>(n * plane), CV_32F);
    }
    float * blob_base = impl_->patch_blob.ptr<float>();
    for (size_t k = 0; k < n; ++k) {
      impl_->fill_wilor_blob(patches[base + k].patch, blob_base + k * plane);
    }
    const std::array<int64_t, 4> wilor_shape{
      static_cast<int64_t>(n), 3, kImageSize, kImageSize};
    auto outs = impl_->run_wilor(impl_->patch_blob, wilor_shape);

    for (size_t k = 0; k < n; ++k) {
      WilorHand & hand = hands[base + k];
      const auto & raw = raws[base + k];
      const float * inv = patches[base + k].inv.ptr<float>(0);
      const float * kp = outs[0].GetTensorData<float>() + k * 21 * 2;
      const float * cam = outs[1].GetTensorData<float>() + k * 3;
      const float * vt = outs[2].GetTensorData<float>() +
                         k * kManoVertCount * 3;

      // kpts_2d 是 (patch 像素 - 128)/256 的归一化坐标 (与 3D 关键点投
      // 影基准对齐) → 还原 patch 像素 → (左手先镜像回 x = 255-x) →
      // 逆仿射 → 原图
      hand.box = raw.box;
      hand.is_right = raw.is_right;
      hand.conf = raw.conf;
      hand.kpts2d.resize(21);
      for (int j = 0; j < 21; ++j) {
        float px = kp[2 * j] * kImageSize + kImageSize / 2.f;
        float py = kp[2 * j + 1] * kImageSize + kImageSize / 2.f;
        if (!raw.is_right) {
          px = static_cast<float>(kImageSize - 1) - px;
        }
        const float u = inv[0] * px + inv[1] * py + inv[2];
        const float v = inv[3] * px + inv[4] * py + inv[5];
        hand.kpts2d[j] = {u, v};
      }

      // MANO 网格: verts (778x3, 裁块空间) + pred_cam_t → 相机系 3D (米),
      // 官方 perspective_projection (f' = 5000/256, 光心 0) → patch 归一
      // 化坐标 → patch 像素 → (左手镜像) → 逆仿射 → 原图像素。深度 Z 仅
      // 用于着色/遮挡, 单位米
      hand.verts3d.resize(kManoVertCount);
      hand.verts2d.resize(kManoVertCount);
      for (int j = 0; j < kManoVertCount; ++j) {
        const float X = vt[3 * j] + cam[0];
        const float Y = vt[3 * j + 1] + cam[1];
        const float Z = std::max(vt[3 * j + 2] + cam[2], 1e-4f);
        float px = kFocalNorm * X / Z * kImageSize + kImageSize / 2.f;
        const float py = kFocalNorm * Y / Z * kImageSize + kImageSize / 2.f;
        float Xc = X;
        if (!raw.is_right) {
          px = static_cast<float>(kImageSize - 1) - px;
          Xc = -X;   // patch 镜像后 3D 相机系同样沿 x 翻转
        }
        hand.verts3d[j] = {Xc, Y, Z};
        hand.verts2d[j] = {
          inv[0] * px + inv[1] * py + inv[2],
          inv[3] * px + inv[4] * py + inv[5]};
      }
    }
  }
  return hands;
}

std::string to_annotations_json(
  const std::vector<WilorHand> & hands, const std::string & algo)
{
  std::ostringstream os;
  os << "{\"algo\":\"" << algo << "\",\"items\":[";
  bool first = true;
  for (const auto & hand : hands) {
    // 手框 (右手橙 / 左手天蓝, overlay schema 颜色为 RGB)
    const int r = hand.is_right ? 255 : 60;
    const int b = hand.is_right ? 0 : 255;
    if (!first) {
      os << ",";
    }
    first = false;
    os << "{\"type\":\"bbox\",\"box\":[" << num(hand.box.x) << ","
       << num(hand.box.y) << "," << num(hand.box.width) << ","
       << num(hand.box.height) << "],"
       << "\"label\":\"" << (hand.is_right ? "Right" : "Left") << " "
       << num(hand.conf, 2) << "\","
       << "\"color\":[" << r << ",170," << b << "]}";
    // 骨架: 每链一条 polyline (按 21 点顺序连线会串成锯齿)
    for (const auto & chain : kChains) {
      os << ",{\"type\":\"polyline\",\"points\":[";
      bool first_pt = true;
      for (const int idx : chain) {
        if (idx < 0) {
          break;
        }
        if (!first_pt) {
          os << ",";
        }
        first_pt = false;
        os << "[" << num(hand.kpts2d[idx].x) << ","
           << num(hand.kpts2d[idx].y) << "]";
      }
      os << "],\"close\":false,\"color\":[80,255,120]}";
    }
    // MANO 网格点云: 778 顶点散点叠加 (与手框同色, 整数像素省带宽)
    for (const auto & p : hand.verts2d) {
      os << ",{\"type\":\"point\",\"point\":[" << num(p.x, 0) << ","
         << num(p.y, 0) << "],\"color\":[" << r << ",170," << b << "]}";
    }
  }
  os << "]}";
  return os.str();
}

namespace
{

constexpr char kBase64Tab[] =
  "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

std::string base64(const unsigned char * data, size_t n)
{
  std::string out;
  out.reserve((n + 2) / 3 * 4);
  for (size_t i = 0; i < n; i += 3) {
    const unsigned b0 = data[i];
    const unsigned b1 = i + 1 < n ? data[i + 1] : 0;
    const unsigned b2 = i + 2 < n ? data[i + 2] : 0;
    const unsigned v = b0 << 16 | b1 << 8 | b2;
    out += kBase64Tab[v >> 18 & 63];
    out += kBase64Tab[v >> 12 & 63];
    out += i + 1 < n ? kBase64Tab[v >> 6 & 63] : '=';
    out += i + 2 < n ? kBase64Tab[v & 63] : '=';
  }
  return out;
}

}  // namespace

std::string to_mesh_jpeg(const WilorHand & hand, int size, int quality)
{
  if (hand.verts2d.size() != static_cast<size_t>(kManoVertCount) ||
      hand.verts3d.size() != static_cast<size_t>(kManoVertCount) ||
      size < 16) {
    return {};
  }

  // verts2d 包围盒等比缩放进画布 (8% 边距, y 翻转: 图像 y 向下)
  cv::Point2f lo(hand.verts2d[0].x, hand.verts2d[0].y);
  cv::Point2f hi = lo;
  float zlo = hand.verts3d[0].z, zhi = zlo;
  for (size_t j = 1; j < hand.verts2d.size(); ++j) {
    lo.x = std::min(lo.x, hand.verts2d[j].x);
    lo.y = std::min(lo.y, hand.verts2d[j].y);
    hi.x = std::max(hi.x, hand.verts2d[j].x);
    hi.y = std::max(hi.y, hand.verts2d[j].y);
    zlo = std::min(zlo, hand.verts3d[j].z);
    zhi = std::max(zhi, hand.verts3d[j].z);
  }
  const float span = std::max(hi.x - lo.x, hi.y - lo.y);
  if (span < 1e-3f) {
    return {};
  }
  const float scale = size * 0.84f / span;
  const float ox = (size - (hi.x - lo.x) * scale) * 0.5f - lo.x * scale;
  const float oy = (size - (hi.y - lo.y) * scale) * 0.5f - lo.y * scale;
  std::vector<cv::Point2f> pts(kManoVertCount);
  std::vector<float> depth(kManoVertCount);
  for (int j = 0; j < kManoVertCount; ++j) {
    pts[j] = {
      (hand.verts2d[j].x - lo.x) * scale + ox,
      size - ((hand.verts2d[j].y - lo.y) * scale + oy)};
    depth[j] = hand.verts3d[j].z;
  }
  const float zspan = std::max(zhi - zlo, 1e-6f);

  // z-buffer 平面着色光栅化 (3D 法向 + 头灯 + 深度微染色, 深色底)
  cv::Mat canvas(size, size, CV_8UC3, cv::Scalar(26, 28, 32));
  cv::Mat zbuf(size, size, CV_32F, cv::Scalar(FLT_MAX));
  const cv::Vec3f base(120.f, 165.f, 210.f);   // 肤色基调 (BGR)
  for (const auto & f : kManoFaces) {
    // 左手是右手模型镜像, 面片绕序翻转 (同官方 faces_left)
    const int a = f[0];
    const int b = hand.is_right ? f[1] : f[2];
    const int c = hand.is_right ? f[2] : f[1];
    const auto & p0 = pts[a], & p1 = pts[b], & p2 = pts[c];
    const float area =
      (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
    if (std::abs(area) < 1e-6f) {
      continue;
    }
    // 3D 面片法向 (相机系) → 头灯方向 (0,0,-1) 平面着色
    const auto & q0 = hand.verts3d[a], & q1 = hand.verts3d[b],
                & q2 = hand.verts3d[c];
    const cv::Point3f n = (q1 - q0).cross(q2 - q0);
    const float nn = static_cast<float>(cv::norm(n));
    const float shade = nn > 1e-9f ? std::abs(n.z) / nn : 0.f;
    const float d0 = depth[a], d1 = depth[b], d2 = depth[c];
    const int xmin = std::max(0, static_cast<int>(std::floor(
      std::min({p0.x, p1.x, p2.x}))));
    const int xmax = std::min(size - 1, static_cast<int>(std::ceil(
      std::max({p0.x, p1.x, p2.x}))));
    const int ymin = std::max(0, static_cast<int>(std::floor(
      std::min({p0.y, p1.y, p2.y}))));
    const int ymax = std::min(size - 1, static_cast<int>(std::ceil(
      std::max({p0.y, p1.y, p2.y}))));
    for (int py = ymin; py <= ymax; ++py) {
      float * zrow = zbuf.ptr<float>(py);
      cv::Vec3b * crow = canvas.ptr<cv::Vec3b>(py);
      for (int px = xmin; px <= xmax; ++px) {
        const float fx = px + 0.5f, fy = py + 0.5f;
        const float w0 = ((p1.x - fx) * (p2.y - fy) -
                          (p1.y - fy) * (p2.x - fx)) / area;
        const float w1 = ((p2.x - fx) * (p0.y - fy) -
                          (p2.y - fy) * (p0.x - fx)) / area;
        const float w2 = 1.f - w0 - w1;
        if (w0 < -1e-3f || w1 < -1e-3f || w2 < -1e-3f) {
          continue;
        }
        const float z = w0 * d0 + w1 * d1 + w2 * d2;
        if (z >= zrow[px]) {
          continue;
        }
        zrow[px] = z;
        // 近亮远暗 + 头灯亮度
        const float depth_tint = 1.f - 0.35f * (z - zlo) / zspan;
        const float k = std::max(0.25f, shade) * depth_tint;
        crow[px] = {static_cast<uchar>(base[0] * k),
                    static_cast<uchar>(base[1] * k),
                    static_cast<uchar>(base[2] * k)};
      }
    }
  }

  std::vector<uchar> buf;
  if (!cv::imencode(".jpg", canvas, buf,
                    {cv::IMWRITE_JPEG_QUALITY, quality})) {
    return {};
  }
  return base64(buf.data(), buf.size());
}

}  // namespace flylab_vision
