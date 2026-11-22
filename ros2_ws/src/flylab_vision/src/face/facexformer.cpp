// Copyright 2026 flylab
/// FaceXFormer 统一人脸分析器实现 —— onnxruntime 推理 + 6 任务解码。
///
/// 模型: facexformer.onnx (export_onnx.py 导出, 8 输出全量返回, 由调用方
///   按 task 选择解码 —— 与官方 inference.py 的 task 布尔筛选等价)
///   输入  [B,3,224,224] RGB, /255 + ImageNet mean/std
///   输出  [0] landmark [B,136] 归一化 [-1,1] 交错 x,y
///         [1] headpose [B,3] 弧度 (pitch/yaw/roll)
///         [2] attribute [B,40] logits (sigmoid >= 0.5 为开)
///         [3] visibility [B,29] logits (sigmoid >= 0.5 为可见)
///         [4] age [B,Na] logits (argmax -> 10 岁段 bin)
///         [5] gender [B,2] logits (argmax: 0 女 1 男)
///         [6] race [B,5] logits (argmax: White/Black/Asian/Indian/Others)
///         [7] segmentation [B,11,224,224] logits (softmax -> argmax)
/// 预处理对齐官方 inference.py/realtime_app.py: 双三次缩放 224 +
/// /255 + ImageNet 归一化 (cv::INTER_CUBIC 与 PIL BICUBIC 等价近似)。
#include "face/facexformer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <onnxruntime_cxx_api.h>
#include <stdexcept>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace flylab_vision
{

// CUDA EP 的专用注册接口 (与 scrfd_detector.cpp 同因: 字符串版
// AppendExecutionProvider("CUDAExecutionProvider") 不认 CUDA, 会抛
// "Unknown provider name", 必须走这个 C 接口)
extern "C" OrtStatus * OrtSessionOptionsAppendExecutionProvider_CUDA(
  OrtSessionOptions * options, int device_id);

namespace
{

constexpr int kInput = 224;                       // 网络输入边长 (正方形)
constexpr float kMean[3] = {0.485f, 0.456f, 0.406f};  // ImageNet mean
constexpr float kStd[3] = {0.229f, 0.224f, 0.225f};   // ImageNet std

struct TaskName {const char * name; int id;};
constexpr TaskName kTaskNames[] = {
  {"none", FACE_TASK_NONE},
  {"parsing", FACE_TASK_PARSING},
  {"landmarks", FACE_TASK_LANDMARKS},
  {"headpose", FACE_TASK_HEADPOSE},
  {"attributes", FACE_TASK_ATTRIBUTES},
  {"age_gender_race", FACE_TASK_AGE_GENDER_RACE},
  {"visibility", FACE_TASK_VISIBILITY},
};

size_t tensor_numel(const Ort::Value & v)
{
  const auto info = v.GetTensorTypeAndShapeInfo();
  size_t n = 1;
  for (const auto d : info.GetShape()) {
    n *= static_cast<size_t>(d);
  }
  return n;
}

}  // namespace

int facex_task_id(const char * name)
{
  if (name == nullptr) {
    return -1;
  }
  for (const auto & t : kTaskNames) {
    if (std::strcmp(t.name, name) == 0) {
      return t.id;
    }
  }
  return -1;
}

const char * facex_task_name(int id)
{
  for (const auto & t : kTaskNames) {
    if (t.id == id) {
      return t.name;
    }
  }
  return "unknown";
}

/// pimpl: onnxruntime 对象全部藏在实现里, 头文件不暴露其依赖
struct FaceXFormer::Impl
{
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "facexformer"};
  std::unique_ptr<Ort::Session> session;
  Ort::MemoryInfo mem{Ort::MemoryInfo::CreateCpu(
      OrtArenaAllocator, OrtMemTypeDefault)};
  std::string input_name;
  std::vector<std::string> output_names;
  std::vector<const char *> output_name_ptrs;   // Run 用, 构造时与上面同步填好
  cv::Mat blob;         // NCHW float 输入 (批量拼接, 帧间复用)
  int reg_batch = 1;    // 输入 batch 维: 1 = 静态 (逐张回退), -1 = 动态 (≤8 拼批)
  bool cuda_ok = false;
  std::string cuda_err; // CUDA EP 注册失败原因 (成功则为空)
};

FaceXFormer::FaceXFormer(const FaceXConfig & config)
: impl_(std::make_unique<Impl>()), config_(config)   // 顺序须与声明一致 (impl_ 在前)
{
  if (config_.model_path.empty()) {
    throw std::runtime_error("FaceXFormer: model_path 为空");
  }
  Ort::SessionOptions opts;
  opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  if (config_.use_cuda) {
    // 失败时保留状态串由上层日志暴露并自动回退 CPU (同 scrfd_detector 约定)
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
    if (n_out != 8) {
      throw std::runtime_error(
        "FaceXFormer: 输出数不是 8 (须为 export_onnx.py 导出的 facexformer.onnx)");
    }
    impl_->output_names.reserve(n_out);
    impl_->output_name_ptrs.reserve(n_out);
    for (size_t i = 0; i < n_out; ++i) {
      impl_->output_names.push_back(
        impl_->session->GetOutputNameAllocated(i, alloc).get());
      impl_->output_name_ptrs.push_back(impl_->output_names.back().c_str());
    }
    // 输入 batch 维: symbolic (≤0) = 动态拼批, 固定 1 = 静态逐张
    const auto in_shape = impl_->session->GetInputTypeInfo(0)
      .GetTensorTypeAndShapeInfo().GetShape();
    if (in_shape.size() == 4 && in_shape[0] <= 0) {
      impl_->reg_batch = -1;
    }
  }
}

FaceXFormer::~FaceXFormer() = default;

bool FaceXFormer::cuda_active() const
{
  return impl_->cuda_ok;
}

std::string FaceXFormer::cuda_error() const
{
  return impl_->cuda_err;
}

void FaceXFormer::run_batch(
  const std::vector<cv::Mat> & faces, int task,
  std::vector<FaceXResult> & results)
{
  if (task < FACE_TASK_PARSING || task > FACE_TASK_VISIBILITY) {
    throw std::invalid_argument("FaceXFormer::run_batch: task 越界");
  }
  results.assign(faces.size(), FaceXResult{});
  if (faces.empty()) {
    return;
  }

  const int plane = kInput * kInput;
  const size_t sample = 3 * static_cast<size_t>(plane);

  // ---------- 分块批量推理: 动态 batch 模型一次 ≤8 脸 (摊薄算子调度
  // 开销, 实测 batch=4 时 12ms/脸 vs 单张 30ms), 静态 batch=1 模型自动
  // 退化为逐张, 行为与旧版一致 ----------
  constexpr size_t kBatchCap = 8;
  const size_t batch_max = impl_->reg_batch < 0 ? kBatchCap : 1;
  cv::Mat resized;
  std::vector<cv::Mat> ch(3);
  for (size_t base = 0; base < faces.size(); base += batch_max) {
    const size_t n = std::min(batch_max, faces.size() - base);
    if (impl_->blob.total() != n * sample || impl_->blob.type() != CV_32F) {
      impl_->blob.create(1, static_cast<int>(n * sample), CV_32F);
    }
    float * blob_base = impl_->blob.ptr<float>();
    for (size_t k = 0; k < n; ++k) {
      // 预处理: 224 RGB /255 + ImageNet 归一化 -> NCHW (样本 k 起点)
      cv::resize(faces[base + k], resized, cv::Size(kInput, kInput),
        0, 0, cv::INTER_CUBIC);
      cv::split(resized, ch);   // BGR 三通道
      float * dst = blob_base + k * sample;
      for (int c = 0; c < 3; ++c) {
        cv::Mat cf;
        ch[c].convertTo(cf, CV_32F, 1.0 / 255.0);
        // BGR 通道序 -> RGB 平面序 (R=ch[2], G=ch[1], B=ch[0])
        const int src = 2 - c;
        cf = (cf - kMean[src]) / kStd[src];
        std::memcpy(dst + static_cast<size_t>(c) * plane,
          cf.ptr<float>(), sizeof(float) * plane);
      }
    }
    const std::array<int64_t, 4> in_shape{
      static_cast<int64_t>(n), 3, kInput, kInput};
    auto in = Ort::Value::CreateTensor<float>(
      impl_->mem, impl_->blob.ptr<float>(), impl_->blob.total(),
      in_shape.data(), in_shape.size());
    const char * in_names[] = {impl_->input_name.c_str()};
    auto outs = impl_->session->Run(
      Ort::RunOptions{nullptr}, in_names, &in, 1,
      impl_->output_name_ptrs.data(), impl_->output_name_ptrs.size());

    // ---------- 按 task 解码 (输出顺序见文件头注释, 逐样本偏移) ----------
    for (size_t k = 0; k < n; ++k) {
      FaceXResult & out = results[base + k];
      switch (task) {
        case FACE_TASK_PARSING: {
            // [B,11,224,224] 逐像素取 logits 最大通道 (softmax 单调, argmax 等价)
            const auto shape = outs[7].GetTensorTypeAndShapeInfo().GetShape();
            const int64_t ncls = shape.size() == 4 ? shape[1] : 11;
            const float * logits = outs[7].GetTensorData<float>() +
                                   k * static_cast<size_t>(ncls) * plane;
            out.seg_mask.create(kInput, kInput, CV_8UC1);
            for (int y = 0; y < kInput; ++y) {
              for (int x = 0; x < kInput; ++x) {
                const size_t px = static_cast<size_t>(y) * kInput + x;
                int best = 0;
                float best_v = -std::numeric_limits<float>::infinity();
                for (int64_t c = 0; c < ncls; ++c) {
                  const float v = logits[static_cast<size_t>(c) * plane + px];
                  if (v > best_v) {
                    best_v = v;
                    best = static_cast<int>(c);
                  }
                }
                out.seg_mask.at<uchar>(y, x) = static_cast<uchar>(best);
              }
            }
            break;
          }
        case FACE_TASK_LANDMARKS: {
            // [B,136] 归一化坐标交错 x,y ([B,68,2] 展平后同布局)
            const float * p = outs[0].GetTensorData<float>() + k * 136;
            for (size_t i = 0; i < 136; ++i) {
              out.landmark[i / 2][i % 2] = p[i];
            }
            break;
          }
        case FACE_TASK_HEADPOSE: {
            const float * p = outs[1].GetTensorData<float>() + k * 3;
            for (int i = 0; i < 3; ++i) {
              out.headpose[i] = p[i];
            }
            break;
          }
        case FACE_TASK_ATTRIBUTES: {
            const float * p = outs[2].GetTensorData<float>() + k * 40;
            for (int i = 0; i < 40; ++i) {
              out.attribute[i] = 1.f / (1.f + std::exp(-p[i])) >= 0.5f;
            }
            break;
          }
        case FACE_TASK_VISIBILITY: {
            const float * p = outs[3].GetTensorData<float>() + k * 29;
            for (int i = 0; i < 29; ++i) {
              out.visibility[i] = 1.f / (1.f + std::exp(-p[i])) >= 0.5f;
            }
            break;
          }
        case FACE_TASK_AGE_GENDER_RACE: {
            // age/gender/race 各自 logits argmax ([B,Na] 按样本偏移)
            auto argmax_at = [&](const Ort::Value & v, size_t b) {
                const auto sh = v.GetTensorTypeAndShapeInfo().GetShape();
                const size_t na = tensor_numel(v) /
                  std::max<size_t>(1, static_cast<size_t>(sh[0]));
                const float * p = v.GetTensorData<float>() + b * na;
                size_t best = 0;
                for (size_t i = 1; i < na; ++i) {
                  if (p[i] > p[best]) {
                    best = i;
                  }
                }
                return static_cast<int>(best);
              };
            out.age_bin = argmax_at(outs[4], k);
            out.gender = argmax_at(outs[5], k);
            out.race = argmax_at(outs[6], k);
            break;
          }
        default:
          break;
      }
    }
  }
}

}  // namespace flylab_vision
