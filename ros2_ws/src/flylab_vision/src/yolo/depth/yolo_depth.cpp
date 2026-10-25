// Copyright 2026 flylab
/// YOLO 单目深度估计器实现 —— onnxruntime 推理 + 伪彩编码 + 地面站标注 JSON。
///
/// 模型: Ultralytics depth 任务 ONNX 导出 (models/depth/yolo26s-depth.onnx)
///   输入  1x3xSxS (RGB, /255, letterbox 正方形)  —— S 由模型固定输入尺寸决定
///   输出  1x1xSxS 逐像素 (log-)深度, 与输入画布同分辨率
///
/// 与检测/分类的差别: 输出是稠密图像而非稀疏量, 处理链是
/// 裁 letterbox 无效区 -> (裸 log 导出时才 exp) -> 缩放回原帧
/// (逐像素对齐原图); 展示走 min-max 归一化 + turbo 伪彩 + JPEG(base64)
/// 经 image 图元画中画叠加。
#include "yolo/yolo_depth.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <onnxruntime_cxx_api.h>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace flylab_vision
{

// 同 YoloDetector: GPU 构建的 libonnxruntime 导出该 C 接口, 字符串版
// AppendExecutionProvider("CUDAExecutionProvider") 不认 CUDA 会抛异常
extern "C" OrtStatus * OrtSessionOptionsAppendExecutionProvider_CUDA(
  OrtSessionOptions * options, int device_id);

namespace
{

/// 等比缩放 + 114 灰边填充到 size x size, 返回画布并带回缩放比 / pad
/// (与 Ultralytics LetterBox 一致, 同 YoloDetector 预处理)
cv::Mat letterbox(
  const cv::Mat & bgr, int size, float & scale, int & pad_x, int & pad_y)
{
  const float r = std::min(size / (float)bgr.rows, size / (float)bgr.cols);
  const int nh = std::round(bgr.rows * r), nw = std::round(bgr.cols * r);
  cv::Mat resized;
  cv::resize(bgr, resized, cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);
  pad_x = (size - nw) / 2;
  pad_y = (size - nh) / 2;
  cv::Mat canvas(size, size, CV_8UC3, cv::Scalar(114, 114, 114));
  resized.copyTo(canvas(cv::Rect(pad_x, pad_y, nw, nh)));
  scale = r;
  return canvas;
}

/// letterbox 画布 -> NCHW float blob (RGB, /255), 行级单次遍历平面化
cv::Mat make_blob(const cv::Mat & canvas)
{
  const int size = canvas.rows;
  const int plane = size * size;
  cv::Mat blob(1, 3 * plane, CV_32F);
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
  return blob;
}

/// base64 编码 (RFC 4648, 无换行) —— 纯算法库不引 OpenSSL 等额外依赖
std::string base64_encode(const unsigned char * data, size_t len)
{
  static const char * table =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve((len + 2) / 3 * 4);
  size_t i = 0;
  for (; i + 3 <= len; i += 3) {
    const uint32_t n = (data[i] << 16) | (data[i + 1] << 8) | data[i + 2];
    out += table[(n >> 18) & 63];
    out += table[(n >> 12) & 63];
    out += table[(n >> 6) & 63];
    out += table[n & 63];
  }
  if (i + 1 == len) {
    const uint32_t n = data[i] << 16;
    out += table[(n >> 18) & 63];
    out += table[(n >> 12) & 63];
    out += "==";
  } else if (i + 2 == len) {
    const uint32_t n = (data[i] << 16) | (data[i + 1] << 8);
    out += table[(n >> 18) & 63];
    out += table[(n >> 12) & 63];
    out += table[(n >> 6) & 63];
    out += "=";
  }
  return out;
}

std::string num(double v, int prec = 2)
{
  std::ostringstream os;
  os << std::fixed << std::setprecision(prec) << v;
  return os.str();
}

}  // namespace

/// pimpl: onnxruntime 对象全部藏在实现里, 头文件不暴露其依赖
struct YoloDepthEstimator::Impl
{
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "yolo_depth"};
  std::unique_ptr<Ort::Session> session;
  Ort::MemoryInfo mem{Ort::MemoryInfo::CreateCpu(
      OrtArenaAllocator, OrtMemTypeDefault)};
  std::string input_name, output_name;
  int in_size = 0;          // 实际输入边长
  bool cuda_ok = false;
  std::string cuda_err;     // CUDA EP 注册失败原因 (成功则为空)
};

YoloDepthEstimator::YoloDepthEstimator(const DepthConfig & config)
: impl_(std::make_unique<Impl>()), config_(config)
{
  if (config_.model_path.empty()) {
    throw std::runtime_error("YoloDepthEstimator: model_path 为空");
  }
  Ort::SessionOptions opts;
  opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  if (config_.use_cuda) {
    // 失败时保留状态串, 由上层日志暴露并自动回退 CPU
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
    impl_->output_name =
      impl_->session->GetOutputNameAllocated(0, alloc).get();
  }

  // 输入尺寸: 显式参数优先; 否则取模型固定输入尺寸 (动态尺寸 -1 时报错)
  {
    auto shape = impl_->session->GetInputTypeInfo(0)
                 .GetTensorTypeAndShapeInfo().GetShape();
    int fixed = 0;
    if (shape.size() >= 4 && shape[2] > 0 && shape[2] == shape[3]) {
      fixed = static_cast<int>(shape[2]);
    }
    if (config_.input_size > 0) {
      impl_->in_size = config_.input_size;
    } else if (fixed > 0) {
      impl_->in_size = fixed;
      config_.input_size = fixed;   // 回写, config() 反映实际值
    } else {
      throw std::runtime_error(
              "YoloDepthEstimator: 模型输入为动态尺寸, 须显式指定 input_size");
    }
  }

  // 空跑一次确认模型可用且输出为 SxS 单通道深度图
  // (加载失败在这里抛, 不拖到首帧)
  cv::Mat probe(
    impl_->in_size, impl_->in_size, CV_8UC3, cv::Scalar(114, 114, 114));
  cv::Mat blob = make_blob(probe);
  const std::array<int64_t, 4> in_shape{
    1, 3, impl_->in_size, impl_->in_size};
  auto in = Ort::Value::CreateTensor<float>(
    impl_->mem, blob.ptr<float>(),
    static_cast<size_t>(3) * impl_->in_size * impl_->in_size,
    in_shape.data(), in_shape.size());
  const char * in_names[] = {impl_->input_name.c_str()};
  const char * out_names[] = {impl_->output_name.c_str()};
  auto outs = impl_->session->Run(
    Ort::RunOptions{nullptr}, in_names, &in, 1, out_names, 1);
  const auto out_shape = outs[0].GetTensorTypeAndShapeInfo().GetShape();
  if (out_shape.size() != 4 || out_shape.back() != impl_->in_size) {
    throw std::runtime_error("YoloDepthEstimator: 不支持的输出布局");
  }
}

YoloDepthEstimator::~YoloDepthEstimator() = default;

bool YoloDepthEstimator::cuda_active() const
{
  return impl_->cuda_ok;
}

std::string YoloDepthEstimator::cuda_error() const
{
  return impl_->cuda_err;
}

int YoloDepthEstimator::input_size() const
{
  return impl_->in_size;
}

DepthResult YoloDepthEstimator::estimate(const cv::Mat & bgr)
{
  DepthResult out;
  if (bgr.empty()) {
    return out;
  }
  float scale = 1.f;
  int pad_x = 0, pad_y = 0;
  const cv::Mat canvas = letterbox(bgr, impl_->in_size, scale, pad_x, pad_y);
  cv::Mat blob = make_blob(canvas);
  const std::array<int64_t, 4> in_shape{
    1, 3, impl_->in_size, impl_->in_size};
  auto in = Ort::Value::CreateTensor<float>(
    impl_->mem, blob.ptr<float>(),
    static_cast<size_t>(3) * impl_->in_size * impl_->in_size,
    in_shape.data(), in_shape.size());
  const char * in_names[] = {impl_->input_name.c_str()};
  const char * out_names[] = {impl_->output_name.c_str()};
  auto outs = impl_->session->Run(
    Ort::RunOptions{nullptr}, in_names, &in, 1, out_names, 1);

  // 输出 1x1xSxS -> 包成 Mat (零拷贝) -> 裁掉 letterbox 无效区 ->
  // (log 深度) exp 还原 -> 缩放回原帧尺寸, 与原图逐像素对齐
  const int s = impl_->in_size;
  const float * raw = outs[0].GetTensorData<float>();
  const cv::Mat depth_canvas(s, s, CV_32F, const_cast<float *>(raw));
  const int vw = std::max(1, s - 2 * pad_x);
  const int vh = std::max(1, s - 2 * pad_y);
  cv::Mat valid = depth_canvas(cv::Rect(pad_x, pad_y, vw, vh)).clone();
  if (config_.log_depth) {
    // 限幅防溢出 (异常输出单点不会拖垮整帧), 再 exp 还原为深度
    valid = cv::min(cv::max(valid, -20.f), 20.f);
    cv::exp(valid, valid);
  }
  cv::resize(valid, out.depth, bgr.size(), 0, 0, cv::INTER_LINEAR);
  double dmin = 0.0, dmax = 0.0;
  cv::minMaxLoc(valid, &dmin, &dmax);
  out.dmin = static_cast<float>(dmin);
  out.dmax = static_cast<float>(dmax);
  return out;
}

std::string to_depth_annotations_json(
  const DepthResult & result, const std::string & algo,
  float pip_scale, int jpeg_quality)
{
  std::vector<uchar> jpeg;
  std::string b64;
  int draw_w = 0, draw_h = 0, pos_x = 0, pos_y = 0;
  if (!result.depth.empty() && result.dmax > result.dmin) {
    // min-max 归一化 -> turbo 伪彩 (近处暖色/远处冷色, 相对深度可视化惯例)
    cv::Mat norm, colored;
    cv::normalize(result.depth, norm, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::applyColorMap(norm, colored, cv::COLORMAP_TURBO);
    const int fw = colored.cols, fh = colored.rows;
    // 画中画: 右下角小窗 (留 8px 边距), 编码即窗尺寸,
    // 地面站按原尺寸贴不放大, 相机画面不被遮挡
    pip_scale = std::max(0.1f, std::min(1.f, pip_scale));
    draw_w = std::max(1, static_cast<int>(std::lround(fw * pip_scale)));
    draw_h = std::max(1, static_cast<int>(std::lround(fh * pip_scale)));
    pos_x = fw - draw_w - 8;
    pos_y = fh - draw_h - 8;
    cv::resize(colored, colored, cv::Size(draw_w, draw_h), 0, 0,
      cv::INTER_AREA);
    cv::imencode(".jpg", colored, jpeg,
      {cv::IMWRITE_JPEG_QUALITY, std::max(1, std::min(100, jpeg_quality))});
    b64 = base64_encode(jpeg.data(), jpeg.size());
  }

  std::ostringstream os;
  os << "{\"algo\":\"" << algo << "\",\"items\":[";
  if (!b64.empty()) {
    os << "{\"type\":\"image\",\"pos\":[" << pos_x << "," << pos_y << "],"
       << "\"w\":" << draw_w << ","
       << "\"alpha\":1,"
       << "\"data\":\"" << b64 << "\"},";
  }
  // 本帧深度范围: 画面左下角 (HUD 左上角 / 分类右上角, 左下角空闲)
  if (!result.depth.empty()) {
    os << "{\"type\":\"text\",\"pos\":[12," << result.depth.rows - 12
       << "],\"text\":\"depth " << num(result.dmin) << " ~ "
       << num(result.dmax) << "\"}";
  }
  os << "]}";
  return os.str();
}

}  // namespace flylab_vision
