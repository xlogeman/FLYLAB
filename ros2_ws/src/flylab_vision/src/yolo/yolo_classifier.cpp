// Copyright 2026 flylab
/// YOLO 图像分类器实现 —— onnxruntime 推理 + softmax + top-k + 地面站标注 JSON。
///
/// 模型: Ultralytics ONNX 标准导出 (models/classify/yolo26s-cls.onnx)
///   输入  1x3xSxS (RGB, /255, letterbox 正方形)  —— S 由模型固定输入尺寸决定
///   输出  1xC 类别分数 (C=1000 为 ImageNet-1k); yolo26-cls 导出已内建
///         softmax, 输出即概率, 纯线性导出则运行时自动补 softmax
///
/// 标签解析: Ultralytics 导出把类别名表写进 ONNX metadata "names"
///   ("{0: 'tench, Tinca tinca', 1: 'goldfish, Carassius auratus', ...}"),
///   直接从模型里取, 无需随包分发标签文件; 取不到再走外部文件/占位名。
#include "yolo/yolo_classifier.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <onnxruntime_cxx_api.h>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace flylab_vision
{

// 同 YoloDetector: GPU 构建的 libonnxruntime 导出该 C 接口, 字符串版
// AppendExecutionProvider("CUDAExecutionProvider") 不认 CUDA 会抛异常
extern "C" OrtStatus * OrtSessionOptionsAppendExecutionProvider_CUDA(
  OrtSessionOptions * options, int device_id);

namespace
{

/// 等比缩放 + 114 灰边填充到 size x size (与 Ultralytics ClassifyLetterBox 一致)
cv::Mat letterbox(const cv::Mat & bgr, int size)
{
  const float r = std::min(size / (float)bgr.rows, size / (float)bgr.cols);
  const int nh = std::round(bgr.rows * r), nw = std::round(bgr.cols * r);
  cv::Mat resized;
  cv::resize(bgr, resized, cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);
  cv::Mat canvas(size, size, CV_8UC3, cv::Scalar(114, 114, 114));
  resized.copyTo(canvas(cv::Rect((size - nw) / 2, (size - nh) / 2, nw, nh)));
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

/// 数值稳定 softmax (logits -> 概率)
std::vector<float> softmax(const float * logits, int n)
{
  float m = logits[0];
  for (int i = 1; i < n; ++i) {
    m = std::max(m, logits[i]);
  }
  std::vector<float> p(n);
  float sum = 0.f;
  for (int i = 0; i < n; ++i) {
    p[i] = std::exp(logits[i] - m);
    sum += p[i];
  }
  for (auto & v : p) {
    v /= sum;
  }
  return p;
}

/// 解析 Ultralytics metadata "names": "{0: 'tench, Tinca tinca', ...}"。
/// 类别名里可以带逗号 (英文俗名, 学名), 逐条匹配 "id: 'name'" 而非按逗号切分。
std::vector<std::string> parse_embedded_names(const std::string & s)
{
  static const std::regex re(R"((\d+)\s*:\s*'([^']*)')");
  std::vector<std::string> out;
  int max_id = -1;
  for (std::sregex_iterator it(s.begin(), s.end(), re); it != std::sregex_iterator{}; ++it) {
    const int id = std::stoi((*it)[1]);
    if (id >= static_cast<int>(out.size())) {
      out.resize(id + 1);
    }
    out[id] = (*it)[2];
    max_id = std::max(max_id, id);
  }
  if (max_id < 0) {
    out.clear();
  }
  return out;
}

/// 外部标签文件: 每行一个类别名, 忽略空行
std::vector<std::string> load_labels_file(const std::string & path)
{
  std::ifstream f(path);
  if (!f) {
    throw std::runtime_error("YoloClassifier: 标签文件读不了: " + path);
  }
  std::vector<std::string> out;
  std::string line;
  while (std::getline(f, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (!line.empty()) {
      out.push_back(line);
    }
  }
  return out;
}

std::string num(double v, int prec = 2)
{
  std::ostringstream os;
  os << std::fixed << std::setprecision(prec) << v;
  return os.str();
}

/// 按类别 id 生成稳定的 BGR 颜色 (与 YoloDetector::class_color 同款逻辑,
/// 库间不互相链接, 各自带一份)
cv::Scalar cls_color(int class_id)
{
  const int hue = (class_id * 37) % 180;
  cv::Mat hsv(1, 1, CV_8UC3), bgr;
  hsv.at<cv::Vec3b>(0, 0) = cv::Vec3b(static_cast<uchar>(hue), 235, 255);
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
  const cv::Vec3b v = bgr.at<cv::Vec3b>(0, 0);
  return cv::Scalar(v[0], v[1], v[2]);
}

/// JSON 字符串转义 (标签来自模型 metadata, 保守处理引号/反斜杠)
std::string json_escape(const std::string & s)
{
  std::string out;
  for (const char c : s) {
    if (c == '"' || c == '\\') {
      out += '\\';
    }
    out += c;
  }
  return out;
}

}  // namespace

/// pimpl: onnxruntime 对象全部藏在实现里, 头文件不暴露其依赖
struct YoloClassifier::Impl
{
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "yolo_classifier"};
  std::unique_ptr<Ort::Session> session;
  Ort::MemoryInfo mem{Ort::MemoryInfo::CreateCpu(
      OrtArenaAllocator, OrtMemTypeDefault)};
  std::string input_name, output_name;
  int in_size = 0;          // 实际输入边长
  int classes = 0;          // 类别总数
  bool cuda_ok = false;
  std::string cuda_err;     // CUDA EP 注册失败原因 (成功则为空)
  std::vector<std::string> labels;
};

YoloClassifier::YoloClassifier(const ClsConfig & config)
: impl_(std::make_unique<Impl>()), config_(config)
{
  if (config_.model_path.empty()) {
    throw std::runtime_error("YoloClassifier: model_path 为空");
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
              "YoloClassifier: 模型输入为动态尺寸, 须显式指定 input_size");
    }
  }

  // 类别总数 + 标签表: 模型内嵌 metadata > 外部文件 > 占位名
  {
    auto out_shape = impl_->session->GetOutputTypeInfo(0)
                     .GetTensorTypeAndShapeInfo().GetShape();
    size_t total = 1;
    for (const auto d : out_shape) {
      if (d > 0) {
        total *= static_cast<size_t>(d);
      }
    }
    if (total < 2) {
      throw std::runtime_error("YoloClassifier: 不支持的输出布局");
    }
    impl_->classes = static_cast<int>(total);

    std::vector<std::string> names;
    Ort::ModelMetadata md = impl_->session->GetModelMetadata();
    Ort::AllocatorWithDefaultOptions alloc;
    auto embedded = md.LookupCustomMetadataMapAllocated("names", alloc);
    if (embedded != nullptr) {
      names = parse_embedded_names(embedded.get());
    }
    if (static_cast<int>(names.size()) == impl_->classes) {
      impl_->labels = std::move(names);
    } else if (!config_.labels_file.empty()) {
      impl_->labels = load_labels_file(config_.labels_file);
      if (static_cast<int>(impl_->labels.size()) != impl_->classes) {
        throw std::runtime_error(
                "YoloClassifier: 标签文件行数 (" +
                std::to_string(impl_->labels.size()) + ") 与模型类别数 (" +
                std::to_string(impl_->classes) + ") 不一致");
      }
    } else {
      for (int i = 0; i < impl_->classes; ++i) {
        impl_->labels.push_back("class_" + std::to_string(i));
      }
    }
  }

  // 空跑一次确认模型可用 (加载失败在这里抛, 不拖到首帧)
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
  (void)out_shape;   // 类别数已在上面按输出张量元素总数校验
}

YoloClassifier::~YoloClassifier() = default;

bool YoloClassifier::cuda_active() const
{
  return impl_->cuda_ok;
}

std::string YoloClassifier::cuda_error() const
{
  return impl_->cuda_err;
}

int YoloClassifier::num_classes() const
{
  return impl_->classes;
}

int YoloClassifier::input_size() const
{
  return impl_->in_size;
}

const std::vector<std::string> & YoloClassifier::labels() const
{
  return impl_->labels;
}

std::vector<ClsPrediction> YoloClassifier::classify(const cv::Mat & bgr)
{
  if (bgr.empty()) {
    return {};
  }
  const cv::Mat canvas = letterbox(bgr, impl_->in_size);
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

  // 输出 1xC -> (必要时 softmax) -> top-k (置信度降序)。
  // yolo26-cls 导出已内建 softmax, 输出恒为非负且总和≈1 的概率;
  // 其他导出 (末层纯线性) 才是 logits, 需要自己 softmax。按数值特征判别:
  // 对已是概率的输出再套一层 softmax 会把分布压成近似均匀 (全部显示 0.00)
  const float * raw = outs[0].GetTensorData<float>();
  bool is_prob = true;
  float sum = 0.f;
  for (int i = 0; i < impl_->classes; ++i) {
    const float v = raw[i];
    if (v < 0.f || v > 1.0001f) {
      is_prob = false;
      break;
    }
    sum += v;
  }
  std::vector<float> prob;
  if (is_prob && std::abs(sum - 1.f) < 0.05f) {
    prob.assign(raw, raw + impl_->classes);
  } else {
    prob = softmax(raw, impl_->classes);
  }
  const int k = std::min(std::max(config_.topk, 1), impl_->classes);
  std::vector<int> idx(impl_->classes);
  for (int i = 0; i < impl_->classes; ++i) {
    idx[i] = i;
  }
  std::partial_sort(
    idx.begin(), idx.begin() + k, idx.end(),
    [&](int a, int b) {return prob[a] > prob[b];});

  std::vector<ClsPrediction> results;
  results.reserve(k);
  for (int i = 0; i < k; ++i) {
    const int id = idx[i];
    results.push_back({id, impl_->labels[id], prob[id]});
  }
  return results;
}

std::string to_cls_annotations_json(
  const std::vector<ClsPrediction> & preds, const std::string & algo,
  int frame_width)
{
  std::ostringstream os;
  os << "{\"algo\":\"" << algo << "\",\"items\":[";
  // 右上角逐行文字 (右对齐, 右边距 12px): 地面站 HUD 在左上角, 右上角空闲;
  // top-1 用类别稳定色 (同检测框配色逻辑), 其余灰白。
  // 帧宽未知 (frame_width<=0) 时退化为左上角
  const bool right = frame_width > 0;
  const int x = right ? frame_width - 12 : 12;
  const int y0 = right ? 26 : 70, dy = 28;
  for (size_t i = 0; i < preds.size(); ++i) {
    const auto & p = preds[i];
    if (i > 0) {
      os << ",";
    }
    std::ostringstream item;
    item << "{\"type\":\"text\""
         << (right ? ",\"align\":\"right\"" : "")
         << ",\"pos\":[" << x << "," << y0 + dy * static_cast<int>(i) << "],"
         << "\"text\":\"" << json_escape(p.name) << " " << num(p.conf) << "\"";
    if (i == 0) {
      const cv::Scalar c = cls_color(p.class_id);
      item << ",\"color\":[" << static_cast<int>(c[0]) << ","
           << static_cast<int>(c[1]) << "," << static_cast<int>(c[2]) << "]}";
    } else {
      item << ",\"color\":[255,255,255]}";
    }
    os << item.str();
  }
  os << "]}";
  return os.str();
}

}  // namespace flylab_vision
