// Copyright 2026 flylab
/// YOLO 语义分割器实现 —— onnxruntime 推理 + un-letterbox + 配色着色 + 标注 JSON。
///
/// 模型: Ultralytics 语义分割 ONNX 标准导出 (models/semantic/yolo26s-sem.onnx,
/// Cityscapes 19 类, SemanticSegment 头)
///   输入  1x3x1024x1024 (RGB, /255, letterbox)
///   输出  1x1024x1024 uint8 逐像素类别图 (argmax 已烘焙进图内)
///
/// 后处理: 类别图与 letterbox 画布同分辨率, 裁掉灰边无效区 -> 最近邻缩放
/// 回原帧尺寸 (INTER_NEAREST 避免类别插值混色), 与原图逐像素对齐。
///
/// 与 yolo_segment (实例分割) 的差异: 无检测框/原型掩码/NMS, 输出是整幅
/// 像素级类别图, 展示走 image 图元半透明全幅叠加。
#include "yolo/yolo_semantic.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <onnxruntime_cxx_api.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace flylab_vision
{

namespace
{

/// 等比缩放 + 灰边填充到 size x size, 返回画布并带回缩放比 / pad (同 yolo_detector)
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

/// letterbox 画布 -> NCHW float blob (RGB, /255), 行级单次遍历手写平面化
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

std::string num(double v, int prec = 1)
{
  std::ostringstream os;
  os << std::fixed << std::setprecision(prec) << v;
  return os.str();
}

/// base64 编码 (无外部依赖, 手写查表)
const char kBase64Table[] =
  "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

std::string base64_encode(const uint8_t * data, size_t len)
{
  std::string out;
  out.reserve((len + 2) / 3 * 4);
  for (size_t i = 0; i < len; i += 3) {
    const uint32_t b0 = data[i];
    const uint32_t b1 = i + 1 < len ? data[i + 1] : 0;
    const uint32_t b2 = i + 2 < len ? data[i + 2] : 0;
    const uint32_t tri = (b0 << 16) | (b1 << 8) | b2;
    out += kBase64Table[(tri >> 18) & 0x3f];
    out += kBase64Table[(tri >> 12) & 0x3f];
    out += i + 1 < len ? kBase64Table[(tri >> 6) & 0x3f] : '=';
    out += i + 2 < len ? kBase64Table[tri & 0x3f] : '=';
  }
  return out;
}

}  // namespace

const std::vector<std::string> & cityscapes_names()
{
  static const std::vector<std::string> names = {
    "road", "sidewalk", "building", "wall", "fence", "pole",
    "traffic light", "traffic sign", "vegetation", "terrain", "sky",
    "person", "rider", "car", "truck", "bus", "train", "motorcycle",
    "bicycle"};
  return names;
}

const std::vector<cv::Vec3b> & cityscapes_palette_rgb()
{
  static const std::vector<cv::Vec3b> palette = {
    {128, 64, 128},    // 0  road
    {244, 35, 232},    // 1  sidewalk
    {70, 70, 70},      // 2  building
    {102, 102, 156},   // 3  wall
    {190, 153, 153},   // 4  fence
    {153, 153, 153},   // 5  pole
    {250, 170, 30},    // 6  traffic light
    {220, 220, 0},     // 7  traffic sign
    {107, 142, 35},    // 8  vegetation
    {152, 251, 152},   // 9  terrain
    {70, 130, 180},    // 10 sky
    {220, 20, 60},     // 11 person
    {255, 0, 0},       // 12 rider
    {0, 0, 142},       // 13 car
    {0, 0, 70},        // 14 truck
    {0, 60, 100},      // 15 bus
    {0, 80, 100},      // 16 train
    {0, 0, 230},       // 17 motorcycle
    {119, 11, 32}};    // 18 bicycle
  return palette;
}

/// pimpl: onnxruntime 对象全部藏在实现里, 头文件不暴露其依赖
struct YoloSemanticSegmenter::Impl
{
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "yolo_semantic"};
  std::unique_ptr<Ort::Session> session;
  Ort::MemoryInfo mem{Ort::MemoryInfo::CreateCpu(
      OrtArenaAllocator, OrtMemTypeDefault)};
  std::string input_name, output_name;
  int input_size = 0;     // 实际输入边长 (模型固定或显式指定)
  int out_h = 0, out_w = 0;   // 输出类别图分辨率 (空跑时确定)
  bool cuda_ok = false;
  std::string cuda_err;   // CUDA EP 注册失败原因 (成功则为空)
};

YoloSemanticSegmenter::YoloSemanticSegmenter(const YoloSemanticConfig & config)
: impl_(std::make_unique<Impl>()), config_(config)   // 顺序须与声明一致 (impl_ 在前)
{
  if (config_.model_path.empty()) {
    throw std::runtime_error("YoloSemanticSegmenter: model_path 为空");
  }
  Ort::SessionOptions opts;
  opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  if (config_.use_cuda) {
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
    impl_->output_name = impl_->session->GetOutputNameAllocated(0, alloc).get();
  }

  // 输入尺寸: 显式指定优先, 否则取模型固定输入尺寸 (动态 -1 则报错)
  const auto in_shape = impl_->session->GetInputTypeInfo(0)
    .GetTensorTypeAndShapeInfo().GetShape();
  if (config_.input_size > 0) {
    impl_->input_size = config_.input_size;
  } else if (in_shape.size() == 4 && in_shape[2] > 0 && in_shape[3] > 0) {
    impl_->input_size = static_cast<int>(in_shape[2]);
    if (in_shape[3] != in_shape[2]) {
      throw std::runtime_error(
              "YoloSemanticSegmenter: 模型输入非正方形 " +
              std::to_string(in_shape[2]) + "x" + std::to_string(in_shape[3]));
    }
  } else {
    throw std::runtime_error(
            "YoloSemanticSegmenter: 模型为动态输入尺寸, 须显式指定 input_size");
  }

  // 空跑一次确认模型可用并记录输出分辨率 (加载失败在这里抛, 不拖到首帧)
  cv::Mat probe(
    impl_->input_size, impl_->input_size, CV_8UC3, cv::Scalar(114, 114, 114));
  cv::Mat blob = make_blob(probe);
  const std::array<int64_t, 4> shape{
    1, 3, impl_->input_size, impl_->input_size};
  auto in = Ort::Value::CreateTensor<float>(
    impl_->mem, blob.ptr<float>(), blob.total(), shape.data(), shape.size());
  const char * in_names[] = {impl_->input_name.c_str()};
  const char * out_names[] = {impl_->output_name.c_str()};
  auto outs = impl_->session->Run(
    Ort::RunOptions{nullptr}, in_names, &in, 1, out_names, 1);
  const auto out_shape = outs[0].GetTensorTypeAndShapeInfo().GetShape();
  const auto out_type =
    outs[0].GetTensorTypeAndShapeInfo().GetElementType();
  if (out_shape.size() != 3 || out_type != ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8) {
    throw std::runtime_error("YoloSemanticSegmenter: 不支持的输出布局 (期望 1xHxW uint8)");
  }
  impl_->out_h = static_cast<int>(out_shape[1]);
  impl_->out_w = static_cast<int>(out_shape[2]);
  if (impl_->out_h != impl_->input_size || impl_->out_w != impl_->input_size) {
    throw std::runtime_error("YoloSemanticSegmenter: 输出分辨率与输入不一致");
  }
}

YoloSemanticSegmenter::~YoloSemanticSegmenter() = default;

int YoloSemanticSegmenter::input_size() const
{
  return impl_->input_size;
}

bool YoloSemanticSegmenter::cuda_active() const
{
  return impl_->cuda_ok;
}

std::string YoloSemanticSegmenter::cuda_error() const
{
  return impl_->cuda_err;
}

YoloSemanticResult YoloSemanticSegmenter::segment(const cv::Mat & bgr)
{
  YoloSemanticResult result;
  if (bgr.empty()) {
    return result;
  }
  float scale = 1.f;
  int pad_x = 0, pad_y = 0;
  const cv::Mat canvas = letterbox(bgr, impl_->input_size, scale, pad_x, pad_y);
  cv::Mat blob = make_blob(canvas);
  const std::array<int64_t, 4> shape{
    1, 3, impl_->input_size, impl_->input_size};
  auto in = Ort::Value::CreateTensor<float>(
    impl_->mem, blob.ptr<float>(),
    static_cast<size_t>(3) * impl_->input_size * impl_->input_size,
    shape.data(), shape.size());
  const char * in_names[] = {impl_->input_name.c_str()};
  const char * out_names[] = {impl_->output_name.c_str()};
  auto outs = impl_->session->Run(
    Ort::RunOptions{nullptr}, in_names, &in, 1, out_names, 1);

  // 输出 1xSxS uint8 -> S x S 单通道 Mat (数据在 outs 生命周期内, 先拷贝)
  const auto & out = outs[0];
  const uint8_t * raw = out.GetTensorData<uint8_t>();
  const int s = impl_->input_size;
  cv::Mat pred(s, s, CV_8UC1);
  std::copy(raw, raw + static_cast<size_t>(s) * s, pred.data);

  // un-letterbox: 裁掉灰边无效区 -> 最近邻缩放回原帧尺寸 (类别图禁用线性插值)
  const int nw = std::round(bgr.cols * scale), nh = std::round(bgr.rows * scale);
  cv::Mat valid;
  cv::resize(
    pred(cv::Rect(pad_x, pad_y, nw, nh)), valid,
    bgr.size(), 0, 0, cv::INTER_NEAREST);

  // 中值滤波: 去单像素椒盐噪声 (中值近似类别多数投票), 边界更整、叠加不闪
  if (config_.smooth_kernel >= 3) {
    cv::medianBlur(valid, valid, config_.smooth_kernel | 1);
  }

  // 类别过滤: 不在白名单的像素置 void
  if (!config_.class_filter.empty()) {
    const auto & keep = config_.class_filter;
    for (int y = 0; y < valid.rows; ++y) {
      uint8_t * row = valid.ptr<uint8_t>(y);
      for (int x = 0; x < valid.cols; ++x) {
        if (std::find(keep.begin(), keep.end(), row[x]) == keep.end()) {
          row[x] = kSemanticVoid;
        }
      }
    }
  }

  // 统计本帧出现的类别 (按像素占比降序, 供图例展示)
  std::array<int, 256> hist{};
  for (int y = 0; y < valid.rows; ++y) {
    const uint8_t * row = valid.ptr<uint8_t>(y);
    for (int x = 0; x < valid.cols; ++x) {
      ++hist[row[x]];
    }
  }
  std::vector<std::pair<int, long>> counts;
  for (int c = 0; c < static_cast<int>(cityscapes_names().size()); ++c) {
    if (hist[c] > 0 && (config_.class_filter.empty() ||
      std::find(config_.class_filter.begin(), config_.class_filter.end(), c) !=
      config_.class_filter.end()))
    {
      counts.push_back({c, hist[c]});
    }
  }
  std::sort(counts.begin(), counts.end(),
    [](const auto & a, const auto & b) {return a.second > b.second;});
  for (const auto & [c, cnt] : counts) {
    (void)cnt;
    result.present.push_back(c);
  }

  result.class_map = std::move(valid);
  return result;
}

std::string to_semantic_annotations_json(
  const YoloSemanticResult & result, const std::string & algo, float pip_scale)
{
  if (result.class_map.empty()) {
    return "{\"algo\":\"" + algo + "\",\"items\":[]}";
  }
  const cv::Mat & cls = result.class_map;
  const auto & palette = cityscapes_palette_rgb();

  // 类别图 -> 配色 BGRA 图: 类别像素不透明, 无效像素半透明深灰打底
  // (保持画中画窗口完整感, 同时暗示"无类别归属")。
  // PNG 编码: 平坦色块体积小、无 JPEG 伪影闪烁, 且保留透明通道
  cv::Mat bgra(cls.size(), CV_8UC4);
  for (int y = 0; y < cls.rows; ++y) {
    const uint8_t * src = cls.ptr<uint8_t>(y);
    cv::Vec4b * dst = bgra.ptr<cv::Vec4b>(y);
    for (int x = 0; x < cls.cols; ++x) {
      const uint8_t c = src[x];
      if (c != kSemanticVoid && c < palette.size()) {
        // 调色板存 RGB, Mat 用 BGRA
        dst[x] = cv::Vec4b(palette[c][2], palette[c][1], palette[c][0], 255);
      } else {
        dst[x] = cv::Vec4b(40, 40, 40, 200);   // 深灰半透明底
      }
    }
  }

  // 画中画 (同深度估计): 右下角小窗 (留 8px 边距), 编码即窗尺寸,
  // 地面站按原尺寸贴不放大, 相机画面不被遮挡
  pip_scale = std::max(0.1f, std::min(1.f, pip_scale));
  const int draw_w = std::max(1, static_cast<int>(std::lround(cls.cols * pip_scale)));
  const int draw_h = std::max(1, static_cast<int>(std::lround(cls.rows * pip_scale)));
  const int pos_x = cls.cols - draw_w - 8;
  const int pos_y = cls.rows - draw_h - 8;
  cv::Mat pip;
  cv::resize(bgra, pip, cv::Size(draw_w, draw_h), 0, 0, cv::INTER_AREA);

  std::vector<uchar> png;
  if (!cv::imencode(".png", pip, png)) {
    return "{\"algo\":\"" + algo + "\",\"items\":[]}";
  }
  const std::string b64 = base64_encode(png.data(), png.size());

  std::ostringstream os;
  os << "{\"algo\":\"" << algo << "\",\"items\":[";
  os << "{\"type\":\"image\",\"pos\":[" << num(pos_x, 0) << "," << num(pos_y, 0)
     << "],\"w\":" << num(draw_w, 0)
     << ",\"alpha\":1"
     << ",\"data\":\"" << b64 << "\"}";
  // text 图元: 本帧出现的类别图例 (按像素占比降序), 左下角偏上一行 ——
  // 左上角是地面站 FPS HUD, 左下角同一行留给深度估计的范围文字
  if (!result.present.empty()) {
    const auto & names = cityscapes_names();
    os << ",{\"type\":\"text\",\"pos\":[12," << num(cls.rows - 42, 0)
       << "],\"text\":\"";
    for (size_t i = 0; i < result.present.size() && i < 8; ++i) {
      const int c = result.present[i];
      if (i > 0) {
        os << " / ";
      }
      os << (c < static_cast<int>(names.size()) ? names[c] :
        "class_" + std::to_string(c));
    }
    if (result.present.size() > 8) {
      os << " / +" << result.present.size() - 8;
    }
    os << "\",\"color\":[255,255,255]}";
  }
  os << "]}";
  return os.str();
}

}  // namespace flylab_vision
