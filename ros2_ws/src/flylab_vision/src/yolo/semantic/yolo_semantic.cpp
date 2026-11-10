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
/// 像素级类别图, 展示走 cityscapes 配色缩略图进右栏面板 (to_semantic_info_json)。
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

/// 类别图 -> cityscapes 配色 BGR 图: 类别像素不透明着色, 无效像素深灰
/// (暗示"无类别归属"), 等比缩放到 target_w 宽 (0 = 保持原尺寸)。
/// 供右栏面板缩略图编码
cv::Mat colorize_semantic(const cv::Mat & cls, int target_w = 0)
{
  cv::Mat colored(cls.size(), CV_8UC3);
  const auto & palette = cityscapes_palette_rgb();
  for (int y = 0; y < cls.rows; ++y) {
    const uint8_t * src = cls.ptr<uint8_t>(y);
    cv::Vec3b * dst = colored.ptr<cv::Vec3b>(y);
    for (int x = 0; x < cls.cols; ++x) {
      const uint8_t c = src[x];
      if (c != kSemanticVoid && c < palette.size()) {
        // 调色板存 RGB, Mat 用 BGR
        dst[x] = cv::Vec3b(palette[c][2], palette[c][1], palette[c][0]);
      } else {
        dst[x] = cv::Vec3b(214, 218, 224);   // 浅灰底 (无类别归属)
      }
    }
  }
  if (target_w > 0 && target_w < colored.cols) {
    const int h = std::max(1, static_cast<int>(std::lround(
      colored.rows * target_w / static_cast<double>(colored.cols))));
    cv::resize(colored, colored, cv::Size(target_w, h), 0, 0, cv::INTER_AREA);
  }
  return colored;
}

}  // namespace

std::string to_semantic_info_json(
  const YoloSemanticResult & result, int frame_w, int frame_h, int img_width)
{
  std::ostringstream os;
  os << "{\"src\":\"semantic\"";
  if (!result.class_map.empty()) {
    const auto & names = cityscapes_names();
    os << ",\"nclass\":" << result.present.size()
       << ",\"w\":" << frame_w << ",\"h\":" << frame_h;
    // 本帧出现的类别图例 (按像素占比降序): [名, 占比%, [R,G,B]] 全量发送,
    // 显示截断 (前 3 项) 在面板侧 —— 截断上限属显示关注点, 改图例行数只动
    // flylab_gcs 不用重编本包; RGB 取 cityscapes 标准配色, 面板画同色小方块
    os << ",\"present\":[";
    const auto & palette = cityscapes_palette_rgb();
    for (size_t i = 0; i < result.present.size(); ++i) {
      const int c = result.present[i];
      if (i > 0) {
        os << ",";
      }
      os << "[\"" << (c < static_cast<int>(names.size()) ? names[c] :
        "class_" + std::to_string(c)) << "\","
         << std::fixed << std::setprecision(1)
         << (i < result.present_share.size() ? result.present_share[i] : 0.f)
         << ",[" << static_cast<int>(palette[c][0]) << ","
         << static_cast<int>(palette[c][1]) << ","
         << static_cast<int>(palette[c][2]) << "]]";
    }
    os << "]";
    // 配色缩略图: 缩到面板显示宽度再编码, 体积随宽度而非原帧尺寸;
    // PNG: 平坦色块体积小、无 JPEG 伪影闪烁
    if (img_width > 0) {
      const cv::Mat colored = colorize_semantic(result.class_map, img_width);
      std::vector<uchar> png;
      if (cv::imencode(".png", colored, png)) {
        os << ",\"img\":\"" << base64_encode(png.data(), png.size()) << "\"";
      }
    }
  }
  os << "}";
  return os.str();
}

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
  // 自定义清新配色 (RGB, 下标 = 类别 id): 替代 Cityscapes 标准配色
  // (整体偏深偏闷), 保持语义直觉 (天空蓝/植被绿/路面雾蓝紫) 且
  // 相邻易混类别拉开色相; 类别图缩略图与图例色块共用本表
  static const std::vector<cv::Vec3b> palette = {
    {106, 130, 230},   // 0  road       雾蓝紫
    {255, 195, 120},   // 1  sidewalk   杏黄
    {140, 142, 148},   // 2  building   中灰
    {226, 168, 216},   // 3  wall       粉紫
    {255, 216, 96},    // 4  fence      明黄
    {168, 172, 190},   // 5  pole       浅灰
    {255, 84, 84},     // 6  traffic light  亮红
    {250, 200, 0},     // 7  traffic sign   柠黄
    {104, 200, 112},   // 8  vegetation 清新绿
    {192, 232, 128},   // 9  terrain    嫩芽黄绿
    {96, 182, 255},    // 10 sky        天蓝
    {255, 128, 56},    // 11 person     活力橙
    {255, 172, 190},   // 12 rider      樱粉
    {78, 142, 255},    // 13 car        亮蓝
    {58, 192, 202},    // 14 truck      青
    {255, 158, 64},    // 15 bus        橙黄
    {150, 122, 232},   // 16 train      紫罗兰
    {255, 96, 160},    // 17 motorcycle 品红粉
    {118, 222, 180}};  // 18 bicycle    薄荷绿
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
  const float total = static_cast<float>(valid.total());
  for (const auto & [c, cnt] : counts) {
    result.present.push_back(c);
    result.present_share.push_back(
      total > 0.f ? 100.f * cnt / total : 0.f);
  }

  result.class_map = std::move(valid);
  return result;
}

}  // namespace flylab_vision
