// Copyright 2026 flylab
/// YOLO OBB (旋转框) 检测器实现 —— onnxruntime 推理 + 解码 + 旋转 NMS + 标注 JSON。
///
/// 模型: Ultralytics OBB ONNX 标准导出 (models/obb/yolo26s-obb.onnx)
///   输入  1x3x1024x1024 (RGB, /255, letterbox)
///   输出  1x20x21504: 前 4 通道 = cx/cy/w/h (letterbox 输入坐标系, 像素),
///         第 5 通道 = 角度 (弧度), 后 15 通道 = DOTA v1 各类分数;
///         兼容 1x21504x20 anchor-major 布局
///
/// 与 yolo_detector 的差异: 第 5 通道是角度, NMS 必须用旋转框 IoU
/// (cv::dnn::NMSBoxes 只支持水平框) —— 自定义贪心抑制, 交叠面积用
/// cv::rotatedRectangleIntersection 求。
#include "yolo/yolo_obb.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <onnxruntime_cxx_api.h>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace flylab_vision
{

// CUDA EP 的专用注册接口 (GPU 构建的 libonnxruntime 导出; 同 yolo_detector)
extern "C" OrtStatus * OrtSessionOptionsAppendExecutionProvider_CUDA(
  OrtSessionOptions * options, int device_id);

namespace
{

/// DOTA v1 15 类
const std::vector<std::string> kDotaNames = {
  "plane", "ship", "storage tank", "baseball diamond", "tennis court",
  "basketball court", "ground track field", "harbor", "bridge",
  "large vehicle", "small vehicle", "helicopter", "roundabout",
  "soccer ball field", "swimming pool"};

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

/// 输出张量形状归一化: 去掉首尾的 1 维 (同 yolo_detector)
struct Layout
{
  int channels = 0;
  int anchors = 0;
  bool channel_major = true;
};

Layout layout_of(const std::vector<int64_t> & shape)
{
  std::vector<int64_t> s = shape;
  while (s.size() > 2 && s.front() == 1) {
    s.erase(s.begin());
  }
  while (s.size() > 2 && s.back() == 1) {
    s.pop_back();
  }
  if (s.size() != 2) {
    return {};
  }
  Layout lay;
  // 通道维 (类别数+5) 必然小于 anchor 维, 据此判别两种布局
  lay.channel_major = s[0] < s[1];
  lay.channels = static_cast<int>(lay.channel_major ? s[0] : s[1]);
  lay.anchors = static_cast<int>(lay.channel_major ? s[1] : s[0]);
  return lay;
}

/// 两旋转框 IoU: 交叠多边形面积 / (两框面积和 - 交叠面积)
float rotated_iou(const cv::RotatedRect & a, const cv::RotatedRect & b)
{
  std::vector<cv::Point2f> sect;
  const auto status = cv::rotatedRectangleIntersection(a, b, sect);
  if (status == cv::INTERSECT_NONE || sect.size() < 3) {
    return 0.f;
  }
  const float inter = static_cast<float>(cv::contourArea(sect));
  const float uni = a.size.area() + b.size.area() - inter;
  return uni > 1e-6f ? inter / uni : 0.f;
}

std::string num(double v, int prec = 1)
{
  std::ostringstream os;
  os << std::fixed << std::setprecision(prec) << v;
  return os.str();
}

/// 按类别 id 生成稳定 BGR 颜色 (与 yolo_detector::class_color 同算法, HSV 色环取色)
cv::Scalar obb_class_color(int class_id)
{
  const int hue = (class_id * 37) % 180;
  cv::Mat hsv(1, 1, CV_8UC3), bgr;
  hsv.at<cv::Vec3b>(0, 0) = cv::Vec3b(static_cast<uchar>(hue), 235, 255);
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
  const cv::Vec3b v = bgr.at<cv::Vec3b>(0, 0);
  return cv::Scalar(v[0], v[1], v[2]);
}

}  // namespace

const std::vector<std::string> & dota_names()
{
  return kDotaNames;
}

/// pimpl: onnxruntime 对象全部藏在实现里, 头文件不暴露其依赖
struct YoloObbDetector::Impl
{
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "yolo_obb"};
  std::unique_ptr<Ort::Session> session;
  Ort::MemoryInfo mem{Ort::MemoryInfo::CreateCpu(
      OrtArenaAllocator, OrtMemTypeDefault)};
  std::string input_name, output_name;
  Layout layout;          // 首次空跑时确定, 之后复用
  bool cuda_ok = false;
  std::string cuda_err;   // CUDA EP 注册失败原因 (成功则为空)
};

YoloObbDetector::YoloObbDetector(const YoloObbConfig & config)
: impl_(std::make_unique<Impl>()), config_(config)   // 顺序须与声明一致 (impl_ 在前)
{
  if (config_.model_path.empty()) {
    throw std::runtime_error("YoloObbDetector: model_path 为空");
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
    impl_->output_name =
      impl_->session->GetOutputNameAllocated(0, alloc).get();
  }

  // 空跑一次确认模型可用并记录输出布局 (加载失败在这里抛, 不拖到首帧)
  cv::Mat probe(
    config_.input_size, config_.input_size, CV_8UC3, cv::Scalar(114, 114, 114));
  cv::Mat blob = make_blob(probe);
  const std::array<int64_t, 4> in_shape{
    1, 3, config_.input_size, config_.input_size};
  auto in = Ort::Value::CreateTensor<float>(
    impl_->mem, blob.ptr<float>(), blob.total(), in_shape.data(), in_shape.size());
  const char * in_names[] = {impl_->input_name.c_str()};
  const char * out_names[] = {impl_->output_name.c_str()};
  auto outs = impl_->session->Run(
    Ort::RunOptions{nullptr}, in_names, &in, 1, out_names, 1);
  impl_->layout = layout_of(
    outs[0].GetTensorTypeAndShapeInfo().GetShape());
  if (impl_->layout.channels < 6) {   // 至少 4 坐标 + 1 角度 + 1 类
    throw std::runtime_error("YoloObbDetector: 不支持的输出布局");
  }
}

YoloObbDetector::~YoloObbDetector() = default;

bool YoloObbDetector::cuda_active() const
{
  return impl_->cuda_ok;
}

std::string YoloObbDetector::cuda_error() const
{
  return impl_->cuda_err;
}

std::vector<YoloObbDetection> YoloObbDetector::detect(const cv::Mat & bgr)
{
  if (bgr.empty()) {
    return {};
  }
  float scale = 1.f;
  int pad_x = 0, pad_y = 0;
  const cv::Mat canvas = letterbox(bgr, config_.input_size, scale, pad_x, pad_y);
  cv::Mat blob = make_blob(canvas);
  const std::array<int64_t, 4> in_shape{
    1, 3, config_.input_size, config_.input_size};
  auto in = Ort::Value::CreateTensor<float>(
    impl_->mem, blob.ptr<float>(),
    static_cast<size_t>(3) * config_.input_size * config_.input_size,
    in_shape.data(), in_shape.size());
  const char * in_names[] = {impl_->input_name.c_str()};
  const char * out_names[] = {impl_->output_name.c_str()};
  auto outs = impl_->session->Run(
    Ort::RunOptions{nullptr}, in_names, &in, 1, out_names, 1);

  const auto & out = outs[0];
  const Layout & lay = impl_->layout;
  const float * data = out.GetTensorData<float>();
  const int n_class = lay.channels - 5;
  const int n_anchor = lay.anchors;

  // 通道取值 (兼容 channel-major / anchor-major 两种布局)
  auto at = [&](int c, int a) {
      return lay.channel_major ?
             data[static_cast<size_t>(c) * n_anchor + a] :
             data[static_cast<size_t>(a) * lay.channels + c];
    };

  // 置信度过滤 + 类别过滤, 解码为原图坐标系旋转框
  struct Cand
  {
    cv::RotatedRect box;
    float conf;
    int cls;
  };
  std::vector<Cand> cands;
  for (int a = 0; a < n_anchor; ++a) {
    float best = 0.f;
    int best_id = 0;
    for (int c = 0; c < n_class; ++c) {
      const float s = at(5 + c, a);
      if (s > best) {
        best = s;
        best_id = c;
      }
    }
    if (best < config_.conf_thres) {
      continue;
    }
    if (!config_.class_filter.empty() && std::find(
        config_.class_filter.begin(), config_.class_filter.end(), best_id) ==
      config_.class_filter.end())
    {
      continue;
    }
    // letterbox 坐标 -> 原图坐标 (旋转框: 平移缩放中心与尺寸, 角度不变)
    const float cx = (at(0, a) - pad_x) / scale;
    const float cy = (at(1, a) - pad_y) / scale;
    const float w = at(2, a) / scale;
    const float h = at(3, a) / scale;
    if (cx < -w || cy < -h || cx > bgr.cols + w || cy > bgr.rows + h) {
      continue;   // 完全出画
    }
    // Ultralytics 输出角度为弧度, 转 cv::RotatedRect 的度
    const float deg = static_cast<float>(at(4, a) * 180.0 / CV_PI);
    cands.push_back({cv::RotatedRect({cx, cy}, {w, h}, deg), best, best_id});
  }
  if (cands.empty()) {
    return {};
  }

  // 旋转框贪心 NMS: 按置信度降序, 与已保留框同类且 IoU 超阈的抑制
  std::sort(cands.begin(), cands.end(),
    [](const Cand & a, const Cand & b) {return a.conf > b.conf;});
  std::vector<YoloObbDetection> results;
  for (const auto & c : cands) {
    bool suppressed = false;
    for (const auto & r : results) {
      if (r.class_id == c.cls &&
        rotated_iou(c.box, r.box) > config_.iou_thres)
      {
        suppressed = true;
        break;
      }
    }
    if (!suppressed) {
      results.push_back({c.box, c.cls, kDotaNames[c.cls], c.conf});
    }
  }
  return results;
}

std::string to_annotations_json(
  const std::vector<YoloObbDetection> & results, const std::string & algo)
{
  std::ostringstream os;
  os << "{\"algo\":\"" << algo << "\",\"items\":[";
  for (size_t i = 0; i < results.size(); ++i) {
    const auto & d = results[i];
    if (i > 0) {
      os << ",";
    }
    const cv::Scalar c = obb_class_color(d.class_id);
    const int r = static_cast<int>(c[2]);   // BGR -> RGB 给前端
    const int g = static_cast<int>(c[1]);
    const int b = static_cast<int>(c[0]);
    cv::Point2f verts[4];
    d.box.points(verts);
    os << "{\"type\":\"polyline\",\"points\":[";
    for (int k = 0; k < 4; ++k) {
      if (k > 0) {
        os << ",";
      }
      os << "[" << num(verts[k].x) << "," << num(verts[k].y) << "]";
    }
    os << "],\"close\":true,\"color\":[" << r << "," << g << "," << b << "]}";
    // 标签放在旋转框最高顶点上方, 避免与框线重叠
    cv::Point2f top = verts[0];
    for (int k = 1; k < 4; ++k) {
      if (verts[k].y < top.y) {
        top = verts[k];
      }
    }
    os << ",{\"type\":\"text\",\"pos\":[" << num(top.x) << ","
       << num(top.y - 4) << "],\"text\":\"" << d.name << " "
       << num(d.conf, 2) << "\",\"color\":[" << r << "," << g << "," << b << "]}";
  }
  os << "]}";
  return os.str();
}

}  // namespace flylab_vision
