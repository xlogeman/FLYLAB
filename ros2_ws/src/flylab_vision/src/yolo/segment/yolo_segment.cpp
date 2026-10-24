// Copyright 2026 flylab
/// YOLO 实例分割器实现 —— onnxruntime 推理 + 解码 + NMS + 掩码还原 + 标注 JSON。
///
/// 模型: Ultralytics 分割 ONNX 标准导出 (models/segment/yolo26s-seg.onnx)
///   输入   1x3x640x640 (RGB, /255, letterbox)
///   输出0  1x116x8400: 前 4 通道 = cx/cy/w/h (letterbox 输入坐标系, 像素),
///          中间 80 通道 = COCO 各类分数, 末尾 32 通道 = 掩码系数;
///          兼容 1x8400x116 anchor-major 布局
///   输出1  原型掩码 1x32x160x160
///
/// 掩码还原 (同 Ultralytics 后处理): mask = sigmoid(系数ᵀ · 原型) 得到
/// 160x160 全图概率, 按检测框裁剪 (掩码只保留框内区域), 上采样到原图
/// 框尺寸后按阈值二值化, 贴回原图尺寸的 0/255 掩码图。
///
/// 与 yolo_detector 的差异: 输出多一路原型掩码 + 32 个掩码系数通道,
/// 解码后多做掩码还原与轮廓提取; 检测框部分流程完全相同。
#include "yolo/yolo_segment.hpp"

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

#include "yolo/yolo_detector.hpp"   // coco_names() (分割同为 COCO 80 类)

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

/// 检测输出张量形状归一化: 去掉首尾的 1 维 (同 yolo_detector)
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
  // 通道维 (类别数+4+系数数) 必然小于 anchor 维, 据此判别两种布局
  lay.channel_major = s[0] < s[1];
  lay.channels = static_cast<int>(lay.channel_major ? s[0] : s[1]);
  lay.anchors = static_cast<int>(lay.channel_major ? s[1] : s[0]);
  return lay;
}

std::string num(double v, int prec = 1)
{
  std::ostringstream os;
  os << std::fixed << std::setprecision(prec) << v;
  return os.str();
}

/// 按类别 id 生成稳定 BGR 颜色 (与 yolo_detector::class_color 同算法, HSV 色环取色)
cv::Scalar seg_class_color(int class_id)
{
  const int hue = (class_id * 37) % 180;
  cv::Mat hsv(1, 1, CV_8UC3), bgr;
  hsv.at<cv::Vec3b>(0, 0) = cv::Vec3b(static_cast<uchar>(hue), 235, 255);
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
  const cv::Vec3b v = bgr.at<cv::Vec3b>(0, 0);
  return cv::Scalar(v[0], v[1], v[2]);
}

}  // namespace

/// pimpl: onnxruntime 对象全部藏在实现里, 头文件不暴露其依赖
struct YoloSegmentDetector::Impl
{
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "yolo_segment"};
  std::unique_ptr<Ort::Session> session;
  Ort::MemoryInfo mem{Ort::MemoryInfo::CreateCpu(
      OrtArenaAllocator, OrtMemTypeDefault)};
  std::string input_name, det_name, proto_name;
  Layout layout;          // 检测输出布局, 首次空跑时确定, 之后复用
  int proto_c = 0;        // 原型掩码: 系数通道数 (32)
  int proto_h = 0;        // 原型掩码高 (输入边长 / 4)
  int proto_w = 0;        // 原型掩码宽
  bool cuda_ok = false;
  std::string cuda_err;   // CUDA EP 注册失败原因 (成功则为空)
};

YoloSegmentDetector::YoloSegmentDetector(const YoloSegmentConfig & config)
: impl_(std::make_unique<Impl>()), config_(config)   // 顺序须与声明一致 (impl_ 在前)
{
  if (config_.model_path.empty()) {
    throw std::runtime_error("YoloSegmentDetector: model_path 为空");
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
    impl_->det_name = impl_->session->GetOutputNameAllocated(0, alloc).get();
    impl_->proto_name = impl_->session->GetOutputNameAllocated(1, alloc).get();
  }

  // 空跑一次确认模型可用并记录两个输出的布局 (加载失败在这里抛, 不拖到首帧)
  cv::Mat probe(
    config_.input_size, config_.input_size, CV_8UC3, cv::Scalar(114, 114, 114));
  cv::Mat blob = make_blob(probe);
  const std::array<int64_t, 4> in_shape{
    1, 3, config_.input_size, config_.input_size};
  auto in = Ort::Value::CreateTensor<float>(
    impl_->mem, blob.ptr<float>(), blob.total(), in_shape.data(), in_shape.size());
  const char * in_names[] = {impl_->input_name.c_str()};
  const char * out_names[] = {impl_->det_name.c_str(), impl_->proto_name.c_str()};
  auto outs = impl_->session->Run(
    Ort::RunOptions{nullptr}, in_names, &in, 1, out_names, 2);
  impl_->layout = layout_of(
    outs[0].GetTensorTypeAndShapeInfo().GetShape());
  const auto proto_shape = outs[1].GetTensorTypeAndShapeInfo().GetShape();
  if (impl_->layout.channels < 7 || proto_shape.size() != 4) {   // 至少 4 坐标 + 1 类 + 2 系数
    throw std::runtime_error("YoloSegmentDetector: 不支持的输出布局");
  }
  impl_->proto_c = static_cast<int>(proto_shape[1]);
  impl_->proto_h = static_cast<int>(proto_shape[2]);
  impl_->proto_w = static_cast<int>(proto_shape[3]);
}

YoloSegmentDetector::~YoloSegmentDetector() = default;

bool YoloSegmentDetector::cuda_active() const
{
  return impl_->cuda_ok;
}

std::string YoloSegmentDetector::cuda_error() const
{
  return impl_->cuda_err;
}

std::vector<YoloSegmentDetection> YoloSegmentDetector::segment(const cv::Mat & bgr)
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
  const char * out_names[] = {impl_->det_name.c_str(), impl_->proto_name.c_str()};
  auto outs = impl_->session->Run(
    Ort::RunOptions{nullptr}, in_names, &in, 1, out_names, 2);

  const auto & det = outs[0];
  const auto & proto = outs[1];
  const Layout & lay = impl_->layout;
  const float * data = det.GetTensorData<float>();
  const int n_coef = impl_->proto_c;
  const int n_class = lay.channels - 4 - n_coef;
  const int n_anchor = lay.anchors;
  const float * proto_data = proto.GetTensorData<float>();
  const int ph = impl_->proto_h, pw = impl_->proto_w;

  // 通道取值 (兼容 channel-major / anchor-major 两种布局)
  auto at = [&](int c, int a) {
      return lay.channel_major ?
             data[static_cast<size_t>(c) * n_anchor + a] :
             data[static_cast<size_t>(a) * lay.channels + c];
    };

  // 置信度过滤 + 类别过滤, 解码为原图坐标系检测框
  struct Cand
  {
    cv::Rect2f box;        // 原图坐标
    cv::Rect2f box_lb;     // letterbox 坐标 (掩码裁剪用)
    float conf;
    int cls;
    std::vector<float> coefs;   // n_coef 个掩码系数
  };
  std::vector<Cand> cands;
  for (int a = 0; a < n_anchor; ++a) {
    float best = 0.f;
    int best_id = 0;
    for (int c = 0; c < n_class; ++c) {
      const float s = at(4 + c, a);
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
    // letterbox 坐标 -> 原图坐标
    const float cx = (at(0, a) - pad_x) / scale;
    const float cy = (at(1, a) - pad_y) / scale;
    const float w = at(2, a) / scale;
    const float h = at(3, a) / scale;
    if (cx < -w || cy < -h || cx > bgr.cols + w || cy > bgr.rows + h) {
      continue;   // 完全出画
    }
    Cand cand;
    cand.box = {cx - w / 2, cy - h / 2, w, h};
    cand.box_lb = {at(0, a) - at(2, a) / 2, at(1, a) - at(3, a) / 2, at(2, a), at(3, a)};
    cand.conf = best;
    cand.cls = best_id;
    cand.coefs.resize(n_coef);
    for (int k = 0; k < n_coef; ++k) {
      cand.coefs[k] = at(4 + n_class + k, a);
    }
    cands.push_back(std::move(cand));
  }
  if (cands.empty()) {
    return {};
  }

  // 水平框贪心 NMS: 按置信度降序, 与已保留框同类且 IoU 超阈的抑制
  std::sort(cands.begin(), cands.end(),
    [](const Cand & a, const Cand & b) {return a.conf > b.conf;});
  std::vector<Cand> kept;
  for (const auto & c : cands) {
    bool suppressed = false;
    for (const auto & k : kept) {
      if (k.cls != c.cls) {
        continue;
      }
      const float ix = std::max(0.f, std::min(c.box.br().x, k.box.br().x) -
        std::max(c.box.x, k.box.x));
      const float iy = std::max(0.f, std::min(c.box.br().y, k.box.br().y) -
        std::max(c.box.y, k.box.y));
      const float inter = ix * iy;
      const float uni = c.box.area() + k.box.area() - inter;
      if (uni > 1e-6f && inter / uni > config_.iou_thres) {
        suppressed = true;
        break;
      }
    }
    if (!suppressed) {
      kept.push_back(c);
    }
  }

  // 掩码还原: 系数 · 原型 -> sigmoid, 框内裁剪 -> 上采样 -> 二值化 -> 贴回原图
  // 原型掩码内存布局为通道优先 [32][ph][pw] (元素 (k,i,j) 在 k*ph*pw+i*pw+j),
  // 行主序 Mat(n_coef, ph*pw) 恰好逐元素对齐: 行 = 系数通道, 列 = 掩码像素
  const cv::Mat proto_2d(n_coef, ph * pw, CV_32F,
      const_cast<float *>(proto_data));
  std::vector<YoloSegmentDetection> results;
  results.reserve(kept.size());
  for (const auto & c : kept) {
    cv::Mat coef(1, n_coef, CV_32F, const_cast<float *>(c.coefs.data()));
    cv::Mat prob;                        // ph x pw 概率图 (未过 sigmoid)
    cv::gemm(coef, proto_2d, 1.f, cv::Mat(), 0.f, prob);   // 1 x (ph*pw)
    prob = prob.reshape(1, ph);          // 展平向量 -> ph x pw
    // 框内裁剪 (掩码空间), 掩码坐标 = letterbox 坐标 / 下采样比
    const float down = static_cast<float>(config_.input_size) / ph;
    const int mx0 = std::clamp(static_cast<int>(std::floor(c.box_lb.x / down)), 0, pw - 1);
    const int my0 = std::clamp(static_cast<int>(std::floor(c.box_lb.y / down)), 0, ph - 1);
    const int mx1 = std::clamp(static_cast<int>(std::ceil((c.box_lb.x + c.box_lb.width) / down)),
      mx0 + 1, pw);
    const int my1 = std::clamp(static_cast<int>(std::ceil((c.box_lb.y + c.box_lb.height) / down)),
      my0 + 1, ph);
    cv::Mat crop = prob(cv::Rect(mx0, my0, mx1 - mx0, my1 - my0)).clone();
    cv::exp(-crop, crop);                // sigmoid: 1 / (1 + exp(-x))
    crop = 1.f / (1.f + crop);

    // 掩码空间框区域 -> 原图框区域 (宽高按原图框尺寸插值), 再二值化
    cv::Mat up;
    cv::resize(crop, up,
      cv::Size(std::max(1, static_cast<int>(std::round(c.box.width))),
        std::max(1, static_cast<int>(std::round(c.box.height)))), 0, 0, cv::INTER_LINEAR);

    cv::Mat mask = cv::Mat::zeros(bgr.size(), CV_8UC1);
    const int bx = std::clamp(static_cast<int>(std::round(c.box.x)), 0, bgr.cols - 1);
    const int by = std::clamp(static_cast<int>(std::round(c.box.y)), 0, bgr.rows - 1);
    const int bw = std::min(up.cols, bgr.cols - bx);
    const int bh = std::min(up.rows, bgr.rows - by);
    if (bw <= 0 || bh <= 0) {
      continue;
    }
    const cv::Mat roi = mask(cv::Rect(bx, by, bw, bh));
    // threshold 输出与输入同深度 (32F), 先二值化再转 8U 写入掩码 ROI
    cv::Mat bin;
    cv::threshold(up(cv::Rect(0, 0, bw, bh)), bin, config_.mask_thres, 255,
      cv::THRESH_BINARY);
    bin.convertTo(roi, CV_8UC1);

    results.push_back({c.box, mask, c.cls,
        static_cast<int>(c.cls) < static_cast<int>(coco_names().size()) ?
        coco_names()[c.cls] : "class_" + std::to_string(c.cls), c.conf});
  }
  return results;
}

std::string to_annotations_json(
  const std::vector<YoloSegmentDetection> & results, const std::string & algo)
{
  std::ostringstream os;
  os << "{\"algo\":\"" << algo << "\",\"items\":[";
  const double eps = 2.0;   // 轮廓多边形简化容差 (原图像素)
  bool first = true;        // 每实例发 轮廓 x N + bbox + text, 条目数不定, 全局计数分隔
  auto sep = [&os, &first]() {
      if (!first) {
        os << ",";
      }
      first = false;
    };
  for (const auto & d : results) {
    const cv::Scalar c = seg_class_color(d.class_id);
    const int r = static_cast<int>(c[2]);   // BGR -> RGB 给前端
    const int g = static_cast<int>(c[1]);
    const int b = static_cast<int>(c[0]);

    // 掩码轮廓: 取最大的 3 个轮廓, 多边形简化控制点数, 闭合 polyline
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(d.mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::sort(contours.begin(), contours.end(),
      [](const auto & a, const auto & b) {
        return cv::contourArea(a) > cv::contourArea(b);
      });
    int emitted = 0;
    cv::Point label_anchor{-1, -1};   // 最大轮廓最高顶点, 标签锚定其上方
    for (const auto & cont : contours) {
      if (emitted >= 3 || cont.size() < 3) {
        break;
      }
      std::vector<cv::Point> poly;
      cv::approxPolyDP(cont, poly, eps, true);
      if (poly.size() < 3) {
        continue;
      }
      if (emitted == 0) {
        label_anchor = poly[0];
        for (size_t k = 1; k < poly.size(); ++k) {
          if (poly[k].y < label_anchor.y) {
            label_anchor = poly[k];
          }
        }
      }
      sep();
      os << "{\"type\":\"polyline\",\"points\":[";
      for (size_t k = 0; k < poly.size(); ++k) {
        if (k > 0) {
          os << ",";
        }
        os << "[" << num(poly[k].x) << "," << num(poly[k].y) << "]";
      }
      os << "],\"close\":true,\"fill\":true,\"fill_alpha\":0.45,\"color\":["
         << r << "," << g << "," << b << "]}";
      ++emitted;
    }

    // 标签贴着掩码边界最高点 (无轮廓时回退检测框顶部)
    const double tx = label_anchor.x >= 0 ? label_anchor.x : d.box.x;
    const double ty =
      (label_anchor.y >= 0 ? label_anchor.y : d.box.y) - 4.0;
    sep();
    os << "{\"type\":\"text\",\"pos\":[" << num(tx) << "," << num(ty)
       << "],\"text\":\"" << d.name << " "
       << num(d.conf, 2) << "\",\"color\":[" << r << "," << g << "," << b << "]}";
  }
  os << "]}";
  return os.str();
}

}  // namespace flylab_vision
