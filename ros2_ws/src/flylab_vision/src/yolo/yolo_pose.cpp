// Copyright 2026 flylab
/// YOLO Pose (人体关键点) 估计器实现 —— onnxruntime 推理 + 解码 + NMS + 标注 JSON。
///
/// 模型: Ultralytics Pose ONNX 标准导出 (models/pose/yolo26s-pose.onnx)
///   输入  1x3x640x640 (RGB, /255, letterbox)
///   输出  1x56x8400: 前 4 通道 = cx/cy/w/h (letterbox 输入坐标系, 像素),
///         第 5 通道 = 类别分数 (COCO-pose 单类 person),
///         后 51 通道 = 17 关键点 x/y/可见性 (letterbox 坐标系);
///         兼容 1x8400x56 anchor-major 布局
///
/// 与 yolo_detector 的差异: 通道尾部是关键点 (x/y/conf 交错), 解码时按
/// (x - pad)/scale 映射回原图; NMS 用水平框即可 (cv::dnn::NMSBoxes 同思路,
/// 这里手写贪心保持与 obb 一致的风格)。
#include "yolo/yolo_pose.hpp"

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

namespace
{

/// COCO 17 人体关键点
const std::vector<std::string> kCocoKptNames = {
  "nose", "left_eye", "right_eye", "left_ear", "right_ear",
  "left_shoulder", "right_shoulder", "left_elbow", "right_elbow",
  "left_wrist", "right_wrist", "left_hip", "right_hip",
  "left_knee", "right_knee", "left_ankle", "right_ankle"};

/// COCO 骨架连接 (Ultralytics 官方 skeleton)
const std::vector<std::pair<int, int>> kCocoSkeleton = {
  {15, 13}, {13, 11}, {16, 14}, {14, 12}, {11, 12},
  {5, 11}, {6, 12}, {5, 6}, {5, 7}, {6, 8}, {7, 9}, {8, 10},
  {1, 2}, {0, 1}, {0, 2}, {1, 3}, {2, 4}, {3, 5}, {4, 6}};

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
  // 通道维 (5+3*nk) 必然小于 anchor 维, 据此判别两种布局
  lay.channel_major = s[0] < s[1];
  lay.channels = static_cast<int>(lay.channel_major ? s[0] : s[1]);
  lay.anchors = static_cast<int>(lay.channel_major ? s[1] : s[0]);
  return lay;
}

/// 两水平框 IoU
float box_iou(const cv::Rect2f & a, const cv::Rect2f & b)
{
  const float ix = std::max(0.f, std::min(a.x + a.width, b.x + b.width) -
    std::max(a.x, b.x));
  const float iy = std::max(0.f, std::min(a.y + a.height, b.y + b.height) -
    std::max(a.y, b.y));
  const float inter = ix * iy;
  const float uni = a.area() + b.area() - inter;
  return uni > 1e-6f ? inter / uni : 0.f;
}

std::string num(double v, int prec = 1)
{
  std::ostringstream os;
  os << std::fixed << std::setprecision(prec) << v;
  return os.str();
}

/// 按类别 id 生成稳定 BGR 颜色 (与 yolo_detector::class_color 同算法, HSV 色环取色)
cv::Scalar pose_class_color(int class_id)
{
  const int hue = (class_id * 37) % 180;
  cv::Mat hsv(1, 1, CV_8UC3), bgr;
  hsv.at<cv::Vec3b>(0, 0) = cv::Vec3b(static_cast<uchar>(hue), 235, 255);
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
  const cv::Vec3b v = bgr.at<cv::Vec3b>(0, 0);
  return cv::Scalar(v[0], v[1], v[2]);
}

}  // namespace

const std::vector<std::string> & coco_kpt_names()
{
  return kCocoKptNames;
}

const std::vector<std::pair<int, int>> & coco_skeleton()
{
  return kCocoSkeleton;
}

/// pimpl: onnxruntime 对象全部藏在实现里, 头文件不暴露其依赖
struct YoloPoseDetector::Impl
{
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "yolo_pose"};
  std::unique_ptr<Ort::Session> session;
  Ort::MemoryInfo mem{Ort::MemoryInfo::CreateCpu(
      OrtArenaAllocator, OrtMemTypeDefault)};
  std::string input_name, output_name;
  Layout layout;          // 首次空跑时确定, 之后复用
  int n_kpt = 0;          // 关键点个数 = (channels - 5) / 3
  bool cuda_ok = false;
  std::string cuda_err;   // CUDA EP 注册失败原因 (成功则为空)
};

YoloPoseDetector::YoloPoseDetector(const YoloPoseConfig & config)
: impl_(std::make_unique<Impl>()), config_(config)   // 顺序须与声明一致 (impl_ 在前)
{
  if (config_.model_path.empty()) {
    throw std::runtime_error("YoloPoseDetector: model_path 为空");
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
  std::vector<float> blob = make_blob(probe);
  const std::array<int64_t, 4> in_shape{
    1, 3, config_.input_size, config_.input_size};
  auto in = Ort::Value::CreateTensor<float>(
    impl_->mem, blob.data(), blob.size(), in_shape.data(), in_shape.size());
  const char * in_names[] = {impl_->input_name.c_str()};
  const char * out_names[] = {impl_->output_name.c_str()};
  auto outs = impl_->session->Run(
    Ort::RunOptions{nullptr}, in_names, &in, 1, out_names, 1);
  impl_->layout = layout_of(
    outs[0].GetTensorTypeAndShapeInfo().GetShape());
  const int tail = impl_->layout.channels - 5;
  if (tail < 3 || tail % 3 != 0) {   // 至少 4 坐标 + 1 类 + 1 关键点 (x/y/conf)
    throw std::runtime_error("YoloPoseDetector: 不支持的输出布局");
  }
  impl_->n_kpt = tail / 3;
}

YoloPoseDetector::~YoloPoseDetector() = default;

int YoloPoseDetector::kpt_count() const
{
  return impl_->n_kpt;
}

bool YoloPoseDetector::cuda_active() const
{
  return impl_->cuda_ok;
}

std::string YoloPoseDetector::cuda_error() const
{
  return impl_->cuda_err;
}

std::vector<YoloPoseDetection> YoloPoseDetector::detect(const cv::Mat & bgr)
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
  const int n_class = 1;                    // pose 模型单类 (COCO person)
  const int n_kpt = impl_->n_kpt;
  const int n_anchor = lay.anchors;

  // 通道取值 (兼容 channel-major / anchor-major 两种布局)
  auto at = [&](int c, int a) {
      return lay.channel_major ?
             data[static_cast<size_t>(c) * n_anchor + a] :
             data[static_cast<size_t>(a) * lay.channels + c];
    };

  // 置信度过滤, 解码为原图坐标系水平框 + 关键点
  struct Cand
  {
    cv::Rect2f box;
    float conf;
    int cls;
    std::vector<cv::Point2f> kpts;
    std::vector<float> kconf;
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
    // letterbox 坐标 -> 原图坐标
    const float cx = (at(0, a) - pad_x) / scale;
    const float cy = (at(1, a) - pad_y) / scale;
    const float w = at(2, a) / scale;
    const float h = at(3, a) / scale;
    if (cx < -w || cy < -h || cx > bgr.cols + w || cy > bgr.rows + h) {
      continue;   // 完全出画
    }
    Cand cand;
    cand.box = cv::Rect2f(cx - w / 2, cy - h / 2, w, h);
    cand.conf = best;
    cand.cls = best_id;
    cand.kpts.reserve(n_kpt);
    cand.kconf.reserve(n_kpt);
    for (int k = 0; k < n_kpt; ++k) {
      const int base = 5 + 3 * k;
      cand.kpts.emplace_back(
        (at(base, a) - pad_x) / scale, (at(base + 1, a) - pad_y) / scale);
      cand.kconf.push_back(at(base + 2, a));
    }
    cands.push_back(std::move(cand));
  }
  if (cands.empty()) {
    return {};
  }

  // 水平框贪心 NMS: 按置信度降序, 与已保留框 IoU 超阈的抑制
  std::sort(cands.begin(), cands.end(),
    [](const Cand & a, const Cand & b) {return a.conf > b.conf;});
  std::vector<YoloPoseDetection> results;
  for (const auto & c : cands) {
    bool suppressed = false;
    for (const auto & r : results) {
      if (box_iou(c.box, r.box) > config_.iou_thres) {
        suppressed = true;
        break;
      }
    }
    if (!suppressed) {
      YoloPoseDetection d{c.box, "person", c.conf, c.kpts, c.kconf};
      // 低于可见性阈值的关键点置信度清零 —— 标注序列化以 >0 判定可见,
      // 阈值统一由 config_.kpt_conf 控制
      for (auto & kc : d.kpt_conf) {
        if (kc < config_.kpt_conf) {
          kc = 0.f;
        }
      }
      results.push_back(std::move(d));
    }
  }
  return results;
}

std::string to_annotations_json(
  const std::vector<YoloPoseDetection> & results, const std::string & algo)
{
  std::ostringstream os;
  os << "{\"algo\":\"" << algo << "\",\"items\":[";
  for (size_t i = 0; i < results.size(); ++i) {
    const auto & d = results[i];
    if (i > 0) {
      os << ",";
    }
    const cv::Scalar c = pose_class_color(i);   // 多人时按序号稳定取色
    const int r = static_cast<int>(c[2]);   // BGR -> RGB 给前端
    const int g = static_cast<int>(c[1]);
    const int b = static_cast<int>(c[0]);
    const int nk = static_cast<int>(d.keypoints.size());

    // 骨架连线 (两端关键点均可见才画, 每段一条不闭合 polyline)
    bool first_item = true;
    for (const auto & seg : kCocoSkeleton) {
      if (seg.first >= nk || seg.second >= nk) {
        continue;
      }
      if (d.kpt_conf[seg.first] <= 0.f || d.kpt_conf[seg.second] <= 0.f) {
        continue;
      }
      if (!first_item) {
        os << ",";
      }
      first_item = false;
      const auto & p1 = d.keypoints[seg.first];
      const auto & p2 = d.keypoints[seg.second];
      os << "{\"type\":\"polyline\",\"points\":["
         << "[" << num(p1.x) << "," << num(p1.y) << "],"
         << "[" << num(p2.x) << "," << num(p2.y) << "]"
         << "],\"close\":false,\"color\":[" << r << "," << g << "," << b << "]}";
    }

    // 可见关键点 (绿色, 不带文字标注避免遮挡)
    for (int k = 0; k < nk; ++k) {
      if (d.kpt_conf[k] <= 0.f) {
        continue;
      }
      if (!first_item) {
        os << ",";
      }
      first_item = false;
      os << "{\"type\":\"point\",\"point\":[" << num(d.keypoints[k].x) << ","
         << num(d.keypoints[k].y) << "],\"color\":[0,255,0]}";
    }
  }
  os << "]}";
  return os.str();
}

}  // namespace flylab_vision
