// Copyright 2026 flylab
/// YOLO 目标检测器实现 —— onnxruntime 推理 + 解码 + NMS + 地面站标注 JSON。
///
/// 模型: Ultralytics ONNX 标准导出 (models/detect/yolo26s.onnx)
///   输入  1x3x640x640 (RGB, /255, letterbox)
///   输出  1x84x8400: 前 4 通道 = cx/cy/w/h (letterbox 输入坐标系, 像素),
///         后 80 通道 = 各类分数; 兼容 1x8400x84 anchor-major 布局
///
/// 为什么用 onnxruntime 而不是 OpenCV DNN: 系统 OpenCV 4.5.x 的 DNN
/// 解析不了 yolo26 attention 里的动态 Split 节点 (已实测), python 侧
/// pip OpenCV 5.0 虽然支持但 C++ 侧不能依赖。库随包发布在
/// thirdparty/onnxruntime/, 安装时拷到 lib/ 由 rpath 解析。
#include "yolo/yolo_detector.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <iomanip>
#include <onnxruntime_cxx_api.h>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include <opencv2/dnn.hpp>       // cv::dnn::NMSBoxes (仅 NMS 工具)
#include <opencv2/imgproc.hpp>

namespace flylab_vision
{

// CUDA EP 的专用注册接口 (GPU 构建的 libonnxruntime 导出; 字符串版
// AppendExecutionProvider("CUDAExecutionProvider") 不认 CUDA, 会抛
// "Unknown provider name", 必须走这个 C 接口)
extern "C" OrtStatus * OrtSessionOptionsAppendExecutionProvider_CUDA(
  OrtSessionOptions * options, int device_id);

namespace
{

/// COCO 80 类
const std::vector<std::string> kCocoNames = {
  "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
  "truck", "boat", "traffic light", "fire hydrant", "stop sign",
  "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
  "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
  "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
  "baseball bat", "baseball glove", "skateboard", "surfboard",
  "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon",
  "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
  "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
  "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
  "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
  "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
  "hair drier", "toothbrush"};

/// 等比缩放 + 灰边填充到 size x size, 返回画布并带回缩放比 / pad
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

std::string num(double v, int prec = 1)
{
  std::ostringstream os;
  os << std::fixed << std::setprecision(prec) << v;
  return os.str();
}

/// letterbox 画布 -> NCHW float blob (RGB, /255)。
/// 行级单次遍历手写平面化 (不依赖 dnn 模块, 无中间分配)
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

/// 输出张量形状归一化: 去掉首尾的 1 维。
/// 兼容 [1,84,8400] / [1,8400,84] / 4 维带尾 1 的导出。
/// channel_major=true 表示类别维在前 (pred[c][a]), 否则 anchor 在前 (pred[a][c])。
struct Layout
{
  int classes = 0;
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
  // 通道维 (类别数+4) 必然小于 anchor 维, 据此判别两种布局
  lay.channel_major = s[0] < s[1];
  lay.classes = static_cast<int>(lay.channel_major ? s[0] : s[1]);
  lay.anchors = static_cast<int>(lay.channel_major ? s[1] : s[0]);
  return lay;
}

}  // namespace

const std::vector<std::string> & coco_names()
{
  return kCocoNames;
}

cv::Scalar class_color(int class_id)
{
  // 色环按 37 取模错开相邻类别, HSV->BGR 得到饱和度较高的可辨识颜色
  const int hue = (class_id * 37) % 180;
  cv::Mat hsv(1, 1, CV_8UC3), bgr;
  hsv.at<cv::Vec3b>(0, 0) = cv::Vec3b(static_cast<uchar>(hue), 235, 255);
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
  const cv::Vec3b v = bgr.at<cv::Vec3b>(0, 0);
  return cv::Scalar(v[0], v[1], v[2]);
}

/// pimpl: onnxruntime 对象全部藏在实现里, 头文件不暴露其依赖
struct YoloDetector::Impl
{
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "yolo_detector"};
  std::unique_ptr<Ort::Session> session;
  Ort::MemoryInfo mem{Ort::MemoryInfo::CreateCpu(
      OrtArenaAllocator, OrtMemTypeDefault)};
  std::string input_name, output_name;
  Layout layout;          // 首次空跑时确定, 之后复用
  bool cuda_ok = false;
  std::string cuda_err;   // CUDA EP 注册失败原因 (成功则为空)
};

YoloDetector::YoloDetector(const YoloConfig & config)
: impl_(std::make_unique<Impl>()), config_(config)   // 顺序须与声明一致 (impl_ 在前)
{
  if (config_.model_path.empty()) {
    throw std::runtime_error("YoloDetector: model_path 为空");
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
  if (impl_->layout.classes < 6) {
    throw std::runtime_error("YoloDetector: 不支持的输出布局");
  }
}

YoloDetector::~YoloDetector() = default;

bool YoloDetector::cuda_active() const
{
  return impl_->cuda_ok;
}

std::string YoloDetector::cuda_error() const
{
  return impl_->cuda_err;
}

std::vector<YoloDetection> YoloDetector::detect(const cv::Mat & bgr)
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
  const int n_class = lay.classes - 4;
  const int n_anchor = lay.anchors;

  // 求 8400 个 anchor 的最优类别。必须沿内存连续方向扫描 (cache 友好):
  // 逐 anchor 跨步访问 channel-major 数据每次跳 33KB, 纯 CPU 解码就要几十 ms
  std::vector<int> best_id(n_anchor, 0);
  std::vector<float> best_score(n_anchor, 0.f);
  if (lay.channel_major) {
    for (int c = 0; c < n_class; ++c) {
      const float * row = data + static_cast<size_t>(4 + c) * n_anchor;
      for (int a = 0; a < n_anchor; ++a) {
        if (row[a] > best_score[a]) {
          best_score[a] = row[a];
          best_id[a] = c;
        }
      }
    }
  } else {
    for (int a = 0; a < n_anchor; ++a) {
      const float * row = data + static_cast<size_t>(a) * lay.classes + 4;
      for (int c = 0; c < n_class; ++c) {
        if (row[c] > best_score[a]) {
          best_score[a] = row[c];
          best_id[a] = c;
        }
      }
    }
  }
  // 坐标通道 (0..3) 取值跨步, 但只对过阈值的少数 anchor 读取
  auto box_at = [&](int c, int a) {
      return lay.channel_major ?
             data[static_cast<size_t>(c) * n_anchor + a] :
             data[static_cast<size_t>(a) * lay.classes + c];
    };

  // 置信度过滤 + letterbox 坐标 -> 原图像素左上角 (x, y, w, h)
  std::vector<cv::Rect2f> boxes;
  std::vector<float> confs;
  std::vector<int> cls_ids;
  for (int a = 0; a < n_anchor; ++a) {
    const int best = best_id[a];
    const float score = best_score[a];
    if (score < config_.conf_thres) {
      continue;
    }
    if (!config_.class_filter.empty() && std::find(
        config_.class_filter.begin(), config_.class_filter.end(), best) ==
      config_.class_filter.end())
    {
      continue;
    }
    const float cx = box_at(0, a), cy = box_at(1, a);
    const float w = box_at(2, a), h = box_at(3, a);
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
    boxes.push_back(box);
    confs.push_back(score);
    cls_ids.push_back(best);
  }
  if (boxes.empty()) {
    return {};
  }

  // 逐类 NMS (同类内抑制, 跨类保留)
  std::vector<YoloDetection> results;
  for (int cid = 0; cid < n_class; ++cid) {
    std::vector<cv::Rect> cls_rects;
    std::vector<float> cls_confs;
    std::vector<size_t> cls_idx;
    for (size_t i = 0; i < boxes.size(); ++i) {
      if (cls_ids[i] != cid) {
        continue;
      }
      const auto & b = boxes[i];
      cls_rects.emplace_back(
        cvRound(b.x), cvRound(b.y), cvRound(b.width), cvRound(b.height));
      cls_confs.push_back(confs[i]);
      cls_idx.push_back(i);
    }
    if (cls_rects.empty()) {
      continue;
    }
    std::vector<int> keep;
    cv::dnn::NMSBoxes(cls_rects, cls_confs, config_.conf_thres,
      config_.iou_thres, keep);
    for (const int k : keep) {
      const size_t i = cls_idx[k];
      results.push_back({boxes[i], cid, kCocoNames[cid], confs[i]});
    }
  }
  return results;
}

std::string to_annotations_json(
  const std::vector<YoloDetection> & results, const std::string & algo)
{
  std::ostringstream os;
  os << "{\"algo\":\"" << algo << "\",\"items\":[";
  for (size_t i = 0; i < results.size(); ++i) {
    const auto & d = results[i];
    if (i > 0) {
      os << ",";
    }
    const cv::Scalar c = class_color(d.class_id);
    os << "{\"type\":\"bbox\",\"box\":[" << num(d.box.x) << ","
       << num(d.box.y) << "," << num(d.box.width) << ","
       << num(d.box.height) << "],"
       << "\"label\":\"" << d.name << " " << num(d.conf, 2) << "\","
       << "\"color\":[" << static_cast<int>(c[0]) << ","
       << static_cast<int>(c[1]) << "," << static_cast<int>(c[2]) << "]}";
  }
  os << "]}";
  return os.str();
}

}  // namespace flylab_vision
