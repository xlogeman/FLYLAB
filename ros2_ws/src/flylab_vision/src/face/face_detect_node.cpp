// Copyright 2026 flylab
/// SCRFD 人脸检测 + FaceXFormer 下游任务节点 (接入地面站视觉工具体系,
/// 启停/收帧/标注通信见 common/display_vision_node.hpp)。
/// 每帧对置信度最高的人脸裁剪 (外扩 facex_margin%) 送 FaceXFormer 跑当前
/// 任务 (none = 仅检测不推理); 任务经 /flylab/face_task 热切换, 下一帧生效。
/// 模型默认 models/face/ 下 scrfd_10g_bnkps.onnx + facexformer.onnx
/// (model / facex_model 参数可覆盖)。
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "common/display_vision_node.hpp"
#include "face/facexformer.hpp"
#include "face/scrfd_detector.hpp"

namespace flylab_vision
{

namespace
{

/// base64 编码 (无外部依赖, 手写查表; parsing/visibility 标注的 PNG 用)
std::string base64_encode(const uint8_t * data, size_t len)
{
  static const char * table =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve((len + 2) / 3 * 4);
  for (size_t i = 0; i < len; i += 3) {
    const uint32_t b0 = data[i];
    const uint32_t b1 = i + 1 < len ? data[i + 1] : 0;
    const uint32_t b2 = i + 2 < len ? data[i + 2] : 0;
    const uint32_t tri = (b0 << 16) | (b1 << 8) | b2;
    out += table[(tri >> 18) & 0x3f];
    out += table[(tri >> 12) & 0x3f];
    out += i + 1 < len ? table[(tri >> 6) & 0x3f] : '=';
    out += i + 2 < len ? table[tri & 0x3f] : '=';
  }
  return out;
}

/// 人像分割 11 类配色 (RGB, 对齐 facexformer/realtime_app.py;
/// 填 BGRA 掩码时反序成 BGR)
constexpr uint8_t kParsingColorsRGB[11][3] = {
  {0, 0, 0}, {0, 153, 255}, {102, 255, 153}, {0, 204, 153},
  {255, 255, 102}, {255, 255, 204}, {255, 153, 0}, {255, 102, 255},
  {102, 0, 51}, {255, 204, 255}, {255, 0, 102},
};

constexpr int kFaceXInput = 224;   // FaceXFormer 网络输入边长 (掩码/坐标基准)

/// 种族 argmax -> 中文 (模型输出序: White/Black/Asian/Indian/Others)
const char * race_zh(int r)
{
  static const char * names[] = {"白人", "黑人", "亚裔", "印度裔", "其他"};
  return r >= 0 && r < 5 ? names[r] : "未知";
}

/// 年龄/性别/种族 -> 面板与画面文字共用的文案
std::string agr_text(const FaceXResult & fx)
{
  std::ostringstream os;
  os << (fx.gender == 1 ? "男" : "女") << " · "
     << fx.age_bin * 10 << "-" << fx.age_bin * 10 + 9 << " 岁 · "
     << race_zh(fx.race);
  return os.str();
}

/// 可见关键点计数 (面板与画面文字共用)
int vis_count(const FaceXResult & fx)
{
  int n = 0;
  for (const bool v : fx.visibility) {
    n += v ? 1 : 0;
  }
  return n;
}

/// CelebA 40 属性中文名 (顺序与模型输出一致; 面板同名表在
/// flylab_gcs/panel_sections.py _CELEBA_ATTRS_ZH, 两边须同步)
constexpr const char * kCelebaAttrsZh[40] = {
  "胡渣", "弓形眉", "有魅力", "眼袋", "秃头", "刘海", "厚唇", "大鼻",
  "黑发", "金发", "画面模糊", "棕发", "浓眉", "圆脸", "双下巴", "眼镜",
  "山羊胡", "灰白头发", "浓妆", "高颧骨", "男性", "微张嘴", "八字胡",
  "细长眼", "无须", "瓜子脸", "苍白肤色", "尖鼻", "发际线后移",
  "红润脸颊", "鬓角", "微笑", "直发", "卷发", "戴耳环", "戴帽",
  "涂口红", "戴项链", "戴领带", "年轻",
};

/// 开启属性 -> 画面文字 (顿号连接; 全无时给占位避免空图元)
std::string attrs_text(const FaceXResult & fx)
{
  std::ostringstream os;
  bool first = true;
  for (int i = 0; i < 40; ++i) {
    if (fx.attribute[i]) {
      os << (first ? "" : "、") << kCelebaAttrsZh[i];
      first = false;
    }
  }
  return first ? std::string("无开启属性") : os.str();
}

}  // namespace

class FaceDetectNode final : public DisplayVisionNode
{
public:
  FaceDetectNode()
  : DisplayVisionNode("face_detect_node",
      {"face_start", "face_stop", "face", "人脸检测", true})
  {
    // ---------- 参数 ----------
    const auto model = declare_parameter<std::string>(
      "model", default_model_path("face", "scrfd_10g_bnkps.onnx"));
    const auto input_size = declare_parameter<int>("input_size", 640);
    const auto conf = declare_parameter<double>("conf", 0.5);
    const auto iou = declare_parameter<double>("iou", 0.4);
    const auto device = declare_parameter<std::string>("device", "cpu");
    const auto info_topic =
      declare_parameter<std::string>("info_topic", "/flylab/aruco_info");
    const auto facex_model = declare_parameter<std::string>(
      "facex_model", default_model_path("face", "facexformer.onnx"));
    const auto task_name =
      declare_parameter<std::string>("facex_task", "none");
    margin_ = declare_parameter<double>("facex_margin", 50.0);
    const auto task_topic =
      declare_parameter<std::string>("face_task_topic", "/flylab/face_task");

    // ---------- SCRFD 检测 (启动即加载, 失败抛异常直接终止暴露配置问题) ----------
    ScrfdConfig cfg;
    cfg.model_path = model;
    cfg.input_size = input_size;
    cfg.conf_thres = static_cast<float>(conf);
    cfg.iou_thres = static_cast<float>(iou);
    cfg.use_cuda = device == "cuda";
    detector_ = std::make_unique<ScrfdDetector>(cfg);
    RCLCPP_INFO(
      get_logger(),
      "检测模型加载完成: %s [%s, input %d, conf %.2f, iou %.2f]",
      model.c_str(), detector_->cuda_active() ? "cuda" : "cpu",
      cfg.input_size, cfg.conf_thres, cfg.iou_thres);
    if (cfg.use_cuda && !detector_->cuda_active()) {
      RCLCPP_WARN(
        get_logger(), "SCRFD CUDA EP 注册失败, 已回退 CPU: %s",
        detector_->cuda_error().c_str());
    }

    // ---------- FaceXFormer 下游任务模型 ----------
    FaceXConfig xcfg;
    xcfg.model_path = facex_model;
    xcfg.use_cuda = device == "cuda";
    facex_ = std::make_unique<FaceXFormer>(xcfg);
    RCLCPP_INFO(
      get_logger(), "任务模型加载完成: %s [%s]",
      facex_model.c_str(), facex_->cuda_active() ? "cuda" : "cpu");
    if (xcfg.use_cuda && !facex_->cuda_active()) {
      RCLCPP_WARN(
        get_logger(), "FaceXFormer CUDA EP 注册失败, 已回退 CPU: %s",
        facex_->cuda_error().c_str());
    }
    task_id_ = facex_task_id(task_name.c_str());
    if (task_id_ < FACE_TASK_NONE) {
      task_id_ = FACE_TASK_NONE;
      RCLCPP_WARN(
        get_logger(), "未知任务名 '%s', 已回退 none (仅检测)", task_name.c_str());
    }

    log_ready("SCRFD 人脸检测 + FaceXFormer 任务");

    // ---------- 通信 (基类之外: 任务切换指令 + 右栏算法信息面板数据) ----------
    task_sub_ = create_subscription<std_msgs::msg::String>(
      task_topic, 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        const int id = facex_task_id(msg->data.c_str());
        if (id < FACE_TASK_NONE) {   // none = -1 合法, 只有真未知名 (< -1) 才拒收
          RCLCPP_WARN(
            get_logger(), "未知人脸任务 '%s', 保持当前任务", msg->data.c_str());
          return;
        }
        task_id_ = id;
        RCLCPP_INFO(get_logger(), "人脸任务切换: %s", facex_task_name(id));
      });
    info_pub_ = create_publisher<std_msgs::msg::String>(info_topic, 10);
  }

protected:
  std::string process(const cv::Mat & bgr, int /*frame_width*/) override
  {
    const auto faces = detector_->detect(bgr);
    last_count_ = faces.size();

    // ---------- 图元流: 人脸框/关键点 + 单脸任务输出 ----------
    std::ostringstream items;
    face_items_json(items, faces);

    const int task = task_id_;          // 帧内取一次, 切换下一帧生效
    FaceXResult fx;
    bool has_fx = false;
    if (task != FACE_TASK_NONE && !faces.empty()) {
      // 置信度最高的一张脸
      const ScrfdFace * best = &faces[0];
      for (const auto & f : faces) {
        if (f.conf > best->conf) {
          best = &f;
        }
      }
      // 外扩 margin% 边距裁剪 (对齐 facexformer realtime_app 的 adjust_bbox),
      // 夹回画幅; 裁剪过小 (远脸/角脸) 则跳过任务推理
      const cv::Rect2f & b = best->box;
      const float dw = static_cast<float>(b.width * margin_ / 200.0);
      const float dh = static_cast<float>(b.height * margin_ / 200.0);
      const float x0 = std::max(0.f, b.x - dw);
      const float y0 = std::max(0.f, b.y - dh);
      const float x1 =
        std::min(static_cast<float>(bgr.cols), b.x + b.width + dw);
      const float y1 =
        std::min(static_cast<float>(bgr.rows), b.y + b.height + dh);
      const cv::Rect crop(
        static_cast<int>(x0), static_cast<int>(y0),
        static_cast<int>(x1 - x0), static_cast<int>(y1 - y0));
      if (crop.width >= 10 && crop.height >= 10) {
        facex_->run(bgr(crop), task, fx);
        has_fx = true;
        append_task_items(items, fx, task, best->box, crop);
      }
    }

    publish_info(faces, static_cast<double>(bgr.cols) * bgr.rows,
      task, fx, has_fx);

    std::ostringstream os;
    os << "{\"algo\":\"" << algo_group() << "\",\"items\":["
       << items.str() << "]}";
    return os.str();
  }

  std::string clear_json() const override
  {
    return std::string("{\"algo\":\"") + algo_group() + "\",\"items\":[]}";
  }

  void log_first_infer(long ms) override
  {
    RCLCPP_INFO(
      get_logger(), "首帧推理耗时 %ldms, 检出 %zu 张人脸, 任务 %s",
      ms, last_count_, facex_task_name(task_id_));
  }

private:
  /// 任务输出 -> 画面叠加图元 (追加进 items 流; schema 见 flylab_gcs/overlay.py)
  /// box = 任务脸的 SCRFD 框 (文字钉框顶用, 比 crop 顶贴近脸)
  void append_task_items(
    std::ostringstream & os, const FaceXResult & fx, int task,
    const cv::Rect2f & box, const cv::Rect & crop)
  {
    const float cw = static_cast<float>(crop.width);
    const float ch = static_cast<float>(crop.height);
    // 68 点归一化 [-1,1] -> 裁剪空间 -> 原帧坐标 (224 归一化与裁剪缩放
    // 两次换算可合并成 (p+1)*cw/2)
    auto to_frame = [&](float px, float py) {
        return cv::Point2f(
          crop.x + (px + 1.f) * cw / 2.f, crop.y + (py + 1.f) * ch / 2.f);
      };

    switch (task) {
      case FACE_TASK_LANDMARKS: {
          for (const auto & p : fx.landmark) {
            const cv::Point2f q = to_frame(p[0], p[1]);
            os << ",{\"type\":\"point\",\"point\":["
               << num(q.x) << "," << num(q.y) << "],"
               << "\"color\":[255,255,0]}";
          }
          break;
        }
      case FACE_TASK_HEADPOSE: {
          // 三姿态轴 (对齐 facexformer realtime_app: 轴心 = 裁剪中心,
          // 轴长 = 裁剪宽 1/4; 标注颜色约定为 RGB, X 红 / Y 绿 / Z 蓝)
          const float pitch = fx.headpose[0], yaw = fx.headpose[1],
            roll = fx.headpose[2];
          const float cx = crop.x + cw / 2.f, cy = crop.y + ch / 2.f;
          const float size = cw / 4.f;
          const float dx[3] = {
            size * (std::cos(yaw) * std::cos(roll)),
            size * (-std::cos(yaw) * std::sin(roll)),
            size * std::sin(yaw)};
          const float dy[3] = {
            size * (std::cos(pitch) * std::sin(roll) +
            std::cos(roll) * std::sin(pitch) * std::sin(yaw)),
            size * (std::cos(pitch) * std::cos(roll) -
            std::sin(pitch) * std::sin(yaw) * std::sin(roll)),
            size * (-std::cos(yaw) * std::sin(pitch))};
          const char * colors[3] = {"[255,0,0]", "[0,255,0]", "[0,0,255]"};
          for (int i = 0; i < 3; ++i) {
            os << ",{\"type\":\"polyline\",\"points\":[["
               << num(cx) << "," << num(cy) << "],["
               << num(cx + dx[i]) << "," << num(cy - dy[i]) << "]],"
               << "\"color\":" << colors[i] << "}";
          }
          break;
        }
      case FACE_TASK_PARSING: {
          // 类别掩码 -> BGRA (非背景类半透明, 背景全透明) PNG base64,
          // image 图元铺回裁剪区域 (alpha 逐像素, 不用整图 alpha)
          cv::Mat bgra(kFaceXInput, kFaceXInput, CV_8UC4);
          for (int y = 0; y < kFaceXInput; ++y) {
            for (int x = 0; x < kFaceXInput; ++x) {
              const int cls = fx.seg_mask.at<uchar>(y, x);
              auto & px = bgra.at<cv::Vec4b>(y, x);
              px = {
                kParsingColorsRGB[cls][2], kParsingColorsRGB[cls][1],
                kParsingColorsRGB[cls][0],
                static_cast<uint8_t>(cls == 0 ? 0 : 128)};
            }
          }
          std::vector<uchar> png;
          if (!cv::imencode(".png", bgra, png)) {
            break;
          }
          os << ",{\"type\":\"image\",\"pos\":["
             << num(static_cast<float>(crop.x)) << ","
             << num(static_cast<float>(crop.y)) << "],"
             << "\"w\":" << num(cw) << ",\"h\":" << num(ch) << ",\"data\":\""
             << base64_encode(png.data(), png.size()) << "\"}";
          break;
        }
      case FACE_TASK_AGE_GENDER_RACE:
      case FACE_TASK_ATTRIBUTES: {
          // 结果文字钉在 SCRFD 框顶 (贴脸; 用 crop 顶会高出 25% 框高外扩)
          os << ",{\"type\":\"text\",\"pos\":["
             << num(static_cast<float>(box.x)) << ","
             << num(std::max(0.f, static_cast<float>(box.y) - 8.f)) << "],"
             << "\"text\":\"" << (task == FACE_TASK_AGE_GENDER_RACE ?
              agr_text(fx) : attrs_text(fx))
             << "\",\"color\":[255,255,255]}";
          break;
        }
      default:   // attributes/visibility 只进面板, 无画面叠加
        break;
    }
  }

  /// 面板数据 {"src":"face", "task":.., "count":N, "items":[...]}: ~30Hz
  /// 限频; items = 按置信度降序的前 3 张脸, items[0] 额外携带任务结果
  /// ("result" 文本, attributes 传 "attrs" 开启索引表由面板映射中文)
  void publish_info(
    const std::vector<ScrfdFace> & faces, double frame_area, int task,
    const FaceXResult & fx, bool has_fx)
  {
    const auto now = std::chrono::steady_clock::now();
    if (now - info_last_ < std::chrono::milliseconds(33)) {
      return;
    }
    info_last_ = now;
    std::vector<const ScrfdFace *> sorted;
    sorted.reserve(faces.size());
    for (const auto & f : faces) {
      sorted.push_back(&f);
    }
    std::sort(sorted.begin(), sorted.end(),
      [](const ScrfdFace * a, const ScrfdFace * b) {
        return a->conf > b->conf;
      });
    const size_t n = std::min<size_t>(sorted.size(), 3);
    std::ostringstream os;
    // count = 全部检出数 (可能 > 3, 不能用 len(items) 代替, 面板汇总行用)
    os << "{\"src\":\"face\",\"count\":" << faces.size()
       << ",\"task\":\"" << facex_task_name(task) << "\",\"items\":[";
    for (size_t i = 0; i < n; ++i) {
      const auto & f = *sorted[i];
      float area_pct = 0.f;
      if (frame_area > 0) {
        area_pct = static_cast<float>(100.0 * f.box.area() / frame_area);
      }
      if (i > 0) {
        os << ',';
      }
      os << "{\"conf\":" << num(f.conf, 2) << ","
         << "\"x\":" << num(f.box.x + f.box.width / 2.f, 0) << ","
         << "\"y\":" << num(f.box.y + f.box.height / 2.f, 0) << ","
         << "\"w\":" << num(f.box.width, 0) << ","
         << "\"h\":" << num(f.box.height, 0) << ","
         << "\"area\":" << num(area_pct, 1);
      if (i == 0 && has_fx) {
        os << "," << task_result_json(fx, task);
      }
      os << "}";
    }
    os << "]}";
    std_msgs::msg::String out;
    out.data = os.str();
    info_pub_->publish(out);
  }

  /// 最高置信度脸的任务结果 -> 面板 JSON 片段 (不加大括号)
  std::string task_result_json(const FaceXResult & fx, int task)
  {
    std::ostringstream os;
    switch (task) {
      case FACE_TASK_PARSING: {
          int present[11] = {0};   // 出现的非背景类数
          for (int y = 0; y < kFaceXInput; ++y) {
            for (int x = 0; x < kFaceXInput; ++x) {
              present[fx.seg_mask.at<uchar>(y, x)] = 1;
            }
          }
          int cnt = 0;
          for (const int p : present) {
            cnt += p;
          }
          os << "\"result\":\"分割 " << cnt << " 类\"";
          break;
        }
      case FACE_TASK_LANDMARKS:
        os << "\"result\":\"68 关键点\"";
        break;
      case FACE_TASK_HEADPOSE:
        os << "\"result\":\"P" << num(deg(fx.headpose[0])) << "° Y"
           << num(deg(fx.headpose[1])) << "° R" << num(deg(fx.headpose[2]))
           << "°\"";
        break;
      case FACE_TASK_ATTRIBUTES: {
          os << "\"attrs\":[";
          bool first = true;
          for (int i = 0; i < 40; ++i) {
            if (fx.attribute[i]) {
              os << (first ? "" : ",") << i;
              first = false;
            }
          }
          os << "]";
          break;
        }
      case FACE_TASK_AGE_GENDER_RACE:
        os << "\"result\":\"" << agr_text(fx) << "\"";
        break;
      case FACE_TASK_VISIBILITY:
        os << "\"result\":\"" << vis_count(fx) << "/29 可见\"";
        break;
      default:
        break;
    }
    return os.str();
  }

  static double deg(double rad)
  {
    return rad * 180.0 / 3.14159265358979323846;
  }

  static std::string num(double v, int prec = 1)
  {
    std::ostringstream os;
    os << std::fixed << std::setprecision(prec) << v;
    return os.str();
  }

  std::unique_ptr<ScrfdDetector> detector_;
  std::unique_ptr<FaceXFormer> facex_;
  double margin_ = 50.0;                 // 裁剪外扩边距 (%)
  std::atomic<int> task_id_{FACE_TASK_NONE};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_sub_;
  size_t last_count_ = 0;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr info_pub_;
  std::chrono::steady_clock::time_point info_last_;
};

}  // namespace flylab_vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<flylab_vision::FaceDetectNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
