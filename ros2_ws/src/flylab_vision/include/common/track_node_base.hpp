// Copyright 2026 flylab
#ifndef FLYLAB_VISION__COMMON__TRACK_NODE_BASE_HPP_
#define FLYLAB_VISION__COMMON__TRACK_NODE_BASE_HPP_

#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include <opencv2/core.hpp>

#include "common/display_vision_node.hpp"
#include "common/gcs_topics.hpp"
#include "control/track/gimbal_servo.hpp"
#include "control/track/track_takeover.hpp"

namespace flylab_vision
{

/// 单目标追踪展示节点公共层 —— yolo_track_node / ortrack_node 的共用脚手架。
///
/// 在 DisplayVisionNode (图像输入/启停/标注) 之上, 封装两条追踪链路完全
/// 一致的地面站接口 (话题名可用参数覆盖, 契约见 common/gcs_topics.hpp):
///   1. ROI 框选: topics::TRACK_ROI_TOPIC (latched) 收 {"roi":[x,y,w,h]} /
///      {"reset":true}; 松开框选即请求锁定, begin 落到帧上下文处理
///      (锁定需要当前帧), 追踪中重新框选即换目标
///   2. 指令: topics::CMD_TOPIC 收 track_begin / track_takeover_start[模式] /
///      track_takeover_stop; 反控同时驱动云台伺服 (GimbalServo) 与
///      接管层 (TrackTakeover), 停止反控两者一并复位
///   3. 面板数据: {"src":"track", ...} 发 topics::ALGO_INFO_TOPIC, ~30Hz
///      限频 (对齐地面站 33ms 轮询, 值不变时面板端自动去重); state/宽高/
///      跟随/俯角/距离公共字段在此组装, cls/conf/sim 等差异字段由子类
///      append_info_fields 注入 (基类在 "state", 与 ,"w" 之间留空位)
///
/// 子类差异点:
///   - 三态接口: is_tracking() / stop_tracking() / begin_tracking(bgr)
///     (begin_tracking 调用方保证 has_roi_ 为真)
///   - process(): 各自主循环 (检测+ReID 或 模板匹配); 帧首调
///     consume_begin(bgr), 目标/丢失分支调 servo_follow(box, bgr, has)
///   - append_info_fields(os, force): 面板差异字段
///   - 可选: min_roi_px_ 调整框选最小边长 (默认 1.0)
///
/// 线程约定同基类: 追踪状态在 proc_group_ 单线程访问; on_roi/on_ctl 在
/// ROS 回调线程处理, begin 经 begin_req_ 落到帧上下文。
class TrackNodeBase : public DisplayVisionNode
{
public:
  TrackNodeBase(const std::string & node_name, const Hooks & hooks)
  : DisplayVisionNode(node_name, hooks)
  {
    const auto roi_topic =
      declare_parameter<std::string>("roi_topic", topics::TRACK_ROI_TOPIC);
    const auto ctl_topic =
      declare_parameter<std::string>("ctl_topic", topics::CMD_TOPIC);
    const auto info_topic =
      declare_parameter<std::string>("info_topic", topics::ALGO_INFO_TOPIC);

    // ---------- 通信 (基类之外: ROI 框选 / 开始取消指令 / 面板数据) ----------
    // latched: 地面站先框选后启动节点也能拿到当前框选区域
    roi_sub_ = create_subscription<std_msgs::msg::String>(
      roi_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {on_roi(*msg);});
    ctl_sub_ = create_subscription<std_msgs::msg::String>(
      ctl_topic, 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {on_ctl(*msg);});
    info_pub_ = create_publisher<std_msgs::msg::String>(info_topic, 10);
    servo_ = std::make_unique<GimbalServo>(*this);
    takeover_ = std::make_unique<TrackTakeover>(*this);
  }

protected:
  // ---------- 子类差异点 ----------
  virtual bool is_tracking() const = 0;
  /// 复位内部追踪器与追踪标志 (consume_begin 换目标时调用)
  virtual void stop_tracking() = 0;
  /// 开始追踪 (调用方保证 has_roi_): 用当前帧锁定框选区域内的目标
  virtual void begin_tracking(const cv::Mat & bgr) = 0;
  /// 面板差异字段 (cls/conf/sim[/count]); 前后无逗号要求:
  /// 基类在 "state", 之后与 ,"w" 之前留空位
  virtual void append_info_fields(std::ostringstream & os, bool force) = 0;

  // ---------- 帧上下文公共流程 ----------
  /// begin 请求消费 (process 帧首调用): 追踪中带新框选即换目标
  /// (复位后重新锁定; 锁定需要当前帧, 故不在 ROS 回调里直接做)
  void consume_begin(const cv::Mat & bgr)
  {
    if (!begin_req_) {
      return;
    }
    begin_req_ = false;
    if (has_roi_) {
      if (is_tracking()) {
        stop_tracking();
      }
      begin_tracking(bgr);
    }
  }

  /// 云台伺服 + 反控测量统一入口: 有目标按框中心偏移驱动云台并喂反控
  /// 测量 (视轴角 + 目标像素偏角); 无目标伺服清零 + 停控测量
  void servo_follow(const cv::Rect2f & box, const cv::Mat & bgr, bool has)
  {
    servo_->update(box, bgr, has);
    takeover_->notify_target(servo_->angle_y_deg(), servo_->angle_x_deg(), has);
  }

  /// 锁定成功收尾: ROI 已消费 (画面黄框随之消失, 停止算法后再框选重新开始)
  void clear_roi()
  {
    has_roi_ = false;
    roi_ = cv::Rect2f();
  }

  // ---------- 标注 ----------
  /// 追踪中标注: 单目标框 (锁定青色 / 丢失红色), 画面干净聚焦目标
  std::string target_box_json(
    const cv::Rect2f & box, const std::string & label, bool lost) const
  {
    std::ostringstream os;
    os << "{\"algo\":\"" << algo_group() << "\",\"items\":["
       << "{\"type\":\"bbox\",\"box\":[" << num(box.x) << ","
       << num(box.y) << "," << num(box.width) << ","
       << num(box.height) << "],"
       << "\"label\":\"" << label << "\","
       << (lost ? "\"color\":[255,0,0]}" : "\"color\":[0,255,255]}")
       << "]}";
    return os.str();
  }

  /// 框选区域标注图元 (黄色折线框, 未追踪浏览态回显)
  std::string roi_polyline_item() const
  {
    std::ostringstream os;
    os << "{\"type\":\"polyline\",\"close\":true,\"color\":[255,255,0],"
       << "\"points\":[[" << num(roi_.x) << "," << num(roi_.y) << "],"
       << "[" << num(roi_.x + roi_.width) << "," << num(roi_.y) << "],"
       << "[" << num(roi_.x + roi_.width) << "," << num(roi_.y + roi_.height) << "],"
       << "[" << num(roi_.x) << "," << num(roi_.y + roi_.height) << "]]}";
    return os.str();
  }

  // ---------- 面板数据 ----------
  /// {"src":"track", ...}: ~30Hz 限频, force=True 状态切换立即发
  void publish_info(bool force = false)
  {
    const auto now = std::chrono::steady_clock::now();
    if (!force && now - info_last_ < std::chrono::milliseconds(33)) {
      return;
    }
    info_last_ = now;
    std::ostringstream os;
    os << "{\"src\":\"" << algo_group() << "\","
       << "\"state\":\"" << track_state_ << "\",";
    append_info_fields(os, force);
    os << ",\"w\":" << num(track_box_.width, 0)
       << ",\"h\":" << num(track_box_.height, 0)
       << ",\"follow\":" << (servo_->enabled() ? 1 : 0);
    // 当前俯角测量 (度, 下正; 缺失 = null 面板隐藏"俯角"行)
    const double pitch_meas = takeover_->measure_down_deg();
    if (std::isnan(pitch_meas)) {
      os << ",\"pitch\":null";
    } else {
      os << ",\"pitch\":" << num(pitch_meas);
    }
    // 当前目标距离 (米; 斜距 dist + 水平距 dist_h, 缺失 = null 面板隐藏行)
    double dist_h = 0.0;
    const double dist_meas = takeover_->measure_distance(&dist_h);
    if (std::isnan(dist_meas)) {
      os << ",\"dist\":null,\"dist_h\":null}";
    } else {
      os << ",\"dist\":" << num(dist_meas)
         << ",\"dist_h\":" << num(dist_h) << "}";
    }
    std_msgs::msg::String out;
    out.data = os.str();
    info_pub_->publish(out);
  }

  static std::string num(double v, int prec = 1)
  {
    std::ostringstream os;
    os << std::fixed << std::setprecision(prec) << v;
    return os.str();
  }

  /// 停止时的清屏标注 (两个追踪节点分组一致, 在此统一实现)
  std::string clear_json() const override
  {
    return "{\"algo\":\"" + std::string(algo_group()) + "\",\"items\":[]}";
  }

  // ---------- 可调差异参数与共享状态 (proc_group_ 单线程访问) ----------
  double min_roi_px_ = 1.0;                      // 框选最小边长 (像素)
  std::unique_ptr<GimbalServo> servo_;           // 云台伺服 (目标保持画面中央)
  std::unique_ptr<TrackTakeover> takeover_;      // 反控接管层
  cv::Rect2f roi_;
  bool has_roi_ = false;
  std::string track_state_ = "IDLE";
  cv::Rect2f track_box_;   // 最近目标框 (面板宽高 + 丢失时最后已知位置)

private:
  void on_roi(const std_msgs::msg::String & msg)
  {
    // {"reset":true} 清除框选 (追踪算法停止时地面站发, latched 话题上的
    // 旧 ROI 不再投递给重连的节点)
    if (msg.data.find("\"reset\"") != std::string::npos) {
      if (has_roi_) {
        clear_roi();
        if (!is_tracking()) {
          track_state_ = "IDLE";
          publish_info(true);
        }
        RCLCPP_INFO(get_logger(), "框选区域已清除");
      }
      return;
    }
    // {"roi":[x,y,w,h]} (显示帧像素, 与转发帧同一坐标系); 手写提取数字
    // 数组, 不引 JSON 依赖 (schema 由地面站保证)
    const auto pos = msg.data.find("\"roi\"");
    if (pos == std::string::npos) {
      return;
    }
    size_t i = msg.data.find('[', pos);
    if (i == std::string::npos) {
      return;
    }
    ++i;   // 进入数组
    double v[4] = {0, 0, 0, 0};
    int n = 0;
    while (n < 4 && i < msg.data.size()) {
      const size_t end = msg.data.find_first_of(",]", i);
      if (end == std::string::npos) {
        return;
      }
      try {
        v[n++] = std::stod(msg.data.substr(i, end - i));
      } catch (const std::exception &) {
        return;
      }
      if (msg.data[end] == ']') {
        break;
      }
      i = end + 1;
    }
    if (n != 4 || v[2] < min_roi_px_ || v[3] < min_roi_px_) {
      RCLCPP_WARN(get_logger(), "框选区域无效: %s", msg.data.c_str());
      return;
    }
    roi_ = cv::Rect2f(
      static_cast<float>(v[0]), static_cast<float>(v[1]),
      static_cast<float>(v[2]), static_cast<float>(v[3]));
    has_roi_ = true;
    // 松开即开始: 帧上下文里锁定 (不依赖后续 begin; 追踪中即换目标)
    begin_req_ = true;
    RCLCPP_INFO(
      get_logger(), "收到目标框选区域: (%.0f, %.0f, %.0fx%.0f)",
      roi_.x, roi_.y, roi_.width, roi_.height);
    publish_info(true);
  }

  void on_ctl(const std_msgs::msg::String & msg)
  {
    if (msg.data == "track_begin") {
      // 帧上下文里锁定 (需要当前帧); 追踪中收到且带新框选即换目标
      begin_req_ = true;
    } else if (msg.data.rfind("track_takeover_start", 0) == 0) {
      // 目标追踪反控 (接管控制菜单): 可带模式后缀 "track_takeover_start
      // dive_curve"=曲线打击 / " dive_straight"=直线打击 / " bodyfollow_horiz"=
      // 机体跟随-水平 (保持水平距离) / " bodyfollow_slant"=机体跟随-直线
      // (斜距半径+俯角观察角球面跟随), 不带 = 云台追踪 (悬停跟拍); 五串互不包含,
      // 顺序无关; 先开亦可, 框选锁定目标后伺服自动介入; 已接管时为模式切换
      servo_->set_enabled(true);
      takeover_->activate(
        msg.data.find("bodyfollow_slant") != std::string::npos ?
        TrackTakeover::Mode::BodyfollowSlant :
        msg.data.find("bodyfollow_horiz") != std::string::npos ?
        TrackTakeover::Mode::BodyfollowHoriz :
        msg.data.find("dive_straight") != std::string::npos ?
        TrackTakeover::Mode::DiveStraight :
        msg.data.find("dive_curve") != std::string::npos ?
        TrackTakeover::Mode::DiveCurve : TrackTakeover::Mode::Gimbal);
    } else if (msg.data == "track_takeover_stop") {
      servo_->set_enabled(false);
      takeover_->deactivate("地面站取消");
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr roi_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ctl_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr info_pub_;
  bool begin_req_ = false;   // 框选/track_begin 锁定请求 (帧上下文消费)
  std::chrono::steady_clock::time_point info_last_;
};

}  // namespace flylab_vision

#endif  // FLYLAB_VISION__COMMON__TRACK_NODE_BASE_HPP_
