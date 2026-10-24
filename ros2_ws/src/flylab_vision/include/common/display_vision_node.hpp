// Copyright 2026 flylab
#ifndef FLYLAB_VISION__COMMON__DISPLAY_VISION_NODE_HPP_
#define FLYLAB_VISION__COMMON__DISPLAY_VISION_NODE_HPP_

#include <chrono>
#include <string>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include "common/image_utils.hpp"

namespace flylab_vision
{

/// 接入地面站视觉工具体系的展示型算法节点基类 (flylab_gcs 既有约定)。
///
/// 封装统一链路 (话题名与 flylab_gcs/config.py 保持一致, 均可用参数覆盖):
///   1. 图像输入: 订阅地面站"当前显示的帧" /flylab/display/image_raw
///   2. 工具按钮: 订阅 /flylab/vision_cmd, 收 cmd_start / cmd_stop 启停
///      (常驻运行, 不用每次重启节点); 停止时发 clear_json() 清自己的分组标注
///   3. 画面叠加: 每帧把 process() 返回的 JSON 发往 /flylab/annotations
///      (schema 见 flylab_gcs/overlay.py, 各算法独立分组, 可并存)
///   4. 诊断: 每 2 秒检查收帧情况, 收不到帧主动告警; 首帧图像/首帧推理日志
///
/// 子类职责 (三步接入, 不用再碰任何通信代码):
///   - 构造函数: 声明自己的参数并构建算法, 最后 log_ready()
///   - 实现 process()   —— 逐帧推理 + 序列化成标注 JSON (空串 = 本帧不发)
///   - 实现 clear_json() —— 停止时的清屏标注
///   - 可选重写 log_first_infer() 定制首帧推理日志 (调用时 process() 已执行,
///     子类可把首帧结果暂存成员再打印)
///
/// 注: 基类构造即建好订阅, 子类构造体在其后运行——rclcpp 在 spin() 前不派发
/// 回调, 首帧不会先于模型加载到达, 时序安全。
class DisplayVisionNode : public rclcpp::Node
{
public:
  /// 子类差异点聚合: 指令词 / 标注分组 / 日志文案
  struct Hooks
  {
    const char * cmd_start;       // 启动指令词, 如 "detect_start"
    const char * cmd_stop;        // 停止指令词, 如 "detect_stop"
    const char * algo_group;      // 标注分组名, 如 "yolo" (schema 见 overlay.py)
    const char * start_label;     // 启停日志文案, 如 "检测" -> "检测已启动"
    bool large_payload = false;   // 标注含 base64 大图 (深度/语义) -> 发布 QoS 限深 5
    bool log_first_infer = true;  // 无首帧推理日志诉求 (如 aruco) 时置 false
  };

  DisplayVisionNode(const std::string & node_name, const Hooks & hooks)
  : Node(node_name), hooks_(hooks)
  {
    // ---------- 通用参数 ----------
    image_topic_ = declare_parameter<std::string>(
      "image_topic", "/flylab/display/image_raw");
    const auto cmd_topic =
      declare_parameter<std::string>("cmd_topic", "/flylab/vision_cmd");
    const auto anno_topic =
      declare_parameter<std::string>("annotation_topic", "/flylab/annotations");

    // ---------- 通信 ----------
    // base64 大图标注体积大 (几十 KB), 队列限深防积压旧标注
    anno_pub_ = create_publisher<std_msgs::msg::String>(
      anno_topic, hooks_.large_payload ? rclcpp::QoS(5) : rclcpp::QoS(10));
    cmd_sub_ = create_subscription<std_msgs::msg::String>(
      cmd_topic, 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {on_cmd(*msg);});
    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {on_image(*msg);});

    // 诊断: 每 2 秒检查收帧情况, 收不到帧时主动告警
    stats_timer_ = create_wall_timer(std::chrono::seconds(2), [this]() {
        if (!enabled_) {
          return;
        }
        if (rx_since_log_ == 0) {
          RCLCPP_WARN(
            get_logger(),
            "近 2 秒未收到任何图像帧! 检查: 1) 地面站是否在运行 2) 画面源"
            "是否有帧 3) 图像话题 %s 是否一致",
            image_topic_.c_str());
        }
        rx_since_log_ = 0;
      });
  }

protected:
  /// 处理一帧 (BGR), 返回该帧的标注 JSON; 返回空串则本帧不发
  virtual std::string process(const cv::Mat & bgr, int frame_width) = 0;

  /// 停止时的清屏标注 JSON (发空 items 清掉自己的分组)
  virtual std::string clear_json() const = 0;

  /// 首帧推理完成后的定制日志 (仅 log_first_infer 为 true 时调用一次;
  /// 此前 process() 已执行, 子类可把首帧结果暂存成员再打印)
  virtual void log_first_infer(long ms) {(void)ms;}

  /// 算法加载完成后的就绪日志 (label 如 "YOLO 检测")
  void log_ready(const std::string & label)
  {
    RCLCPP_INFO(
      get_logger(), "%s节点就绪: 订阅 %s, 等待 %s 指令",
      label.c_str(), image_topic_.c_str(), hooks_.cmd_start);
  }

  /// 标注分组名 (传给各算法库的 to_xxx_annotations_json)
  const char * algo_group() const {return hooks_.algo_group;}
  /// 图像输入话题 (子类就绪日志需要时用)
  const std::string & image_topic() const {return image_topic_;}

private:
  void on_cmd(const std_msgs::msg::String & msg)
  {
    if (msg.data == hooks_.cmd_start && !enabled_) {
      enabled_ = true;
      RCLCPP_INFO(get_logger(), "%s已启动", hooks_.start_label);
    } else if (msg.data == hooks_.cmd_stop && enabled_) {
      enabled_ = false;
      std_msgs::msg::String out;
      out.data = clear_json();   // 清自己的分组标注
      anno_pub_->publish(out);
      RCLCPP_INFO(get_logger(), "%s已停止", hooks_.start_label);
    }
  }

  void on_image(const sensor_msgs::msg::Image & msg)
  {
    if (!enabled_) {
      return;
    }
    if (!got_first_frame_) {
      got_first_frame_ = true;
      RCLCPP_INFO(
        get_logger(), "收到首帧图像: %ux%u (%s)",
        msg.width, msg.height, msg.encoding.c_str());
    }
    const cv::Mat bgr = to_bgr(msg);
    if (bgr.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "图像数据不完整或编码不支持 (%s), 跳过", msg.encoding.c_str());
      return;
    }
    const auto t0 = std::chrono::steady_clock::now();
    const std::string json = process(bgr, static_cast<int>(msg.width));
    if (!first_infer_done_) {
      first_infer_done_ = true;
      if (hooks_.log_first_infer) {
        const auto ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - t0).count();
        log_first_infer(static_cast<long>(ms));
      }
    }
    rx_since_log_ += 1;
    if (json.empty()) {
      return;
    }
    std_msgs::msg::String out;
    out.data = json;
    anno_pub_->publish(out);
  }

  const Hooks hooks_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr anno_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  std::string image_topic_;
  uint32_t rx_since_log_ = 0;   // 距上次统计周期收到的帧数
  bool enabled_ = false;
  bool got_first_frame_ = false;
  bool first_infer_done_ = false;
};

}  // namespace flylab_vision

#endif  // FLYLAB_VISION__COMMON__DISPLAY_VISION_NODE_HPP_
