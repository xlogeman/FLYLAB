// Copyright 2026 flylab
#ifndef FLYLAB_VISION__COMMON__DISPLAY_VISION_NODE_HPP_
#define FLYLAB_VISION__COMMON__DISPLAY_VISION_NODE_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
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
    // 收帧/推理双回调组 (须配 MultiThreadedExecutor, 见各节点 main):
    // 推理一帧耗时可达几十 ms, 若与收帧同线程, SHM 传输的 buffer 要等
    // take 才归还, 发布端会被背压阻塞 (实测拖慢地面站抓取线程)。
    // 故收帧回调只缓存最新帧引用立即释放, 推理由 proc 定时器驱动、
    // 永远处理最新帧 (慢了就丢帧, 与图传语义一致)
    img_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    proc_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions img_opts;
    img_opts.callback_group = img_group_;
    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {on_image(msg);},
      img_opts);

    // 推理驱动: 10ms 轮询最新帧 (推理慢时每轮只做一次, 不排队)
    process_timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      [this]() {process_tick();},
      proc_group_);

    // 诊断: 每 2 秒汇报链路帧率 —— 收帧 fps (地面站转发能力) vs 推理 fps +
    // 平均耗时 (本节点消费能力), 据此可判断瓶颈在转发侧还是推理侧
    stats_timer_ = create_wall_timer(std::chrono::seconds(2), [this]() {
        if (!enabled_) {
          rx_since_log_ = 0;
          proc_since_log_ = 0;
          infer_ms_total_ = 0.0;
          tick_since_log_ = 0;
          tick_ms_total_ = 0.0;
          bgr_ms_total_ = 0.0;
          pub_ms_total_ = 0.0;
          return;
        }
        const uint32_t rx = rx_since_log_.exchange(0);
        const uint32_t proc = proc_since_log_.exchange(0);
        const double infer_ms = infer_ms_total_.exchange(0.0);
        if (rx == 0) {
          RCLCPP_WARN(
            get_logger(),
            "近 2 秒未收到任何图像帧! 检查: 1) 地面站是否在运行 2) 画面源"
            "是否有帧 3) 图像话题 %s 是否一致",
            image_topic_.c_str());
          return;
        }
        // 帧龄: 最新缓存帧距现在多久, 持续偏大说明推理消费不过来
        double age_ms = -1.0;
        {
          std::lock_guard<std::mutex> lk(latest_mutex_);
          if (latest_) {
            age_ms = (get_clock()->now() - latest_->header.stamp).seconds() *
              1000.0;
          }
        }
        const uint32_t tick = tick_since_log_.exchange(0);
        const double tick_ms = tick_ms_total_.exchange(0.0);
        const double bgr_ms = bgr_ms_total_.exchange(0.0);
        const double pub_ms = pub_ms_total_.exchange(0.0);
        RCLCPP_INFO(
          get_logger(),
          "链路 2s 统计: 收帧 %u (%.1ffps) / 推理 %u (%.1ffps, %.1fms) / "
          "tick %u (avg %.1fms = 转码 %.1f + 发布 %.1f) / 帧龄 %.0fms",
          rx, rx / 2.0, proc, proc / 2.0,
          proc > 0 ? infer_ms / proc : 0.0,
          tick, tick > 0 ? tick_ms / tick : 0.0,
          tick > 0 ? bgr_ms / tick : 0.0,
          tick > 0 ? pub_ms / tick : 0.0, age_ms);
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

  void on_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!enabled_) {
      return;
    }
    if (!got_first_frame_) {
      got_first_frame_ = true;
      RCLCPP_INFO(
        get_logger(), "收到首帧图像: %ux%u (%s)",
        msg->width, msg->height, msg->encoding.c_str());
    }
    // 只缓存引用 (O(1)), SHM buffer 由 rclcpp take 后立即归还发布端;
    // 真正的转码/推理在 process_tick (独立回调组)
    {
      std::lock_guard<std::mutex> lk(latest_mutex_);
      latest_ = msg;
    }
    rx_since_log_ += 1;
  }

  /// 推理驱动回调 (proc_group_): 取最新帧做转码+推理+发标注。
  /// 每轮只处理一帧, 推理慢时自动丢帧积压, 永远追最新画面
  void process_tick()
  {
    const auto tick_t0 = std::chrono::steady_clock::now();
    sensor_msgs::msg::Image::SharedPtr msg;
    {
      std::lock_guard<std::mutex> lk(latest_mutex_);
      msg = std::move(latest_);   // 取走即清槽, 下一帧到达即可覆盖
    }
    if (!msg || !enabled_) {
      return;
    }
    const auto t_bgr0 = std::chrono::steady_clock::now();
    const cv::Mat bgr = to_bgr(*msg);
    if (bgr.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "图像数据不完整或编码不支持 (%s), 跳过", msg->encoding.c_str());
      return;
    }
    const auto t0 = std::chrono::steady_clock::now();
    const std::string json = process(bgr, static_cast<int>(msg->width));
    {
      const double ms =
        std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
      // 仅 proc_group_ 单回调组写, store 规避 C++17 浮点原子无 fetch_add
      infer_ms_total_.store(infer_ms_total_.load() + ms);
      proc_since_log_ += 1;
    }
    if (!first_infer_done_) {
      first_infer_done_ = true;
      if (hooks_.log_first_infer) {
        const auto ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - t0).count();
        log_first_infer(static_cast<long>(ms));
      }
    }
    const auto t_pub0 = std::chrono::steady_clock::now();
    if (!json.empty()) {
      std_msgs::msg::String out;
      out.data = json;
      anno_pub_->publish(out);
    }
    // tick 分段累计: 周期 (含回调调度等待) / 转码 / 发布 —— 推理吞吐
    // 低于收帧速率时, 差额就在这三段里
    const auto tick_t1 = std::chrono::steady_clock::now();
    tick_since_log_ += 1;
    tick_ms_total_.store(tick_ms_total_.load() +
      std::chrono::duration<double, std::milli>(tick_t1 - tick_t0).count());
    bgr_ms_total_.store(bgr_ms_total_.load() +
      std::chrono::duration<double, std::milli>(t0 - t_bgr0).count());
    pub_ms_total_.store(pub_ms_total_.load() +
      std::chrono::duration<double, std::milli>(tick_t1 - t_pub0).count());
  }

  const Hooks hooks_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr anno_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::CallbackGroup::SharedPtr img_group_;
  rclcpp::CallbackGroup::SharedPtr proc_group_;
  rclcpp::TimerBase::SharedPtr process_timer_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  std::string image_topic_;
  sensor_msgs::msg::Image::SharedPtr latest_;
  std::mutex latest_mutex_;
  std::atomic<uint32_t> rx_since_log_{0};   // 距上次统计周期收到的帧数
  std::atomic<uint32_t> proc_since_log_{0}; // 距上次统计周期实际推理的帧数
  std::atomic<double> infer_ms_total_{0.0}; // 距上次统计周期的推理耗时累计 ms
  std::atomic<uint32_t> tick_since_log_{0}; // 距上次统计周期实际处理的 tick 数
  std::atomic<double> tick_ms_total_{0.0};  // tick 全周期累计 ms (含调度等待)
  std::atomic<double> bgr_ms_total_{0.0};   // to_bgr 转码累计 ms
  std::atomic<double> pub_ms_total_{0.0};   // 标注发布累计 ms
  std::atomic<bool> enabled_{false};
  bool got_first_frame_ = false;
  bool first_infer_done_ = false;
};

}  // namespace flylab_vision

#endif  // FLYLAB_VISION__COMMON__DISPLAY_VISION_NODE_HPP_
