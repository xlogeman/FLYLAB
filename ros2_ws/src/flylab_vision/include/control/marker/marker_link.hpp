// Copyright 2026 flylab
#ifndef FLYLAB_VISION__CONTROL__ARUCO__MARKER_LINK_HPP_
#define FLYLAB_VISION__CONTROL__ARUCO__MARKER_LINK_HPP_

/// 码检测三节点 (aruco_node / fractal_node / marker_landing_node) 共用的
/// 地面站同步小件 —— 原三节点各写一份的重复在此单源化:
///   MarkerParamSync  四路 latched 运行时参数通道 (码边长/视场角/字典/分形配置)
///   TrackedIdRx      降落跟踪 ID 接收 (检测节点 "MAX 让位" 依据)
///   physical_width   像素宽 → 物理宽换算 (针孔模型, 内参与检测器同源)
///   largest_fields_json  面板 "largest" 摘要对象 (marker/fractal 同构)

#include <cmath>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include "marker/aruco_detector.hpp"
#include "common/gcs_topics.hpp"

namespace flylab_vision
{

/// 地面站运行时参数同步通道: latched 话题的订阅、QoS、非法值门控与去重
/// 在组件内统一, 值实际变化时经 Hooks 回调通知使用方 —— 回调返回 true =
/// 已生效 (组件更新去重状态并记 INFO 日志), false = 拒绝生效 (未知字典/
/// 未标定拒跟 FOV/模式不符), 组件保持旧值不记成功日志, 拒绝原因由回调
/// 自行告警。fov 生效不记日志 (与原实现一致, 原实现静默)。
///
/// 话题名参数 (marker_length_topic / fov_topic / dict_topic /
/// fractal_config_topic) 在组件内声明, 仅对启用通道 (回调非空) 声明;
/// 三节点的参数面与原实现一致 (aruco 无分形配置、fractal 无字典)。
///
/// 线程约定: 回调在订阅所属回调组内串行执行, 去重状态无需加锁
/// (与原实现相同 —— 三节点的同步回调均在默认组)。
class MarkerParamSync
{
public:
  struct Hooks
  {
    std::function<bool(double)> marker_length;    // 码边长 (米, 已过 >0 门控)
    std::function<bool(double)> fov;              // 视场角 (度, 已过 >0 门控)
    std::function<bool(const std::string &)> dict;             // 字典名
    std::function<bool(const std::string &)> fractal_config;   // 分形配置名
  };

  /// marker_length / fov_deg / dict_name / fractal_config 为初始值
  /// (去重状态起点, 来自节点自身的同名值参数; 未启用的通道传空串)
  MarkerParamSync(
    rclcpp::Node & node, const Hooks & hooks, double marker_length,
    double fov_deg, const std::string & dict_name,
    const std::string & fractal_config)
  : node_(node), hooks_(hooks), marker_length_(marker_length),
    fov_deg_(fov_deg), dict_name_(dict_name), fractal_config_(fractal_config)
  {
    // ---------- 地面站码边长同步 ----------
    // 场景含多种尺寸码时, 地面站右栏滑杆运行中同步 marker_length
    // (影响位姿解算尺度/距离显示, 检测不受影响); latched 话题,
    // 后启动的节点一接入即收到最近值
    if (hooks_.marker_length) {
      const auto topic = node_.declare_parameter<std::string>(
        "marker_length_topic", topics::ARUCO_MARKER_LENGTH_TOPIC);
      marker_len_sub_ = node_.create_subscription<std_msgs::msg::Float64>(
        topic, rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          if (msg->data <= 0.0 || msg->data == marker_length_) {
            return;   // 非法值/值未变化跳过 (滑杆拖动重发布降噪)
          }
          if (!hooks_.marker_length(msg->data)) {
            return;
          }
          marker_length_ = msg->data;
          RCLCPP_INFO(
            node_.get_logger(),
            "码边长同步: %.2fm (位姿尺度随之更新, 来自地面站)", msg->data);
        });
    }

    // ---------- 地面站视场角同步 ----------
    // 未标定时内参按 FOV 估算, 地面站调整视场滑杆实时跟随; 是否已标定
    // 由回调判定 (拒绝即保持旧值), 组件只做非法值/去重门控
    if (hooks_.fov) {
      const auto topic = node_.declare_parameter<std::string>(
        "fov_topic", topics::CAMERA_FOV_TOPIC);
      fov_sub_ = node_.create_subscription<std_msgs::msg::Float64>(
        topic, rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          if (msg->data <= 0.0 || msg->data == fov_deg_) {
            return;
          }
          if (hooks_.fov(msg->data)) {
            fov_deg_ = msg->data;
          }
        });
    }

    // ---------- 地面站字典同步 ----------
    // 地面站右栏下拉切换 ArUco 字典即发布, 运行中换字典下一帧生效
    if (hooks_.dict) {
      const auto topic = node_.declare_parameter<std::string>(
        "dict_topic", topics::ARUCO_DICT_TOPIC);
      dict_sub_ = node_.create_subscription<std_msgs::msg::String>(
        topic, rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
          if (msg->data.empty() || msg->data == dict_name_) {
            return;   // 空值/未变化跳过 (下拉重复选择降噪)
          }
          if (!hooks_.dict(msg->data)) {
            return;
          }
          dict_name_ = msg->data;
          RCLCPP_INFO(
            node_.get_logger(),
            "字典同步: %s (下一帧检测生效, 来自地面站)", msg->data.c_str());
        });
    }

    // ---------- 地面站分形配置同步 ----------
    // 运行中换配置重建检测器 (重建在回调内), 下一帧生效
    if (hooks_.fractal_config) {
      const auto topic = node_.declare_parameter<std::string>(
        "fractal_config_topic", topics::FRACTAL_CONFIG_TOPIC);
      config_sub_ = node_.create_subscription<std_msgs::msg::String>(
        topic, rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
          if (msg->data.empty() || msg->data == fractal_config_) {
            return;
          }
          if (!hooks_.fractal_config(msg->data)) {
            return;
          }
          fractal_config_ = msg->data;
          RCLCPP_INFO(
            node_.get_logger(),
            "分形配置同步: %s (下一帧检测生效, 来自地面站)", msg->data.c_str());
        });
    }
  }

  /// 当前生效的码边长 (米) / 视场角 (度) —— 物理宽度换算等处使用
  double marker_length() const {return marker_length_;}
  double fov_deg() const {return fov_deg_;}
  /// 当前生效的字典 / 分形配置 (就绪日志等处使用)
  const std::string & dict_name() const {return dict_name_;}
  const std::string & fractal_config() const {return fractal_config_;}

private:
  rclcpp::Node & node_;
  Hooks hooks_;
  double marker_length_;
  double fov_deg_;
  std::string dict_name_;
  std::string fractal_config_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr marker_len_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr fov_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dict_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr config_sub_;
};

/// 降落跟踪 ID 接收 (aruco_node / fractal_node): 订阅降落节点每帧广播的
/// 跟踪码 ID (Int32, 无跟踪发 -1), 供检测节点 "MAX 让位" —— 广播
/// timeout_sec 内有效, 停发 (降落结束/节点退出) 超时后自动恢复 MAX
class TrackedIdRx
{
public:
  explicit TrackedIdRx(rclcpp::Node & node)
  : node_(node)
  {
    const auto topic = node.declare_parameter<std::string>(
      "tracked_id_topic", topics::LANDING_TRACKED_ID_TOPIC);
    sub_ = node.create_subscription<std_msgs::msg::Int32>(
      topic, 10,
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        id_ = msg->data;
        t_ = node_.now();
      });
  }

  /// 当前有效跟踪 ID; 无广播 / 超时 / 负值返回 -1 (不让位)
  int current(double timeout_sec = 1.0) const
  {
    if (id_ < 0 || (node_.now() - t_).seconds() >= timeout_sec) {
      return -1;
    }
    return id_;
  }

private:
  rclcpp::Node & node_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
  int id_ = -1;                                    // 最新广播值
  rclcpp::Time t_{0, 0, RCL_ROS_TIME};             // 广播到达时刻 (有效期判定)
};

/// 码的实际物理宽度 (米): w_px × Z / fx (针孔模型), 无位姿/宽度无效
/// 返回 -1。fx 与检测器同源: 有标定用标定 K, 否则按当前同步 FOV 估算
inline double physical_width(
  double img_w, bool calibrated, double fx, double fov_deg,
  const Detection & d)
{
  if (!d.has_pose || img_w <= 0.0) {
    return -1.0;
  }
  const double fx_eff = calibrated ? fx :
    img_w / (2.0 * std::tan(fov_deg * CV_PI / 360.0));
  return d.bbox_wh.x * d.tvec[2] / fx_eff;
}

/// 右栏面板 "largest" 摘要对象 (aruco_node / fractal_node 同构, 面板字段
/// 提取复用): id/area/w/h/dx/dy/dist/wm[,"tvec"] —— area=画面占比,
/// w/h=外接框宽高 (原图像素), dx/dy=中心相对画面中心偏移 (归一化,
/// 右/下为正), dist=tvec z 前向距离 (米, 无位姿 -1), wm=实际物理宽度
/// (米), tvec=[x,y,z] 码中心相机系位置 (仅 has_pose 时有, 否则缺省 ——
/// 地面站显示 '— (无位姿)')。返回完整对象 (含花括号)。
inline std::string largest_fields_json(
  double img_w, double img_h, const Detection & d, double wm)
{
  std::ostringstream js;
  const double area = img_w > 0.0 ?
    (d.bbox_wh.x * d.bbox_wh.y) /
    (img_w * (img_h > 0.0 ? img_h : 1.0)) : 0.0;
  js << std::fixed << std::setprecision(3)
     << "{\"id\":" << d.id
     << ",\"area\":" << area
     << ",\"w\":" << static_cast<int>(d.bbox_wh.x + 0.5)
     << ",\"h\":" << static_cast<int>(d.bbox_wh.y + 0.5)
     << ",\"dx\":" << (d.center_norm.x - 0.5)
     << ",\"dy\":" << (d.center_norm.y - 0.5)
     << ",\"dist\":" << (d.has_pose ? d.tvec[2] : -1.0)
     << ",\"wm\":" << wm;
  if (d.has_pose) {
    js << ",\"tvec\":[" << d.tvec[0] << ',' << d.tvec[1] << ','
       << d.tvec[2] << ']';
  }
  js << "}";
  return js.str();
}

}  // namespace flylab_vision

#endif  // FLYLAB_VISION__CONTROL__ARUCO__MARKER_LINK_HPP_
