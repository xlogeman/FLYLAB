// Copyright 2026 flylab
#ifndef FLYLAB_VISION__CONTROL__TRACK__TRACK_TAKEOVER_HPP_
#define FLYLAB_VISION__CONTROL__TRACK__TRACK_TAKEOVER_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include "control/common/gimbal_follow.hpp"
#include "control/px4_offboard_client.hpp"

namespace flylab_vision
{

/// 目标追踪反控接管层 (yolo_track / ortrack 共用): 接管 offboard 拿到飞机
/// 控制权, 按模式二选一:
///   云台追踪 (Gimbal): 原地悬停 + 机头跟随 (视线方位收敛), 云台伺服
///     (GimbalServo) 继续由帧上下文驱动让追踪目标保持在画面中央, 即
///     "悬停跟拍"形态; 地面站设置了俯视角度 (/flylab/track_pitch) 时再
///     叠加高度环: 以"视轴俯角 (gimbal_state) + 目标像素偏角"为测量,
///     P 律控制飞机升降收敛到设定俯视角
///   曲线打击 (Dive): 纯角度制导两相制导 —— 沿"视轴俯角 + 目标像素偏角"
///     合成的目标方向定速撞击 (见 dive_velocity), 机头跟随收口方位;
///   直线打击 (Straight): 全程沿视线飞, 不分相 —— 合速度 = min(V, D/sinθ)
///     使垂直分量恒 ≤ 垂直域 D, 航迹始终贴合视线方向 (θ 在锁定时冻结,
///     近似直线收口; 陡角自动降速是直线打击的固有代价 —— 曲线打击用
///     拉角段绕弯换末段全速); 目标方向缺失超过 dive_lost_timeout_sec
///     均交还
///   机体跟随 (Bodyfollow): 保持设定水平距离跟随目标水平移动, 高度环
///     维持俯视角度 (见 bodyfollow_velocity);
///   机体跟随-直线 (BodyfollowSlant): 以"跟随距离"为斜距半径 d、
///     "俯视角度"为观察角度的双环解耦跟随 —— 高度环维持俯视角, 水平环
///     保持 L = d·cos(p), 调整观察角度时飞机沿半径球面滑动到新观察点
///     (俯角未设 = 保持当前角度只锁半径), 摆脱贴视线尾随形态
///
/// 与 aruco_landing_node 同一套组件与流程 (px4_offboard_client, offboard
/// 单写者原则 —— 接管期间偏航指令由本节点注入, 地面站遥控必须退出):
///   激活: takeoverReset → 定时 tick 跑接管状态机 (心跳预热 → 周期请求 →
///         确认), ENGAGED 后持续 sendVelocityFRDYawRate 维持 offboard
///         心跳 (云台追踪: 速度=悬停/高度环升降; 俯冲打击: 速度=沿目标
///         方向定速; 机体跟随: 保持设定水平距离收放; yaw 均为机头跟随);
///         已激活时 activate 即模式切换
///   交还: requestLoiter 切回 AUTO.LOITER; 上锁 / OFFBOARD 被外部切换 /
///         图像断流 / 接管超时 自动交还并回 idle
/// 机头跟随: 经 GimbalFollow 组件 (control/common/gimbal_follow.hpp, 通道
/// 约定同 aruco_landing) 请求相机侧反卷 + 订阅误差, 按误差 P 律注入偏航
/// 角速率; 可经 /flylab/track_follow
/// (latched Bool, 地面站云台面板"跟随"按钮) 开关 —— 关闭时机头保持
/// 航向仅注入零偏航, 云台伺服 (相机侧反卷) 不受影响, 目标仍保持画面
/// 中央, 云台偏航被 ±45° 限位收口。
/// 状态回传 /flylab/track_status (engaged/idle/failed), 地面站据此联动按钮。
///
/// 线程约定: activate/deactivate 在指令回调 (默认组), tick 在定时器
/// (默认组) —— 同组串行, 状态无需加锁。
class TrackTakeover
{
public:
  /// 反控模式 (vision_cmd track_takeover_start [mode], 字符串与
  /// 地面站 config TRACK_TAKEOVER_* 一致)
  enum class Mode
  {
    Gimbal,          // 云台追踪: 悬停跟拍 (悬停/高度环 + 机头跟随, 无后缀指令)
    DiveCurve,       // 曲线打击: 两相制导沿视线撞击 (拉角段收敛到撞击带)
    DiveStraight,    // 直线打击: 全程沿视线飞 (合速度 min(V, D/sinθ), 不分相)
    BodyfollowHoriz, // 机体跟随 (水平): 保持设定水平距离跟随目标移动
    BodyfollowSlant  // 机体跟随 (直线): 斜距半径 + 俯角观察角球面跟随
  };
  static const char * mode_name(Mode m)
  {
    return m == Mode::DiveCurve ? "曲线打击" :
      m == Mode::DiveStraight ? "直线打击" :
      m == Mode::BodyfollowSlant ? "直线跟随" :
      m == Mode::BodyfollowHoriz ? "水平跟随" : "云台追踪";
  }

  explicit TrackTakeover(rclcpp::Node & node)
  : node_(node)
  {
    // ---------- 参数 ----------
    const auto link = node.declare_parameter<std::string>("link", "mavros");
    const auto control_hz = node.declare_parameter<double>("takeover_hz", 20.0);
    const auto warmup =
      node.declare_parameter<double>("offboard_warmup_sec", 1.0);
    const auto retry =
      node.declare_parameter<double>("offboard_retry_sec", 10.0);
    const auto follow_kp = node.declare_parameter<double>("follow_kp", 1.2);
    const auto follow_rate_max = node.declare_parameter<double>(
      "follow_yaw_rate_max_deg", 45.0);
    const auto follow_deadband = node.declare_parameter<double>(
      "follow_deadband_deg", 2.0);
    const auto follow_slow = node.declare_parameter<double>(
      "follow_slow_deg", 30.0);
    frame_timeout_sec_ = node.declare_parameter<double>("frame_timeout_sec", 2.0);
    warmup_sec_ = warmup;
    retry_sec_ = retry;
    follow_kp_ = follow_kp;
    follow_rate_max_ = follow_rate_max;
    follow_deadband_deg_ = follow_deadband;
    follow_slow_deg_ = follow_slow;

    // ---------- PX4 执行层 (复用组件, 链路按 link 参数选择) ----------
    if (link == "mavros") {
      px4_ = std::make_unique<Px4OffboardClientMavros>(node);
      RCLCPP_INFO(
        node.get_logger(),
        "反控 PX4 执行链路: MAVROS (/mavros/*, fcu_url:=udp://:14542@ 专属实例)");
    } else if (link == "uxrce") {
      px4_ = std::make_unique<Px4OffboardClientUxrce>(node);
      RCLCPP_INFO(
        node.get_logger(),
        "反控 PX4 执行链路: uXRCE-DDS (/fmu/*, 需 MicroXRCEAgent udp4 -p 8888)");
    } else {
      RCLCPP_FATAL(
        node.get_logger(), "未知 link 参数 '%s' (可选 mavros|uxrce)",
        link.c_str());
      throw std::invalid_argument("link must be mavros or uxrce");
    }

    // ---------- 地面站通道 (状态回传 / 机头跟随, 约定同 aruco_landing) ----------
    const auto status_topic = node.declare_parameter<std::string>(
      "status_topic", "/flylab/track_status");
    status_pub_ =
      node.create_publisher<std_msgs::msg::String>(status_topic, 10);
    // 机头跟随通道 (复用组件: 请求 0.5s 保活 + 误差 latched 订阅)
    const auto follow_req_topic = node.declare_parameter<std::string>(
      "follow_req_topic", "/flylab/gimbal_follow");
    const auto follow_err_topic = node.declare_parameter<std::string>(
      "follow_err_topic", "/flylab/gimbal_yaw_err");
    follow_ = std::make_unique<GimbalFollow>(node, follow_req_topic, follow_err_topic);
    // 机头跟随开关 (地面站云台面板"跟随"按钮, latched): 默认开 —— 与
    // 地面站未发布时的旧行为一致; 关闭仅停偏航注入, 云台伺服照常
    const auto follow_switch_topic = node.declare_parameter<std::string>(
      "follow_switch_topic", "/flylab/track_follow");
    follow_switch_sub_ = node.create_subscription<std_msgs::msg::Bool>(
      follow_switch_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (nose_follow_ == msg->data) {
          return;
        }
        nose_follow_ = msg->data;
        RCLCPP_INFO(
          node_.get_logger(), "机头跟随: %s",
          nose_follow_ ? "\033[32m开启 (机头转向视线方向)\033[0m"
                       : "\033[33m关闭 (机头保持航向, 仅云台伺服跟目标)\033[0m");
      });

    // ---------- 高度环 (俯视角度, 地面站"俯视角度"输入框) ----------
    const auto alt_kp = node.declare_parameter<double>("alt_kp_deg", 0.08);
    const auto alt_rate_max =
      node.declare_parameter<double>("alt_rate_max_ms", 2.0);
    const auto alt_deadband = node.declare_parameter<double>(
      "alt_deadband_deg", 2.0);
    const auto alt_slow =
      node.declare_parameter<double>("alt_slow_deg", 5.0);
    alt_kp_ = alt_kp;
    alt_rate_max_ = alt_rate_max;
    alt_deadband_deg_ = alt_deadband;
    alt_slow_deg_ = alt_slow;
    // 俯冲打击 (纯角度制导): 撞击速度/垂直域/拉角增益/方向缺失容忍
    dive_speed_ms_ = node.declare_parameter<double>("dive_speed_ms", 12.0);
    dive_down_max_ms_ = node.declare_parameter<double>(
      "dive_down_max_ms", 2.5);   // 须 ≤ PX4 MPC_Z_VEL_MAX_DN (机型文件当前
                                  // 2.5), 超了被飞控钳位导致航迹角压平;
                                  // 撞击带 φ_max = asin(D/V), 想要 45° 带
                                  // (D=V·sin45°=4.25) 需运行时把飞控该参数
                                  // 提上去后再设本参数
    dive_plunge_gain_ = node.declare_parameter<double>(
      "dive_plunge_gain", 0.7);   // 拉角段水平收敛比 (须 <1, 见 dive_velocity)
    dive_lost_timeout_sec_ = node.declare_parameter<double>(
      "dive_lost_timeout_sec", 1.0);
    // 拉角段稳定四闸 (2026-11-25 02_57_39 俯冲自激振荡修复): 该机气动可达
    // 下降速度仅 ~2.8m/s (俯冲入段推力即贴 THR_MIN=0.12, ulog 实测), 撞击带
    // φ_max≈13°, 拉角段就是主要工作段 —— 指令反演 cotθ 对 θ 测量抖动极敏感,
    // 需钳制/滤波/迟滞/限斜率, 防 θ→0 指令发散 (曾输出 62~477m/s) 与 θ 穿零
    // 变负误判"目标在头顶"输出急刹+爬升
    dive_theta_min_deg_ = node.declare_parameter<double>(
      "dive_theta_min_deg", 3.0);   // 视线俯角下限钳制 (度)
    dive_theta_filter_tc_ = node.declare_parameter<double>(
      "dive_theta_filter_tc", 0.4); // 视线角一阶低通时间常数 (s, 0=不过滤)
    dive_band_hysteresis_deg_ = node.declare_parameter<double>(
      "dive_band_hysteresis_deg", 3.0);   // 撞击带段切换迟滞角 (度)
    dive_vh_accel_ms2_ = node.declare_parameter<double>(
      "dive_vh_accel_ms2", 3.0);    // 水平指令限加速度 (m/s²)
    // 机体跟随 (水平跟随: 水平距离 P 律 + 视线方位快闭环; 视线滤波/
    // 限加速度/丢失超时复用俯冲状态)
    bodyfollow_dist_m_ = node.declare_parameter<double>(
      "bodyfollow_dist_m", 10.0);   // 保持水平距离默认值 (m, 运行中经
                                    // /flylab/track_dist 同步覆盖)
    bodyfollow_kp_ = node.declare_parameter<double>(
      "bodyfollow_kp", 0.5);        // 距离环 P 增益 (m/s 每 m)
    bodyfollow_speed_max_ms_ = node.declare_parameter<double>(
      "bodyfollow_speed_max_ms", 4.0);   // 水平速度限幅 (m/s)
    bodyfollow_deadband_m_ = node.declare_parameter<double>(
      "bodyfollow_deadband_m", 1.0);     // 距离死区 (m, 回差 1.5×)
    bodyfollow_slow_m_ = node.declare_parameter<double>(
      "bodyfollow_slow_m", 5.0);         // 中心减速段宽度 (m)
    bodyfollow_lost_timeout_sec_ = node.declare_parameter<double>(
      "bodyfollow_lost_timeout_sec", 5.0);   // 曾跟住后持续丢失交还超时 (s)
    // 俯视角度目标 (latched): 度, 向下正; 未动过不发布 = NaN 不启用
    // (维持悬停)。接管中修改立即生效, 未接管先设置则接管时生效
    const auto pitch_target_topic = node.declare_parameter<std::string>(
      "pitch_target_topic", "/flylab/track_pitch");
    pitch_target_sub_ = node.create_subscription<std_msgs::msg::Float64>(
      pitch_target_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        const double v = msg->data;
        if (v < 5.0 || v > 85.0) {
          return;   // 越界拒绝, 保持原值 (地面站侧同样限幅)
        }
        std::lock_guard<std::mutex> lk(mutex_);
        if (v != pitch_target_) {
          pitch_target_ = v;
        }
      });
    // 跟随距离目标 (latched, 地面站"跟随距离"输入框): 米, 机体跟随-水平
    // 时距离环保持的水平距离; 直线跟随时同一输入框语义为斜距半径 (节点
    // 经 cos(俯角) 折算成水平距目标, 值域共用); 未动过不发布 = 用
    // bodyfollow_dist_m 参数默认值 (与俯视角度同步同一约定)。接管中
    // 修改立即生效
    const auto dist_target_topic = node.declare_parameter<std::string>(
      "dist_target_topic", "/flylab/track_dist");
    dist_target_sub_ = node.create_subscription<std_msgs::msg::Float64>(
      dist_target_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        const double v = msg->data;
        if (v < 3.0 || v > 100.0) {
          return;   // 越界拒绝, 保持原值 (地面站侧同样限幅)
        }
        std::lock_guard<std::mutex> lk(mutex_);
        if (v != bodyfollow_dist_m_) {
          bodyfollow_dist_m_ = v;
          RCLCPP_INFO(
            node_.get_logger(), "跟随距离目标: %.1f m", v);
        }
      });
    // 云台姿态 (地面站实际下发的相机机体系位姿, 10Hz): 视轴转到机体 FRD
    // 取俯角作测量基础 (悬停机体俯仰近 0, 残差由死区 + 像素偏角吸收)
    const auto gimbal_state_topic = node.declare_parameter<std::string>(
      "gimbal_state_topic", "/flylab/gimbal_state");
    gimbal_sub_ = node.create_subscription<geometry_msgs::msg::Quaternion>(
      gimbal_state_topic, 10,
      [this](const geometry_msgs::msg::Quaternion::SharedPtr msg) {
        const double x = msg->x, y = msg->y, z = msg->z, w = msg->w;
        // 视轴 (相机 x 轴, 机体系): 单位四元数主动旋转
        const double vx = 1.0 - 2.0 * (y * y + z * z);
        const double vy = 2.0 * (w * z + x * y);
        const double vz = 2.0 * (x * z - w * y);
        const double down =
          std::atan2(vz, std::hypot(vx, vy)) * 180.0 / M_PI;
        const double az = std::atan2(vy, vx) * 180.0 / M_PI;   // 右+
        std::lock_guard<std::mutex> lk(mutex_);
        boresight_down_deg_ = down;
        boresight_az_deg_ = az;
        gimbal_t_ = std::chrono::steady_clock::now();
      });

    // ---------- 控制主循环 (固定频率, 不随图像帧率波动) ----------
    dt_ = 1.0 / std::max(1.0, control_hz);
    timer_ = node.create_wall_timer(
      std::chrono::duration<double>(dt_),
      [this] {tick();});
  }

  /// 激活反控 (vision_cmd track_takeover_start [mode]): 幂等; 未激活时
  /// 完整接管, 已激活时仅切换模式 (云台追踪↔俯冲打击↔机体跟随, 接管
  /// 状态不断)
  void activate(Mode mode)
  {
    if (active_) {
      if (mode_ == mode) {
        return;
      }
      mode_ = mode;
      {   // 视线/距离状态复位 (俯冲/跟随共用): 丢失超时窗口从切换时刻
          // 重新起算
        std::lock_guard<std::mutex> lk(mutex_);
        reset_los_locked();
      }
      RCLCPP_INFO(
        node_.get_logger(), "\033[32m目标追踪反控切换模式: %s\033[0m",
        mode_name(mode));
      return;
    }
    active_ = true;
    mode_ = mode;
    ever_engaged_ = false;
    {   // 死区迟滞复位 (重新收敛; 与 alt_vz 同一把锁, 纪律一致)
      std::lock_guard<std::mutex> lk(mutex_);
      alt_in_deadband_ = false;
      reset_los_locked();
    }
    px4_->takeoverReset(node_.now());
    notify_frame();
    RCLCPP_INFO(
      node_.get_logger(),
      "\033[32m目标追踪反控 (%s): 接管 offboard, 预热 %.1fs 后切模式\033[0m",
      mode_name(mode), warmup_sec_);
  }

  /// 交还反控 (vision_cmd track_takeover_stop / 异常自愈): 幂等
  void deactivate(const char * reason)
  {
    if (!active_) {
      return;
    }
    active_ = false;
    if (ever_engaged_ && px4_->armed() && px4_->isOffboard()) {
      RCLCPP_INFO(
        node_.get_logger(), "目标追踪反控结束 (%s), 切回 AUTO_LOITER 交还控制权",
        reason);
      px4_->requestLoiter();
    } else {
      RCLCPP_INFO(node_.get_logger(), "目标追踪反控结束 (%s)", reason);
    }
    ever_engaged_ = false;
    follow_->request(false);
    publish_status("idle");
  }

  /// 帧上下文每帧调用 (process 开头): 喂图像断流看门狗
  void notify_frame()
  {
    frame_t_ = std::chrono::steady_clock::now();
  }

  /// 帧上下文每帧调用 (process 里伺服 update 之后): 喂目标像素俯仰/方位
  /// 偏角 (GimbalServo::angle_y_deg/angle_x_deg) 与目标在跟与否 —— 俯冲
  /// 制导测量 = 视轴角 (gimbal_state) + 对应像素偏角, 无目标置 NaN (停控)
  void notify_target(double angle_y_deg, double angle_x_deg, bool has_target)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    angle_y_ = has_target ? angle_y_deg :
      std::numeric_limits<double>::quiet_NaN();
    angle_x_ = has_target ? angle_x_deg :
      std::numeric_limits<double>::quiet_NaN();
  }

  /// 当前目标俯角测量 (度, 下正): 视轴俯角 + 像素偏角; 缺失/过期 = NaN
  /// (面板显示用, 帧上下文调用)
  double measure_down_deg() const
  {
    std::lock_guard<std::mutex> lk(mutex_);
    double v = 0.0;
    return measure_locked(&v) ? v :
      std::numeric_limits<double>::quiet_NaN();
  }

  /// 当前目标距离测量 (米): 斜距 = h / sinθ, 水平距离 = h / tanθ;
  /// θ = 视轴俯角 + 目标像素偏角 (下正), h = localPosition NED z 取负
  /// (假定 EKF 原点在地面)。返回斜距; horiz_out 非 null 时回写水平距离
  /// (始终回写, 失败时 NaN)。测量缺失 (云台姿态过期/目标丢失/位置无效/
  /// 在地面以下) 或 θ < 3° (视线近水平, 反演发散, 数值不可信) 返回 NaN
  /// —— 面板显示用, 帧上下文调用 (与 measure_down_deg 同约定)
  double measure_distance(double * horiz_out = nullptr) const
  {
    if (horiz_out != nullptr) {
      *horiz_out = std::numeric_limits<double>::quiet_NaN();
    }
    std::lock_guard<std::mutex> lk(mutex_);
    double deg = 0.0;
    if (!measure_locked(&deg) || deg < 3.0) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const auto pos = px4_->localPosition();
    if (!pos.z_valid || pos.z >= 0.0f) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const double h = -static_cast<double>(pos.z);
    const double rad = deg * M_PI / 180.0;
    if (horiz_out != nullptr) {
      *horiz_out = h / std::tan(rad);
    }
    return h / std::sin(rad);
  }

private:
  /// 视线/距离状态复位 (调用方持锁; 俯冲打击与机体跟随共用): 丢失超时
  /// 窗口从复位时刻重新起算; 滤波/迟滞/限斜率状态一并复位 (重新激活时
  /// 飞机近悬停, 水平指令从 0 起坡是安全方向)
  void reset_los_locked()
  {
    dive_theta_raw_deg_ = std::numeric_limits<double>::quiet_NaN();
    dive_az_raw_deg_ = std::numeric_limits<double>::quiet_NaN();
    dive_theta_f_rad_ = std::numeric_limits<double>::quiet_NaN();
    dive_az_f_rad_ = std::numeric_limits<double>::quiet_NaN();
    bodyfollow_dist_raw_m_ = std::numeric_limits<double>::quiet_NaN();
    bodyfollow_dist_f_m_ = std::numeric_limits<double>::quiet_NaN();
    bodyfollow_in_deadband_ = false;
    dive_los_t_ = std::chrono::steady_clock::now();
    dive_in_plunge_ = false;
    dive_vh_cmd_ms_ = 0.0;
  }

  void tick()
  {
    if (!active_) {
      return;
    }
    // 图像断流自动交还: 算法停/GCS 关时没人发停令, 悬停心跳与云台
    // 伺服都已失依, 交还控制权比维持盲飞安全
    if (std::chrono::steady_clock::now() - frame_t_ >
        std::chrono::duration<double>(frame_timeout_sec_))
    {
      deactivate("图像断流");
      return;
    }
    if (!px4_->gotStatus()) {
      return;   // 链路未就绪, 等 takeoverTick 内部重试
    }
    if (!px4_->armed()) {
      deactivate("飞机已上锁");
      return;
    }
    // 外部切模式 = 有人抢走控制权 (QGC/遥控器), 交还退出
    if (ever_engaged_ && !px4_->isOffboard()) {
      RCLCPP_WARN(
        node_.get_logger(),
        "\033[33mOFFBOARD 被外部切换 (当前模式: %s), 判定为外部接管, "
        "交还控制权并退出反控\033[0m", px4_->modeName().c_str());
      deactivate("外部接管");
      return;
    }
    if (!ever_engaged_) {
      // 接管中: PX4 要求先收到 setpoint 流才接受切模式, takeoverTick
      // 内部已在发心跳+悬停
      const auto res =
        px4_->takeoverTick(node_.now(), warmup_sec_, retry_sec_);
      if (res == Px4OffboardClientBase::Takeover::ENGAGED) {
        ever_engaged_ = true;
        publish_status("engaged");
        RCLCPP_INFO(
          node_.get_logger(),
          "\033[32m已接管 offboard: 悬停跟拍中 (云台伺服保持目标居中)\033[0m");
      } else if (res == Px4OffboardClientBase::Takeover::FAILED) {
        publish_status("failed");
        deactivate("接管超时");
      }
      return;
    }
    // 已接管: 机头跟随 + 模式速度指令 (云台追踪 = 悬停/高度环升降;
    // 曲线/直线打击 = 沿目标方向, 机体跟随 = 保持水平距/斜距半径收放; 测量
    // 缺失超时交还), 心跳随 setpoint 持续发
    follow_->request(true);
    if (mode_ == Mode::DiveCurve || mode_ == Mode::DiveStraight) {
      if (!dive_velocity()) {
        deactivate("打击目标方向丢失");
      }
      return;
    }
    if (mode_ == Mode::BodyfollowHoriz || mode_ == Mode::BodyfollowSlant) {
      if (!bodyfollow_velocity(mode_ == Mode::BodyfollowSlant)) {
        deactivate("跟随目标丢失");
      }
      return;
    }
    px4_->sendVelocityFRDYawRate(0.0, 0.0, alt_vz(), follow_yaw_rate());
  }

  /// 俯冲打击速度指令 (纯角度制导): 定速 dive_speed_ms 沿"视轴俯角 +
  /// 目标像素偏角"合成的目标方向撞过去。两条修正保证航迹贴合视线:
  ///   1. 机体俯仰补偿: 方向测量是机体相对量, 云台伺服把相机稳在
  ///      世界系 → 机体低头的差值全落在相对量里, 比真实视线角浅出
  ///      机体俯仰角; 不补偿则航迹比视线平, 俯冲加速越猛偏得越多,
  ///      表现为从目标头顶飞过。世界系俯角 = 相对量 + 机体俯仰 (下正,
  ///      从 attitudeNED 四元数提取 asin(2(xz+wy)), 已对照恒等/朝东/
  ///      低头 30° 三基准验证)。
  ///   2. 垂直域两相制导: 沿视线全速撞要求垂直分量 V·sinθ ≤ 垂直域 D
  ///      (dive_down_max_ms, 须 ≤ PX4 MPC_Z_VEL_MAX_DN), 即视线角
  ///      θ ≤ φ_max = asin(D/V), 超过则域装不下、沿视线飞必被钳位:
  ///      - θ ≤ φ_max (终端撞击带): 全速沿视线, v_dn = V·sinθ,
  ///        vh = V·cosθ, 撞击合速度恒 V 无减速;
  ///      - θ > φ_max (拉角段): 飞机近目标顶上方, 视线过陡。垂直压满
  ///        D, 水平 = plunge_gain·D·cotθ (严格 < D·cotθ = 视线跟随值):
  ///        高度消耗快于水平接近 → tanθ=h/d 下降 → 视线角逐渐张开到
  ///        φ_max 后自动切终端带。k<1 保证不越过目标上空 (k=1 即视线
  ///        跟随, θ 恒定不收敛); 拉角段总速 √(D²+(kDcotθ)²) 随 θ 张开
  ///        单调升, 终端切回 V。若视线中途从浅变陡越过 φ_max (垂直
  ///        滞后所致), 入拉角段有一次性小减速, 属垂直域物理边界。
  ///   3. 拉角段稳定四闸 (02_57_39 自激振荡修复): 该机气动可达下降速度仅
  ///      ~2.8m/s (俯冲入段推力即贴 THR_MIN, ulog 实测), φ_max≈13°, 拉角段
  ///      就是主要工作段。实测教训: θ 测量抖动 (像素偏角 + 机体俯仰补偿
  ///      ±20°) 经 cotθ 反演放大成水平指令翻打 (2.8↔12m/s @2Hz), 飞控跟
  ///      不上、收敛到摆动均值 → 航迹角≈视线角 → "视线张开"机制失效自激,
  ///      最终 θ 穿零变负误判目标在头顶, 输出急刹+爬升 (vh 5.9→1.0, vz
  ///      翻正爬升)。对策 = 视线角下限钳制 + 一阶低通 (闸 1/2), 段切换
  ///      迟滞 (闸 3), 水平指令限加速度 (闸 4); 拉角段水平钳到 ≤V·cosφ_max。
  /// 发送走机体系分量: sendVelocityFRD 实现只按当前航向旋水平分量、
  /// 垂直分量直通世界系, 垂直闭环不含机体俯仰; 水平方位由机头跟随
  /// 收口 (视线方位 ≈ 机头方位)。θ 浅→深自然过渡, 平冲渐变俯冲。
  /// 目标方向短缺失 (<dive_lost_timeout_sec) 沿最近有效方向惯性直飞
  /// (单帧漏检不刹车), 超时返回 false 由调用方交还 (不盲飞)。
  bool dive_velocity()
  {
    double t_raw = 0.0, az_raw = 0.0;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      double th = 0.0, a = 0.0;
      if (measure_locked(&th, &a)) {
        dive_theta_raw_deg_ = th;
        dive_az_raw_deg_ = a;
        dive_los_t_ = std::chrono::steady_clock::now();
      } else if (std::chrono::steady_clock::now() - dive_los_t_ >
                 std::chrono::duration<double>(dive_lost_timeout_sec_))
      {
        return false;
      }
      t_raw = dive_theta_raw_deg_ * M_PI / 180.0;
      az_raw = dive_az_raw_deg_ * M_PI / 180.0;
    }
    // 机体俯仰补偿 → 世界系俯角 (下正)
    const auto att = px4_->attitudeNED();
    if (att.valid) {
      t_raw += std::asin(
        std::clamp(2.0 * (att.x * att.z + att.w * att.y), -1.0, 1.0));
    }
    // 闸 1/2 —— 视线角下限钳制 + 一阶低通: 俯冲目标物理上总在下方, θ 抖到
    // 0 附近会使 cotθ 指令发散, 穿零变负则误判"目标在头顶"输出急刹+爬升;
    // 低通抑制像素偏角/俯仰补偿噪声。方位一并低通 (拉角段方位抖动同样喂
    // 振荡; 代价是接近段方位响应略滞后, tc 可调, 0=不过滤)
    t_raw = std::max(t_raw, dive_theta_min_deg_ * M_PI / 180.0);
    const double alpha = dive_theta_filter_tc_ > 0.0 ?
      dt_ / (dive_theta_filter_tc_ + dt_) : 1.0;
    dive_theta_f_rad_ = std::isnan(dive_theta_f_rad_) ? t_raw :
      dive_theta_f_rad_ + alpha * (t_raw - dive_theta_f_rad_);
    dive_az_f_rad_ = std::isnan(dive_az_f_rad_) ? az_raw :
      dive_az_f_rad_ + alpha * (az_raw - dive_az_f_rad_);
    const double t = dive_theta_f_rad_;
    const double az = dive_az_f_rad_;
    // 两相制导: 视线角在撞击带内 (sinθ ≤ D/V) 全速沿视线; 过陡进拉角段
    // (垂直压满域, 水平按 plunge_gain·D·cotθ 收敛, 视线角张开到 φ_max
    // 自动切回终端带)
    const double s = std::sin(t);
    const double s_max = std::min(
      1.0, dive_down_max_ms_ / dive_speed_ms_);   // sin(φ_max)
    double v_dn, vh;
    if (mode_ == Mode::DiveStraight) {
      // 直线打击: 全程沿视线, 不分相 —— 合速度 = min(V, D/sinθ) 使垂直
      // 分量恒 ≤ 垂直域 D, 航迹始终贴合视线方向 (θ 在锁定时冻结, 纯追踪
      // 直线收口); 陡角自动降速是直线打击的固有代价 (曲线打击用拉角段
      // 绕弯换末段全速)。θ ≥ theta_min 钳制保证 s > 0, 合速度天然有界
      const double v = std::min(
        dive_speed_ms_, dive_down_max_ms_ / std::max(s, 0.01));
      v_dn = v * s;
      vh = v * std::cos(t);
    } else {
    // 闸 3 —— 段切换迟滞: θ 在 φ_max 边界抖动时两相来回切, 指令在
    // V·cosθ ↔ k·D·cotθ 间阶跃 (锯齿减速); 回终端带需 θ < φ_max − 迟滞角
    const double s_exit = std::max(0.0, std::sin(
      std::max(0.0, std::asin(s_max) - dive_band_hysteresis_deg_ * M_PI / 180.0)));
    const bool in_plunge = dive_in_plunge_ ? (s > s_exit) : (s > s_max);
    if (!in_plunge) {
      // 终端撞击带: 全速沿视线 (v_dn 未触顶, 还原为 V·sinθ/V·cosθ)
      v_dn = dive_speed_ms_ * s;
      vh = v_dn * std::cos(t) / std::max(s, 0.01);
      if (dive_in_plunge_) {
        dive_in_plunge_ = false;
        RCLCPP_INFO(
          node_.get_logger(),
          "\033[32m曲线打击: 视线角 %.0f° 进入撞击带 (<=%.0f°), 全速沿视线\033[0m",
          t * 180.0 / M_PI, std::asin(s_max) * 180.0 / M_PI);
      }
    } else {
      // 拉角段: 垂直压满 D, 水平 < 视线跟随值 → 视线角张开 (推导见上)
      v_dn = dive_down_max_ms_;
      vh = dive_plunge_gain_ * v_dn * std::cos(t) / std::max(s, 0.01);
      // 闸 1 兜底 —— cotθ 发散钳制: θ 小时上式 → ∞ (02_57_39 曾输出
      // 62~477m/s 被飞控钳成 14 引发急刹), 钳到带边界视线跟随值
      // V·cosφ_max —— 拉角段水平本就不该超过它
      vh = std::min(vh,
        dive_speed_ms_ * std::sqrt(std::max(0.0, 1.0 - s_max * s_max)));
      if (!dive_in_plunge_) {
        dive_in_plunge_ = true;
        RCLCPP_INFO(
          node_.get_logger(),
          "\033[33m曲线打击: 视线角 %.0f° 过陡 (>%.0f° 域限), 拉角段垂直 %.1f "
          "压满张开视线角\033[0m",
          t * 180.0 / M_PI, std::asin(s_max) * 180.0 / M_PI, v_dn);
      }
    }
    }
    // 闸 4 —— 水平指令限加速度: 指令大幅摆动时实际速度收敛到摆动均值 →
    // 航迹角 ≈ 视线角, 拉角段"视线张开"机制失效自激; 限斜率后指令本身
    // 平滑, 实际速度贴指令走, 视线角单调张开收敛
    const double dv_max = dive_vh_accel_ms2_ * dt_;
    dive_vh_cmd_ms_ =
      std::clamp(vh, dive_vh_cmd_ms_ - dv_max, dive_vh_cmd_ms_ + dv_max);
    vh = dive_vh_cmd_ms_;
    // 方位快闭环: 水平速度方向直接指向视线方位 (视轴方位 + 像素偏角,
    // sendVelocityFRD 按当前航向旋水平分量 → right 分量即横向修正)。
    // 不再单靠"云台伺服→机头跟随"慢链路收方位 —— 接近段视线方位角速度
    // 随距离减小发散, 慢链路 (死区+限幅+多级延迟) 跟不上, 残余方位误差
    // 以横移积累, 表现为从目标左右侧掠过。机头跟随仍运行 (把 nose 拉回
    // 视线, 恢复前向视场), 与本环同向不冲突。±45° 限幅 = 云台方位限位
    const double a = std::clamp(az, -45.0 * M_PI / 180.0, 45.0 * M_PI / 180.0);
    px4_->sendVelocityFRDYawRate(
      vh * std::cos(a), vh * std::sin(a), v_dn, follow_yaw_rate());
    return true;
  }

  /// 机体跟随速度指令 (水平跟随 slant=false / 直线跟随 slant=true):
  /// 保持设定距离 bodyfollow_dist_m 跟随目标移动 (水平距/斜距半径, 同一
  /// 地面站"跟随距离"输入框按模式解释)。速度标量 = 距离误差 P 律
  /// (死区回差 + 中心减速抑摆, 同 alt_vz 手法) + 接近段制动距离限幅
  /// (按剩余误差平方根提前减速, 防冲过设定距离后倒退拉回的极限环),
  /// 方向 = 视线方位快闭环 (同 dive_velocity: 视轴方位 + 像素偏角,
  /// sendVelocityFRD 按航向旋水平分量); 垂直通道随模式:
  ///   水平跟随: 高度环 alt_vz 独立维持俯视角度 (俯角收敛后两环解耦:
  ///     高度保持视线角 θ, 水平收放水平距离);
  ///   直线跟随: "跟随距离"语义 = 斜距半径 d, "俯视角度"作为观察角度
  ///     —— 高度环照常维持俯视角 (未设置 = 保持当前视角悬停), 水平环
  ///     保持 L = d·cos(p) (p = 俯视角度设定, 未设用滤波视线角 θ 兜底,
  ///     即只锁半径观察角自由)。两环与水平跟随同一双环解耦结构, 平衡
  ///     态 L = d·cosθ ⇔ 斜距 = d 一致收敛; 调整观察角度时飞机沿半径
  ///     d 的球面滑动到新观察点 (高度与水平距同时改, 半径不变) ——
  ///     摆脱"贴视线尾随"形态 (尾随来自视线分解把飞机锁在云台-目标
  ///     连线上, 高度不可控)。方位维度仍自由 (目标直线移动时自然拖到
  ///     其运动方向后方), 固定方位/绕飞需目标航向估计, 未实现。
  /// 测量复用俯冲视线状态 (低通/限加速度共用, activate 切模式时统一
  /// 复位)。丢失三段式 (跟随与打击语义不同: 打击丢目标 = 任务终止,
  /// 跟随丢目标 = 等追踪器重捕获):
  ///   1. < dive_lost_timeout_sec: 沿最近滤波值惯性跟随 (单帧漏检不刹车)
  ///   2. < bodyfollow_lost_timeout_sec: 原地跟随 (速度经限加速度收到 0,
  ///      机头跟随维持, 等追踪器重捕获 —— 机体不动则目标外观稳定, ReID
  ///      相似度回升自动回 TRACK)
  ///   3. 曾跟住后持续丢失超长窗: 返回 false 由调用方交还 (不盲飞);
  ///      尚无有效测量 (未框选目标) 时始终悬停等待, 不交还
  /// 距离 = h / tanθ (θ = 视轴俯角 + 像素偏角 + 机体俯仰补偿 = 世界系
  /// 视线角, h = localPosition NED z 取负, 假定 EKF 原点在地面/目标同
  /// 平面), θ ≥ 3° 才可信 (视线近水平反演发散, 视为缺失)。直线跟随的
  /// 半径语义在目标端折算 (err 对水平距), 面板显示的斜距由
  /// measure_distance 另行提供, 不走本函数
  bool bodyfollow_velocity(bool slant)
  {
    double az_raw = 0.0, dist_raw = 0.0, dist_target = 0.0;
    double pitch_target = std::numeric_limits<double>::quiet_NaN();
    bool fresh = false, stale = false;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      double th = 0.0, a = 0.0;
      if (measure_locked(&th, &a)) {
        // 机体俯仰补偿 (同 dive_velocity 的教训): 测量是机体相对量,
        // 前飞时机体前倾使相对俯角浅于真实视线角 → h/tanθ 把距离高估
        // (θ=30° 前倾 12° 时高估 ~1.8 倍) → 距离环误判"目标还远"继续
        // 加速 → 前倾更大 → 正反馈撞上目标。补偿成世界系视线角后距离
        // 才可信 (悬停时补偿 ≈ 0)
        const auto att = px4_->attitudeNED();
        if (att.valid) {
          th += std::asin(
            std::clamp(2.0 * (att.x * att.z + att.w * att.y), -1.0, 1.0)) *
            180.0 / M_PI;
        }
        const auto pos = px4_->localPosition();
        if (pos.z_valid && pos.z < 0.0f && th >= 3.0) {
          dive_az_raw_deg_ = a;
          // 控制用量统一为水平距 h/tanθ (直线跟随的半径语义在目标端经
          // cos(p) 折算, 见下 err 计算)
          bodyfollow_dist_raw_m_ =
            -static_cast<double>(pos.z) / std::tan(th * M_PI / 180.0);
          dive_los_t_ = std::chrono::steady_clock::now();
          if (slant) {
            dive_theta_raw_deg_ = th;   // 直线跟随: 世界系视线角进滤波
          }
          fresh = true;
        }
      }
      if (!fresh) {
        // 丢失处理 (见函数头三段式); had = 曾有效测量过
        const double miss = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - dive_los_t_).count();
        const bool had = !std::isnan(bodyfollow_dist_raw_m_);
        if (had && miss > bodyfollow_lost_timeout_sec_) {
          return false;   // 持续丢失超长窗: 交还 (不盲飞)
        }
        stale = had && miss > dive_lost_timeout_sec_;
      }
      az_raw = dive_az_raw_deg_;
      dist_raw = bodyfollow_dist_raw_m_;
      dist_target = bodyfollow_dist_m_;
      pitch_target = pitch_target_;   // 直线跟随观察角 (度, 下正, NaN=未设)
    }
    // 低通 (同 dive 闸 2): 像素偏角噪声经 1/tanθ 反演放大进距离测量,
    // 方位抖动同样进水平方向; tc 可调, 0 = 不过滤
    const double alpha = dive_theta_filter_tc_ > 0.0 ?
      dt_ / (dive_theta_filter_tc_ + dt_) : 1.0;
    const double az_f = az_raw * M_PI / 180.0;
    dive_az_f_rad_ = std::isnan(dive_az_f_rad_) ? az_f :
      dive_az_f_rad_ + alpha * (az_f - dive_az_f_rad_);
    if (slant) {
      // 直线跟随: 世界系视线俯角一并低通 (观察角兜底折算的方向输入;
      // 状态复用俯冲 dive_theta_f_rad_, activate 切模式统一复位; raw
      // 与方位/距离一样取最近有效值 —— 惯性跟随段方向不突变)
      const double tr = dive_theta_raw_deg_ * M_PI / 180.0;
      dive_theta_f_rad_ = std::isnan(dive_theta_f_rad_) ? tr :
        dive_theta_f_rad_ + alpha * (tr - dive_theta_f_rad_);
    }
    bodyfollow_dist_f_m_ = std::isnan(bodyfollow_dist_f_m_) ? dist_raw :
      bodyfollow_dist_f_m_ + alpha * (dist_raw - bodyfollow_dist_f_m_);
    if (std::isnan(bodyfollow_dist_f_m_) || std::isnan(dive_az_f_rad_)) {
      // 尚无有效测量 (未框选目标/丢失窗口内首拍): 悬停保持, 不盲飞
      px4_->sendVelocityFRDYawRate(0.0, 0.0, alt_vz(), follow_yaw_rate());
      return true;
    }
    // 原地跟随 (函数头三段式第 2 段): 斜率状态同步衰减平滑刹停,
    // 高度环/机头跟随维持, 等追踪器重捕获
    if (stale) {
      const double dv_max = dive_vh_accel_ms2_ * dt_;
      dive_vh_cmd_ms_ = std::clamp(
        0.0, dive_vh_cmd_ms_ - dv_max, dive_vh_cmd_ms_ + dv_max);
      px4_->sendVelocityFRDYawRate(
        0.0, 0.0, alt_vz(), follow_yaw_rate());
      return true;
    }
    // 距离环 P 律: err > 0 = 目标比设定远 → 前进接近, err < 0 = 太近 →
    // 倒退拉开; 死区回差 + 中心减速段防测量噪声在零附近推速度抖动。
    // 直线跟随: 跟随距离语义 = 斜距半径, 折算成水平距目标 L = d·cos(p)
    // (p = 俯视角度设定; 未设用滤波视线角 θ —— 只锁半径观察角自由);
    // 平衡态 L = d·cosθ ⇔ 斜距 = d, 与高度环 (维持俯角 p) 一致收敛,
    // 结构同水平跟随的双环解耦 (俯角未收敛期间两环同时动作, 飞机沿
    // 半径球面滑向新观察点, P 增益 + 制动限幅约束瞬态)
    double dist_target_eff = dist_target;
    if (slant) {
      const double ang = std::isnan(pitch_target) ?
        dive_theta_f_rad_ : pitch_target * M_PI / 180.0;
      if (!std::isnan(ang)) {
        dist_target_eff = dist_target * std::cos(ang);
      }
    }
    double vh = 0.0;
    const double err = bodyfollow_dist_f_m_ - dist_target_eff;
    const double abs_err = std::fabs(err);
    if (bodyfollow_in_deadband_) {
      if (abs_err < bodyfollow_deadband_m_ * 1.5) {
        vh = 0.0;
      } else {
        bodyfollow_in_deadband_ = false;
      }
    } else if (abs_err <= bodyfollow_deadband_m_) {
      bodyfollow_in_deadband_ = true;
    }
    if (!bodyfollow_in_deadband_) {
      const double u = std::min(1.0, abs_err / bodyfollow_slow_m_);
      vh = std::clamp(
        bodyfollow_kp_ * err * u,
        -bodyfollow_speed_max_ms_, bodyfollow_speed_max_ms_);
      // 制动距离限幅 (接近段, err > 0): 允许速度 ≤ √(2·a·(err − 余量)),
      // 即按剩余误差平方根提前减速, 到设定距离前自然减到停 —— 否则全速
      // 冲进死区后动量+滤波滞后刹不住, 冲过头再触发倒退拉回, 往复成
      // 极限环。余量含低通滞后补偿: 测量滞后 ≈ 当前指令速度 × tc, 一并
      // 扣掉 (用斜率状态当前值作速度估计)。后退方向 (err < 0) 远离目标
      // 无撞风险, 不加限幅
      if (err > 0.0) {
        const double margin = bodyfollow_deadband_m_ +
          dive_vh_cmd_ms_ * dive_theta_filter_tc_;
        const double brake = std::sqrt(std::max(
          0.0, 2.0 * dive_vh_accel_ms2_ * (err - margin)));
        vh = std::min(vh, brake);
      }
    }
    // 限加速度 (同 dive 闸 4, 复用斜率状态): 测量缺失/恢复切换与距离环
    // 输出的指令阶跃平滑, 实际速度贴指令走
    const double dv_max = dive_vh_accel_ms2_ * dt_;
    dive_vh_cmd_ms_ =
      std::clamp(vh, dive_vh_cmd_ms_ - dv_max, dive_vh_cmd_ms_ + dv_max);
    vh = dive_vh_cmd_ms_;
    // 方位快闭环 (同 dive_velocity): 水平速度直接指向视线方位, 机头
    // 跟随仍运行把 nose 拉回视线; ±45° 限幅 = 云台方位限位。
    // 垂直统一走高度环 alt_vz (水平跟随维持俯视角度 / 直线跟随维持
    // 观察角度, 未设置 = 悬停) —— 直线跟随不再做视线分解, 半径由
    // "水平环 L = d·cos(p) + 高度环俯角 p" 的几何关系间接保证
    const double a = std::clamp(
      dive_az_f_rad_, -45.0 * M_PI / 180.0, 45.0 * M_PI / 180.0);
    px4_->sendVelocityFRDYawRate(
      vh * std::cos(a), vh * std::sin(a), alt_vz(), follow_yaw_rate());
    return true;
  }

  /// 机头跟随偏航角速率 (rad/s, NED 正=右转): 误差 (度, 视线在机头右侧
  /// 为正) × KP, 与转向同号; 开关关闭 / NaN (未跟随/过期) / 死区内为 0。
  /// 收敛段两条抑摆措施 (误差链路有图像+云台伺服延迟, 纯 P 律在零附近
  /// 极限环 = 左右摇摆反复对准):
  ///   1. 死区带回差: |err| < deadband 停, 须偏离 1.5× deadband 才重启
  ///      (回差防测量噪声在死区边界抖动)
  ///   2. 中心减速段: |err| < slow_deg 时增益随误差线性衰减, 越接近
  ///      对准转得越慢, 平滑收敛不冲过; 超过后恢复线性全增益
  double follow_yaw_rate()
  {
    const double err = follow_->yaw_err_deg();   // NaN = 未跟随/测量过期
    if (!nose_follow_ || std::isnan(err)) {
      return 0.0;
    }
    const double abs_err = std::fabs(err);
    if (in_deadband_) {
      if (abs_err < follow_deadband_deg_ * 1.5) {
        return 0.0;
      }
      in_deadband_ = false;
    } else if (abs_err <= follow_deadband_deg_) {
      in_deadband_ = true;
      return 0.0;
    }
    const double u = std::min(1.0, abs_err / follow_slow_deg_);
    const double rate_deg = std::clamp(
      follow_kp_ * err * u, -follow_rate_max_, follow_rate_max_);
    return rate_deg * M_PI / 180.0;
  }

  /// 高度环升降速度 (m/s, FRD 下正): 俯角误差 (设定 − 当前, 度) × KP,
  /// 误差 >0 = 当前太浅 (要更俯视) → 上升 → vz 取负返回。测量 = 视轴
  /// 俯角 (gimbal_state, >0.5s 未更新视为缺失) + 目标像素偏角 (帧上下
  /// 文, 丢失 = NaN); 任一缺失即停控 (vz=0 悬停, 不盲飞)。
  /// 抑摆措施同 follow_yaw_rate (死区带回差 + 中心减速段): 高度外环经
  /// 升降惯性 + 云台伺服两级延迟, 纯 P 律在零附近会推油门抖动
  double alt_vz()
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (std::isnan(pitch_target_)) {
      return 0.0;   // 未设置俯视角度: 维持悬停
    }
    double meas = 0.0;
    if (!measure_locked(&meas)) {
      return 0.0;   // 测量缺失: 云台姿态断流 / 目标丢失
    }
    const double err = pitch_target_ - meas;
    const double abs_err = std::fabs(err);
    if (alt_in_deadband_) {
      if (abs_err < alt_deadband_deg_ * 1.5) {
        return 0.0;
      }
      alt_in_deadband_ = false;
    } else if (abs_err <= alt_deadband_deg_) {
      alt_in_deadband_ = true;
      return 0.0;
    }
    const double u = std::min(1.0, abs_err / alt_slow_deg_);
    return -std::clamp(
      alt_kp_ * err * u, -alt_rate_max_, alt_rate_max_);
  }

  /// 目标方向测量合成 (须已持锁): 俯角 = 视轴俯角 + 像素俯仰偏角 (下+),
  /// 方位 = 视轴方位 + 像素方位偏角 (右+); 云台姿态过期 (>0.5s 未更新) /
  /// 目标像素偏角缺失 (丢失) 返回 false。az_out 可为 nullptr (高度环只用俯角)
  bool measure_locked(double * out, double * az_out = nullptr) const
  {
    if (gimbal_t_ == std::chrono::steady_clock::time_point{} ||
        std::chrono::steady_clock::now() - gimbal_t_ >
          std::chrono::milliseconds(500) ||
        std::isnan(angle_y_))
    {
      return false;
    }
    *out = boresight_down_deg_ + angle_y_;
    if (az_out != nullptr) {
      *az_out = boresight_az_deg_ +
        (std::isnan(angle_x_) ? 0.0 : angle_x_);
    }
    return true;
  }

  void publish_status(const char * status)
  {
    std_msgs::msg::String m;
    m.data = status;
    status_pub_->publish(m);
  }

  rclcpp::Node & node_;
  std::unique_ptr<Px4OffboardClientBase> px4_;
  std::unique_ptr<GimbalFollow> follow_;      // 机头跟随通道 (复用组件)
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr follow_switch_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pitch_target_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr dist_target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr gimbal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  mutable std::mutex mutex_;   // 高度环测量/目标 (帧上下文与 ROS 回调并发)

  bool active_ = false;
  bool ever_engaged_ = false;        // 曾确认进入 offboard (外部切换判定)
  Mode mode_ = Mode::Gimbal;         // 反控模式 (activate 切换)
  bool nose_follow_ = true;          // 机头跟随开关 (/flylab/track_follow)
  std::chrono::steady_clock::time_point frame_t_;
  // ---------- 高度环测量/目标 (受 mutex_ 保护) ----------
  // 俯视角度目标 (度, 下正); NaN = 未设置 (不启用)
  double pitch_target_ = std::numeric_limits<double>::quiet_NaN();
  double boresight_down_deg_ = 0.0;  // 视轴俯角测量 (度, 下正, gimbal_state)
  double boresight_az_deg_ = 0.0;    // 视轴方位测量 (度, 右正, gimbal_state)
  double angle_y_ = std::numeric_limits<double>::quiet_NaN();
  // 目标像素俯仰偏角 (度, 下正, 帧上下文喂入; 丢失 = NaN)
  double angle_x_ = std::numeric_limits<double>::quiet_NaN();
  // 目标像素方位偏角 (度, 右正, 帧上下文喂入; 丢失 = NaN)
  std::chrono::steady_clock::time_point gimbal_t_;  // 最近云台姿态时刻
  double warmup_sec_ = 1.0;
  double retry_sec_ = 10.0;
  double follow_kp_ = 1.2;
  double follow_rate_max_ = 45.0;
  double follow_deadband_deg_ = 2.0;   // 对准死区 (度, 回差 1.5×)
  double follow_slow_deg_ = 30.0;      // 中心减速段宽度 (度)
  bool in_deadband_ = false;           // 死区状态 (回差迟滞)
  double frame_timeout_sec_ = 2.0;
  // ---------- 高度环 (俯视角度) ----------
  double alt_kp_ = 0.08;               // P 增益 (m/s 每 度)
  double alt_rate_max_ = 2.0;          // 升降速度限幅 (m/s)
  double alt_deadband_deg_ = 2.0;      // 俯角对准死区 (度, 回差 1.5×)
  double alt_slow_deg_ = 5.0;          // 中心减速段宽度 (度)
  bool alt_in_deadband_ = false;       // 死区状态 (回差迟滞)
  // ---------- 俯冲打击 (纯角度制导) ----------
  double dive_speed_ms_ = 12.0;        // 撞击速度 (m/s, 沿目标方向定速)
  double dive_down_max_ms_ = 2.5;      // 垂直分量限速 (m/s, 须 ≤ MPC_Z_VEL_
                                       // MAX_DN; 撞击带 φ_max = asin(D/V))
  double dive_plunge_gain_ = 0.7;      // 拉角段水平收敛比 (<1 = 视线角张开)
  bool dive_in_plunge_ = false;        // 当前在拉角段 (迟滞 + 日志切换用)
  double dive_lost_timeout_sec_ = 1.0; // 目标方向缺失容忍 (超时交还)
  double dive_theta_min_deg_ = 3.0;    // 视线俯角下限钳制 (度)
  double dive_theta_filter_tc_ = 0.4;  // 视线角低通时间常数 (s, 0=关)
  double dive_band_hysteresis_deg_ = 3.0;  // 撞击带段切换迟滞角 (度)
  double dive_vh_accel_ms2_ = 3.0;     // 水平指令限加速度 (m/s²)
  double dt_ = 0.05;                   // tick 周期 (s, 构造时按 takeover_hz 定)
  double dive_theta_raw_deg_ =
    std::numeric_limits<double>::quiet_NaN();  // 最近有效原始测量 (度, 下正)
  double dive_az_raw_deg_ =
    std::numeric_limits<double>::quiet_NaN();  // 最近有效原始测量 (度, 右正)
  double dive_theta_f_rad_ =
    std::numeric_limits<double>::quiet_NaN();  // 滤波后视线俯角 (rad, 下正)
  double dive_az_f_rad_ =
    std::numeric_limits<double>::quiet_NaN();  // 滤波后视线方位 (rad, 右正)
  double dive_vh_cmd_ms_ = 0.0;        // 水平指令斜率状态 (m/s)
  std::chrono::steady_clock::time_point dive_los_t_;  // 最近有效方向时刻
  // ---------- 机体跟随 (水平: 距离环; 滤波/限加速度/丢失复用俯冲状态) ----------
  double bodyfollow_dist_m_ = 10.0;    // 保持水平距离 (m)
  double bodyfollow_kp_ = 0.5;         // 距离环 P 增益 (m/s 每 m)
  double bodyfollow_speed_max_ms_ = 4.0;   // 水平速度限幅 (m/s)
  double bodyfollow_deadband_m_ = 1.0;     // 距离死区 (m, 回差 1.5×)
  double bodyfollow_slow_m_ = 5.0;         // 中心减速段宽度 (m)
  double bodyfollow_lost_timeout_sec_ = 5.0;   // 持续丢失交还超时 (s)
  bool bodyfollow_in_deadband_ = false;    // 死区状态 (回差迟滞)
  double bodyfollow_dist_raw_m_ =
    std::numeric_limits<double>::quiet_NaN();  // 最近有效原始水平距 (m)
  double bodyfollow_dist_f_m_ =
    std::numeric_limits<double>::quiet_NaN();  // 滤波后水平距 (m)
};

}  // namespace flylab_vision

#endif  // FLYLAB_VISION__CONTROL__TRACK__TRACK_TAKEOVER_HPP_
