// Copyright 2026 flylab
/// ArUco 视觉降落节点 —— 三态状态机
///
///   IDLE     等待 landing_start 指令, 不发任何飞控指令
///   TRACKING 接管 offboard → 图像空间水平对齐 → 按目标面积调速下降
///            → (对齐 + 面积阈值 + 降落趋势角) 连续 N 帧后进入 LANDING
///   LANDING  切 PX4 AUTO.LAND, 上锁后复位回 IDLE
///
/// 只用 cx/cy/面积做图像空间控制, 不依赖 3D 位姿 (未标定时尺度不可信)。
/// 多尺度嵌套码支持: target_id 指定精准落点码 (小码), 不可见时回退跟踪画面
/// 最大码 (高空大码), 指定码出现后自动切换对齐目标 (target_fallback:=false 关闭回退)。
/// 执行层经 px4_offboard_client (link:=mavros|uxrce 选链路)。
///
/// 话题: 图像 /flylab/display/image_raw (地面站转发帧, 需俯视看到码);
///       云台 /flylab/gimbal_state (相机相对机体姿态, 必需, 断流则悬停等待);
///       指令 /flylab/vision_cmd; 叠加 /flylab/annotations; 状态 /flylab/landing_status
///
/// 使用前提: 已解锁升空, 相机画面垂直向下看到 ArUco 码。
/// 运行: ros2 run flylab_vision aruco_landing_node
///       参数走配置文件 configs/aruco.yaml (--params-file 携带, 地面站默认带上),
///       也可 -p 覆盖单项: -p vel_gain:=1.0 -p target_id:=92

#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <opencv2/imgproc.hpp>

#include "aruco/aruco_detector.hpp"
#include "common/image_utils.hpp"
#include "control/px4_offboard_client.hpp"

namespace flylab_vision
{

namespace
{
constexpr const char kCmdStart[] = "landing_start";
constexpr const char kCmdStop[] = "landing_stop";
constexpr const char kAnnoAlgo[] = "aruco_landing";     // 检测结果叠加分组
constexpr const char kStatusTracking[] = "tracking";     // 接管 offboard, 视觉降落中
constexpr const char kStatusLanding[] = "landing";       // 已切 AUTO.LAND
constexpr const char kStatusLanded[] = "landed";         // 降落成功 (飞机上锁)
constexpr const char kStatusIdle[] = "idle";             // 流程结束/取消/中止
constexpr double kStartPendingTimeout = 15.0;  // 挂起的 landing_start 等待飞控状态的最长时间 (s)

double clampd(double v, double lim)
{
  lim = std::abs(lim);
  return std::fmin(std::fmax(v, -lim), lim);
}

/// R(q)·v: 四元数旋转向量 (标准公式, q 为 body←camera 旋转)
cv::Point3d quat_rot(const geometry_msgs::msg::Quaternion & q, double x, double y, double z)
{
  // t = 2 * (q_vec × v)
  const double tx = 2.0 * (q.y * z - q.z * y);
  const double ty = 2.0 * (q.z * x - q.x * z);
  const double tz = 2.0 * (q.x * y - q.y * x);
  return {
    x + q.w * tx + (q.y * tz - q.z * ty),
    y + q.w * ty + (q.z * tx - q.x * tz),
    z + q.w * tz + (q.x * ty - q.y * tx)
  };
}

/// 降落趋势角: 位移向量与竖直向下方向的夹角 (度)
double landing_angle_deg(const std::vector<cv::Point3d> & positions)
{
  const auto & a = positions.front();
  const auto & b = positions.back();
  const double vx = b.x - a.x, vy = b.y - a.y, vz = b.z - a.z;
  const double len = std::sqrt(vx * vx + vy * vy + vz * vz);
  if (len < 1e-6) {
    return 0.0;  // 位置未变化, 视为已垂直
  }
  double cos_theta = -vz / len;  // ENU 竖直向下 = (0,0,-1)
  cos_theta = std::fmax(-1.0, std::fmin(1.0, cos_theta));
  return std::acos(cos_theta) * 180.0 / M_PI;
}

}  // namespace

class ArucoLandingNode : public rclcpp::Node
{
public:
  ArucoLandingNode()
  : Node("aruco_landing_node")
  {
    // ---------- 参数 ----------
    const auto image_topic = declare_parameter<std::string>(
      "image_topic", "/flylab/display/image_raw");
    const auto cmd_topic = declare_parameter<std::string>("cmd_topic", "/flylab/vision_cmd");
    const auto anno_topic =
      declare_parameter<std::string>("annotation_topic", "/flylab/annotations");
    const auto status_topic =
      declare_parameter<std::string>("status_topic", "/flylab/landing_status");
    const auto dict_name = declare_parameter<std::string>("dict_name", "DICT_4X4_1000");
    const auto marker_length = declare_parameter<double>("marker_length", 0.2);
    const auto fov_deg = declare_parameter<double>("fov_deg", 60.0);
    const auto calib_path = declare_parameter<std::string>("calib", "");
    link_ = declare_parameter<std::string>("link", "mavros");    // PX4 执行链路: mavros|uxrce

    // 云台姿态话题 (相机 NED 体约定: x 视轴 y 图右 z 图下, 相对机体 FRD)。
    // 水平控制用它把图像误差旋到机体系; 无数据/断流则悬停等待。
    const auto gimbal_topic =
      declare_parameter<std::string>("gimbal_topic", "/flylab/gimbal_state");
    gimbal_timeout_sec_ = declare_parameter<double>("gimbal_timeout_sec", 0.5);

    target_id_ = declare_parameter<int>("target_id", 41);        // 指定精准落点码 ID, 不可见时回退最大码;
                                                                 // -1 = 任一 (取画面里最大的并锁定)
    target_fallback_ = declare_parameter<bool>("target_fallback", true);  // 指定 ID 不可见时回退画面最大码
    land_cx_ = declare_parameter<double>("land_cx", 0.5);        // 期望对中的画面位置 (归一化)
    land_cy_ = declare_parameter<double>("land_cy", 0.5);
    vel_gain_ = declare_parameter<double>("vel_gain", 1.0);      // 水平对齐 P 增益
    max_h_vel_ = declare_parameter<double>("max_h_vel", 1.2);    // 水平速度限幅 (m/s)
    align_ref_alt_ = declare_parameter<double>("align_ref_alt", 2.5);    // 高度增益缩放基准高度 (m, 0=禁用)
    max_height_scale_ = declare_parameter<double>("max_height_scale", 3.0);   // 高度缩放上限
    far_descend_tol_ = declare_parameter<double>("far_descend_tol", 0.20);    // 未对齐时允许慢降的最大误差
    far_descend_area_ = declare_parameter<double>("far_descend_area", 0.05);  // 仅远处(面积小于此)允许边对齐边降
    far_descend_scale_ = declare_parameter<double>("far_descend_scale", 0.5); // 边对齐边降的速度折扣
    descend_close_ = declare_parameter<double>("descend_close", 0.3);    // 面积大(近)时下降速度 (正值=向下, m/s)
    descend_far_ = declare_parameter<double>("descend_far", 0.5);        // 面积小(远)时下降速度 (正值=向下, m/s)
    area_s_thresh_ = declare_parameter<double>("area_s_thresh", 0.85);   // 速度调度分段点
    align_tol_ = declare_parameter<double>("align_tol", 0.06);   // 对齐容差 (归一化画面)
    land_area_thresh_ = declare_parameter<double>("land_area_thresh", 0.12);
    vertical_threshold_deg_ =
      declare_parameter<double>("vertical_threshold_deg", 15.0);  // 降落趋势角门限
    min_landing_frames_ = declare_parameter<int>("min_landing_frames", 4);
    angle_window_min_ = declare_parameter<int>("angle_window_min", 5);
    angle_window_max_ = declare_parameter<int>("angle_window_max", 50);
    lost_timeout_sec_ = declare_parameter<double>("lost_timeout_sec", 8.0);
    reacquire_sec_ = declare_parameter<double>("reacquire_sec", 0.5);  // 锁定 ID 持续丢失多久后重新选目标
    offboard_warmup_sec_ = declare_parameter<double>("offboard_warmup_sec", 1.0);
    offboard_retry_sec_ = declare_parameter<double>("offboard_retry_sec", 10.0);
    control_hz_ = declare_parameter<double>("control_hz", 20.0);

    DetectorConfig cfg;
    cfg.dict_name = dict_name;
    cfg.marker_length = marker_length;
    cfg.fov_deg = fov_deg;
    if (!calib_path.empty()) {
      load_calib(calib_path, cfg.K, cfg.D);
    }
    detector_ = std::make_unique<ArucoDetector>(cfg);

    // ---------- PX4 执行层 (复用组件: 通信/诊断/接管时序全在里面, 链路按 link 参数选择) ----------
    if (link_ == "mavros") {
      px4_ = std::make_unique<Px4OffboardClientMavros>(*this);
      RCLCPP_INFO(
        get_logger(), "PX4 执行链路: MAVROS (/mavros/*, fcu_url:=udp://:14542@ 专属实例, "
        "参照 fly_ego.launch.py 启动 MAVROS)");
    } else if (link_ == "uxrce") {
      px4_ = std::make_unique<Px4OffboardClientUxrce>(*this);
      RCLCPP_INFO(
        get_logger(), "PX4 执行链路: uXRCE-DDS (/fmu/*, 需 MicroXRCEAgent udp4 -p 8888)");
    } else {
      RCLCPP_FATAL(get_logger(), "未知 link 参数 '%s' (可选 mavros|uxrce)", link_.c_str());
      throw std::invalid_argument("link must be mavros or uxrce");
    }

    // ---------- 地面站链路 ----------
    anno_pub_ = create_publisher<std_msgs::msg::String>(anno_topic, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic, 10);
    cmd_sub_ = create_subscription<std_msgs::msg::String>(
      cmd_topic, 10, [this](const std_msgs::msg::String::SharedPtr msg) {on_cmd(*msg);});
    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {on_image(*msg);});
    gimbal_sub_ = create_subscription<geometry_msgs::msg::Quaternion>(
      gimbal_topic, 10,
      [this](const geometry_msgs::msg::Quaternion::SharedPtr msg) {
        cam_ori_ = *msg;
        cam_ori_time_ = now();
        gimbal_ever_ = true;  // 标记数据源存在过 (中断时悬停告警区分诊断用)
      });

    // ---------- 地面站视场角同步 ----------
    // 未标定时内参按 FOV 估算 (位姿轴叠加显示), 地面站调整视场滑杆实时跟随;
    // latched 话题, 节点后启动也能收到最近一次发布的值。话题名可用参数覆盖。
    const auto fov_topic =
      declare_parameter<std::string>("fov_topic", "/flylab/camera_fov");
    fov_deg_ = fov_deg;
    fov_sub_ = create_subscription<std_msgs::msg::Float64>(
      fov_topic, 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {on_fov(*msg);});

    // ---------- 跟踪码 ID 广播 ----------
    // 每帧发布当前锁定/回退跟踪的码 ID (无则 -1): 检测节点 (aruco_node)
    // 据此让位 —— 跟踪码恰为画面最大码时不再叠画洋红 MAX, 由本节点橙色
    // TRACK 独占该码的可视化; 广播停发后检测节点超时自行恢复 MAX
    const auto tracked_id_topic = declare_parameter<std::string>(
      "tracked_id_topic", "/flylab/landing_tracked_id");
    tracked_id_pub_ = create_publisher<std_msgs::msg::Int32>(
      tracked_id_topic, 10);

    // ---------- 右栏"算法输出"面板数据 ----------
    // 降落状态结构化摘要 (10Hz 限频), 地面站右栏面板显示
    const auto info_topic =
      declare_parameter<std::string>("info_topic", "/flylab/aruco_info");
    info_pub_ = create_publisher<std_msgs::msg::String>(info_topic, 10);

    // ---------- 控制主循环 (固定频率, 不随图像帧率波动) ----------
    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_hz_), [this]() {tick();});
    last_tick_ = now();

    RCLCPP_INFO(
      get_logger(),
      "ArUco 降落节点就绪 (三态状态机): 等待 %s 指令; 图像 %s; PX4 链路=%s (需已解锁升空的飞机)",
      kCmdStart, image_topic.c_str(), link_.c_str());
  }

private:
  enum class State
  {
    IDLE,      // 等待触发
    TRACKING,  // offboard 接管 + 对齐下降
    LANDING    // 已满足条件, 切 PX4 AUTO.LAND
  };

  // ---------------- 地面站指令 ----------------
  void on_cmd(const std_msgs::msg::String & msg)
  {
    if (msg.data == kCmdStart) {
      onStart();
    } else if (msg.data == kCmdStop) {
      onCancel();
    }
  }

  void onStart()
  {
    if (state_ != State::IDLE) {
      RCLCPP_WARN(get_logger(), "已在降落流程中, 忽略重复 %s", kCmdStart);
      return;
    }
    start_pending_ = false;
    if (!px4_->gotStatus()) {
      // 飞控状态未到 (地面站只在点击时发一次指令): 挂起待状态到达后在
      // tick() 里补执行, 限时防永久悬挂
      if (!start_pending_) {
        start_pending_ = true;
        start_pending_deadline_ =
          now() + rclcpp::Duration::from_seconds(kStartPendingTimeout);
        RCLCPP_WARN(
          get_logger(), "\033[33m尚未收到飞控状态, %s 已挂起, "
          "状态到达后自动触发 (限时 %.0fs)\033[0m", kCmdStart, kStartPendingTimeout);
      }
      return;
    }
    if (!px4_->armed()) {
      RCLCPP_WARN(get_logger(), "飞机未解锁, 无法开始视觉降落 (请先起飞)");
      return;
    }
    // 复位全部状态
    lost_time_ = 0.0;
    landing_count_ = 0;
    land_confirmed_ = false;
    had_target_ = false;
    ever_offboard_ = false;
    tracked_id_ = -1;      // 重新进入降落流程重新选目标
    lost_id_time_ = 0.0;
    trend_angle_ = 90.0;
    positions_.clear();
    last_mode_cmd_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    px4_->takeoverReset(now());
    state_ = State::TRACKING;
    RCLCPP_INFO(
      get_logger(),
      "\033[32m开始 ArUco 视觉降落: 接管 offboard, 预热 %.1fs 后切模式\033[0m", offboard_warmup_sec_);
    publishStatus(kStatusTracking);
  }

  void onCancel()
  {
    if (state_ == State::IDLE) {
      clear_annotations();
      return;
    }
    // 控制权交还: offboard 中切回 AUTO_LOITER, 否则直接复位
    if (px4_->armed() && px4_->isOffboard()) {
      RCLCPP_INFO(get_logger(), "降落取消, 切回 AUTO_LOITER 交还控制权");
      px4_->requestLoiter();
    } else {
      RCLCPP_INFO(get_logger(), "降落取消");
    }
    toIdle();
  }

  void toIdle(bool notify = true)
  {
    state_ = State::IDLE;
    start_pending_ = false;  // 取消/完成时丢弃尚未触发的挂起指令
    if (notify) {
      publishStatus(kStatusIdle);
    }
    clear_annotations();
    publishInfo("{\"src\":\"landing\",\"state\":\"IDLE\"}");  // 面板复位
  }

  // ---------------- 状态回传 (地面站据此复位按钮/给反馈) ----------------
  // tracking: 已接管 offboard 开始视觉降落; landing: 已切 AUTO.LAND;
  // landed: 降落成功 (飞机上锁); idle: 流程结束/取消/中止
  void publishStatus(const char * status)
  {
    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "[状态] %s", status);
  }

  // ---------------- 右栏"算法输出"面板 (JSON 见 aruco_panel.py 注释) ----------------
  void publishInfo(const std::string & json)
  {
    std_msgs::msg::String out;
    out.data = json;
    info_pub_->publish(out);
  }

  /// 面板信息 10Hz 限频判定 (到点返回 true 并刷新上次发布时刻)
  bool infoDue(const rclcpp::Time & t)
  {
    if ((t - last_info_pub_).seconds() < 0.1) {
      return false;
    }
    last_info_pub_ = t;
    return true;
  }

  /// 码的实际物理宽度 (米): w_px × Z / fx (针孔模型), 无位姿返回 -1。
  /// fx 与检测器同源: 有标定用标定 K, 否则按当前同步 FOV 估算
  double physicalWidth(const Detection & d) const
  {
    if (!d.has_pose || img_w_ <= 0.0) {
      return -1.0;
    }
    const double fx = detector_->calibrated() ? detector_->fx() :
      img_w_ / (2.0 * std::tan(fov_deg_ * CV_PI / 360.0));
    return d.bbox_wh.x * d.tvec[2] / fx;
  }

  // ---------------- 地面站视场角同步 ----------------
  void on_fov(const std_msgs::msg::Float64 & msg)
  {
    // 已标定 (真实内参) 或非法值不跟随; 值未变化跳过 (滑杆拖动/重发布降噪)
    if (detector_->calibrated() || msg.data <= 0.0 || msg.data == fov_deg_) {
      return;
    }
    fov_deg_ = msg.data;
    detector_->set_fov(msg.data);
    // RCLCPP_INFO(
    //   get_logger(), "视场角同步: %.1f° (内参按 FOV 估算, 来自地面站)",
    //   msg.data);
  }

  // ---------------- 图像: 检测 (仅在降落流程中工作) ----------------
  void on_image(const sensor_msgs::msg::Image & msg)
  {
    if (state_ == State::IDLE) {
      return;
    }
    const cv::Mat bgr = to_bgr(msg);
    if (bgr.empty()) {
      return;
    }
    image_area_ = static_cast<double>(bgr.cols) * bgr.rows;
    img_w_ = bgr.cols;
    detections_ = detector_->detect(bgr);
    // 跟踪码选取与检测节点同规则 (锁定 ID 内取面积最大者); 每帧广播 ID
    // (无跟踪目标发 -1), 供检测节点判断是否让位
    const Detection * tracked =
      tracked_id_ >= 0 ? largestOfId(tracked_id_) : nullptr;
    std_msgs::msg::Int32 tid;
    tid.data = tracked ? tracked_id_ : -1;
    tracked_id_pub_->publish(tid);
    // 检测结果叠加到地面站画面 (与 aruco_node 同一标注接口, 独立分组);
    // 正在跟踪的码用强调样式 (橙色 + TRACK 后缀, 与检测节点最大码的洋红
    // 区分), 一眼看出飞机在定位哪个码
    if (!detections_.empty()) {
      std_msgs::msg::String out;
      out.data = tracked ?
        to_annotations_json(
          detections_, kAnnoAlgo, tracked, "TRACK", cv::Scalar(255, 165, 0)) :
        to_annotations_json(detections_, kAnnoAlgo);
      anno_pub_->publish(out);
    }
  }

  /// 指定 ID 的检出码中取画面面积最大者; 无该 ID 返回 nullptr
  const Detection * largestOfId(int id) const
  {
    const Detection * best = nullptr;
    double best_area = -1.0;
    for (const auto & d : detections_) {
      if (d.id != id) {
        continue;
      }
      const double a = d.bbox_wh.x * d.bbox_wh.y;
      if (a > best_area) {
        best_area = a;
        best = &d;
      }
    }
    return best;
  }

  /// 目标选择 (指定 ID 优先 + 最大码回退, 适配多尺度嵌套码):
  ///   target_id = -1  自动模式: 首帧选画面最大的码并锁定, 持续丢失超 reacquire_sec 重选
  ///   target_id >= 0  指定模式: 指定 ID 出现即切为对齐目标; 不可见则回退
  ///                   画面最大码 (target_fallback=true) 或只认指定 ID
  /// 回退目标软锁定防跳变; 指定 ID 重新出现立即切回 (同心码无水平跳变)。
  /// 同 ID 多码一律取画面面积最大者, 不随检出顺序跳变。
  const Detection * pickTarget(double dt)
  {
    if (detections_.empty()) {
      noteTargetMissed(dt);
      return nullptr;
    }
    // 指定 ID 优先: 出现即切换对齐目标
    if (target_id_ >= 0) {
      const Detection * m = largestOfId(target_id_);
      if (m != nullptr) {
        if (tracked_id_ != target_id_) {
          RCLCPP_INFO(
            get_logger(), "\033[32m对齐目标切换: id=%d (指定目标出现%s)\033[0m",
            target_id_, tracked_id_ >= 0 ? ", 从回退码接管" : "");
        }
        tracked_id_ = target_id_;
        lost_id_time_ = 0.0;
        return m;
      }
      if (!target_fallback_) {
        noteTargetMissed(dt);
        return nullptr;  // 严格模式: 只认指定 ID
      }
      // 指定 ID 未出现: 回退跟踪画面最大码 (软锁定, 防跳变)
    }
    // 锁定目标仍在画面: 直接跟它 (指定模式下 tracked_id_ 已是回退码)
    if (tracked_id_ >= 0 && tracked_id_ != target_id_) {
      const Detection * m = largestOfId(tracked_id_);
      if (m != nullptr) {
        lost_id_time_ = 0.0;
        return m;
      }
      // 锁定目标本帧缺失: 短暂丢失 (检测抖动/遮挡) 仍认锁定目标, 悬停等待
      noteTargetMissed(dt);
      if (lost_id_time_ <= reacquire_sec_) {
        return nullptr;
      }
      RCLCPP_INFO(
        get_logger(), "锁定目标 id=%d 持续丢失 %.1fs, 重新选择", tracked_id_, lost_id_time_);
      tracked_id_ = -1;
    }
    // (重新)选择: 取画面里面积最大的码并锁定
    const Detection * best = nullptr;
    double best_area = -1.0;
    for (const auto & d : detections_) {
      const double a = d.bbox_wh.x * d.bbox_wh.y;
      if (a > best_area) {
        best_area = a;
        best = &d;
      }
    }
    if (best != nullptr) {
      if (target_id_ >= 0) {
        RCLCPP_INFO(
          get_logger(), "\033[33m指定目标 id=%d 不可见, 回退画面最大码 id=%d "
          "(共检出 %zu 个码)\033[0m", target_id_, best->id, detections_.size());
      } else {
        RCLCPP_INFO(
          get_logger(), "\033[32m选定跟踪目标: id=%d (画面最大, 共检出 %zu 个码, "
          "此后锁定该 ID)\033[0m", best->id, detections_.size());
      }
      tracked_id_ = best->id;
      lost_id_time_ = 0.0;
    }
    return best;
  }

  void noteTargetMissed(double dt)
  {
    if (tracked_id_ >= 0) {
      lost_id_time_ += dt;
    }
  }

  /// 云台姿态数据是否新鲜 (不新鲜则水平控制悬停等待)
  bool gimbalFresh() const
  {
    return (now() - cam_ori_time_).seconds() <= gimbal_timeout_sec_;
  }

  // ---------------- 控制主循环 ----------------
  void tick()
  {
    const rclcpp::Time now_t = now();
    const double dt = std::fmax(0.001, (now_t - last_tick_).seconds());
    last_tick_ = now_t;
    if (state_ == State::IDLE) {
      // 有挂起的 landing_start: 飞控状态已到则补执行, 超时则放弃
      if (start_pending_) {
        if (px4_->gotStatus()) {
          start_pending_ = false;
          onStart();
        } else if ((now_t - start_pending_deadline_).seconds() > 0.0) {
          start_pending_ = false;
          RCLCPP_WARN(
            get_logger(), "\033[33m挂起的 %s 等待飞控状态超时 (%.0fs) 已放弃, "
            "请检查链路后重新点击\033[0m", kCmdStart, kStartPendingTimeout);
        }
      }
      return;
    }
    // 链路断链告警 (只报警不中断, 与 tracker 链路一致)
    if (!px4_->linkFresh(3.0)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "\033[33mPX4 飞控数据中断, 飞控链路 (%s) 可能已断开!\033[0m", link_.c_str());
    }
    switch (state_) {
      case State::TRACKING:
        tickTracking(dt, now_t);
        break;
      case State::LANDING:
        tickLanding(now_t);
        break;
      default:
        break;
    }
  }

  // ---------------- TRACKING: 接管 + 对齐 + 调速下降 ----------------
  void tickTracking(double dt, const rclcpp::Time & now_t)
  {
    if (!px4_->armed()) {
      RCLCPP_WARN(get_logger(), "飞机已上锁, 退出降落流程");
      toIdle();
      return;
    }

    // 阶段 1: 接管 offboard (心跳预热/周期请求/成功判定都在执行层状态机里)
    if (!px4_->isOffboard()) {
      if (ever_offboard_) {
        // 曾进入过 offboard 又被切走 = 外部接管 (飞手/failsafe), 不抢控制权
        RCLCPP_WARN(
          get_logger(), "\033[33mOFFBOARD 被外部切换 (当前模式: %s), "
          "判定为外部接管, 交还控制权并退出降落流程\033[0m",
          px4_->modeName().c_str());
        toIdle();
        return;
      }
      if (px4_->takeoverTick(now_t, offboard_warmup_sec_, offboard_retry_sec_) ==
        Px4OffboardClientBase::Takeover::FAILED)
      {
        toIdle();
      }
      return;
    }
    ever_offboard_ = true;

    // 阶段 2: 视觉跟踪 (已确认 offboard, 心跳随每拍 sendVelocityFRD 持续发送)
    const Detection * tgt = pickTarget(dt);
    if (tgt == nullptr) {
      handleTargetLost(dt);
      return;
    }
    lost_time_ = 0.0;
    if (!had_target_) {
      had_target_ = true;
      RCLCPP_INFO(
        get_logger(), "\033[32m目标捕获: id=%d area=%.3f cx=%.2f cy=%.2f\033[0m",
        tgt->id,
        image_area_ > 0 ? (tgt->bbox_wh.x * tgt->bbox_wh.y) / image_area_ : 0.0,
        tgt->center_norm.x, tgt->center_norm.y);
    }

    // 图像空间量: 归一化中心 + 面积占比
    const double cx = tgt->center_norm.x;
    const double cy = tgt->center_norm.y;
    const double area =
      image_area_ > 0 ? (tgt->bbox_wh.x * tgt->bbox_wh.y) / image_area_ : 0.0;
    const double dist_factor = (1.0 - area) * (1.0 - area);  // 距离越近增益越小, 防近距振荡

    // 高度增益缩放: 归一化误差在高空对应更大水平距离, 按高度/基准高度放大
    // 增益使收敛速率与高度无关; 封顶防高空过冲, 低空不放大
    const auto & pos = px4_->localPosition();
    double height_scale = 1.0;
    if (align_ref_alt_ > 0.0 && pos.z_valid) {
      const double h = std::fmax(0.5, -pos.z);
      height_scale = std::fmin(
        std::fmax(h / align_ref_alt_, 1.0), max_height_scale_);
    }
    // 水平对齐 (输出机体系 FRD 速度): 用相机姿态四元数把"图右/图下"方向
    // R(q)·(0,1,0)、R(q)·(0,0,1) 旋到机体系, 垂直分量只取水平投影 ——
    // 不假设相机垂直向下, 刚性/自稳/yaw 随动云台全适用
    if (!gimbalFresh()) {
      px4_->sendHover();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "\033[33m云台姿态数据不可用 (%s), 原地悬停等待, 数据恢复后自动继续降落\033[0m",
        gimbal_ever_ ? "此前曾收到后中断" : "未收到过, 需地面站 AirSim 相机源");
      return;
    }
    // 光轴偏离垂直向下过远 = 云台未俯拍到位, 告警但不中断 (增益退化仍可对齐)
    const cv::Point3d optical = quat_rot(cam_ori_, 1.0, 0.0, 0.0);
    const double down_cos = std::fmax(-1.0, std::fmin(1.0, optical.z));
    const double tilt_deg = std::acos(down_cos) * 180.0 / M_PI;
    if (tilt_deg > 45.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "\033[33m相机光轴偏离垂直向下 %.0f°, 云台未俯拍到位, 降落几何不成立\033[0m",
        tilt_deg);
    }
    const cv::Point3d u_b = quat_rot(cam_ori_, 0.0, 1.0, 0.0);  // 图右 (机体系)
    const cv::Point3d v_b = quat_rot(cam_ori_, 0.0, 0.0, 1.0);  // 图下 (机体系)
    const double k = vel_gain_ * dist_factor * height_scale;
    double v_fwd = k * ((cx - land_cx_) * u_b.x + (cy - land_cy_) * v_b.x);
    double v_right = k * ((cx - land_cx_) * u_b.y + (cy - land_cy_) * v_b.y);
    v_fwd = clampd(v_fwd, max_h_vel_);
    v_right = clampd(v_right, max_h_vel_);

    // 对齐达标后才开始下降 (先水平对齐再下降, 防止高度低时目标出画)。
    // 下降速度按目标面积分段线性调度: 近慢远快
    const double align_err = std::hypot(cx - land_cx_, cy - land_cy_);
    const bool aligned = align_err < align_tol_;
    double v_down = 0.0;
    if (aligned) {
      v_down = descendSchedule(area);
    } else if (area < far_descend_area_ && align_err < far_descend_tol_) {
      // 高空边对齐边慢降: 下降会放大归一化误差, 超限即停降回去对齐, 自平衡
      v_down = descendSchedule(area) * far_descend_scale_;
    }

    // 降落趋势角: 滑动窗口内位移与竖直方向的夹角 (窗口随面积自适应)
    updateTrendAngle(area);

    // 触发条件: 对齐 + 面积达标(足够近) + 趋势角够垂直, 连续 N 帧确认
    if (aligned && area >= land_area_thresh_ && trend_angle_ < vertical_threshold_deg_) {
      landing_count_++;
    } else {
      landing_count_ = 0;
    }
    if (landing_count_ >= min_landing_frames_) {
      RCLCPP_INFO(
        get_logger(),
        "\033[32m降落条件满足 (面积 %.3f, 对齐误差 %.3f, 趋势角 %.1f°), 切 PX4 AUTO.LAND\033[0m",
        area, align_err, trend_angle_);
      state_ = State::LANDING;
      last_mode_cmd_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      publishStatus(kStatusLanding);
      return;
    }

    px4_->sendVelocityFRD(v_fwd, v_right, v_down);
    // 右栏"算法输出"面板: 对齐下降状态摘要 (10Hz 限频)
    if (infoDue(now_t)) {
      const double h = pos.z_valid ? -pos.z : -1.0;
      std::ostringstream js;
      js << std::fixed << std::setprecision(3)
         << "{\"src\":\"landing\",\"state\":\"TRACKING\",\"tracked_id\":"
         << tracked_id_ << ",\"count\":" << detections_.size()
         << ",\"area\":" << area
         << ",\"w\":" << static_cast<int>(tgt->bbox_wh.x + 0.5)
         << ",\"h\":" << static_cast<int>(tgt->bbox_wh.y + 0.5)
         << ",\"wm\":" << physicalWidth(*tgt)
         << ",\"err\":" << align_err
         << ",\"aligned\":" << (aligned ? "true" : "false")
         << ",\"lost\":false"
         << std::setprecision(2)
         << ",\"height\":" << h
         << std::setprecision(1)
         << ",\"trend\":" << trend_angle_
         << std::setprecision(2)
         << ",\"vel\":[" << v_fwd << ',' << v_right << ',' << v_down << ']'
         << ",\"trigger\":" << landing_count_
         << ",\"trigger_need\":" << min_landing_frames_ << '}';
      publishInfo(js.str());
    }
    // 1Hz 详细状态行: 降落每一步在干什么、为什么还不触发, 一眼可见
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[TRACKING][云台泛化] 高度=%.2fm 检出=%zu 选中 id=%d area=%.3f cx=%.2f cy=%.2f "
      "对齐误差=%.3f (容差%.3f%s 增益缩放%.1f) 趋势角=%.0f°(门限%.0f°) "
      "输出速度(前,右,下)=(%.2f, %.2f, %.2f) 触发计数=%d/%d",
      pos.z_valid ? -pos.z : 0.0,
      detections_.size(), tgt->id, area, cx, cy,
      align_err, align_tol_, aligned ? "已对齐" : "未对齐", height_scale,
      trend_angle_, vertical_threshold_deg_,
      v_fwd, v_right, v_down, landing_count_, min_landing_frames_);
  }

  /// 目标丢失处理 (TRACKING): 原地悬停, 超时切 AUTO_LOITER 交还控制权
  void handleTargetLost(double dt)
  {
    // 原地悬停, 超时交还控制权
    px4_->sendHover();
    lost_time_ += dt;
    // 右栏面板: 丢失状态 (10Hz 限频)
    if (infoDue(now())) {
      const auto & pos = px4_->localPosition();
      const double h = pos.z_valid ? -pos.z : -1.0;
      std::ostringstream js;
      js << std::fixed << std::setprecision(3)
         << "{\"src\":\"landing\",\"state\":\"TRACKING\",\"tracked_id\":"
         << tracked_id_ << ",\"count\":" << detections_.size()
         << ",\"lost\":true"
         << std::setprecision(2)
         << ",\"lost_t\":" << lost_time_
         << ",\"height\":" << h << '}';
      publishInfo(js.str());
    }
    if (had_target_) {
      had_target_ = false;
      RCLCPP_WARN(get_logger(), "\033[33m目标丢失 (画面中无码), 原地悬停等待...\033[0m");
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "目标丢失中 (%.1f/%.1fs): 检出 %zu 个码, 悬停不降; 检查画面是否对准码 / dict_name 是否正确",
      lost_time_, lost_timeout_sec_, detections_.size());
    if (lost_time_ > lost_timeout_sec_) {
      RCLCPP_WARN(get_logger(), "目标丢失超时, 切回 AUTO_LOITER 交还控制权");
      px4_->requestLoiter();
      toIdle();
    }
  }

  /// 下降速度调度 (正值=向下): 按 s=(1-面积)² 两段线性插值,
  /// 近处(s→0) descend_close 慢降, 远处(s→1) descend_far 快降
  double descendSchedule(double area) const
  {
    double s = std::fmax(0.0, std::fmin(1.0, 1.0 - area));
    s *= s;
    const double v_close = descend_close_;              // s=0 (最近)
    const double v_far = descend_far_;                  // s=1 (最远)
    const double v_mid = 0.5 * (v_close + v_far);       // 分段点处速度
    const double t2 = std::fmax(0.01, std::fmin(0.99, area_s_thresh_));
    if (s <= t2) {
      return v_close + (v_mid - v_close) * (s / t2);
    }
    return v_mid + (v_far - v_mid) * ((s - t2) / (1.0 - t2));
  }

  void updateTrendAngle(double area)
  {
    const auto & pos = px4_->localPosition();
    if (!pos.xy_valid || !pos.z_valid) {
      return;
    }
    const cv::Point3d p{pos.y, pos.x, -pos.z};       // NED -> ENU
    // 窗口大小随面积自适应: 距离越远窗口越大
    double win = (1.0 - area) * (1.0 - area) * 12.0;
    win = std::fmax(
      static_cast<double>(angle_window_min_),
      std::fmin(static_cast<double>(angle_window_max_), win));
    const size_t n = std::max<size_t>(2, static_cast<size_t>(std::lround(win)));
    if (positions_.size() >= n) {
      positions_.erase(positions_.begin());
    }
    positions_.push_back(p);
    if (positions_.size() >= n) {
      trend_angle_ = landing_angle_deg(positions_);
    }
  }

  // ---------------- LANDING: 切 AUTO.LAND, 由 PX4 完成降落 ----------------
  void tickLanding(const rclcpp::Time & now_t)
  {
    if (!px4_->armed()) {
      RCLCPP_INFO(get_logger(), "\033[32m飞机已上锁, ArUco 视觉降落完成\033[0m");
      publishStatus(kStatusLanded);   // 成功: 地面站据此复位按钮并给反馈
      toIdle(false);                   // 不再发 idle, 避免覆盖"成功"事件
      return;
    }
    // 右栏面板: PX4 自动降落中 (10Hz 限频)
    if (infoDue(now_t)) {
      const auto & pos = px4_->localPosition();
      const double h = pos.z_valid ? -pos.z : -1.0;
      std::ostringstream js;
      js << std::fixed << std::setprecision(2)
         << "{\"src\":\"landing\",\"state\":\"LANDING\",\"height\":" << h << '}';
      publishInfo(js.str());
    }
    if (px4_->isAutoLand()) {
      if (!land_confirmed_) {
        land_confirmed_ = true;
        RCLCPP_INFO(
          get_logger(), "\033[32m已确认进入 AUTO.LAND, 停发 offboard, "
          "PX4 自动下降中 (等待落地自动上锁)\033[0m");
      }
      return;
    }
    // AUTO.LAND 确认前被外部切走: 同 TRACKING, 不抢回, 交还控制权
    if (ever_offboard_ && !px4_->isOffboard()) {
      RCLCPP_WARN(
        get_logger(), "\033[33mOFFBOARD 被外部切换 (当前模式: %s), "
        "AUTO.LAND 未及确认, 判定为外部接管, 交还控制权并退出降落流程\033[0m",
        px4_->modeName().c_str());
      toIdle();
      return;
    }
    // AUTO.LAND 确认前继续维持 offboard 心跳+悬停 (断流会触发 failsafe 抢先接管)
    px4_->sendHover();
    if ((now_t - last_mode_cmd_).seconds() > 0.5) {
      px4_->requestAutoLand();
      last_mode_cmd_ = now_t;
    }
  }

  void clear_annotations()
  {
    for (const char * algo : {kAnnoAlgo}) {
      std::ostringstream json;
      json << "{\"algo\":\"" << algo << "\",\"items\":[]}";
      std_msgs::msg::String out;
      out.data = json.str();
      anno_pub_->publish(out);
    }
  }

  // ---------------- 成员 ----------------
  // PX4 执行层 (可复用组件, link 参数选择 MAVROS / uXRCE 实现, 见 px4_offboard_client.hpp)
  std::unique_ptr<Px4OffboardClientBase> px4_;
  std::string link_;

  // 视觉与地面站
  std::unique_ptr<ArucoDetector> detector_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr anno_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;  // 降落状态回传
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr tracked_id_pub_;  // 跟踪码 ID 广播 (检测节点让位依据)
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr info_pub_;  // 右栏面板数据
  rclcpp::Time last_info_pub_{0, 0, RCL_ROS_TIME};                // 10Hz 限频
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr gimbal_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr fov_sub_;
  double fov_deg_ = 60.0;   // 当前生效的 FOV (视场角同步去重用)
  rclcpp::TimerBase::SharedPtr control_timer_;

  // 泛化云台映射: 最近一次相机机体系姿态 (相机 NED 体约定) + 到达时刻
  geometry_msgs::msg::Quaternion cam_ori_;
  rclcpp::Time cam_ori_time_{0, 0, RCL_ROS_TIME};
  double gimbal_timeout_sec_ = 0.5;
  bool gimbal_ever_ = false;  // 本次运行曾收到过云台数据 (中断悬停判定用)

  // 状态机
  State state_ = State::IDLE;

  // 检测缓存 (单线程执行器, 回调与定时器串行, 无需加锁)
  std::vector<Detection> detections_;
  double image_area_ = 0.0;

  // 调参项
  int target_id_;
  bool target_fallback_;
  double land_cx_, land_cy_, vel_gain_, max_h_vel_;
  double align_ref_alt_, max_height_scale_;
  double far_descend_tol_, far_descend_area_, far_descend_scale_;
  double descend_close_, descend_far_, area_s_thresh_;
  double align_tol_, land_area_thresh_, vertical_threshold_deg_;
  int min_landing_frames_, angle_window_min_, angle_window_max_;
  double lost_timeout_sec_, offboard_warmup_sec_, offboard_retry_sec_, control_hz_;
  double reacquire_sec_;

  // 运行时状态
  double lost_time_ = 0.0;          // 目标连续丢失时长
  int landing_count_ = 0;           // 降落条件连续满足帧数
  double trend_angle_ = 90.0;       // 降落趋势角 (度)
  std::vector<cv::Point3d> positions_;  // 趋势角滑动窗口 (ENU)
  bool land_confirmed_ = false;     // 已确认 AUTO.LAND (打一次日志)
  bool had_target_ = false;         // 目标捕获/丢失事件日志用
  bool ever_offboard_ = false;      // 本次流程曾进入过 OFFBOARD (判定外部接管用)
  int tracked_id_ = -1;             // 锁定的目标码 ID (-1 = 未锁定)
  double lost_id_time_ = 0.0;       // 锁定目标持续丢失时长
  bool start_pending_ = false;      // landing_start 已挂起 (等飞控状态到达后补执行)
  rclcpp::Time start_pending_deadline_{0, 0, RCL_ROS_TIME};  // 挂起超时时刻
  rclcpp::Time last_mode_cmd_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tick_{0, 0, RCL_ROS_TIME};
  double img_w_ = 0.0;   // 最近一帧图像宽 (物理宽度换算的 fx 估算用)
};

}  // namespace flylab_vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<flylab_vision::ArucoLandingNode>());
  rclcpp::shutdown();
  return 0;
}
