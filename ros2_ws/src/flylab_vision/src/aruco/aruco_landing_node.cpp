// Copyright 2026 flylab
/// ArUco 视觉降落节点 —— 三态状态机
///
///   IDLE     等待 landing_start 指令, 不发任何飞控指令
///   TRACKING 接管 offboard → 全并行耦合控制: 图像空间水平对中 + 按目标面积
///            调速下降 + 云台对准码朝向 (+可调偏置, 可关) 同时输出, 下降速度
///            受对齐/航向门控平滑调度 (误差超容差线性收小, 近地且航向未锁停降)
///            → (对齐 + 面积阈值 + 降落趋势角 + 云台/机头双对齐锁定) 连续 N 帧后进入 LANDING
///   LANDING  切 PX4 AUTO.LAND, 上锁后复位回 IDLE
///
/// 降落模式 (landing_mode 参数, 地面站"接管控制→视觉反控"子菜单选择):
///   image  (默认) 图像空间控制: 只用 cx/cy/面积/码朝向角, 不依赖 3D 位姿
///          (未标定时尺度不可信)
///   pose3d 3D 位姿控制: 位姿尺度可信 (已标定或码边长准确) 时, tvec 经云台
///          姿态旋到机体系, 再经机体姿态旋到 NED 加机体位置 = 码的世界系
///          (NED) 落点估计 —— 落点固定于世界, 对其做野值剔除 + EMA 低通
///          (顺带压掉 IPPE 深度噪声), 误差与速度指令全在世界系里算, 只把
///          最终速度转回机体系下发。不在机体系闭环的原因: 机体前倾 θ 时
///          "世界正下方的码"在机体系读出 h·sinθ 的虚假前向误差, 而前飞
///          又需要前倾 —— 正反馈, k·h≥g 即失稳 (12m 高度恰在边界);
///          世界系落点固定后该耦合消失。水平 P 增益直接作用于米级误差,
///          下降速度随离码高度线性调度, 切 AUTO.LAND 条件换算成米级
///          (水平误差 + 离码高度); 云台对准码朝向 + 机头跟随逻辑两模式相同
/// 码朝向测量: 码 0° 方向 (OpenCV 角点 c0→c1) 经云台姿态旋到机体系取水平方位,
/// 与水平对齐同一姿态链; 云台对准码 0° 方向 + 偏置 (yaw_offset, 地面站右栏可改),
/// 对准后画面中码 0° 方向停在屏幕上方偏右 yaw_offset 处; 机头经跟随通道
/// 追随云台 (请求相机侧反卷 + 订阅 gimbal_yaw_err 注入偏航角速率),
/// 云台对准且机头跟上 (双对齐连续 N 帧) 后锁定航向。
/// 多尺度嵌套码支持: target_id 指定精准落点码 (小码), 不可见时回退跟踪画面
/// 最大码 (高空大码), 指定码出现后自动切换对齐目标 (target_fallback:=false 关闭回退)。
/// 执行层经 px4_offboard_client (link:=mavros|uxrce 选链路)。
///
/// 话题: 图像 /flylab/display/image_raw (地面站转发帧, 需俯视看到码);
///       云台 /flylab/gimbal_state (相机相对机体姿态, 必需, 断流则悬停等待);
///       云台指令 /flylab/gimbal_cmd ([yaw_rate, pitch_rate] °/s, 地面站软云台);
///       跟随请求 /flylab/gimbal_follow (开相机侧反卷, Bool, 开启期 0.5s 保活);
///       跟随误差 /flylab/gimbal_yaw_err (反卷后的云台偏航, latched);
///       朝向偏置 /flylab/landing_yaw_offset (地面站右栏选择, latched);
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
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <opencv2/core.hpp>

#include "aruco/aruco_detector.hpp"
#include "aruco/landing_math.hpp"
#include "aruco/target_selector.hpp"
#include "aruco/yaw_aligner.hpp"
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

// 纯数学助手 (clampd/quat_rot/landing_angle_deg/wrap180/descend_schedule/
// TrendAngleEstimator) 抽到 include/aruco/landing_math.hpp

// AttitudeNED -> 四元数消息 (quat_rot 的输入约定: R(q)·v)
geometry_msgs::msg::Quaternion quat_from(const Px4OffboardClientBase::AttitudeNED & a)
{
  geometry_msgs::msg::Quaternion q;
  q.w = a.w; q.x = a.x; q.y = a.y; q.z = a.z;
  return q;
}

// 四元数共轭 (单位四元数的逆: R(q*)·v = R(q)^T·v, 反向旋转)
geometry_msgs::msg::Quaternion quat_conj(const geometry_msgs::msg::Quaternion & q)
{
  geometry_msgs::msg::Quaternion r;
  r.w = q.w; r.x = -q.x; r.y = -q.y; r.z = -q.z;
  return r;
}

double point3_dist(const cv::Point3d & a, const cv::Point3d & b)
{
  const cv::Point3d d = a - b;
  return std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
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
    align_gate_width_ = declare_parameter<double>("align_gate_width", 0.15);  // 对齐门控斜坡宽度 (归一化误差): 对齐误差超出 align_tol 后下降线性衰减到 0
    const double yaw_gate_width = declare_parameter<double>("yaw_gate_width", 15.0);  // 航向门控斜坡宽度 (度): 云台/跟随偏差超出 yaw_tol 后下降线性衰减到 0
    floor_alt_ = declare_parameter<double>("floor_alt", 1.0);                 // 近地高度下限 (m): 低于此高度且航向未锁定时停降; 0=禁用
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

    // 降落模式: image = 图像空间伺服 (默认, 不依赖 3D 位姿);
    // pose3d = 3D 位姿闭环 (位姿尺度可信时): tvec 经云台姿态旋到机体系,
    // 按米级水平误差 P 对中 + 下降速度随离码高度调度, 触发条件换米级
    const auto landing_mode = declare_parameter<std::string>("landing_mode", "image");
    pose3d_mode_ = landing_mode == "pose3d";
    if (!pose3d_mode_ && landing_mode != "image") {
      RCLCPP_FATAL(
        get_logger(), "未知 landing_mode '%s' (可选 image|pose3d)",
        landing_mode.c_str());
      throw std::invalid_argument("landing_mode must be image or pose3d");
    }
    // pose3d 模式参数 (image 模式忽略):
    // 水平增益 1/s —— 速度 = 增益 × 米级偏移, 增益即误差收敛时间常数的倒数
    pose_h_gain_ = declare_parameter<double>("pose_h_gain", 0.8);
    // 下降速度 = 增益 × 离码高度, 夹在 [min, max] 内再乘对齐/航向门控
    pose_descend_gain_ = declare_parameter<double>("pose_descend_gain", 0.15);
    pose_descend_max_ = declare_parameter<double>("pose_descend_max", 1.0);
    pose_descend_min_ = declare_parameter<double>("pose_descend_min", 0.2);
    pose_align_tol_ = declare_parameter<double>("pose_align_tol", 0.20);  // 水平对齐容差 (m)
    pose_gate_width_ = declare_parameter<double>("pose_gate_width", 0.50);  // 对齐门控斜坡宽度 (m)
    pose_land_alt_ = declare_parameter<double>("pose_land_alt", 1.5);  // 切 AUTO.LAND 的离码高度 (m)
    // 落点估计滤波 (NED 世界系): 码固定于世界, EMA 低通压 IPPE 深度噪声;
    // 单帧跳变超过野值门限 (IPPE 歧义/角点失败) 直接剔除保旧值
    pose_filter_alpha_ = declare_parameter<double>("pose_filter_alpha", 0.25);
    pose_outlier_m_ = declare_parameter<double>("pose_outlier_m", 1.5);
    // 云台支点相对机体中心的安装偏置 (机体系 FRD, 米): pose3d 模式把码偏移
    // 平移到机体中心; 支点在旋转中心时全 0 (AirSim 云台默认在机体原点)
    cam_offset_fwd_ = declare_parameter<double>("cam_offset_fwd", 0.0);
    cam_offset_right_ = declare_parameter<double>("cam_offset_right", 0.0);
    cam_offset_down_ = declare_parameter<double>("cam_offset_down", 0.0);

    // 降落朝向对齐 (全程并行, 不等水平对齐): 云台对准码 0° 方向 + 偏置,
    // 机头经跟随通道同步追随云台; 云台对准且机头跟上连续 N 帧锁定航向
    // (AUTO.LAND 保持切换瞬间航向); 对齐期间下降受航向门控平滑压低
    yaw_align_ = declare_parameter<bool>("yaw_align", true);
    yaw_offset_deg_ = declare_parameter<double>("yaw_offset_deg", 0.0);
    const double yaw_tol_deg = declare_parameter<double>("yaw_tol_deg", 5.0);
    const double yaw_gain = declare_parameter<double>("yaw_gain", 2.5);
    yaw_rate_max_deg_ = declare_parameter<double>("yaw_rate_max_deg", 30.0);
    const int min_yaw_frames = declare_parameter<int>("min_yaw_frames", 5);
    const double yaw_timeout_sec = declare_parameter<double>("yaw_timeout_sec", 30.0);
    const auto yaw_offset_topic = declare_parameter<std::string>(
      "yaw_offset_topic", "/flylab/landing_yaw_offset");

    // 算法云台通道 (地面站软云台): 朝向对齐经 [yaw_rate, pitch_rate] (°/s)
    // 速率指令驱动, 复用摇杆速率通道与限位; 正偏航 = 视线方位右转
    const auto gimbal_cmd_topic = declare_parameter<std::string>(
      "gimbal_cmd_topic", "/flylab/gimbal_cmd");

    // 机头跟随云台 (offboard 单写者原则: 偏航指令由本节点注入, 地面站只做
    // 相机侧反卷与误差广播): 对齐中请求 /flylab/gimbal_follow 开反卷
    // (世界锚定, 机体转动时相对偏航收敛), 订阅 /flylab/gimbal_yaw_err
    // (反卷后的云台偏航 = 视线世界方位 − 航向, 右+, NaN=未跟随, 1s 保活),
    // 注入偏航角速率 = follow_kp × err; 云台对准且机头跟上才恢复下降
    const double follow_kp = declare_parameter<double>("follow_kp", 0.8);
    const double follow_tol_deg = declare_parameter<double>("follow_tol_deg", 5.0);
    const double follow_rate_max_deg =
      declare_parameter<double>("follow_rate_max_deg", 30.0);
    follow_fresh_sec_ = declare_parameter<double>("follow_fresh_sec", 1.5);
    const auto follow_req_topic = declare_parameter<std::string>(
      "follow_req_topic", "/flylab/gimbal_follow");
    const auto follow_err_topic = declare_parameter<std::string>(
      "follow_err_topic", "/flylab/gimbal_yaw_err");

    // ---------- 目标选择 / 朝向对齐 / 趋势角子系统 (逻辑拆分, 参数在此装配) ----------
    // 逻辑见 include/aruco/target_selector.hpp 与 yaw_aligner.hpp
    selector_ = std::make_unique<TargetSelector>(get_logger(),
      TargetSelector::Params{target_id_, target_fallback_, reacquire_sec_});
    yaw_aligner_ = std::make_unique<YawAligner>(get_logger(), get_clock(),
      YawAligner::Params{yaw_align_, yaw_offset_deg_, yaw_tol_deg, yaw_gain,
        yaw_rate_max_deg_, min_yaw_frames, yaw_timeout_sec, follow_kp,
        follow_tol_deg, follow_rate_max_deg, yaw_gate_width});
    trend_est_.setWindow(angle_window_min_, angle_window_max_);

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

    // ---------- 地面站码边长同步 ----------
    // 场景含多种尺寸码时, 地面站右栏滑杆运行中同步 marker_length:
    // 影响位姿尺度 (面板"实际宽度/距离"换算), 降落控制只用图像空间量
    // 不受影响。latched 话题, 后启动的节点一接入即收到最近值。
    const auto marker_len_topic = declare_parameter<std::string>(
      "marker_length_topic", "/flylab/aruco_marker_length");
    marker_length_ = marker_length;
    marker_len_sub_ = create_subscription<std_msgs::msg::Float64>(
      marker_len_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {on_marker_length(*msg);});

    // ---------- 地面站字典同步 ----------
    // 地面站右栏下拉切换 ArUco 字典即发布 (latched, 后启动的节点一接入即收到
    // 最近值); 运行中换字典后锁定目标码可能不在新字典里, 重选目标并复位
    // 朝向滤波按新字典重新对齐。话题名可用参数覆盖。
    const auto dict_topic = declare_parameter<std::string>(
      "dict_topic", "/flylab/aruco_dict");
    dict_name_ = dict_name;
    dict_sub_ = create_subscription<std_msgs::msg::String>(
      dict_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {on_dict(*msg);});

    // ---------- 地面站视场角同步 ----------
    // 未标定时内参按 FOV 估算 (位姿轴叠加显示), 地面站调整视场滑杆实时跟随;
    // latched 话题, 节点后启动也能收到最近一次发布的值。话题名可用参数覆盖。
    const auto fov_topic =
      declare_parameter<std::string>("fov_topic", "/flylab/camera_fov");
    fov_deg_ = fov_deg;
    fov_sub_ = create_subscription<std_msgs::msg::Float64>(
      fov_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {on_fov(*msg);});

    // ---------- 跟踪码 ID 广播 ----------
    // 每帧发布当前锁定/回退跟踪的码 ID (无则 -1): 检测节点 (aruco_node)
    // 据此让位 —— 跟踪码恰为画面最大码时不再叠画洋红 MAX, 由本节点橙色
    // TRACK 独占该码的可视化; 广播停发后检测节点超时自行恢复 MAX
    const auto tracked_id_topic = declare_parameter<std::string>(
      "tracked_id_topic", "/flylab/landing_tracked_id");
    tracked_id_pub_ = create_publisher<std_msgs::msg::Int32>(
      tracked_id_topic, 10);

    // ---------- 地面站降落朝向偏置同步 ----------
    // 右栏面板选择机头相对码 0° 方向的落点朝向 (度, 右正); latched 话题
    // (订阅端 transient_local, 后启动的节点一接入即收到最近值), 运行中
    // 修改会解锁已锁定的航向、按新朝向重新对齐
    yaw_offset_sub_ = create_subscription<std_msgs::msg::Float64>(
      yaw_offset_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {on_yaw_offset(*msg);});

    // ---------- 算法云台通道 ----------
    // 朝向对齐的执行端: 地面站软云台速率指令 ([yaw_rate, pitch_rate] °/s,
    // 反控中云台面板锁定只锁 UI, 算法通道不受影响)
    gimbal_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      gimbal_cmd_topic, 10);

    // ---------- 机头跟随通道 ----------
    // 跟随请求: 开启 0.5s 保活重发 (自愈地面站侧悄悄退出的失步),
    // 关闭只发一次; 误差广播 latched 订阅 (后启动即拿当前值),
    // 新鲜度靠地面站 1s 保活重发判定
    follow_req_pub_ = create_publisher<std_msgs::msg::Bool>(follow_req_topic, 10);
    follow_err_sub_ = create_subscription<std_msgs::msg::Float64>(
      follow_err_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {on_follow_err(*msg);});

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
      "ArUco 降落节点就绪 (三态状态机, 模式=%s): 等待 %s 指令; 图像 %s; PX4 链路=%s (需已解锁升空的飞机)",
      pose3d_mode_ ? "pose3d" : "image",
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
    selector_->reset();        // 重新进入降落流程重新选目标
    yaw_aligner_->reset();    // 航向重新对齐
    marker_ned_valid_ = false;  // 落点估计滤波复位 (重新捕获)
    marker_ned_id_ = -1;
    gimbal_follow_req_ = false;   // 跟随请求复位 (上次运行若遗留, 地面站心跳超时自退)
    trend_est_.reset();
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
    gimbalYawRate(0.0);      // 停云台 (速率指令持续生效, 必须显式清零)
    requestGimbalFollow(false);   // 停跟随 (相机切回跟随机头)
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

  /// 码 0° 方向 (OpenCV 角点 c0→c1, 即码图案 x 轴) 在机体系的方位角
  /// (度, 右正): 图像内平面方向 (0, dx, dy) 经云台姿态旋到机体系取水平
  /// 方位 —— 与水平对齐同一姿态链, 不假设相机垂直向下。
  /// 须在 gimbalFresh() 成立时调用 (tickTracking 已保证)
  double markerYawDeg(const Detection & d) const
  {
    const double dx = d.corners[1].x - d.corners[0].x;
    const double dy = d.corners[1].y - d.corners[0].y;
    const cv::Point3d b = quat_rot(cam_ori_, 0.0, dx, dy);
    return std::atan2(b.y, b.x) * 180.0 / M_PI;
  }

  /// 算法云台通道: 偏航角速率指令 (°/s, 右正, 限幅), 俯仰恒 0 不动。
  /// 地面站侧速率为持续语义 (置 0 才停): 值变化即发布; 跟随开启期间零值
  /// 也按 0.5s 保活重发 (兼作 follow 心跳, 断流超时地面站自动退反卷);
  /// 跟随关闭后停止只发一次零, 不再刷话题。
  void gimbalYawRate(double yaw_rate_deg)
  {
    const double rate = clampd(yaw_rate_deg, yaw_rate_max_deg_);
    const bool changed = std::fabs(rate - gimbal_rate_sent_) > 0.5;
    const bool keep = (now() - gimbal_pub_t_).seconds() >= 0.5;
    if (!changed && !(keep && (rate != 0.0 || gimbal_follow_req_))) {
      return;
    }
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {rate, 0.0};
    gimbal_cmd_pub_->publish(msg);
    gimbal_rate_sent_ = rate;
    gimbal_pub_t_ = now();
  }

  /// 机头跟随请求 (相机侧反卷, 世界锚定): 开启按 0.5s 保活重发 —— 地面站
  /// 侧跟随可能被悄悄退出 (gimbal_cmd 心跳超时/抓取线程重连复位), 节点
  /// 无感知, 重发可在 0.5s 内拉回跟随自愈失步; 关闭仍为边沿触发 (只发
  /// 一次, GCS 断流超时自退兜底)。
  void requestGimbalFollow(bool on)
  {
    const bool changed = (on != gimbal_follow_req_);
    const bool keep = (now() - follow_req_t_).seconds() >= 0.5;
    if (!changed && !(on && keep)) {
      return;
    }
    std_msgs::msg::Bool msg;
    msg.data = on;
    follow_req_pub_->publish(msg);
    gimbal_follow_req_ = on;
    follow_req_t_ = now();
  }

  // ---------------- 地面站跟随误差广播 ----------------
  void on_follow_err(const std_msgs::msg::Float64 & msg)
  {
    follow_err_valid_ = std::isfinite(msg.data);
    follow_err_ = follow_err_valid_ ? msg.data : 0.0;
    follow_err_t_ = now();
  }

  /// 跟随误差是否可用 (非 NaN 且在新鲜度内; 地面站 1s 保活 + 传输裕量)
  bool followErrUsable() const
  {
    return follow_err_valid_ && (now() - follow_err_t_).seconds() <= follow_fresh_sec_;
  }

  // ---------------- 地面站码边长同步 ----------------
  void on_marker_length(const std_msgs::msg::Float64 & msg)
  {
    // 非法值/值未变化跳过 (滑杆拖动重发布降噪)
    if (msg.data <= 0.0 || msg.data == marker_length_) {
      return;
    }
    marker_length_ = msg.data;
    detector_->set_marker_length(msg.data);
    RCLCPP_INFO(
      get_logger(), "码边长同步: %.2fm (位姿尺度随之更新, 来自地面站)",
      msg.data);
  }

  // ---------------- 地面站字典同步 ----------------
  void on_dict(const std_msgs::msg::String & msg)
  {
    // 非法值/值未变化跳过 (下拉重复选择降噪)
    if (msg.data.empty() || msg.data == dict_name_) {
      return;
    }
    try {
      detector_->set_dictionary(msg.data);
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(), "忽略无效字典 '%s': %s", msg.data.c_str(), e.what());
      return;
    }
    dict_name_ = msg.data;
    RCLCPP_INFO(
      get_logger(), "字典同步: %s (下一帧检测生效, 来自地面站)",
      dict_name_.c_str());
    // 锁定目标码在新字典下可能不存在: 丢弃锁定重新选目标, 并复位
    // 朝向滤波/计数/锁定 (码 ID 集合变了, 朝向基准失效)
    selector_->reset();
    yaw_aligner_->reset();
  }

  // ---------------- 地面站降落朝向偏置同步 ----------------
  void on_yaw_offset(const std_msgs::msg::Float64 & msg)
  {
    // 非法值/值未变化跳过
    if (!std::isfinite(msg.data) || msg.data == yaw_offset_deg_) {
      return;
    }
    yaw_offset_deg_ = msg.data;
    // 偏置变化即重新对齐: 解锁航向、清滤波与计数 (IDLE 中仅存值)
    yaw_aligner_->setOffset(msg.data);
    RCLCPP_INFO(
      get_logger(),
      "降落朝向偏置同步: %.1f° (来自地面站, 将按新朝向重新对齐)", msg.data);
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
    const int tracked_id = selector_->trackedId();
    const Detection * tracked =
      tracked_id >= 0 ? selector_->largestOfId(tracked_id, detections_) : nullptr;
    std_msgs::msg::Int32 tid;
    tid.data = tracked ? tracked_id : -1;
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
    const Detection * tgt = selector_->pick(dt, detections_);
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

    // 对齐输入量: image 模式 = 归一化画面中心/面积; pose3d 模式 = 码 3D 位姿
    const double cx = tgt->center_norm.x;
    const double cy = tgt->center_norm.y;
    const double area =
      image_area_ > 0 ? (tgt->bbox_wh.x * tgt->bbox_wh.y) / image_area_ : 0.0;
    const auto & pos = px4_->localPosition();

    // 水平控制 (输出机体系 FRD 速度) 依赖云台姿态: 数据过期则方位测量不可信
    if (!gimbalFresh()) {
      px4_->sendHover();
      gimbalYawRate(0.0);   // 云台姿态过期, 方位测量不可信, 停转
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

    // 水平对齐 —— 两模式输出同一组量, 下游 (门控下降/触发/面板) 无感差异:
    //   align_err   image=归一化画面误差, pose3d=NED 水平误差 (m)
    //   aligned     分别对 align_tol / pose_align_tol
    //   v_fwd/right image=增益×归一化误差×(1-面积)²×高度缩放, pose3d=世界系
    //               速度指令转机体系
    double align_err = 0.0, align_gate = 1.0, v_fwd = 0.0, v_right = 0.0;
    double e_n = 0.0, e_e = 0.0, h_m = -1.0;  // pose3d: NED 水平误差 (北,东) 与离码高度 (米)
    bool aligned = false;
    if (pose3d_mode_) {
      // 检出但位姿解算失败 (罕见): pose3d 模式无法米级闭环, 悬停等下一帧
      if (!tgt->has_pose) {
        px4_->sendHover();
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "\033[33m[3D位姿] 目标码 id=%d 位姿解算失败 (solvePnP), 悬停等待下一帧\033[0m",
          tgt->id);
        return;
      }
      // tvec (OpenCV 相机系 x右 y下 z前) 先换轴序到云台约定 (x视轴 y图右
      // z图下, 与 gimbal_state 四元数同一约定), 经云台姿态旋到机体系 FRD,
      // 加云台支点安装偏置 = 码相对机体中心的米级偏移 (前, 右, 下)
      cv::Point3d p_b =
        quat_rot(cam_ori_, tgt->tvec[2], tgt->tvec[0], tgt->tvec[1]);
      p_b.x += cam_offset_fwd_;
      p_b.y += cam_offset_right_;
      p_b.z += cam_offset_down_;
      // 世界系落点固定: 机体姿态旋到 NED + 机体位置 = 码的 NED 坐标估计。
      // 不在机体系闭环的原因: 机体前倾 θ 时"世界正下方的码"在机体系读出
      // h·sinθ 的虚假前向误差, 而前飞又需要前倾 —— 正反馈 (k·h≥g 即失稳,
      // 12m 高度恰在边界); 世界系量与机体姿态解耦, 该耦合消失
      const auto att = px4_->attitudeNED();
      if (!att.valid || !pos.xy_valid || !pos.z_valid) {
        px4_->sendHover();
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "\033[33m[3D位姿] 机体%s数据不可用, 原地悬停等待\033[0m",
          !att.valid ? "姿态" : "位置");
        return;
      }
      // 目标码切换即复位滤波 (不同码落点不同)
      if (tgt->id != marker_ned_id_) {
        marker_ned_id_ = tgt->id;
        marker_ned_valid_ = false;
      }
      const geometry_msgs::msg::Quaternion aq = quat_from(att);
      // p_b (FRD) -> NED: NED→FRD 四元数取共轭反向旋转
      const cv::Point3d p_n = quat_rot(quat_conj(aq), p_b.x, p_b.y, p_b.z);
      const cv::Point3d mk{pos.x + p_n.x, pos.y + p_n.y, pos.z + p_n.z};
      // 野值剔除 + EMA 低通: 码固定于世界, 对落点估计滤波合法, 顺带压掉
      // IPPE 深度噪声 (角点 1px ≈ 0.3m @12m); 单帧跳变超门限剔除保旧值
      if (!marker_ned_valid_) {
        marker_ned_ = mk;
        marker_ned_valid_ = true;
      } else if (point3_dist(mk, marker_ned_) <= pose_outlier_m_) {
        marker_ned_ += pose_filter_alpha_ * (mk - marker_ned_);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "\033[33m[3D位姿] 落点估计野值 (%.2fm 跳变) 已剔除, 保旧值\033[0m",
          point3_dist(mk, marker_ned_));
      }
      // 误差与离码高度 (NED, z 下正): e 指向码
      e_n = marker_ned_.x - pos.x;
      e_e = marker_ned_.y - pos.y;
      h_m = marker_ned_.z - pos.z;
      align_err = std::hypot(e_n, e_e);
      aligned = align_err < pose_align_tol_;
      align_gate = std::fmin(1.0, std::fmax(0.0,
        1.0 - (align_err - pose_align_tol_) / std::fmax(1e-3, pose_gate_width_)));
      // 世界系速度指令 -> 机体系 FRD (NED→FRD 四元数正向旋转, 只取水平两轴;
      // 机体俯仰/横滚不再进入误差与指令, 俯仰正反馈消失)
      const cv::Point3d v_b =
        quat_rot(aq, pose_h_gain_ * e_n, pose_h_gain_ * e_e, 0.0);
      v_fwd = clampd(v_b.x, max_h_vel_);
      v_right = clampd(v_b.y, max_h_vel_);
    } else {
      const double dist_factor = (1.0 - area) * (1.0 - area);  // 距离越近增益越小, 防近距振荡
      // 高度增益缩放: 归一化误差在高空对应更大水平距离, 按高度/基准高度放大
      // 增益使收敛速率与高度无关; 封顶防高空过冲, 低空不放大
      double height_scale = 1.0;
      if (align_ref_alt_ > 0.0 && pos.z_valid) {
        const double h = std::fmax(0.5, -pos.z);
        height_scale = std::fmin(
          std::fmax(h / align_ref_alt_, 1.0), max_height_scale_);
      }
      // 用相机姿态四元数把"图右/图下"方向 R(q)·(0,1,0)、R(q)·(0,0,1) 旋到
      // 机体系, 垂直分量只取水平投影 —— 不假设相机垂直向下, 刚性/自稳/yaw
      // 随动云台全适用
      const cv::Point3d u_b = quat_rot(cam_ori_, 0.0, 1.0, 0.0);  // 图右 (机体系)
      const cv::Point3d v_b = quat_rot(cam_ori_, 0.0, 0.0, 1.0);  // 图下 (机体系)
      const double k = vel_gain_ * dist_factor * height_scale;
      v_fwd = clampd(k * ((cx - land_cx_) * u_b.x + (cy - land_cy_) * v_b.x), max_h_vel_);
      v_right = clampd(k * ((cx - land_cx_) * u_b.y + (cy - land_cy_) * v_b.y), max_h_vel_);
      align_err = std::hypot(cx - land_cx_, cy - land_cy_);
      aligned = align_err < align_tol_;
      align_gate = std::fmin(1.0, std::fmax(0.0,
        1.0 - (align_err - align_tol_) / std::fmax(1e-3, align_gate_width_)));
    }

    // 全并行耦合控制: 水平对中 / 下降 / 朝向对齐同时输出, 不再分阶段。
    // 下降速度 = 距离调度 (image=面积, pose3d=离码高度) × 对齐门控 × 航向门控,
    // 门控为线性斜坡: 误差在容差内=1, 超出容差后随误差线性衰减到 0 ——
    // 下降会放大误差, 门控自动收小停降回去对齐, 自平衡

    // ---------------- 朝向对齐 (全程并行: 云台 → 码 0° 方向 + 偏置, 机头跟随) ----------------
    // 状态机/滤波/锁定/门控在 YawAligner (include/aruco/yaw_aligner.hpp);
    // 这里只提供测量输入: 云台方位 = 画面上方 (0,0,-1) 经云台姿态旋到机
    // 体系取水平方位 (与码方位同一姿态链, 不假设相机垂直向下), 码方位见
    // markerYawDeg()。跟随请求的收发 (requestGimbalFollow) 属 ROS 管道留在本节点
    const cv::Point3d up_b = quat_rot(cam_ori_, 0.0, 0.0, -1.0);
    const double gimbal_az = std::atan2(up_b.y, up_b.x) * 180.0 / M_PI;
    const auto yaw = yaw_aligner_->update(
      dt, tgt->id, markerYawDeg(*tgt), gimbal_az, followErrUsable(), follow_err_);
    // 跟随接入: 未锁定期间持续请求相机侧反卷 (0.5s 保活自愈失步), 锁定
    // (含超时放行) 边沿切回跟随机头 (视角连续不跳变); 关闭对齐不请求
    requestGimbalFollow(yaw_align_ && !yaw.locked);
    gimbalYawRate(yaw.gimbal_rate);

    // 下降速度合成: image = 面积调度 (近慢远快); pose3d = 离码高度线性调度
    // (增益×高度, 夹在 [min, max] 保证低空仍有下降进度) —— 均乘对齐/航向门控
    double v_down;
    if (pose3d_mode_) {
      const double vd = pose_descend_gain_ * std::fmax(0.0, h_m);
      v_down = std::fmin(pose_descend_max_, std::fmax(pose_descend_min_, vd)) *
        align_gate * yaw.yaw_gate;
    } else {
      v_down =
        descend_schedule(area, descend_close_, descend_far_, area_s_thresh_) *
        align_gate * yaw.yaw_gate;
    }
    // 近地高度下限: 低于此高度且航向未锁定时停降 (AUTO.LAND 航向在切换
    // 瞬间锁定, 低空带误差下降会来不及锁航向), 悬停等对齐
    if (floor_alt_ > 0.0 && yaw_align_ && !yaw.locked && pos.z_valid &&
      -pos.z < floor_alt_)
    {
      v_down = 0.0;
    }

    // 降落趋势角: 滑动窗口内位移与竖直方向的夹角 (窗口随面积自适应)
    trend_est_.update(area, pos.xy_valid && pos.z_valid,
      cv::Point3d{pos.y, pos.x, -pos.z});   // NED -> ENU

    // 触发条件: 对齐 + 足够近 (image=面积达标, pose3d=离码高度达标) +
    // 趋势角够垂直 + 航向已锁定, 连续 N 帧确认
    const bool near_enough = pose3d_mode_ ? h_m <= pose_land_alt_ :
      area >= land_area_thresh_;
    if (aligned && near_enough &&
      trend_est_.angle() < vertical_threshold_deg_ && (!yaw_align_ || yaw.locked))
    {
      landing_count_++;
    } else {
      landing_count_ = 0;
    }
    if (landing_count_ >= min_landing_frames_) {
      if (pose3d_mode_) {
        RCLCPP_INFO(
          get_logger(),
          "\033[32m降落条件满足 (3D位姿: 水平误差 %.3fm 容差%.2f, 离码高度 %.2fm 门限%.1f, 趋势角 %.1f°), "
          "切 PX4 AUTO.LAND\033[0m",
          align_err, pose_align_tol_, h_m, pose_land_alt_, trend_est_.angle());
      } else {
        RCLCPP_INFO(
          get_logger(),
          "\033[32m降落条件满足 (图像空间: 面积 %.3f, 对齐误差 %.3f, 趋势角 %.1f°), 切 PX4 AUTO.LAND\033[0m",
          area, align_err, trend_est_.angle());
      }
      gimbalYawRate(0.0);          // 停云台
      requestGimbalFollow(false);  // 停跟随
      state_ = State::LANDING;
      last_mode_cmd_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      publishStatus(kStatusLanding);
      return;
    }

    if (yaw.turning) {
      // 全并行: 水平 P 对中 + 门控下降 + 偏航角速率 (rad/s, 正=右转) =
      // follow_kp × 反卷跟随误差, 机头追上视线后云台相对偏航收敛回中,
      // 与云台环共同收敛到 "码方位相对机头 = yaw_offset" 的不动点
      px4_->sendVelocityFRDYawRate(v_fwd, v_right, v_down, yaw.yaw_rate_rad);
    } else {
      px4_->sendVelocityFRD(v_fwd, v_right, v_down);
    }
    // 右栏"算法输出"面板: 对齐下降状态摘要 (10Hz 限频)
    if (infoDue(now_t)) {
      const double h = pos.z_valid ? -pos.z : -1.0;
      std::ostringstream js;
      js << std::fixed << std::setprecision(3)
         << "{\"src\":\"landing\",\"state\":\"TRACKING\",\"mode\":\"" <<
        (pose3d_mode_ ? "pose3d" : "image") << "\",\"tracked_id\":"
         << selector_->trackedId() << ",\"count\":" << detections_.size()
         << ",\"area\":" << area
         << ",\"w\":" << static_cast<int>(tgt->bbox_wh.x + 0.5)
         << ",\"h\":" << static_cast<int>(tgt->bbox_wh.y + 0.5)
         << ",\"wm\":" << physicalWidth(*tgt);
      // 码中心在相机坐标系位置 (x右 y下 z前, 米); 仅 has_pose 时有
      if (tgt->has_pose) {
        js << ",\"tvec\":[" << tgt->tvec[0] << ',' << tgt->tvec[1]
           << ',' << tgt->tvec[2] << ']';
      }
      // pose3d 模式补充米级闭环量: err=机体系水平误差(m), h_m=离码高度(m)
      if (pose3d_mode_) {
        js << ",\"err_m\":" << align_err << ",\"h_m\":" << h_m;
      }
      js << ",\"err\":" << align_err
         << ",\"aligned\":" << (aligned ? "true" : "false")
         << ",\"lost\":false"
         << std::setprecision(2)
         << ",\"height\":" << h
         << std::setprecision(1)
         << ",\"trend\":" << trend_est_.angle()
         << std::setprecision(2)
         << ",\"vel\":[" << v_fwd << ',' << v_right << ',' << v_down << ']'
         << ",\"a_gate\":" << align_gate
         << ",\"y_gate\":" << yaw.yaw_gate
         << ",\"trigger\":" << landing_count_
         << ",\"trigger_need\":" << min_landing_frames_;
      // 航向对齐状态 (面板"航向"行; 未启用不发, 面板显示 '—')
      if (yaw_align_) {
        js << ",\"yaw_en\":true"
           << ",\"yaw_err\":" << yaw.yaw_err_deg
           << std::setprecision(0)
           << ",\"yaw_off\":" << yaw_offset_deg_
           << ",\"yaw_locked\":" << (yaw.locked ? "true" : "false");
      }
      js << '}';
      publishInfo(js.str());
    }
    // 1Hz 详细状态行: 降落每一步在干什么、为什么还不触发, 一眼可见
    if (pose3d_mode_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[TRACKING][3D位姿] 高度=%.2fm 检出=%zu 选中 id=%d "
        "水平误差=(北%.2f, 东%.2f)m 合计=%.3f (容差%.2f%s 门控%.2f) "
        "离码高度=%.2fm (门限%.1fm) 趋势角=%.0f°(门限%.0f°) "
        "云台对齐=%.1f°(偏置%.0f° %s 门控%.2f) 跟随=%.1f°(%s) "
        "输出速度(前,右,下)=(%.2f, %.2f, %.2f) 触发计数=%d/%d",
        pos.z_valid ? -pos.z : 0.0,
        detections_.size(), tgt->id,
        e_n, e_e, align_err, pose_align_tol_,
        aligned ? "已对齐" : "未对齐", align_gate,
        h_m, pose_land_alt_,
        trend_est_.angle(), vertical_threshold_deg_,
        yaw_align_ ? yaw.yaw_err_deg : 0.0, yaw_offset_deg_,
        !yaw_align_ ? "关闭" : (yaw.locked ? "已锁定" : "对齐中"),
        yaw.yaw_gate,
        yaw_align_ && followErrUsable() ? follow_err_ : 0.0,
        yaw_align_ ? (followErrUsable() ? "可用" : "不可用") : "—",
        v_fwd, v_right, v_down, landing_count_, min_landing_frames_);
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[TRACKING][云台泛化] 高度=%.2fm 检出=%zu 选中 id=%d area=%.3f cx=%.2f cy=%.2f "
        "对齐误差=%.3f (容差%.3f%s 门控%.2f) 趋势角=%.0f°(门限%.0f°) "
        "云台对齐=%.1f°(偏置%.0f° %s 门控%.2f) 跟随=%.1f°(%s) "
        "输出速度(前,右,下)=(%.2f, %.2f, %.2f) 触发计数=%d/%d",
        pos.z_valid ? -pos.z : 0.0,
        detections_.size(), tgt->id, area, cx, cy,
        align_err, align_tol_, aligned ? "已对齐" : "未对齐", align_gate,
        trend_est_.angle(), vertical_threshold_deg_,
        yaw_align_ ? yaw.yaw_err_deg : 0.0, yaw_offset_deg_,
        !yaw_align_ ? "关闭" : (yaw.locked ? "已锁定" : "对齐中"),
        yaw.yaw_gate,
        yaw_align_ && followErrUsable() ? follow_err_ : 0.0,
        yaw_align_ ? (followErrUsable() ? "可用" : "不可用") : "—",
        v_fwd, v_right, v_down, landing_count_, min_landing_frames_);
    }
  }

  /// 目标丢失处理 (TRACKING): 原地悬停, 超时切 AUTO_LOITER 交还控制权
  void handleTargetLost(double dt)
  {
    // 原地悬停, 超时交还控制权
    px4_->sendHover();
    gimbalYawRate(0.0);   // 无码可测向, 停转防云台漂到限位
    lost_time_ += dt;
    // 右栏面板: 丢失状态 (10Hz 限频)
    if (infoDue(now())) {
      const auto & pos = px4_->localPosition();
      const double h = pos.z_valid ? -pos.z : -1.0;
      std::ostringstream js;
      js << std::fixed << std::setprecision(3)
         << "{\"src\":\"landing\",\"state\":\"TRACKING\",\"tracked_id\":"
         << selector_->trackedId() << ",\"count\":" << detections_.size()
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

  // 下降速度调度 descend_schedule() 与趋势角估计 TrendAngleEstimator
  // 抽到 include/aruco/landing_math.hpp

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
    std_msgs::msg::String out;
    out.data = std::string("{\"algo\":\"") + kAnnoAlgo + "\",\"items\":[]}";
    anno_pub_->publish(out);
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
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gimbal_cmd_pub_;  // 算法云台通道
  double gimbal_rate_sent_ = 0.0;                  // 最近下发的云台偏航速率 (°/s, 变化检测)
  rclcpp::Time gimbal_pub_t_{0, 0, RCL_ROS_TIME};  // 最近下发时刻 (保活节流)

  // 机头跟随通道 (云台对齐期间机头追随云台)
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr follow_req_pub_;   // 反卷请求
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr follow_err_sub_;
  bool gimbal_follow_req_ = false;   // 已请求跟随 (云台指令保活判定 + 请求去重)
  rclcpp::Time follow_req_t_{0, 0, RCL_ROS_TIME};  // 最近请求发布时刻 (0.5s 保活节流)
  bool follow_err_valid_ = false;    // 最近收到的跟随误差有效 (非 NaN)
  double follow_err_ = 0.0;          // 最近收到的跟随误差 (度, 右+)
  rclcpp::Time follow_err_t_{0, 0, RCL_ROS_TIME};  // 误差到达时刻 (新鲜度)
  rclcpp::Time last_info_pub_{0, 0, RCL_ROS_TIME};                // 10Hz 限频
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr gimbal_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr fov_sub_;
  double fov_deg_ = 60.0;   // 当前生效的 FOV (视场角同步去重用)
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr marker_len_sub_;
  double marker_length_ = 0.2;  // 当前生效的码边长 (同步去重用)
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dict_sub_;
  std::string dict_name_;       // 当前生效的字典 (同步去重用)
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
  double align_tol_, land_area_thresh_, vertical_threshold_deg_;
  double descend_close_, descend_far_, area_s_thresh_;
  int min_landing_frames_, angle_window_min_, angle_window_max_;
  double lost_timeout_sec_, offboard_warmup_sec_, offboard_retry_sec_, control_hz_;
  double reacquire_sec_;
  bool yaw_align_;
  double yaw_offset_deg_;   // 当前生效偏置 (同步去重/面板显示; 状态机内部持有副本)
  double yaw_rate_max_deg_;  // 云台速率限幅 (°/s, gimbalYawRate 通道用)
  double follow_fresh_sec_;
  double align_gate_width_, floor_alt_;

  // 3D 位姿降落模式 (landing_mode:=pose3d; image 模式忽略)
  bool pose3d_mode_ = false;
  double pose_h_gain_;               // 水平 P 增益 (1/s): 速度 = 增益 × 米级偏移
  double pose_descend_gain_;         // 下降 P 增益 (1/s): 速度 = 增益 × 离码高度
  double pose_descend_max_, pose_descend_min_;   // 下降速度限幅 (m/s, 门控前)
  double pose_align_tol_;            // 水平对齐容差 (m)
  double pose_gate_width_;           // 对齐门控斜坡宽度 (m)
  double pose_land_alt_;             // 切 AUTO.LAND 的离码高度门限 (m)
  double cam_offset_fwd_, cam_offset_right_, cam_offset_down_;  // 云台支点安装偏置 (机体系 FRD, m)
  double pose_filter_alpha_;   // 落点估计 EMA 系数 (0~1, 越小越平滑)
  double pose_outlier_m_;      // 落点估计野值剔除门限 (m)

  // 落点估计 (NED 世界系固定, pose3d 模式): EMA 滤波值 + 有效性 + 所属码 ID
  cv::Point3d marker_ned_{0.0, 0.0, 0.0};
  bool marker_ned_valid_ = false;
  int marker_ned_id_ = -1;

  // 运行时状态
  double lost_time_ = 0.0;          // 目标连续丢失时长
  int landing_count_ = 0;           // 降落条件连续满足帧数
  TrendAngleEstimator trend_est_;   // 降落趋势角估计 (landing_math.hpp)

  // 拆分子系统 (构造函数中按参数装配)
  std::unique_ptr<TargetSelector> selector_;  // 目标码选择 (target_selector.hpp)
  std::unique_ptr<YawAligner> yaw_aligner_;   // 朝向对齐状态机 (yaw_aligner.hpp)
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_offset_sub_;
  bool land_confirmed_ = false;     // 已确认 AUTO.LAND (打一次日志)
  bool had_target_ = false;         // 目标捕获/丢失事件日志用
  bool ever_offboard_ = false;      // 本次流程曾进入过 OFFBOARD (判定外部接管用)
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
