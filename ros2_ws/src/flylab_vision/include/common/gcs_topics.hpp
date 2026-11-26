// Copyright 2026 flylab
#ifndef FLYLAB_VISION__COMMON__GCS_TOPICS_HPP_
#define FLYLAB_VISION__COMMON__GCS_TOPICS_HPP_

/// 地面站 <-> 视觉节点的话题契约 (flylab_gcs 的 config/topics.py 是本文件的
/// Python 侧对应物, 两边命名一一对应, 改话题名必须两处同步)。
///
/// 本包内所有 /flylab/* 话题的字符串字面量集中在此; 节点经
/// declare_parameter("xxx_topic", topics::XXX_TOPIC) 引用为默认值,
/// 仍可用 ros2 参数覆盖。QoS / 话题语义见 topics.py 与各节点注释。

namespace flylab_vision
{
namespace topics
{

// ---------- 展示型节点公共链路 (common/display_vision_node.hpp) ----------
constexpr const char * DISPLAY_IMAGE_TOPIC = "/flylab/display/image_raw";  // 地面站当前显示帧 (算法图像输入)
constexpr const char * CMD_TOPIC = "/flylab/vision_cmd";                   // 算法启停指令 (std_msgs/String)
constexpr const char * ANNOTATION_TOPIC = "/flylab/annotations";           // 画面叠加标注 (JSON, schema 见 overlay.py)
constexpr const char * ALGO_INFO_TOPIC = "/flylab/algo_info";              // 右栏算法信息面板 ({"src":...} JSON)

// ---------- 深度估计 ----------
constexpr const char * DEPTH_MAP_TOPIC = "/flylab/depth_map";              // 降采样深度图 (16UC1 毫米, 悬停读数)

// ---------- 云台 / 相机 (算法通道与同步, 见 topics.py 航向跟随一节) ----------
constexpr const char * GIMBAL_STATE_TOPIC = "/flylab/gimbal_state";        // 地面站 10Hz 发布相机姿态四元数
constexpr const char * GIMBAL_CMD_TOPIC = "/flylab/gimbal_cmd";            // 算法 → 云台 [yaw_rate, pitch_rate] (°/s), 兼作 follow 心跳
constexpr const char * GIMBAL_FOLLOW_REQ_TOPIC = "/flylab/gimbal_follow";  // 算法请求相机侧反卷 (Bool)
constexpr const char * GIMBAL_YAW_ERR_TOPIC = "/flylab/gimbal_yaw_err";    // 地面站广播反卷后偏航误差 (度, latched, NaN=未跟随)
constexpr const char * CAMERA_FOV_TOPIC = "/flylab/camera_fov";            // 视场角同步 (Float64, latched, 按 FOV 估内参的算法订阅)

// ---------- latched 运行时参数同步 (约定: 后启动节点一接入即拿到当前值) ----------
constexpr const char * ARUCO_MARKER_LENGTH_TOPIC = "/flylab/aruco_marker_length";  // ArUco 码边长 (米)
constexpr const char * ARUCO_DICT_TOPIC = "/flylab/aruco_dict";                    // ArUco 字典名
constexpr const char * FRACTAL_CONFIG_TOPIC = "/flylab/fractal_config";            // fractal 分形配置名
constexpr const char * FACE_TASK_TOPIC = "/flylab/face_task";                      // 人脸下游任务切换
constexpr const char * FACE_TOPK_TOPIC = "/flylab/face_topk";                      // 人脸批量张数 (0 = 全部)
constexpr const char * LANDING_YAW_OFFSET_TOPIC = "/flylab/landing_yaw_offset";    // 降落航向偏置 (度)
constexpr const char * TRACK_FOLLOW_TOPIC = "/flylab/track_follow";                // 追踪反控机头跟随开关 (latched Bool)
constexpr const char * TRACK_PITCH_TOPIC = "/flylab/track_pitch";                  // 追踪俯视角度 (度, 下正, latched, 高度环目标)
constexpr const char * TRACK_DIST_TOPIC = "/flylab/track_dist";                    // 追踪跟随距离 (米, latched, 距离环目标)

// ---------- ArUco 降落 ----------
constexpr const char * LANDING_STATUS_TOPIC = "/flylab/landing_status";     // 降落状态回传 (tracking/landing/landed/idle)
// 历史遗留: 追踪码 ID 广播话题, 地面站现经 algo_info JSON 的 tracked_id
// 字段消费, 本话题暂无订阅方 (aruco/fractal/landing 三节点仍在发布)
constexpr const char * LANDING_TRACKED_ID_TOPIC = "/flylab/landing_tracked_id";

// ---------- 目标追踪反控 ----------
constexpr const char * TRACK_ROI_TOPIC = "/flylab/track_roi";               // 地面站画面框选区域 ({"roi":[x,y,w,h]}, latched)
constexpr const char * TRACK_STATUS_TOPIC = "/flylab/track_status";         // 反控状态回传 (engaged/idle/failed)

// ---------- 手动遥控 ----------
constexpr const char * TELEOP_CMD_TOPIC = "/flylab/teleop_cmd";             // 归一化遥控量 (Twist, -1..1)
constexpr const char * TELEOP_STATUS_TOPIC = "/flylab/teleop_status";       // 遥控状态回传

}  // namespace topics
}  // namespace flylab_vision

#endif  // FLYLAB_VISION__COMMON__GCS_TOPICS_HPP_
