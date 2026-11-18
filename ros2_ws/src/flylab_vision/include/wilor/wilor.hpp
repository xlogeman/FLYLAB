// Copyright 2026 flylab
#ifndef FLYLAB_VISION__WILOR_HPP_
#define FLYLAB_VISION__WILOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace flylab_vision
{

/// WiLoR 管线配置 (检测器 + MANO 回归两个 ONNX)
struct WilorConfig
{
  std::string detector_path;    // YOLO 手部检测 ONNX (models/hand/detector.onnx)
  std::string wilor_path;       // WiLoR MANO 回归 ONNX (models/hand/wilor_fp16.onnx)
  float conf_thres = 0.3f;      // 手部检测置信度阈值
  float iou_thres = 0.45f;      // 手部检测 NMS IoU 阈值
  int det_stride = 1;           // 检测隔帧: N>1 时每 N 帧跑一次检测, 中间帧
                                // 复用上次手框只做回归 (省 ~11ms/帧, 手部
                                // 快速运动时会跟丢, 默认 1 逐帧检测)
  bool use_cuda = false;        // onnxruntime 无 CUDA EP 时自动回退 CPU
};

/// 单只手的检测结果 (一帧可含左右手各 0~1 个, 一般最多 2 只)
struct WilorHand
{
  cv::Rect2f box;               // 手框, 原图像素 (x, y 为左上角)
  bool is_right = true;         // 右手 / 左手
  float conf = 0.f;             // 检测置信度 0~1
  std::vector<cv::Point2f> kpts2d;   // 21 关键点 (openpose-21 序), 原图像素
  std::vector<cv::Point3f> verts3d;  // MANO 网格 778 顶点, 相机系米
                                     // (verts + pred_cam_t; 左手为右手模型
                                     // 沿 patch x 镜像后的相机系)
  std::vector<cv::Point2f> verts2d;  // 778 顶点投影, 原图像素, 与 verts3d
                                     // 一一对应 (透视投影 + 逆仿射)
};

/// WiLoR 手部姿态管线 —— 纯算法, 不含 ROS / 窗口显示。
///
/// 两级推理:
///   1. YOLO 手部检测器 (输入尺寸取自模型元数据, 输出 pose 布局
///      [box 4 + 左右手分数 2 + 可选关键点 3nk], 行 4:6 为左右手分数)
///   2. 逐手裁 192x256 比例 patch (左手镜像成右手, MANO 只建模右手)
///      归一化后过 WiLoR, 输出三路:
///        kpts_2d (Nx21x2)   21 关键点, patch 归一化坐标 → 逆仿射回原图
///        verts   (Nx778x3)  MANO 网格顶点 (拓扑在 mano_faces.hpp, 原
///                           mano_faces.npy 已烘焙进源码)
///        pred_cam_t (Nx3)   裁块相机平移 [tx, ty, tz]
///      顶点加 pred_cam_t 得相机系 3D, 过官方 perspective_projection
///      (f' = 5000/256, 光心 0) 得 patch 归一化坐标, 与 kpts_2d 同一
///      基准 → 镜像/逆仿射回原图像素
///
/// 批量推理: 模型输入 batch 维为 symbolic 时所有检出手拼批一次回归
/// (单次 ≤8 手, N 手耗时 ≈ batch1 的 0.6~0.7N); 输入 batch 固定为 1
/// 的旧模型自动逐手回退, 行为不变。batch 维在构造时按模型元数据判定。
///
/// 推理后端为 onnxruntime (thirdparty/onnxruntime/, 同 yolo 系列)。
/// wilor 模型用 fp16 版 (wilor_fp16_dyn.onnx 动态 batch, 权重已内联
/// 单文件): 实测 CUDA 单手 ~32ms / batch=4 摊到 ~19ms/手 (旧静态
/// wilor_fp16.onnx 单手 28ms), 关键点相对误差 <0.2%。
/// 单实例单线程使用。
class WilorPipeline
{
public:
  /// 模型加载/空跑校验失败抛 std::runtime_error
  explicit WilorPipeline(const WilorConfig & config);
  ~WilorPipeline();

  /// 单帧检测 + 逐手关键点/MANO 网格回归 (BGR); 无手返回空
  std::vector<WilorHand> detect(const cv::Mat & bgr);

  /// CUDA EP 是否真正生效 (use_cuda=true 但注册失败时为 false, 已回退 CPU)
  bool cuda_active() const;
  /// CUDA EP 注册失败原因 (成功/未启用为空)
  std::string cuda_error() const;
  /// wilor 回归是否动态 batch (动态模型多手拼批一次推理, 静态模型逐手)
  bool batch_dynamic() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;   // pimpl: 隔离 onnxruntime 头文件依赖
  WilorConfig config_;
};

/// 检测结果 -> 地面站叠加标注 JSON (schema 见 flylab_gcs/overlay.py)。
/// 输出为按算法分组的消息 {"algo": algo, "items": [...]}, 每只手:
///   bbox (label = "Right/Left 置信度", 右手橙 / 左手天蓝) +
///   6 段骨架 polyline (openpose-21 拓扑: 拇指/四指各一条链 + 腕-小指,
///   顺序连线会把 21 点串成锯齿线, 不能用) +
///   MANO 网格 778 顶点散点 (与手框同色)
/// 坐标为原图像素, 颜色为 RGB。
std::string to_annotations_json(
  const std::vector<WilorHand> & hands, const std::string & algo = "wilor");

/// 单手 MANO 网格渲染为小图 JPEG (base64): verts2d 平铺缩放进 size×size
/// 画布, kManoFaces 面片 z-buffer 平面着色 (3D 法向 + 头灯), 深色底。
/// 稀疏输出进地面站右栏面板 JSON 的 "img" 字段 (顶部固定高度图片区)。
/// mesh 为空时返回空串。
std::string to_mesh_jpeg(
  const WilorHand & hand, int size = 128, int quality = 70);

}  // namespace flylab_vision

#endif  // FLYLAB_VISION__WILOR_HPP_
