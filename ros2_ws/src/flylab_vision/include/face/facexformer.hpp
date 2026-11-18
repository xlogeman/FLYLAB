// Copyright 2026 flylab
#ifndef FLYLAB_VISION__FACE__FACEXFORMER_HPP_
#define FLYLAB_VISION__FACE__FACEXFORMER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace flylab_vision
{

/// 任务 id (与 FaceXFormer 训练约定一致, inference.py 的 task 序号);
/// FACE_TASK_NONE = 只做人脸检测, 不跑 FaceXFormer
enum FaceXTask
{
  FACE_TASK_NONE = -1,          ///< 无任务 (仅 SCRFD 检测)
  FACE_TASK_PARSING = 0,        ///< 人像分割 (11 类掩码)
  FACE_TASK_LANDMARKS = 1,      ///< 68 关键点
  FACE_TASK_HEADPOSE = 2,       ///< 头部姿态 (pitch/yaw/roll 弧度)
  FACE_TASK_ATTRIBUTES = 3,     ///< CelebA 40 属性
  FACE_TASK_AGE_GENDER_RACE = 4,///< 年龄段/性别/种族
  FACE_TASK_VISIBILITY = 5,     ///< 29 关键点可见性
};

/// 任务名 <-> id (地面站 face_task 指令用, 未知名字返回 -1)
int facex_task_id(const char * name);
const char * facex_task_name(int id);

/// 检测器配置
struct FaceXConfig
{
  std::string model_path;      ///< ONNX 路径 (facexformer.onnx, 8 输出导出)
  bool use_cuda = false;       ///< onnxruntime 无 CUDA EP 时自动回退 CPU
};

/// 单脸下游任务推理结果 (run() 只填 task 对应字段, 其余为默认值)
struct FaceXResult
{
  float landmark[68][2];       ///< [-1,1] 归一化坐标 (224 裁剪空间, 交错 x,y)
  float headpose[3];           ///< pitch / yaw / roll (弧度)
  bool attribute[40];          ///< CelebA 40 属性 (sigmoid >= 0.5)
  bool visibility[29];         ///< 29 关键点可见性 (sigmoid >= 0.5)
  int age_bin = 0;             ///< 年龄段 bin (bin*10 ~ bin*10+9 岁)
  int gender = 0;              ///< 0 女 / 1 男
  int race = 0;                ///< 0 白人 / 1 黑人 / 2 亚裔 / 3 印度裔 / 4 其他
  cv::Mat seg_mask;            ///< CV_8UC1 224x224 类别索引 0..10 (task 0)

  FaceXResult()
  {
    for (auto & p : landmark) {p[0] = p[1] = 0.f;}
    for (auto & v : headpose) {v = 0.f;}
    for (auto & v : attribute) {v = false;}
    for (auto & v : visibility) {v = false;}
  }
};

/// FaceXFormer (ICCV 2025) 统一人脸分析器 —— 单模型 6 任务。
/// 纯算法, 不含 ROS / 窗口显示; onnxruntime 后端 (thirdparty/)。
/// 模型为 export_onnx.py 导出的 8 输出版 (跳过 task 布尔筛选, 全部输出
/// 由本类按 task 解码): 输入 [B,3,224,224] ImageNet 归一化, 输出
/// landmark/headpose/attribute/visibility/age/gender/race/segmentation。
/// 单实例单线程使用。
class FaceXFormer
{
public:
  /// 模型加载/空跑校验失败抛 std::runtime_error
  explicit FaceXFormer(const FaceXConfig & config);
  ~FaceXFormer();
  FaceXFormer(const FaceXFormer &) = delete;
  FaceXFormer & operator=(const FaceXFormer &) = delete;

  /// 单脸推理 (BGR, 任意尺寸, 内部缩放 224), 结果解到 out。
  /// task 超出 0..5 时抛 std::invalid_argument
  void run(const cv::Mat & face_bgr, int task, FaceXResult & out);

  /// 多脸批量推理: 各脸分别缩放 224 拼 [N,3,224,224] 一次前向 (模型输入
  /// batch 维为 symbolic 时分块 ≤8, 静态 batch=1 自动退化为逐张), results
  /// 与 faces 一一对应。task 越界抛 std::invalid_argument
  void run_batch(
    const std::vector<cv::Mat> & faces, int task,
    std::vector<FaceXResult> & results);

  /// CUDA EP 是否真正生效 (use_cuda=true 但注册失败时为 false, 已回退 CPU)
  bool cuda_active() const;
  /// CUDA EP 注册失败原因 (成功/未启用为空)
  std::string cuda_error() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;   // pimpl: 隔离 onnxruntime 头文件依赖
  FaceXConfig config_;
};

}  // namespace flylab_vision

#endif  // FLYLAB_VISION__FACE__FACEXFORMER_HPP_
