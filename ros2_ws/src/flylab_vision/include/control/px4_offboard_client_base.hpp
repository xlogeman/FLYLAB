// Copyright 2026 flylab
#ifndef PX4_OFFBOARD_CLIENT_BASE_HPP_
#define PX4_OFFBOARD_CLIENT_BASE_HPP_

/// PX4 offboard 执行层公共接口 —— MAVROS / uXRCE-DDS 双链路的抽象。
/// 可复用组件, 供 flylab_vision/src/control 下所有"通过 PX4 控制飞机"的
/// 算法节点使用 (视觉降落 / 目标追踪控制 / ...)。
///
/// 两个实现 (见 px4_offboard_client_mavros.hpp / px4_offboard_client_uxrce.hpp)
/// 与 flylab tracker:=mavros|uxrce 约定一致, 上层节点用 link 参数选择。
///
/// 坐标/符号约定 (两链路统一, 上层无感):
///   - 速度指令一律机体系 FRD: x 前 y 右 z 下, "下"为正 = 下降
///   - localPosition() 一律返回 NED 语义 (z 下正), MAVROS 侧由 ENU pose 换算
///
/// 注意: 两种链路都不要与避障 tracker_uxrce/tracker_mavros 同时运行
/// (会争抢 setpoint; tracker 与本组件同时走 MAVROS 时争抢的是同一条 setpoint 流)。

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace flylab_vision
{

/// PX4 offboard 执行层公共接口 (MAVROS / uXRCE 两个实现)
class Px4OffboardClientBase
{
public:
  /// offboard 接管状态机输出
  enum class Takeover
  {
    ENGAGED,     // 已确认进入 OFFBOARD, 上层可以开始发控制 setpoint
    REQUESTING,  // 请求中 (内部已在发心跳+悬停, 继续每 tick 调用)
    FAILED       // 超过 warmup+retry 仍未进入 (上层决定放弃/交还)
  };

  /// 位置快照, NED 语义 (z 下正)。MAVROS 侧由 ENU pose 换算, uXRCE 侧取自 VehicleLocalPosition
  struct LocalPosNED
  {
    bool xy_valid = false;
    bool z_valid = false;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
  };

  virtual ~Px4OffboardClientBase() = default;

  // ---------------- 状态查询 ----------------
  virtual bool gotStatus() const = 0;        // 已收到飞控状态 (链路可用)
  virtual bool armed() const = 0;
  virtual bool isOffboard() const = 0;
  virtual bool isAutoLand() const = 0;
  virtual bool linkFresh(double timeout_sec) const = 0;  // 断链判定
  /// 当前飞控导航模式名 (诊断日志用, 如 "OFFBOARD" / "AUTO.RTL")
  virtual std::string modeName() const = 0;
  virtual LocalPosNED localPosition() const = 0;

  // ---------------- offboard 接管状态机 ----------------
  virtual void takeoverReset(const rclcpp::Time & now) = 0;
  virtual Takeover takeoverTick(const rclcpp::Time & now, double warmup_sec, double retry_sec) = 0;

  // ---------------- 控制指令 ----------------
  /// 机体系 FRD 速度 setpoint (x前 y右 z下, "下"为正) 并同时发 offboard 心跳。
  /// offboard 模式下必须持续调用 (断流会触发 offboard failsafe)
  virtual void sendVelocityFRD(double fwd, double right, double down) = 0;
  /// 同上, 附加偏航角速率 (NED 语义: 正=顺时针/右转, 负=左转)。
  /// 实现需屏蔽 yaw 位置设定, 只用角速率 (手动遥控用)
  virtual void sendVelocityFRDYawRate(
    double fwd, double right, double down, double yaw_rate) = 0;
  /// 原地悬停
  virtual void sendHover() = 0;
  virtual void requestOffboard() = 0;
  virtual void requestAutoLand() = 0;
  virtual void requestLoiter() = 0;
};

}  // namespace flylab_vision

#endif  // PX4_OFFBOARD_CLIENT_BASE_HPP_
