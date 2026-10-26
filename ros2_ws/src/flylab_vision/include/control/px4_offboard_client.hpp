// Copyright 2026 flylab
#ifndef PX4_OFFBOARD_CLIENT_HPP_
#define PX4_OFFBOARD_CLIENT_HPP_

/// PX4 offboard 执行层 (双链路) —— 总入口。
/// 可复用组件, 供 flylab_vision/src/control 下所有"通过 PX4 控制飞机"的
/// 算法节点使用 (视觉降落 / 目标追踪控制 / ...)。
///
/// 文件组织:
///   px4_offboard_client_base.hpp    公共接口 (基类 + Takeover + LocalPosNED + 坐标约定)
///   px4_offboard_client_mavros.hpp  MAVROS 链路实现   (与 tracker_mavros 同一套话题/服务)
///   px4_offboard_client_uxrce.hpp   uXRCE-DDS 直连实现 (与 tracker_uxrce 同一套 /fmu 话题)
///
/// 两条链路与 flylab tracker:=mavros|uxrce 约定一致, 上层节点用 link 参数选择:
///   mavros —— 依赖 MAVROS 桥: fcu_url:=udp://:14542@ (PX4 MAVROS 专属实例,
///             参照 fly_ego.launch.py 的 mavros 节点; 14580 会被 AirSim 抢占勿用)
///   uxrce  —— 依赖 MicroXRCEAgent udp4 -p 8888, 不依赖 MAVROS
///
/// 用法 (在 rclcpp::Node 子类中):
///   std::unique_ptr<Px4OffboardClientBase> px4_ =
///     std::make_unique<Px4OffboardClientMavros>(*this);   // 或 ...Uxrce
///   px4_->takeoverReset(now());                           // 开始接管
///   if (px4_->takeoverTick(now(), 1.0, 10.0) != Px4OffboardClientBase::Takeover::ENGAGED) return;
///   px4_->sendVelocityFRD(0.5, 0.0, 0.1);                 // 机体系速度 (前,右,下)
///   px4_->requestAutoLand();
///
/// 共同封装内容: 诊断日志 (链路建立/模式切换跟踪/failsafe 与断链告警)、
/// 机体系 FRD 速度 setpoint (自动旋到链路约定系)、悬停、
/// 切 OFFBOARD / AUTO.LAND / AUTO.LOITER、offboard 接管状态机
/// takeoverTick() (心跳预热 → 周期请求 → 成功/超时, PX4 要求先收到流才接受切模式)。
///
/// 注意: 不要与避障 tracker_uxrce/tracker_mavros 同时运行 (会争抢 setpoint 流)。

#include "px4_offboard_client_base.hpp"
#include "px4_offboard_client_mavros.hpp"
#include "px4_offboard_client_uxrce.hpp"

#endif  // PX4_OFFBOARD_CLIENT_HPP_
