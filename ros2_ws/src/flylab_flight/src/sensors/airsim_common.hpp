// airsim_common.hpp —— AirSim 直连传感器节点公共设施
//
// 内容: RPC 客户端样板 (StrictMode/rpclib 头文件顺序敏感)、连接辅助、
//       仿真时钟 -> ROS 时钟映射 (SimClockSync)、NED/FRD <-> ENU/FLU 四元数换算。
// 所有 airsim_* 传感器节点 (src/sensors/) 共用;
// 桥接节点 (src/bridges/) 是纯 ROS 节点, 不依赖 AirSim 库, 不使用本头文件。
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>

// AirSim C++ 客户端核心头文件 (以下宏与 include 顺序不可调换)
#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif
#include "rpc/rpc_error.h"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
STRICT_MODE_ON

namespace flylab {

using AirSimClient = msr::airlib::MultirotorRpcLibClient;

// AirSim 主机地址: 读环境变量 PX4_SIM_HOST_ADDR (scripts/env.sh 自动设为 WSL 网关),
// 未设置时回退常见的 WSL 默认网关地址
inline std::string airsim_host() {
    const char* ip_env = std::getenv("PX4_SIM_HOST_ADDR");
    return ip_env ? ip_env : "172.28.144.1";
}

// 建立并确认一条独立 RPC 通道。失败抛异常, 由节点构造函数统一转为 fatal。
// 每路高频数据流(图像/IMU/雷达/里程计)应各用一条独立通道 + 独立定时器线程, 互不阻塞。
inline std::unique_ptr<AirSimClient> connect_airsim(const std::string& ip) {
    auto client = std::make_unique<AirSimClient>(ip);
    client->confirmConnection();
    return client;
}

// =========================================================================
// 仿真时钟 -> ROS 时钟映射
//
// AirSim 传感器 time_stamp (仿真时钟) 与本机时钟不同源; 若部分数据用墙钟、
// 部分用仿真钟, 仿真慢于实时时下游会系统性过积分 -> 平移漂移。
// 约定: 所有 airsim_* 传感器节点统一用本类做线性映射 (ros = offset + sim),
// 使各节点 header.stamp 落在同一条时间轴 (各节点独立估 offset, 残差仅
// ms 级)。仿真卡顿时时间轴暂停, 不产生数据空洞。
// =========================================================================
class SimClockSync {
public:
    // sim_s: AirSim 仿真时间戳 (秒); wall_s: 本机时钟 (秒, 取 this->now().seconds())
    // 返回映射后的 ROS 时间戳 (秒)
    double to_ros(double sim_s, double wall_s) {
        std::lock_guard<std::mutex> lk(mutex_);
        if (!calibrated_) {
            offset_ = wall_s - sim_s;
            calibrated_ = true;
        } else {
            const double dev = (wall_s - sim_s) - offset_;
            if (std::fabs(dev) > 0.5) {
                offset_ = wall_s - sim_s;  // 大偏差 (仿真暂停/跳变/重置): 直接重标定
            } else {
                offset_ += 0.05 * dev;     // 小偏差 (仿真慢于实时): EMA 低通跟踪
            }
        }
        return offset_ + sim_s;
    }

private:
    std::mutex mutex_;
    double offset_ = 0.0;
    bool calibrated_ = false;
};

// =========================================================================
// 坐标系换算四元数 (AirSim 原生 NED/FRD -> ROS 标准 ENU/FLU)
//
// 绝对姿态: q_enu_flu = q_ned_to_enu * q_raw * q_frd_to_flu
//   (左乘参考系旋转, 右乘机体系旋转)
// 相对量 (机体分量 FRD->FLU): x 不变, y/z 取反
// 位置 (NED->ENU): (X,Y,Z)_enu = (Y,X,-Z)_ned
// =========================================================================
inline tf2::Quaternion q_ned_to_enu() {
    tf2::Quaternion q;
    q.setRPY(M_PI, 0, M_PI / 2.0);
    return q;
}

inline tf2::Quaternion q_frd_to_flu() {
    tf2::Quaternion q;
    q.setRPY(M_PI, 0, 0);
    return q;
}

inline tf2::Quaternion quat_nedfrd_to_enuflu(const tf2::Quaternion& q_raw) {
    return q_ned_to_enu() * q_raw * q_frd_to_flu();
}

}  // namespace flylab
