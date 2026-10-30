#pragma once
// 底盘安全速度上限：限幅（chassis_command.h）与安全监测（chassis_safety_monitor.h）共用。
//
// 整机速度控制的安全约束与上层速度规划无关：所有速度来源最终都经 /control/cmd_vel
// 进入本进程，限幅与监测放在实时侧才能覆盖全部来源。上限写死在代码里，不做成 ROS 参数，
// 运行中不能被参数接口改动。

#include <algorithm>
#include <cmath>

#include "chassis_kinematics.h"

namespace chassis_node {

/// 车体线速度上限 (m/s)：平面合速度，差速即 |vx|
constexpr double kChassisSafeLinearSpeedMps = 0.75;
/// 单轮轮缘线速度上限 (m/s)
constexpr double kChassisSafeWheelSpeedMps = 0.75;
/// 角速度上限 (rad/s)。原地转向时，离转动中心 0.75 m 以内的点（含伸出车体的上半身）
/// 线速度都不超过单轮上限
constexpr double kChassisSafeAngularRps = 1.0;

/// 把车体 twist 压到线速度、角速度上限内：两项都超时取更严的系数，vx、vy、wz 同比缩放，
/// 只降速、不改运动方向与转弯半径。与底盘类型无关，全向同样适用；非有限值由调用方先处理。
inline Twist2D ClampTwistToSafeLimits(const Twist2D& t) {
    double k = 1.0;
    const double v = std::hypot(t.vx, t.vy);
    if (v > kChassisSafeLinearSpeedMps) k = std::min(k, kChassisSafeLinearSpeedMps / v);
    if (std::abs(t.wz) > kChassisSafeAngularRps) k = std::min(k, kChassisSafeAngularRps / std::abs(t.wz));
    return {t.vx * k, t.vy * k, t.wz * k};
}

}  // namespace chassis_node
