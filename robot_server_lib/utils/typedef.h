/**
 * @file base_typedef.h
 * @brief 机器人类型定义和宏定义
 * @copyright Copyright (C) 2025 DexForce Technology Co., Ltd. All Rights Reserved.
 */
#pragma once

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <magic_enum.hpp>
#include <string_view>
#include <system_error>
#include <type_traits>
#include <vector>

namespace robot_server_lib {

constexpr int ndof_robot = 20;
constexpr int torque_ctrl_dt_micro = 1000;  // 1kHz
constexpr int manager_dt_micro = 4 * torque_ctrl_dt_micro;
constexpr int simulation_dt_micro = torque_ctrl_dt_micro;
constexpr int publish_dt_micro = 10000;  // 100 hz

constexpr double torque_ctrl_dt = double(torque_ctrl_dt_micro) / 1e6;
constexpr double manager_dt = double(manager_dt_micro) / 1e6;
constexpr double simulation_dt = double(simulation_dt_micro) / 1e6;
constexpr double IdleVelocityThreshold = 0.05 / 180 * M_PI;  // 静止状态的速度阈值
constexpr double PositionThreshold = 1e-6;                   // 位置误差容忍度
constexpr double StreamingCommandTimeout = 1.5;              // 流式命令超时1.5秒

using Int64 = std::int64_t;
using VectorDof = Eigen::Matrix<double, ndof_robot, 1>;
using JointPositionDof = VectorDof;  // 关节位置
using Matrix3d = Eigen::Matrix<double, 3, 3>;
using Matrix4d = Eigen::Matrix<double, 4, 4>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector3d = Eigen::Matrix<double, 3, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector12d = Eigen::Matrix<double, 12, 1>;

enum class RunMode {
    Position,            // position controls
    Torque,              // torque control
    CartesianImpedance,  // impedance control
    Drag,                // zero-stiff control
    Unknown
};

enum class Status { Init, Idle, Running, Error };

enum class ErrorCode { NoError, InvalidRequest, VelocityOverLimit, TorqueOverLimit, IKFailed };

enum class RequestType {
    Empty,  // empty command

    // Set Property Request
    TcpPrimary,
    TcpSecondary,
    SpeedRatio,
    ClearError,
    SafeLevel,
    SwitchRunMode,
    EnableDrag,
    ImpedanceMass,
    ImpedanceStiffness,
    // Move Request
    MoveJoint,
    MoveLine,
    MoveTraj,
    StartJointStreaming,
    StartEEPoseStreaming,
    StartEEVelocityStreaming,
    // Emergency Request
    StopMove,
    ResetToInit,

    Unknown
};

enum class MotionGroup {
    Primary,
    Secondary,
    BothArms,
    WholeBody,
    Torso,

    Unknown
};

enum class ReferenceFrame {
    ArmBase,    // arm base frame
    WorldBase,  // world base frame
    EEFrame,    // end-effector frame
    Unknown
};

enum class RequestStatus {
    ErrorDrop,
    Executed,
    PhaseFinished,
    Queued,
    Stopped,

    Unknown
};

template <typename E, typename = std::enable_if_t<std::is_enum_v<E>>>
inline const char *ToString(E e) {
    auto sv = magic_enum::enum_name(e);
    return sv.empty() ? "Unknown" : sv.data();
}

}  // namespace robot_server_lib