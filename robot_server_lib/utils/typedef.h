/**
 * @file base_typedef.h
 * @brief 机器人类型定义和宏定义
 * @copyright Copyright (C) 2025 DexForce Technology Co., Ltd. All Rights Reserved.
 */
#pragma once

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <system_error>
#include <vector>

namespace robot_server_lib
{

constexpr int ndof_robot = 20;
constexpr int torque_ctrl_dt_micro = 1000;  // 1kHz
constexpr int manager_dt_micro = 4 * torque_ctrl_dt_micro;
constexpr int simulation_dt_micro = 500;  // 2Khz
constexpr int publish_dt_micro = 10000;   // 100 hz

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
using Vector3d = Eigen::Matrix<double, 3, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector12d = Eigen::Matrix<double, 12, 1>;

enum class RunMode
{
    Position,     // position controls
    Impedance,    // impedance control
    Drag,         // zero-stiff control
    Teleoperate,  // teleoperate control
    Unknown
};

enum class Status
{
    Init,
    Idle,
    Running,
    Error
};

enum class RequestType
{
    Empty,  // empty command

    // Set Property Request
    TcpPrimary,
    TcpSecondary,
    SpeedRatio,
    ClearError,
    EnableDrag,
    EnableImpedance,
    SafeLevel,

    // Move Request
    MoveJoint,
    MoveLine,
    MoveTraj,
    StartJointStreaming,
    StartEEPoseStreaming,

    // Emergency Request
    StopMove,
    ResetToInit,

    Unknown
};

enum class MotionGroup
{
    Primary,
    Secondary,
    BothArms,
    WholeBody,
    Torso,

    Unknown
};

enum class ReferenceFrame
{
    ArmBase,    // arm base frame
    WorldBase,  // world base frame

    Unknown
};

enum class RequestStatus
{
    ErrorDrop = 0,
    Executed = 1,
    PhaseFinished = 2,
    Queued = 3,
    Stopped = 4,

    Unknown = 4
};

inline const char *ToString(Status s)
{
    switch (s)
    {
    case Status::Init:
        return "Init";
    case Status::Idle:
        return "Idle";
    case Status::Running:
        return "Running";
    case Status::Error:
        return "Error";
    default:
        return "Unknown";
    }
}

inline const char *ToString(MotionGroup s)
{
    switch (s)
    {
    case MotionGroup::Primary:
        return "Primary";
    case MotionGroup::Secondary:
        return "Secondary";
    case MotionGroup::BothArms:
        return "BothArm";
    case MotionGroup::WholeBody:
        return "WholeBody";
    case MotionGroup::Torso:
        return "Torso";
    default:
        return "Unknown";
    }
}

inline const char *ToString(RequestStatus s)
{
    switch (s)
    {
    case RequestStatus::ErrorDrop:
        return "ErrorDrop";
    case RequestStatus::Executed:
        return "Executed";
    case RequestStatus::Queued:
        return "Queued";
    case RequestStatus::PhaseFinished:
        return "PhaseFinished";
    case RequestStatus::Stopped:
        return "Stopped";
    default:
        return "Unknown";
    }
}

inline const char *ToString(RequestType type)
{
    switch (type)
    {
    case RequestType::Empty:
        return "Empty";
    case RequestType::TcpPrimary:
        return "TcpPrimary";
    case RequestType::TcpSecondary:
        return "TcpSecondary";
    case RequestType::SpeedRatio:
        return "SpeedRatio";
    case RequestType::EnableDrag:
        return "EnableDrag";
    case RequestType::EnableImpedance:
        return "EnableImpedance";
    case RequestType::SafeLevel:
        return "SafeLevel";
    case RequestType::ClearError:
        return "ClearError";
    case RequestType::MoveJoint:
        return "MoveJoint";
    case RequestType::MoveLine:
        return "MoveLine";
    case RequestType::MoveTraj:
        return "MoveTraj";
    case RequestType::StartJointStreaming:
        return "StartJointStreaming";
    case RequestType::StartEEPoseStreaming:
        return "StartEEPoseStreaming";
    case RequestType::StopMove:
        return "StopMove";
    default:
        return "Unknown";
    }
}

inline const char *ToString(RunMode mode)
{
    switch (mode)
    {
    case RunMode::Position:
        return "Position";
    case RunMode::Impedance:
        return "Impedance";
    case RunMode::Drag:
        return "Drag";
    case RunMode::Teleoperate:
        return "Teleoperate";
    default:
        return "Unknown";
    }
}

inline const char *ToString(ReferenceFrame frame)
{
    switch (frame)
    {
    case ReferenceFrame::ArmBase:
        return "ArmBase";
    case ReferenceFrame::WorldBase:
        return "WorldBase";
    default:
        return "Unknown";
    }
}

}  // namespace robot_server_lib