#pragma once
#include "data_structure.h"
#include "inverse_dynamics.h"

namespace robot_server_lib {
constexpr double torque_error_threshold = 2.5;
constexpr double velocity_error_threshold = 2.5;
constexpr double position_limit_vel_threshold = 4 * IdleVelocityThreshold;
constexpr int safety_confirm_frames = 4;              // 连续越限帧数, 防止瞬态误触发
constexpr double safety_velocity_filter_alpha = 0.1;  // 速度低通滤波系数

// 安全检测器: 有状态, 内部维护速度滤波与各类越限连续帧计数
class SafetyChecker {
public:
    SafetyChecker() = default;

    // 执行安全检测, 返回是否安全; error_codes 会被置位
    bool Check(const JointMeasurement& joint_measurement,
               const RobotModel& robot_model,
               std::array<ErrorCode, ndof_robot>& error_codes,
               SafetyCheckConfig config = SafetyCheckConfig());

    void Reset();

private:
    VectorDof filtered_velocity_{VectorDof::Zero()};           // 滤波后的速度(用于位置限位)
    std::array<int, ndof_robot> velocity_over_limit_count_{};  // 速度越限连续帧计数
    std::array<int, ndof_robot> torque_over_limit_count_{};    // 力矩越限连续帧计数
    std::array<int, ndof_robot> position_over_limit_count_{};  // 位置越限连续帧计数
    bool initialized_{false};
};
}  // namespace robot_server_lib
