#pragma once
#include "data_structure.h"
#include "inverse_dynamics.h"

namespace robot_server_lib {
constexpr double torque_error_threshold = 2.5;
constexpr double velocity_error_threshold = 2.5;
constexpr double position_limit_vel_threshold = 4 * IdleVelocityThreshold;

bool CheckSafety(const JointMeasurement& joint_measurement,
                 const RobotModel& robot_model,
                 std::array<ErrorCode, ndof_robot>& error_codes,
                 SafetyCheckConfig config = SafetyCheckConfig());
}  // namespace robot_server_lib