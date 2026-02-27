#pragma once
#include "data_structure.h"
#include "inverse_dynamics.h"

namespace robot_server_lib {
constexpr double torque_error_threshold = 2.0;
constexpr double velocity_error_threshold = 1.5;
constexpr double position_limit_vel_threshold = 4 * IdleVelocityThreshold;

bool CheckSafety(const JointMeasurement& joint_measurement, const RobotModel& robot_model, ErrorCode& error_code);
}  // namespace robot_server_lib