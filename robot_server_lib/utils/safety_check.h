#pragma once
#include "data_structure.h"
namespace robot_server_lib {
constexpr double velocity_error_threshold = 1.5;  // 错误阈值比例
constexpr double torque_error_threshold = 1.5;    // 错误阈值比例

// 检查关节速度是否超出限制
bool CheckMeasurementSafety(const JointMeasurement& joint_measurement,
                            const RobotParams& robot_params,
                            ErrorCode& error_code);
}  // namespace robot_server_lib