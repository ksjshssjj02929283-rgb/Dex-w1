#include "safety_check.h"
#include "NanoLog.hpp"

namespace robot_server_lib
{
bool CheckMeasurementSafety(const JointMeasurement &joint_measurement, const RobotParams &robot_params, ErrorCode &error_code)
{
    // 有错误则覆盖错误码返回false, 无错误则保持原错误码返回true
    for(int i = 0; i < ndof_robot; i++)
    {
        if (joint_measurement.velocity[i] > robot_params.joint_max_velocity_limit_positive[i] * error_threshold ||
            joint_measurement.velocity[i] < robot_params.joint_max_velocity_limit_negative[i] * error_threshold)
        {
            if (error_code != ErrorCode::VelocityOverLimit)
            {
                LOG_CRIT << "Joint " << i << " velocity over limit, current velocity: " << joint_measurement.velocity[i]
                        << ", limits: [" << robot_params.joint_max_velocity_limit_negative[i] * error_threshold
                        << ", " << robot_params.joint_max_velocity_limit_positive[i] * error_threshold << "]";
                error_code = ErrorCode::VelocityOverLimit;
            }
            return false;
        }
        if (joint_measurement.effort[i] > robot_params.joint_max_torque_limit_positive[i] * error_threshold ||
            joint_measurement.effort[i] < robot_params.joint_max_torque_limit_negative[i] * error_threshold)
        {
            if (error_code != ErrorCode::TorqueOverLimit)
            {
                LOG_CRIT << "Joint " << i << " torque over limit, current torque: " << joint_measurement.effort[i]
                        << ", limits: [" << robot_params.joint_max_torque_limit_negative[i] * error_threshold
                        << ", " << robot_params.joint_max_torque_limit_positive[i] * error_threshold << "]";
                error_code = ErrorCode::TorqueOverLimit;
            }
            return false;
        }
    }
    return true;
}
}  // namespace robot_server_lib