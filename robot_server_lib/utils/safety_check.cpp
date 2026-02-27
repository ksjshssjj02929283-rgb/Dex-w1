#include "safety_check.h"

#include "NanoLog.hpp"

namespace robot_server_lib {
bool CheckSafety(const JointMeasurement &joint_measurement, const RobotModel &robot_model, ErrorCode &error_code) {
    const auto &params = *robot_model.params_ptr;
    const auto &model = *robot_model.model_ptr;
    const auto &joint_names = params.joint_command_sequence_by_name;

    for (int i = 0; i < ndof_robot; i++) {
        if (joint_measurement.velocity[i] > params.joint_max_velocity_limit_positive[i] * velocity_error_threshold ||
            joint_measurement.velocity[i] < params.joint_max_velocity_limit_negative[i] * velocity_error_threshold) {
            if (error_code != ErrorCode::VelocityOverLimit) {
                std::stringstream ss;
                ss << "Joint " << i << " velocity over limit, current velocity: " << joint_measurement.velocity[i]
                   << ", limits: [" << params.joint_max_velocity_limit_negative[i] * velocity_error_threshold << ", "
                   << params.joint_max_velocity_limit_positive[i] * velocity_error_threshold << "]";
                std::cout << ss.str() << "\n";
                LOG_CRIT << ss.str();
                error_code = ErrorCode::VelocityOverLimit;
            }
            return false;
        }
        if (joint_measurement.effort[i] > params.joint_max_torque_limit_positive[i] * torque_error_threshold ||
            joint_measurement.effort[i] < params.joint_max_torque_limit_negative[i] * torque_error_threshold) {
            if (error_code != ErrorCode::TorqueOverLimit) {
                std::stringstream ss;
                ss << "Joint " << i << " torque over limit, current torque: " << joint_measurement.effort[i]
                   << ", limits: [" << params.joint_max_torque_limit_negative[i] * torque_error_threshold << ", "
                   << params.joint_max_torque_limit_positive[i] * torque_error_threshold << "]";
                std::cout << ss.str() << "\n";
                LOG_CRIT << ss.str();
                error_code = ErrorCode::TorqueOverLimit;
            }
            return false;
        }
    }

    Eigen::VectorXd lb = dynamics::MapToParamOrder(model, model.lowerPositionLimit, joint_names);
    Eigen::VectorXd ub = dynamics::MapToParamOrder(model, model.upperPositionLimit, joint_names);

    for (int i = 0; i < ndof_robot; i++) {
        if (joint_measurement.position[i] > ub[i] && joint_measurement.velocity[i] > position_limit_vel_threshold) {
            if (error_code != ErrorCode::PositionOverLimit) {
                std::stringstream ss;
                ss << "Joint " << joint_names[i] << " exceeds upper limit, position: " << joint_measurement.position[i]
                   << ", limit: " << ub[i] << ", velocity: " << joint_measurement.velocity[i] << ", velocity threshold: " << position_limit_vel_threshold;
                std::cout << ss.str() << "\n";
                LOG_CRIT << ss.str();
                error_code = ErrorCode::PositionOverLimit;
            }
            return false;
        }
        if (joint_measurement.position[i] < lb[i] && joint_measurement.velocity[i] < -position_limit_vel_threshold) {
            if (error_code != ErrorCode::PositionOverLimit) {
                std::stringstream ss;
                ss << "Joint " << joint_names[i] << " exceeds lower limit, position: " << joint_measurement.position[i]
                   << ", limit: " << lb[i] << ", velocity: " << joint_measurement.velocity[i] << ", velocity threshold: " << -position_limit_vel_threshold;
                std::cout << ss.str() << "\n";
                LOG_CRIT << ss.str();
                error_code = ErrorCode::PositionOverLimit;
            }
            return false;
        }
    }
    return true;
}
}  // namespace robot_server_lib