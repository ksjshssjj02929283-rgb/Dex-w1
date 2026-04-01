#include "safety_check.h"

#include "NanoLog.hpp"

namespace robot_server_lib {
bool CheckSafety(const JointMeasurement &joint_measurement,
                 const RobotModel &robot_model,
                 ErrorCode &error_code,
                 SafetyCheckConfig config) {
    const auto &params = *robot_model.params_ptr;
    const auto &model = *robot_model.model_ptr;
    const auto &joint_names = params.joint_command_sequence_by_name;

    for (int i = 0; i < ndof_robot; i++) {
        if (config.check_velocity_limit &&
            (joint_measurement.velocity[i] > params.joint_max_velocity_limit_positive[i] * velocity_error_threshold ||
             joint_measurement.velocity[i] < params.joint_max_velocity_limit_negative[i] * velocity_error_threshold)) {
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
        if (config.check_torque_limit &&
            (joint_measurement.effort[i] > params.joint_max_torque_limit_positive[i] * torque_error_threshold ||
             joint_measurement.effort[i] < params.joint_max_torque_limit_negative[i] * torque_error_threshold)) {
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

    // 位置限位检测（可在拖动模式下跳过）
    if (config.check_position_limit) {
        Eigen::VectorXd lb = dynamics::MapToParamOrder(model, model.lowerPositionLimit, joint_names);
        Eigen::VectorXd ub = dynamics::MapToParamOrder(model, model.upperPositionLimit, joint_names);

        // 应用安全阈值（将 deg 转换为 rad）
        Eigen::VectorXd margin_rad = params.joint_limit_margin_deg * M_PI / 180.0;
        lb = lb + margin_rad;  // 下限向上调整
        ub = ub - margin_rad;  // 上限向下调整

        for (int i = 0; i < ndof_robot; i++) {
            if (joint_measurement.q_desired_pre[i] > ub[i] &&
                joint_measurement.velocity[i] > position_limit_vel_threshold) {
                if (error_code != ErrorCode::PositionOverLimit) {
                    std::stringstream ss;
                    ss << "Joint " << joint_names[i]
                       << " exceeds upper limit, q_desired_pre: " << joint_measurement.q_desired_pre[i]
                       << ", limit: " << ub[i] << ", original limit: " << ub[i] + margin_rad[i]
                       << ", margin_rad: " << margin_rad[i] << ", velocity: " << joint_measurement.velocity[i]
                       << ", velocity threshold: " << position_limit_vel_threshold;
                    std::cout << ss.str() << "\n";
                    LOG_CRIT << ss.str();
                    error_code = ErrorCode::PositionOverLimit;
                }
                return false;
            }
            if (joint_measurement.q_desired_pre[i] < lb[i] &&
                joint_measurement.velocity[i] < -position_limit_vel_threshold) {
                if (error_code != ErrorCode::PositionOverLimit) {
                    std::stringstream ss;
                    ss << "Joint " << joint_names[i]
                       << " exceeds lower limit, q_desired_pre: " << joint_measurement.q_desired_pre[i]
                       << ", limit: " << lb[i] << ", original limit: " << lb[i] - margin_rad[i]
                       << ", margin_rad: " << margin_rad[i] << ", velocity: " << joint_measurement.velocity[i]
                       << ", velocity threshold: " << -position_limit_vel_threshold;
                    std::cout << ss.str() << "\n";
                    LOG_CRIT << ss.str();
                    error_code = ErrorCode::PositionOverLimit;
                }
                return false;
            }
        }
    }
    return true;
}
}  // namespace robot_server_lib