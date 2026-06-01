#include "safety_check.h"

#include "NanoLog.hpp"

namespace robot_server_lib {
bool CheckSafety(const JointMeasurement& joint_measurement,
                 const RobotModel& robot_model,
                 std::array<ErrorCode, ndof_robot>& error_codes,
                 SafetyCheckConfig config) {
    const auto& params = *robot_model.params_ptr;
    const auto& model = *robot_model.model_ptr;
    bool is_safe = true;

    for (int i = 0; i < ndof_robot; i++) {
        if (config.check_velocity_limit &&
            (joint_measurement.velocity[i] > params.joint_velocity_limit[i] * velocity_error_threshold ||
             joint_measurement.velocity[i] < -params.joint_velocity_limit[i] * velocity_error_threshold)) {
            if (error_codes[i] != ErrorCode::VelocityOverLimit) {
                std::stringstream ss;
                ss << "Joint " << i << " velocity over limit, current velocity: " << joint_measurement.velocity[i]
                   << ", limits: [" << -params.joint_velocity_limit[i] * velocity_error_threshold << ", "
                   << params.joint_velocity_limit[i] * velocity_error_threshold << "]";
                std::cout << ss.str() << "\n";
                LOG_CRIT << ss.str();
                error_codes[i] = ErrorCode::VelocityOverLimit;
            }
            is_safe = false;
        }
        if (config.check_torque_limit &&
            (joint_measurement.effort[i] > params.joint_torque_limit[i] * torque_error_threshold ||
             joint_measurement.effort[i] < -params.joint_torque_limit[i] * torque_error_threshold)) {
            if (error_codes[i] != ErrorCode::TorqueOverLimit) {
                std::stringstream ss;
                ss << "Joint " << i << " torque over limit, current torque: " << joint_measurement.effort[i]
                   << ", limits: [" << -params.joint_torque_limit[i] * torque_error_threshold << ", "
                   << params.joint_torque_limit[i] * torque_error_threshold << "]";
                std::cout << ss.str() << "\n";
                LOG_CRIT << ss.str();
                error_codes[i] = ErrorCode::TorqueOverLimit;
            }
            is_safe = false;
        }
    }

    // 位置限位检测（可在拖动模式下跳过）
    if (config.check_position_limit) {
        Eigen::VectorXd lb = model.lowerPositionLimit;
        Eigen::VectorXd ub = model.upperPositionLimit;

        // 应用安全阈值（将 deg 转换为 rad）
        Eigen::VectorXd margin_rad = params.joint_limit_margin_deg * M_PI / 180.0;
        lb = lb + margin_rad;  // 下限向上调整
        ub = ub - margin_rad;  // 上限向下调整

        for (int i = 0; i < ndof_robot; i++) {
            if (joint_measurement.q_desired_pre[i] > ub[i] &&
                joint_measurement.velocity[i] > position_limit_vel_threshold) {
                if (error_codes[i] != ErrorCode::PositionOverLimit) {
                    std::stringstream ss;
                    ss << "Joint " << model.names[i + 1]
                       << " exceeds upper limit, q_desired_pre: " << joint_measurement.q_desired_pre[i]
                       << ", limit: " << ub[i] << ", original limit: " << ub[i] + margin_rad[i]
                       << ", margin_rad: " << margin_rad[i] << ", velocity: " << joint_measurement.velocity[i]
                       << ", velocity threshold: " << position_limit_vel_threshold;
                    std::cout << ss.str() << "\n";
                    LOG_CRIT << ss.str();
                    error_codes[i] = ErrorCode::PositionOverLimit;
                }
                is_safe = false;
            }
            if (joint_measurement.q_desired_pre[i] < lb[i] &&
                joint_measurement.velocity[i] < -position_limit_vel_threshold) {
                if (error_codes[i] != ErrorCode::PositionOverLimit) {
                    std::stringstream ss;
                    ss << "Joint " << model.names[i + 1]
                       << " exceeds lower limit, q_desired_pre: " << joint_measurement.q_desired_pre[i]
                       << ", limit: " << lb[i] << ", original limit: " << lb[i] - margin_rad[i]
                       << ", margin_rad: " << margin_rad[i] << ", velocity: " << joint_measurement.velocity[i]
                       << ", velocity threshold: " << -position_limit_vel_threshold;
                    std::cout << ss.str() << "\n";
                    LOG_CRIT << ss.str();
                    error_codes[i] = ErrorCode::PositionOverLimit;
                }
                is_safe = false;
            }
        }
    }

    return is_safe;
}
}  // namespace robot_server_lib
