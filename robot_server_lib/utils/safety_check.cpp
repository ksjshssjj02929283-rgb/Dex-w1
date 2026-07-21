#include "safety_check.h"

#include "NanoLog.hpp"

namespace robot_server_lib {

void SafetyChecker::Reset() {
    filtered_velocity_.setZero();
    velocity_over_limit_count_.fill(0);
    torque_over_limit_count_.fill(0);
    position_over_limit_count_.fill(0);
    initialized_ = false;
}

bool SafetyChecker::Check(const JointMeasurement& joint_measurement,
                          const RobotModel& robot_model,
                          std::array<ErrorCode, ndof_robot>& error_codes,
                          SafetyCheckConfig config) {
    const auto& params = *robot_model.params_ptr;
    const auto& model = *robot_model.model_ptr;
    bool is_safe = true;

    // 速度低通滤波: 抑制速度噪声, 仅用于位置限位判定
    if (!initialized_) {
        filtered_velocity_ = joint_measurement.velocity;
        initialized_ = true;
    } else {
        const double alpha = safety_velocity_filter_alpha;
        for (int i = 0; i < ndof_robot; ++i) {
            filtered_velocity_[i] = alpha * joint_measurement.velocity[i] + (1.0 - alpha) * filtered_velocity_[i];
        }
    }

    for (int i = 0; i < ndof_robot; i++) {
        // 速度越限检测 (连续帧确认, 不滤波)
        bool velocity_over = false;
        if (config.check_velocity_limit &&
            (joint_measurement.velocity[i] > params.joint_velocity_limit[i] * velocity_error_threshold ||
             joint_measurement.velocity[i] < -params.joint_velocity_limit[i] * velocity_error_threshold)) {
            velocity_over = true;
            if (error_codes[i] != ErrorCode::VelocityOverLimit) {
                std::stringstream ss;
                ss << "Joint " << model.names[i + 1]
                   << " velocity over limit, current velocity: " << joint_measurement.velocity[i] << ", limits: ["
                   << -params.joint_velocity_limit[i] * velocity_error_threshold << ", "
                   << params.joint_velocity_limit[i] * velocity_error_threshold << "]";
                std::cout << ss.str() << "\n";
                LOG_CRIT << ss.str();
            }
        }
        if (velocity_over) {
            velocity_over_limit_count_[i]++;
            if (velocity_over_limit_count_[i] >= safety_confirm_frames) {
                error_codes[i] = ErrorCode::VelocityOverLimit;
                is_safe = false;
            }
        } else {
            velocity_over_limit_count_[i] = 0;
        }

        // 力矩越限检测 (连续帧确认, 不滤波)
        bool torque_over = false;
        if (config.check_torque_limit &&
            (joint_measurement.effort[i] > params.joint_torque_limit[i] * torque_error_threshold ||
             joint_measurement.effort[i] < -params.joint_torque_limit[i] * torque_error_threshold)) {
            torque_over = true;
            if (error_codes[i] != ErrorCode::TorqueOverLimit) {
                std::stringstream ss;
                ss << "Joint " << model.names[i + 1]
                   << " torque over limit, current torque: " << joint_measurement.effort[i] << ", limits: ["
                   << -params.joint_torque_limit[i] * torque_error_threshold << ", "
                   << params.joint_torque_limit[i] * torque_error_threshold << "]";
                std::cout << ss.str() << "\n";
                LOG_CRIT << ss.str();
            }
        }
        if (torque_over) {
            torque_over_limit_count_[i]++;
            if (torque_over_limit_count_[i] >= safety_confirm_frames) {
                error_codes[i] = ErrorCode::TorqueOverLimit;
                is_safe = false;
            }
        } else {
            torque_over_limit_count_[i] = 0;
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
            // 使用滤波速度判定, 减少噪声导致的误触发
            const double filtered_vel = filtered_velocity_[i];
            bool over_limit_this_frame = false;

            if (joint_measurement.q_desired_pre[i] > ub[i] && filtered_vel > position_limit_vel_threshold) {
                over_limit_this_frame = true;
                if (error_codes[i] != ErrorCode::PositionOverLimit) {
                    std::stringstream ss;
                    ss << "Joint " << model.names[i + 1]
                       << " exceeds upper limit, q_desired_pre: " << joint_measurement.q_desired_pre[i]
                       << ", limit: " << ub[i] << ", original limit: " << ub[i] + margin_rad[i]
                       << ", margin_rad: " << margin_rad[i] << ", filtered velocity: " << filtered_vel
                       << ", velocity threshold: " << position_limit_vel_threshold
                       << ", confirm frames: " << position_over_limit_count_[i] + 1 << "/" << safety_confirm_frames;
                    std::cout << ss.str() << "\n";
                    LOG_CRIT << ss.str();
                }
            }
            if (joint_measurement.q_desired_pre[i] < lb[i] && filtered_vel < -position_limit_vel_threshold) {
                over_limit_this_frame = true;
                if (error_codes[i] != ErrorCode::PositionOverLimit) {
                    std::stringstream ss;
                    ss << "Joint " << model.names[i + 1]
                       << " exceeds lower limit, q_desired_pre: " << joint_measurement.q_desired_pre[i]
                       << ", limit: " << lb[i] << ", original limit: " << lb[i] - margin_rad[i]
                       << ", margin_rad: " << margin_rad[i] << ", filtered velocity: " << filtered_vel
                       << ", velocity threshold: " << -position_limit_vel_threshold
                       << ", confirm frames: " << position_over_limit_count_[i] + 1 << "/" << safety_confirm_frames;
                    std::cout << ss.str() << "\n";
                    LOG_CRIT << ss.str();
                }
            }

            // 连续帧确认: 仅连续超过 safety_confirm_frames 帧才真正报错
            if (over_limit_this_frame) {
                position_over_limit_count_[i]++;
                if (position_over_limit_count_[i] >= safety_confirm_frames) {
                    error_codes[i] = ErrorCode::PositionOverLimit;
                    is_safe = false;
                }
            } else {
                // 本帧未越限, 重置计数
                position_over_limit_count_[i] = 0;
            }
        }
    }

    return is_safe;
}
}  // namespace robot_server_lib
