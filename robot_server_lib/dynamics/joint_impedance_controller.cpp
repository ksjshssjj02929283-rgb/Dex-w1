#include "joint_impedance_controller.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace robot_server_lib {
namespace dynamics {

JointImpedanceController::JointImpedanceController(RobotModel robot_model) : robot_model_(std::move(robot_model)) {}

bool JointImpedanceController::UpdateTorqueCommand(const VectorDof& feedback_q,
                                                   const VectorDof& feedback_dq,
                                                   const VectorDof& cmd_q,
                                                   const VectorDof& cmd_dq,
                                                   const VectorDof& stiffness,
                                                   const VectorDof& damping,
                                                   double integral_ratio,
                                                   const Vector12d& sensor_wrench,
                                                   VectorDof& tau_command,
                                                   VectorDof* torque_ext_out,
                                                   double dt,
                                                   const VectorDof* integral_enable) {
    VectorDof q_error = cmd_q - feedback_q;
    const VectorDof dq_error = cmd_dq - feedback_dq;

    constexpr double kSmallPositionError = 0.005 / 180.0 * 3.14159265358979323846;
    for (int i = 0; i < ndof_robot; i++) {
        if (std::abs(q_error[i]) < kSmallPositionError) {
            q_error[i] = 0.0;
        }
    }

    if (integral_ratio > 0.0) {
        AccumulateIntegralError(q_error, dt, integral_enable);
    } else {
        integral_error_.setZero();
    }
    const VectorDof torque_integral = (stiffness * integral_ratio).asDiagonal() * integral_error_;
    const VectorDof torque_ext = ComputeExternalTorque(feedback_q, sensor_wrench);
    if (torque_ext_out) {
        *torque_ext_out = torque_ext;
    }

    tau_command = stiffness.asDiagonal() * q_error + damping.asDiagonal() * dq_error + torque_ext + torque_integral;
    return true;
}

bool JointImpedanceController::UpdateTorqueCommandWithoutExternalTorque(const VectorDof& feedback_q,
                                                                        const VectorDof& feedback_dq,
                                                                        const VectorDof& cmd_q,
                                                                        const VectorDof& cmd_dq,
                                                                        const VectorDof& stiffness,
                                                                        const VectorDof& damping,
                                                                        double integral_ratio,
                                                                        VectorDof& tau_command,
                                                                        double dt,
                                                                        const VectorDof* integral_enable) {
    VectorDof q_error = cmd_q - feedback_q;
    const VectorDof dq_error = cmd_dq - feedback_dq;

    constexpr double kSmallPositionError = 0.005 / 180.0 * 3.14159265358979323846;
    for (int i = 0; i < ndof_robot; i++) {
        if (std::abs(q_error[i]) < kSmallPositionError) {
            q_error[i] = 0.0;
        }
    }

    if (integral_ratio > 0.0) {
        AccumulateIntegralError(q_error, dt, integral_enable);
    } else {
        integral_error_.setZero();
    }
    const VectorDof torque_integral = (stiffness * integral_ratio).asDiagonal() * integral_error_;
    tau_command = stiffness.asDiagonal() * q_error + damping.asDiagonal() * dq_error + torque_integral;
    return true;
}

void JointImpedanceController::Reset() {
    integral_error_.setZero();
}

VectorDof JointImpedanceController::ComputeExternalTorque(const VectorDof& feedback_q,
                                                          const Vector12d& sensor_wrench) const {
    VectorDof torque_ext = VectorDof::Zero();

    auto add_arm_external_torque = [&](MotionGroup motion_group, const Vector6d& raw_wrench) {
        const Vector6d wrench = ProcessExternalWrench(raw_wrench);
        if (wrench.isZero(FloatZeroTol)) {
            return;
        }

        Eigen::VectorXd q_arm;
        utils::GetJointFromWholeBody(robot_model_, feedback_q, q_arm, motion_group);

        const auto& arm_model =
            (motion_group == MotionGroup::Primary) ? *robot_model_.primary_arm_model_ptr
                                                   : *robot_model_.secondary_arm_model_ptr;
        const Eigen::MatrixXd jacobian = pinocchio::GetJacobiMatrix(arm_model, q_arm, pinocchio::LOCAL);
        const Eigen::VectorXd tau_arm = jacobian.transpose() * wrench;
        utils::MapJointToWholeBody(robot_model_, tau_arm, torque_ext, motion_group);
    };

    add_arm_external_torque(MotionGroup::Primary, sensor_wrench.head<6>());
    add_arm_external_torque(MotionGroup::Secondary, sensor_wrench.tail<6>());

    return torque_ext;
}

Vector6d JointImpedanceController::ProcessExternalWrench(const Vector6d& wrench) const {
    const auto& params = *robot_model_.params_ptr;
    Vector6d processed = Vector6d::Zero();

    auto deadband_saturate = [](double value, double deadband, double saturation) {
        const double magnitude = std::max(0.0, std::abs(value) - deadband);
        return std::copysign(std::min(magnitude, saturation), value);
    };

    for (int i = 0; i < 6; ++i) {
        const bool is_force = i < 3;
        const double deadband =
            is_force ? params.external_wrench_force_deadband_n : params.external_wrench_torque_deadband_nm;
        const double saturation =
            is_force ? params.external_wrench_force_saturation_n : params.external_wrench_torque_saturation_nm;
        processed[i] = deadband_saturate(wrench[i], deadband, saturation);
    }
    processed *= static_cast<double>(params.external_wrench_model_sign);
    return processed;
}

void JointImpedanceController::AccumulateIntegralError(const VectorDof& q_error,
                                                       double dt,
                                                       const VectorDof* integral_enable) {
    if (dt <= 0.0 || !std::isfinite(dt)) {
        return;
    }

    for (int i = 0; i < ndof_robot; ++i) {
        const bool enabled = (integral_enable == nullptr) || ((*integral_enable)[i] > 0.5);
        if (!enabled) {
            continue;
        }
        integral_error_[i] = std::clamp(integral_error_[i] + q_error[i] * dt,
                                        -error_integral_max_,
                                        error_integral_max_);
    }
}

}  // namespace dynamics
}  // namespace robot_server_lib
