#include "joint_impedance_controller.h"

namespace robot_server_lib {
namespace dynamics {

namespace {

Vector6d LimitWrenchForComputation(const Vector6d &wrench) {
    constexpr double kForceNormMax = 50.0;
    constexpr double kForceNormMin = 0.3;
    constexpr double kTorqueNormMax = 10.0;
    constexpr double kTorqueNormMin = 0.1;

    Vector6d out = wrench;

    Vector3d force = out.head<3>();
    const double force_norm = force.norm();
    if (force_norm < kForceNormMin) {
        force.setZero();
    } else if (force_norm > kForceNormMax) {
        force *= (kForceNormMax / force_norm);
    }

    Vector3d torque = out.tail<3>();
    const double torque_norm = torque.norm();
    if (torque_norm < kTorqueNormMin) {
        torque.setZero();
    } else if (torque_norm > kTorqueNormMax) {
        torque *= (kTorqueNormMax / torque_norm);
    }

    out.head<3>() = force;
    out.tail<3>() = torque;
    return out;
}

}  // namespace

JointImpedanceController::JointImpedanceController(const RobotModel &robot_model) : robot_model_(robot_model) {}

JointImpedanceController::Output JointImpedanceController::UpdateTorqueCommand(const VectorDof &feedback_q,
                                                                               const VectorDof &feedback_dq,
                                                                               const VectorDof &cmd_q,
                                                                               const VectorDof &cmd_dq,
                                                                               const Vector12d &sensor_wrench,
                                                                               const VectorDof &joint_stiffness,
                                                                               const VectorDof &joint_damping,
                                                                               double joint_integral_ratio,
                                                                               bool enable_compliance) {
    Output output;
    output.q_desired_base = cmd_q;
    output.q_desired_compliant = cmd_q;
    // joint_compliance_offset and force_trigger_state keep default zero values

    VectorDof q_error = cmd_q - feedback_q;
    VectorDof dq_error = cmd_dq - feedback_dq;

    VectorDof q_error_for_integral = ApplyIntegralDeadzone(q_error);

    integral_error_ =
        (integral_error_ + q_error_for_integral).cwiseMin(error_integral_max_).cwiseMax(-error_integral_max_);
    VectorDof torque_integral = (joint_stiffness * joint_integral_ratio).asDiagonal() * integral_error_;

    VectorDof torque_base =
        joint_stiffness.asDiagonal() * q_error + joint_damping.asDiagonal() * dq_error + torque_integral;

    VectorDof torque_external = VectorDof::Zero();
    if (enable_compliance) {
        torque_external = ComputeExternalTorque(feedback_q, sensor_wrench);
    }

    output.torque_joint = torque_base + torque_external;
    return output;
}

VectorDof JointImpedanceController::ComputeHoldTorque(const VectorDof &feedback_q,
                                                      const VectorDof &feedback_dq,
                                                      const VectorDof &hold_q,
                                                      const VectorDof &joint_stiffness,
                                                      const VectorDof &joint_damping) {
    integral_error_.setZero();
    return joint_stiffness.asDiagonal() * (hold_q - feedback_q) + joint_damping.asDiagonal() * (-feedback_dq);
}

VectorDof JointImpedanceController::ComputeDampingTorque(const VectorDof &feedback_dq,
                                                         const VectorDof &joint_damping) const {
    return joint_damping.asDiagonal() * (-feedback_dq);
}

void JointImpedanceController::Reset(const VectorDof &reference_q) {
    integral_error_.setZero();
    (void)reference_q;
}

VectorDof JointImpedanceController::ApplyIntegralDeadzone(const VectorDof &q_error) const {
    VectorDof output = q_error;
    for (int i = 0; i < ndof_robot; ++i) {
        if (std::abs(output[i]) < integral_deadzone_rad_) {
            output[i] = 0.0;
        }
    }
    return output;
}

VectorDof JointImpedanceController::ComputeExternalTorque(const VectorDof &feedback_q,
                                                          const Vector12d &sensor_wrench) const {
    VectorDof torque_external = VectorDof::Zero();
    const Vector6d sensor_wrench_primary = LimitWrenchForComputation(sensor_wrench.head<6>());
    const Vector6d sensor_wrench_secondary = LimitWrenchForComputation(sensor_wrench.tail<6>());

    // Primary arm: -J_local(q)^T * W_ext_ee
    // sensor_wrench 在末端坐标系(body frame)中表达，Jacobian 必须使用 LOCAL 帧以保持一致
    Eigen::VectorXd q_primary;
    utils::GetJointFromWholeBody(robot_model_, feedback_q, q_primary, MotionGroup::Primary);
    const auto &primary_model = robot_model_.primary_arm_model_ptr;
    Eigen::MatrixXd jacobian_primary = pinocchio::GetJacobiMatrix(*primary_model, q_primary, pinocchio::LOCAL);
    Eigen::VectorXd tau_primary = -jacobian_primary.transpose() * sensor_wrench_primary;
    utils::MapJointToWholeBody(robot_model_, tau_primary, torque_external, MotionGroup::Primary);

    // Secondary arm: -J_local(q)^T * W_ext_ee
    Eigen::VectorXd q_secondary;
    utils::GetJointFromWholeBody(robot_model_, feedback_q, q_secondary, MotionGroup::Secondary);
    const auto &secondary_model = robot_model_.secondary_arm_model_ptr;
    Eigen::MatrixXd jacobian_secondary = pinocchio::GetJacobiMatrix(*secondary_model, q_secondary, pinocchio::LOCAL);
    Eigen::VectorXd tau_secondary = -jacobian_secondary.transpose() * sensor_wrench_secondary;
    utils::MapJointToWholeBody(robot_model_, tau_secondary, torque_external, MotionGroup::Secondary);

    return torque_external;
}

}  // namespace dynamics
}  // namespace robot_server_lib