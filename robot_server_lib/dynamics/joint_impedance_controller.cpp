#include "joint_impedance_controller.h"

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
                                                   VectorDof& tau_command) {
    VectorDof q_error = cmd_q - feedback_q;
    const VectorDof dq_error = cmd_dq - feedback_dq;

    constexpr double kSmallPositionError = 0.005 / 180.0 * 3.14159265358979323846;
    for (int i = 0; i < ndof_robot; i++) {
        if (std::abs(q_error[i]) < kSmallPositionError) {
            q_error[i] = 0.0;
        }
    }

    AccumulateIntegralError(q_error);
    const VectorDof torque_integral = (stiffness * integral_ratio).asDiagonal() * integral_error_;
    const VectorDof torque_ext = ComputeExternalTorque(feedback_q, sensor_wrench);

    tau_command = stiffness.asDiagonal() * q_error + damping.asDiagonal() * dq_error + torque_integral + torque_ext;
    return true;
}

void JointImpedanceController::Reset() {
    integral_error_.setZero();
}

VectorDof JointImpedanceController::ComputeExternalTorque(const VectorDof& feedback_q,
                                                          const Vector12d& sensor_wrench) const {
    VectorDof torque_ext = VectorDof::Zero();

    auto add_arm_external_torque = [&](MotionGroup motion_group, const Vector6d& wrench) {
        if (wrench.isZero(FloatZeroTol)) {
            return;
        }

        Eigen::VectorXd q_arm;
        utils::GetJointFromWholeBody(robot_model_, feedback_q, q_arm, motion_group);

        const auto& arm_model =
            (motion_group == MotionGroup::Primary) ? *robot_model_.primary_arm_model_ptr
                                                   : *robot_model_.secondary_arm_model_ptr;
        const Eigen::MatrixXd jacobian = pinocchio::GetJacobiMatrix(arm_model, q_arm);
        const Eigen::VectorXd tau_arm = jacobian.transpose() * wrench;
        utils::MapJointToWholeBody(robot_model_, tau_arm, torque_ext, motion_group);
    };

    add_arm_external_torque(MotionGroup::Primary, sensor_wrench.head<6>());
    add_arm_external_torque(MotionGroup::Secondary, sensor_wrench.tail<6>());

    return torque_ext;
}

void JointImpedanceController::AccumulateIntegralError(const VectorDof& q_error) {
    integral_error_ = (integral_error_ + q_error).cwiseMin(error_integral_max_).cwiseMax(-error_integral_max_);
}

}  // namespace dynamics
}  // namespace robot_server_lib
