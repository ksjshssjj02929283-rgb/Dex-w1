#include "joint_impedance_controller.h"

namespace robot_server_lib {
namespace dynamics {

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
    UpdateComplianceOffset(feedback_q, sensor_wrench, enable_compliance);

    Output output;
    output.q_desired_base = cmd_q;
    output.joint_compliance_offset = joint_compliance_offset_;
    output.q_desired_compliant = cmd_q + joint_compliance_offset_;
    output.force_trigger_state = force_trigger_state_;

    VectorDof q_error = output.q_desired_compliant - feedback_q;
    VectorDof dq_error = cmd_dq - feedback_dq;

    VectorDof q_error_for_integral = ApplyIntegralDeadzone(q_error);
    if (force_trigger_state_[0] != 0) {
        for (int i = 0; i < robot_model_.params_ptr->joints_primary.size(); ++i) {
            const int joint_id = static_cast<int>(robot_model_.params_ptr->joints_primary[i]);
            q_error_for_integral[joint_id] = 0.0;
        }
    }
    if (force_trigger_state_[1] != 0) {
        for (int i = 0; i < robot_model_.params_ptr->joints_secondary.size(); ++i) {
            const int joint_id = static_cast<int>(robot_model_.params_ptr->joints_secondary[i]);
            q_error_for_integral[joint_id] = 0.0;
        }
    }

    integral_error_ =
        (integral_error_ + q_error_for_integral).cwiseMin(error_integral_max_).cwiseMax(-error_integral_max_);
    VectorDof torque_integral = (joint_stiffness * joint_integral_ratio).asDiagonal() * integral_error_;

    output.torque_joint =
        joint_stiffness.asDiagonal() * q_error + joint_damping.asDiagonal() * dq_error + torque_integral;
    return output;
}

VectorDof JointImpedanceController::ComputeHoldTorque(const VectorDof &feedback_q,
                                                      const VectorDof &feedback_dq,
                                                      const VectorDof &hold_q,
                                                      const VectorDof &joint_stiffness,
                                                      const VectorDof &joint_damping) {
    force_trigger_state_ = {0, 0};
    joint_compliance_offset_.setZero();
    integral_error_.setZero();
    return joint_stiffness.asDiagonal() * (hold_q - feedback_q) + joint_damping.asDiagonal() * (-feedback_dq);
}

VectorDof JointImpedanceController::ComputeDampingTorque(const VectorDof &feedback_dq,
                                                         const VectorDof &joint_damping) const {
    return joint_damping.asDiagonal() * (-feedback_dq);
}

void JointImpedanceController::Reset(const VectorDof &reference_q) {
    integral_error_.setZero();
    joint_compliance_offset_.setZero();
    force_trigger_state_ = {0, 0};
    (void)reference_q;
}

void JointImpedanceController::SetForceNormThreshold(const std::array<double, 2> &threshold) {
    force_norm_threshold_ = threshold;
}

void JointImpedanceController::SetComplianceGain(const std::array<double, 2> &gain) { compliance_gain_ = gain; }

void JointImpedanceController::SetComplianceReturnRatio(double return_ratio) {
    compliance_return_ratio_ = std::clamp(return_ratio, 0.0, 1.0);
}

void JointImpedanceController::SetComplianceOffsetMax(double offset_max) {
    compliance_offset_max_ = std::max(0.0, offset_max);
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

void JointImpedanceController::UpdateComplianceOffset(const VectorDof &feedback_q,
                                                      const Vector12d &sensor_wrench,
                                                      bool enable_compliance) {
    Eigen::VectorXd q_primary;
    Eigen::VectorXd q_secondary;
    utils::GetJointFromWholeBody(robot_model_, feedback_q, q_primary, MotionGroup::Primary);
    utils::GetJointFromWholeBody(robot_model_, feedback_q, q_secondary, MotionGroup::Secondary);

    UpdateArmComplianceOffset(q_primary, sensor_wrench.head<6>(), MotionGroup::Primary, 0, enable_compliance);
    UpdateArmComplianceOffset(q_secondary, sensor_wrench.tail<6>(), MotionGroup::Secondary, 1, enable_compliance);
}

void JointImpedanceController::UpdateArmComplianceOffset(const Eigen::VectorXd &q_arm,
                                                         const Vector6d &wrench_arm,
                                                         MotionGroup motion_group,
                                                         int arm_index,
                                                         bool enable_compliance) {
    const double wrench_norm = wrench_arm.norm();
    force_trigger_state_[arm_index] = (enable_compliance && wrench_norm >= force_norm_threshold_[arm_index]) ? 1 : 0;
    if (force_trigger_state_[arm_index] == 0) {
        DecayArmComplianceOffset(motion_group);
        return;
    }

    const auto &arm_model = motion_group == MotionGroup::Primary ? robot_model_.primary_arm_model_ptr
                                                                 : robot_model_.secondary_arm_model_ptr;
    Eigen::MatrixXd jacobian = pinocchio::GetJacobiMatrix(*arm_model, q_arm);
    Eigen::VectorXd offset_delta = compliance_gain_[arm_index] * jacobian.transpose() * wrench_arm * torque_ctrl_dt;
    Eigen::VectorXd arm_offset;
    utils::GetJointFromWholeBody(robot_model_, joint_compliance_offset_, arm_offset, motion_group);
    arm_offset += offset_delta;
    arm_offset = arm_offset.cwiseMin(compliance_offset_max_).cwiseMax(-compliance_offset_max_);
    utils::MapJointToWholeBody(robot_model_, arm_offset, joint_compliance_offset_, motion_group);
}

void JointImpedanceController::DecayArmComplianceOffset(MotionGroup motion_group) {
    Eigen::VectorXd arm_offset;
    utils::GetJointFromWholeBody(robot_model_, joint_compliance_offset_, arm_offset, motion_group);
    arm_offset *= (1.0 - compliance_return_ratio_);
    utils::MapJointToWholeBody(robot_model_, arm_offset, joint_compliance_offset_, motion_group);
}

}  // namespace dynamics
}  // namespace robot_server_lib
