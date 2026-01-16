#include "torque_controller_inverse_dynamic.h"

namespace robot_server_lib {

TorqueControllerInvDynamic::TorqueControllerInvDynamic(RobotModel robot_model) : TorqueControllerBase(robot_model) {
    impedance_ctrl_primary_ =
        std::make_shared<dynamics::CartesianImpedanceController>(robot_model_, MotionGroup::Primary);
    impedance_ctrl_secondary_ =
        std::make_shared<dynamics::CartesianImpedanceController>(robot_model_, MotionGroup::Secondary);
}

bool TorqueControllerInvDynamic::ComputeCommand(const JointRawMeasurement &raw_measurement,
                                                const PlannerManagerCommand &input_cmd,
                                                TorqueCtrlCommand &output_cmd) {
    VectorDof feedback_q = raw_measurement.position;
    VectorDof feedback_dq = raw_measurement.velocity;
    VectorDof cmd_q = input_cmd.cmd.position;
    VectorDof cmd_dq = input_cmd.cmd.velocity;
    VectorDof cmd_ddq = input_cmd.cmd.acceleration;
    VectorDof torque_ff = input_cmd.cmd.torque_feedforward;
    VectorDof ddq_desired = VectorDof::Zero();
    VectorDof q_error = VectorDof::Zero();
    VectorDof dq_error = VectorDof::Zero();
    this->parameters_.K_p = input_cmd.cmd.joint_stiffness;
    this->parameters_.K_v = 2.0 * this->parameters_.K_p.cwiseSqrt();  // 更新临界阻尼
    switch (state_) {
        case TorqueControllerState::Idle:
            ddq_desired = this->parameters_.K_p.asDiagonal() * (idle_q_ - feedback_q) +
                          this->parameters_.K_v.asDiagonal() * (-feedback_dq);
            output_cmd.torque_feedforward =
                dynamics::InverseDynamics(this->robot_model_, feedback_q, feedback_dq, VectorDof::Zero());
            output_cmd.torque = dynamics::InverseDynamics(this->robot_model_, feedback_q, feedback_dq, ddq_desired);
            output_cmd.q_desired = idle_q_;
            break;
        case TorqueControllerState::Decelerating:
            ddq_desired = this->parameters_.K_v.asDiagonal() * (-feedback_dq);
            output_cmd.torque_feedforward =
                dynamics::InverseDynamics(this->robot_model_, feedback_q, feedback_dq, VectorDof::Zero());
            output_cmd.torque = dynamics::InverseDynamics(this->robot_model_, feedback_q, feedback_dq, ddq_desired);
            output_cmd.q_desired = feedback_q;
            break;
        case TorqueControllerState::Operating: {
            q_error = cmd_q - feedback_q;
            dq_error = cmd_dq - feedback_dq;
            ddq_desired =
                cmd_ddq + this->parameters_.K_p.asDiagonal() * q_error + this->parameters_.K_v.asDiagonal() * dq_error;
            const auto &dyn_base_params_primary = this->robot_model_.params_ptr->dyn_base_params_primary;
            const auto &dyn_base_params_secondary = this->robot_model_.params_ptr->dyn_base_params_secondary;
            if (dyn_base_params_primary.has_value()) {
                Eigen::VectorXd tau_ff_pri, q_cmd_primary, q_feedback_primary, dq_feedback_primary, dq_cmd_primary;
                utils::GetJointFromWholeBody(this->robot_model_, cmd_q, q_cmd_primary, MotionGroup::Primary);
                utils::GetJointFromWholeBody(this->robot_model_, feedback_q, q_feedback_primary, MotionGroup::Primary);
                utils::GetJointFromWholeBody(this->robot_model_, feedback_dq, dq_feedback_primary,
                                             MotionGroup::Primary);
                utils::GetJointFromWholeBody(this->robot_model_, cmd_dq, dq_cmd_primary, MotionGroup::Primary);
                auto ret_pri = impedance_ctrl_primary_->UpdateTorqueCommand(q_feedback_primary, dq_feedback_primary,
                                                                            q_cmd_primary, dq_cmd_primary, tau_ff_pri);
                utils::MapJointToWholeBody(this->robot_model_, tau_ff_pri, torque_ff, MotionGroup::Primary);
            }
            if (dyn_base_params_secondary.has_value()) {
                Eigen::VectorXd tau_ff_sec, q_cmd_secondary, q_feedback_secondary, dq_feedback_secondary,
                    dq_cmd_secondary;
                utils::GetJointFromWholeBody(this->robot_model_, cmd_q, q_cmd_secondary, MotionGroup::Secondary);
                utils::GetJointFromWholeBody(this->robot_model_, feedback_q, q_feedback_secondary,
                                             MotionGroup::Secondary);
                utils::GetJointFromWholeBody(this->robot_model_, feedback_dq, dq_feedback_secondary,
                                             MotionGroup::Secondary);
                utils::GetJointFromWholeBody(this->robot_model_, cmd_dq, dq_cmd_secondary, MotionGroup::Secondary);
                auto ret_sec = impedance_ctrl_secondary_->UpdateTorqueCommand(
                    q_feedback_secondary, dq_feedback_secondary, q_cmd_secondary, dq_cmd_secondary, tau_ff_sec);
                utils::MapJointToWholeBody(this->robot_model_, tau_ff_sec, torque_ff, MotionGroup::Secondary);
            }
            output_cmd.torque_feedforward = dynamics::InverseDynamics(robot_model_, feedback_q, feedback_dq,
                                                                      VectorDof::Zero());  // 补偿重力, 科氏力, 摩擦力
            output_cmd.torque_impedance = output_cmd.torque_feedforward + torque_ff;
            output_cmd.torque = dynamics::InverseDynamics(robot_model_, feedback_q, feedback_dq, ddq_desired);
            output_cmd.q_desired = cmd_q;
        }
        default:
            break;
    }
    return true;
}

bool TorqueControllerInvDynamic::ResetForNewEpisode(const JointRawMeasurement &raw_measurement,
                                                    const TorqueControllerState &state) {
    state_ = state;
    if (state_ == TorqueControllerState::Idle) {
        idle_q_ = raw_measurement.position;
        LOG_INFO << "TorqueControllerInvDynamic set idle_q_: " << idle_q_.transpose();
    }
    return true;
}

}  // namespace robot_server_lib
