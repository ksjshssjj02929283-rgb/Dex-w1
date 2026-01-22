#include "torque_controller_inverse_dynamic.h"

namespace robot_server_lib {

TorqueControllerInvDynamic::TorqueControllerInvDynamic(RobotModel robot_model) : TorqueControllerBase(robot_model) {}

bool TorqueControllerInvDynamic::ComputeCommand(const JointRawMeasurement &raw_measurement,
                                                const PlannerManagerCommand &input_cmd,
                                                TorqueCtrlCommand &output_cmd) {
    VectorDof feedback_q = raw_measurement.position;
    VectorDof feedback_dq = raw_measurement.velocity;
    VectorDof cmd_q = input_cmd.cmd.position;
    VectorDof cmd_dq = input_cmd.cmd.velocity;
    VectorDof cmd_ddq = input_cmd.cmd.acceleration;
    VectorDof torque_ff = input_cmd.cmd.torque_feedforward;
    VectorDof torque_ref = VectorDof::Zero();
    VectorDof q_error = VectorDof::Zero();
    VectorDof dq_error = VectorDof::Zero();
    this->parameters_.K_p = input_cmd.cmd.joint_stiffness;
    this->parameters_.K_v = input_cmd.cmd.joint_damping;
    switch (state_) {
        case TorqueControllerState::Idle:
            torque_ref = this->parameters_.K_p.asDiagonal() * (idle_q_ - feedback_q) +
                         this->parameters_.K_v.asDiagonal() * (-feedback_dq);
            output_cmd.torque_feedforward =
                dynamics::InverseDynamics(this->robot_model_, feedback_q, feedback_dq, VectorDof::Zero());
            output_cmd.torque = output_cmd.torque_feedforward + torque_ref;
            output_cmd.q_desired = idle_q_;
            break;
        case TorqueControllerState::Decelerating:
            torque_ref = this->parameters_.K_v.asDiagonal() * (-feedback_dq);
            output_cmd.torque_feedforward =
                dynamics::InverseDynamics(this->robot_model_, feedback_q, feedback_dq, VectorDof::Zero());
            output_cmd.torque = output_cmd.torque_feedforward + torque_ref;
            output_cmd.q_desired = feedback_q;
            break;
        case TorqueControllerState::Operating: {
            q_error = cmd_q - feedback_q;
            dq_error = cmd_dq - feedback_dq;
            torque_ref = this->parameters_.K_p.asDiagonal() * q_error + this->parameters_.K_v.asDiagonal() * dq_error;
            output_cmd.torque_feedforward = dynamics::InverseDynamics(robot_model_, feedback_q, feedback_dq,
                                                                      VectorDof::Zero());  // 补偿重力, 科氏力, 摩擦力
            output_cmd.torque_impedance = output_cmd.torque_feedforward + torque_ff;
            output_cmd.torque = output_cmd.torque_feedforward + torque_ref;
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
