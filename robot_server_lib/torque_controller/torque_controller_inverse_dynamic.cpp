#include "torque_controller_inverse_dynamic.h"

namespace robot_server_lib
{

TorqueControllerInvDynamic::TorqueControllerInvDynamic(RobotModel robot_model) : TorqueControllerBase(robot_model)
{
#ifndef TORQUE_CONTROL
    this->parameters_.K_p = 50;
    this->parameters_.K_v = 0;
#endif
}

bool TorqueControllerInvDynamic::ComputeCommand(const JointRawMeasurement &raw_measurement, const PlannerManagerCommand &input_cmd,
                                                TorqueCtrlCommand &output_cmd)
{
    VectorDof feedback_q = raw_measurement.position;
    VectorDof feedback_dq = raw_measurement.velocity;
    VectorDof cmd_q = input_cmd.cmd.position;
    VectorDof cmd_dq = input_cmd.cmd.velocity;
    VectorDof cmd_ddq = input_cmd.cmd.acceleration;
    VectorDof torque_feedforward = input_cmd.cmd.torque_feedforward;
    VectorDof ddq_desired{};
    VectorDof external_torque = this->GetJointExternalTorque(raw_measurement.sensor_wrench, feedback_q);

    switch (state_)
    {
    case TorqueControllerState::Idle:
        ddq_desired = this->parameters_.K_p * (idle_q_ - feedback_q) + this->parameters_.K_v * (-feedback_dq) + external_torque / this->parameters_.M_d;
        output_cmd.torque = dynamics::InverseDynamics(*robot_model_.model_ptr, robot_data_, feedback_q, feedback_dq, ddq_desired,
                                                      robot_model_.params_ptr->joint_command_sequence_by_name);
        output_cmd.q_desired = idle_q_;
        break;
    case TorqueControllerState::Decelerating:
        ddq_desired = this->parameters_.K_v * (-feedback_dq) + external_torque / this->parameters_.M_d;
        output_cmd.torque = dynamics::InverseDynamics(*robot_model_.model_ptr, robot_data_, feedback_q, feedback_dq, ddq_desired,
                                                      robot_model_.params_ptr->joint_command_sequence_by_name);
        output_cmd.q_desired = feedback_q;
        break;
    case TorqueControllerState::Operating:
        ddq_desired = cmd_ddq + this->parameters_.K_p * (cmd_q - feedback_q) + this->parameters_.K_v * (cmd_dq - feedback_dq) + external_torque / this->parameters_.M_d;
        output_cmd.torque = torque_feedforward + dynamics::InverseDynamics(*robot_model_.model_ptr, robot_data_, feedback_q, feedback_dq, ddq_desired,
                                                                           robot_model_.params_ptr->joint_command_sequence_by_name);
        output_cmd.q_desired = cmd_q;
        break;
    default:
        break;
    }
    return true;
}

bool TorqueControllerInvDynamic::ResetForNewEpisode(const JointRawMeasurement &raw_measurement, const TorqueControllerState &state)
{
    state_ = state;
    if (state_ == TorqueControllerState::Idle)
    {
        idle_q_ = raw_measurement.position;
    }
    return true;
}

}  // namespace robot_server_lib
