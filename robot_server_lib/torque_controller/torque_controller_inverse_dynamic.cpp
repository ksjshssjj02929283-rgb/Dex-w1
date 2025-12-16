#include "torque_controller_inverse_dynamic.h"

namespace robot_server_lib
{

TorqueControllerInvDynamic::TorqueControllerInvDynamic(RobotModel robot_model) : TorqueControllerBase(robot_model)
{
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
    VectorDof torque_external = this->GetJointExternalTorque(raw_measurement.sensor_wrench, feedback_q);
    output_cmd.torque_external = torque_external;
    switch (state_)
    {
    case TorqueControllerState::Idle:
        ddq_desired = this->parameters_.K_p * (idle_q_ - feedback_q) + this->parameters_.K_v * (-feedback_dq);
        output_cmd.torque = dynamics::InverseDynamics(robot_model_, robot_data_, feedback_q, feedback_dq, ddq_desired);
        output_cmd.torque_feedforward = dynamics::InverseDynamics(robot_model_, robot_data_, feedback_q, VectorDof::Zero(), VectorDof::Zero());
        output_cmd.q_desired = idle_q_;
        break;
    case TorqueControllerState::Decelerating:
        ddq_desired = this->parameters_.K_v * (-feedback_dq);
        output_cmd.torque = dynamics::InverseDynamics(robot_model_, robot_data_, feedback_q, feedback_dq, ddq_desired);
        output_cmd.torque_feedforward = dynamics::InverseDynamics(robot_model_, robot_data_, feedback_q, VectorDof::Zero(), VectorDof::Zero());
        output_cmd.q_desired = feedback_q;
        break;
    case TorqueControllerState::Operating:
        ddq_desired = cmd_ddq + this->parameters_.K_p * (cmd_q - feedback_q) + this->parameters_.K_v * (cmd_dq - feedback_dq);
        output_cmd.torque = torque_feedforward + dynamics::InverseDynamics(robot_model_, robot_data_, feedback_q, feedback_dq, ddq_desired);
        output_cmd.torque_feedforward = dynamics::InverseDynamics(robot_model_, robot_data_, feedback_q, VectorDof::Zero(), VectorDof::Zero());
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
        LOG_INFO << "TorqueControllerInvDynamic set idle_q_: " << idle_q_.transpose();
    }
    return true;
}

}  // namespace robot_server_lib
