#pragma once
#include "torque_controller_base.h"

namespace robot_server_lib
{

class TorqueControllerInvDynamic : public TorqueControllerBase
{
   public:
    TorqueControllerInvDynamic(RobotModel robot_model) : TorqueControllerBase(robot_model)
    {
        parameters_.K_p = 50;
        parameters_.K_v = 0;
    }    
    bool ComputeCommand(const JointRawMeasurement &raw_measurement, const PlannerManagerCommand &input_cmd, TorqueCtrlCommand &output_cmd) override
    {
        VectorDof feedback_q = raw_measurement.position;
        VectorDof feedback_dq = raw_measurement.velocity;
        VectorDof cmd_q = input_cmd.cmd.position;
        VectorDof cmd_dq = input_cmd.cmd.velocity;
        VectorDof cmd_ddq = input_cmd.cmd.acceleration;
        VectorDof torque_feedforward = input_cmd.cmd.torque_feedforward;
        VectorDof ddq_desired{};

        switch (state_)
        {
        case TorqueControllerState::Idle:
            ddq_desired = parameters_.K_p * (idle_q_ - feedback_q) + parameters_.K_v * (-feedback_dq);
            output_cmd.torque = pinocchio::InverseDynamics(*robot_model_.model_ptr, robot_data_, feedback_q, feedback_dq, ddq_desired,
                                                           robot_model_.params_ptr->joint_command_sequence_by_name);
            output_cmd.q_desired = idle_q_;
            break;
        case TorqueControllerState::Decelerating:
            ddq_desired = parameters_.K_v * (-feedback_dq);
            output_cmd.torque = pinocchio::InverseDynamics(*robot_model_.model_ptr, robot_data_, feedback_q, feedback_dq, ddq_desired,
                                                           robot_model_.params_ptr->joint_command_sequence_by_name);
            output_cmd.q_desired = feedback_q;
            break;
        case TorqueControllerState::Operating:
            ddq_desired = cmd_ddq + parameters_.K_p * (cmd_q - feedback_q) + parameters_.K_v * (cmd_dq - feedback_dq);
            output_cmd.torque = torque_feedforward + pinocchio::InverseDynamics(*robot_model_.model_ptr, robot_data_, feedback_q, feedback_dq,
                                                                                ddq_desired, robot_model_.params_ptr->joint_command_sequence_by_name);
            output_cmd.q_desired = cmd_q;
            break;
        default:
            break;
        }
        return true;
    }
    bool ResetForNewEpisode(const JointRawMeasurement &raw_measurement, const TorqueControllerState &state) override
    {
        state_ = state;
        if (state_ == TorqueControllerState::Idle)
        {
            idle_q_ = raw_measurement.position;
        }
        return true;
    }

   protected:
    VectorDof idle_q_;
};

}  // namespace robot_server_lib