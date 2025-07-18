#pragma once
#include "torque_controller_base.h"

namespace robot_server_lib
{

class IdleControllerInvDynamic : public TorqueControllerBase
{
   public:
    using TorqueControllerBase::TorqueControllerBase;
    bool ComputeCommand(const JointRawMeasurement &raw_measurement, const PlannerManagerCommand &input_cmd, TorqueCtrlCommand &output_cmd) override
    {
        VectorDof q = raw_measurement.position;
        VectorDof dq = raw_measurement.velocity;

        VectorDof ddq_desired{};
        ddq_desired = parameters_.K_p * (idle_q_ - q) + parameters_.K_v * (-dq);
        VectorDof tau = pinocchio::rnea(*robot_model_.model_ptr, robot_data_, q, dq, ddq_desired);

        output_cmd.torque = tau;
        output_cmd.q_desired = idle_q_;
        return true;
    }
    bool ResetForNewEpisode(const JointRawMeasurement &raw_measurement) override
    {
        idle_q_ = raw_measurement.position;
        return true;
    }

   protected:
    VectorDof idle_q_;
};

class DecelerationControllerInvDynamic : public TorqueControllerBase
{
   public:
    using TorqueControllerBase::TorqueControllerBase;
    bool ComputeCommand(const JointRawMeasurement &raw_measurement, const PlannerManagerCommand &input_cmd, TorqueCtrlCommand &output_cmd) override
    {
        VectorDof q = raw_measurement.position;
        VectorDof dq = raw_measurement.velocity;

        VectorDof ddq_desired{};
        ddq_desired = parameters_.K_v * (-dq);
        VectorDof tau = pinocchio::rnea(*robot_model_.model_ptr, robot_data_, q, dq, ddq_desired);

        output_cmd.torque = tau;
        output_cmd.q_desired = q;
        return true;
    }
    bool ResetForNewEpisode(const JointRawMeasurement &raw_measurement) override { return true; }
};

class OperatingControllerInvDynamic : public TorqueControllerBase
{
   public:
    using TorqueControllerBase::TorqueControllerBase;
    bool ComputeCommand(const JointRawMeasurement &raw_measurement, const PlannerManagerCommand &input_cmd, TorqueCtrlCommand &output_cmd) override
    {
        VectorDof feedback_q = raw_measurement.position;
        VectorDof feedback_dq = raw_measurement.velocity;
        VectorDof cmd_q = input_cmd.cmd.position;
        VectorDof cmd_dq = input_cmd.cmd.velocity;
        VectorDof cmd_ddq = input_cmd.cmd.acceleration;

        VectorDof ddq_desired{};
        ddq_desired = cmd_ddq + parameters_.K_p * (cmd_q - feedback_q) + parameters_.K_v * (cmd_dq - feedback_dq);
        // ddq_desired = cmd_ddq;
        VectorDof tau = pinocchio::rnea(*robot_model_.model_ptr, robot_data_, cmd_q, cmd_dq, ddq_desired);

        output_cmd.torque = tau + input_cmd.cmd.torque_feedforward;
        output_cmd.q_desired = cmd_q;
        return true;
    }
    bool ResetForNewEpisode(const JointRawMeasurement &raw_measurement) override { return true; }
};

}  // namespace robot_server_lib