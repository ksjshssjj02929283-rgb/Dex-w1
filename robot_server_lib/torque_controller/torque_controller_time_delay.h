#pragma once
#include "torque_controller_base.h"
#include <sstream>
namespace robot_server_lib
{
//todo: param tune
class TorqueControllerTimeDelay : public TorqueControllerBase
{
   public:
    TorqueControllerTimeDelay(RobotModel robot_model)
        : TorqueControllerBase(robot_model)
    {
        idle_q_.setZero();
        last_feedback_dq_.setZero();
        output_tau_.setZero();
        last_measurement_time_point_ = TimePointSystem::Invalid();
        // todo: param tune
        M_bar_ << 0.1, 0.1, 0.1, 0.1, 0.01, 0.01,
                  0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
                  0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
        parameters_.K_p = 10;
        parameters_.K_v = 6;
    }
    bool ComputeCommand(const JointRawMeasurement &raw_measurement, const PlannerManagerCommand &input_cmd, TorqueCtrlCommand &output_cmd) override
    {
        if (last_measurement_time_point_.is_invalid())
        {
            last_measurement_time_point_ = raw_measurement.time_point;
            return true;
        }
        double dt_micro = (raw_measurement.time_point.time_in_ns() - last_measurement_time_point_.time_in_ns()) / 1e3;
        if (dt_micro < torque_ctrl_dt)
        {
            return true;
        }
        VectorDof feedback_q = raw_measurement.position;
        VectorDof feedback_dq = raw_measurement.velocity;
        VectorDof feedback_ddq = (feedback_dq - last_feedback_dq_) / dt_micro;
        VectorDof cmd_q = input_cmd.cmd.position;
        VectorDof cmd_dq = input_cmd.cmd.velocity;
        VectorDof cmd_ddq = input_cmd.cmd.acceleration;
        VectorDof torque_feedforward = input_cmd.cmd.torque_feedforward;
        VectorDof q_desired{};
        VectorDof ddq_desired{};
        
        switch (state_)
        {
            case TorqueControllerState::Idle:
                ddq_desired = parameters_.K_p * (idle_q_ - feedback_q) + parameters_.K_v * (-feedback_dq);
                output_tau_ += M_bar_.cwiseProduct(ddq_desired - feedback_ddq);
                q_desired = idle_q_;
                break;
            case TorqueControllerState::Decelerating:
                ddq_desired = parameters_.K_v * (-feedback_dq);
                output_tau_ += M_bar_.cwiseProduct(ddq_desired - feedback_ddq);
                q_desired = feedback_q;
                break;
            case TorqueControllerState::Operating:
            {
                auto eigen_to_string = [](const Eigen::VectorXd& v) {
                    std::ostringstream oss;
                    oss.precision(4);
                    oss << std::fixed << v.transpose();
                    return oss.str();
                };
                ddq_desired = parameters_.K_p * (cmd_q - feedback_q) + parameters_.K_v * (cmd_dq - feedback_dq);
                LOG_DEBUG << "dt ms: " << dt_micro / 1000.0;
                LOG_DEBUG << "error_q: " << eigen_to_string(cmd_q - feedback_q);
                LOG_DEBUG << "error_dq: " << eigen_to_string(cmd_dq - feedback_dq);
                LOG_DEBUG << "ddq_desired: " << eigen_to_string(ddq_desired);

                output_tau_ += M_bar_.cwiseProduct(ddq_desired - feedback_ddq);

                LOG_DEBUG << "output_tau: " << eigen_to_string(output_tau_);
                q_desired = cmd_q;
                break;
            }
            default:
                break;
        }

        last_feedback_dq_ = feedback_dq;
        last_measurement_time_point_ = raw_measurement.time_point;
        output_cmd.torque = output_tau_;
        output_cmd.q_desired = q_desired;
        return true;
    }
    bool ResetForNewEpisode(const JointRawMeasurement &raw_measurement, const TorqueControllerState &state) override
    {
        state_ = state;
        if (state_ == TorqueControllerState::Idle)
        {
            idle_q_ = raw_measurement.position;
            last_feedback_dq_ = raw_measurement.velocity;
        }
        return true;
    }

   protected:
        VectorDof idle_q_;
        VectorDof last_feedback_dq_;
        TimePointSystem last_measurement_time_point_;
        VectorDof output_tau_;
        VectorDof M_bar_;
};

}  // namespace robot_server_lib