#pragma once

#include "inverse_dynamics.h"

namespace robot_server_lib {
namespace dynamics {

class JointImpedanceController {
public:
    JointImpedanceController() = default;
    explicit JointImpedanceController(RobotModel robot_model);

    bool UpdateTorqueCommand(const VectorDof& feedback_q,
                             const VectorDof& feedback_dq,
                             const VectorDof& cmd_q,
                             const VectorDof& cmd_dq,
                             const VectorDof& stiffness,
                             const VectorDof& damping,
                             double integral_ratio,
                             const Vector12d& sensor_wrench,
                             VectorDof& tau_command);

    void Reset();

private:
    VectorDof ComputeExternalTorque(const VectorDof& feedback_q, const Vector12d& sensor_wrench) const;
    Vector6d ProcessExternalWrench(const Vector6d& wrench) const;
    void AccumulateIntegralError(const VectorDof& q_error);

    RobotModel robot_model_;
    VectorDof integral_error_ = VectorDof::Zero();
    double error_integral_max_ = 0.1;
};

}  // namespace dynamics
}  // namespace robot_server_lib
