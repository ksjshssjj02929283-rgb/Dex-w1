#include "joint_pd_controller.h"

#include <cmath>

namespace robot_server_lib {
namespace dynamics {

bool JointPdController::UpdateTorqueCommand(const VectorDof& q_error,
                                            const VectorDof& dq_error,
                                            const VectorDof& stiffness,
                                            const VectorDof& damping,
                                            const VectorDof& integral_ratio,
                                            VectorDof& tau_command) {
    // 积分项专用误差: 死区抑制噪声导致的积分漂移, PD 项保留原始误差
    VectorDof q_error_for_integral = q_error;
    for (int i = 0; i < ndof_robot; i++) {
        if (std::abs(q_error_for_integral[i]) < PositionThreshold) {
            q_error_for_integral[i] = 0.0;
        }
    }

    UpdateIntegralError(q_error_for_integral);
    const VectorDof torque_integral = (stiffness.cwiseProduct(integral_ratio)).asDiagonal() * integral_error_;
    // tau = K_p·q_error + K_v·dq_error + torque_integral
    tau_command = stiffness.asDiagonal() * q_error + damping.asDiagonal() * dq_error + torque_integral;
    return true;
}

void JointPdController::Reset() { integral_error_.setZero(); }

void JointPdController::UpdateIntegralError(const VectorDof& q_error) {
    integral_error_ = (integral_error_ + q_error).cwiseMin(error_integral_max_).cwiseMax(-error_integral_max_);
}

}  // namespace dynamics
}  // namespace robot_server_lib
