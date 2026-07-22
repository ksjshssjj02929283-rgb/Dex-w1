#pragma once

#include "data_structure.h"

namespace robot_server_lib {
namespace dynamics {

// 关节PD控制器: 实现关节空间 PD 控制 + 积分项
// tau = K_p·q_error + K_v·dq_error + K_p·K_i_ratio·∫q_error
class JointPdController {
public:
    JointPdController() = default;

    // PD 控制: PD + 积分
    bool UpdateTorqueCommand(const VectorDof& q_error,
                             const VectorDof& dq_error,
                             const VectorDof& stiffness,
                             const VectorDof& damping,
                             const VectorDof& integral_ratio,
                             VectorDof& tau_command);

    void Reset();

private:
    // 累加位置误差积分并限幅
    void UpdateIntegralError(const VectorDof& q_error);

    VectorDof integral_error_ = VectorDof::Zero();  // 位置误差积分项
    double error_integral_max_ = 0.5;               // 积分项限幅, 防止积分饱和
};

}  // namespace dynamics
}  // namespace robot_server_lib
