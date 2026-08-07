#pragma once

#include "data_structure.h"

namespace robot_server_lib {
namespace dynamics {

// 关节时延控制器: 根据期望与反馈加速度误差递推输出力矩
// tau[k] = sat(tau[k-1] + M_bar * (a_cmd[k] - a_fbk[k]))
class JointTimeDelayController {
public:
    explicit JointTimeDelayController(const TimeDelayControlConfig& config = TimeDelayControlConfig{});

    bool UpdateTorqueCommand(const VectorDof& cmd_pos,
                             const VectorDof& cmd_vel,
                             const VectorDof& feedback_pos,
                             const VectorDof& feedback_vel,
                             VectorDof& tau_command);

    void Reset();

private:
    VectorDof motor_inertia_ = VectorDof::Constant(0.007);
    VectorDof m_bar_ = VectorDof::Zero();

    VectorDof last_feedback_velocity_ = VectorDof::Zero();
    VectorDof last_command_velocity_ = VectorDof::Zero();
    VectorDof output_torque_ = VectorDof::Zero();
    bool initialized_{false};

    double cycle_time_{torque_ctrl_dt};
    VectorDof stiffness_ = VectorDof::Constant(100.0);
    VectorDof damping_ = VectorDof::Constant(20.0);
};

}  // namespace dynamics
}  // namespace robot_server_lib
