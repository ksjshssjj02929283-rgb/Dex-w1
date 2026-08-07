#include "joint_time_delay_controller.h"

#include <algorithm>
#include <cmath>

namespace robot_server_lib {
namespace dynamics {

JointTimeDelayController::JointTimeDelayController(const TimeDelayControlConfig& config) {
    for (int i = 0; i < ndof_robot; ++i) {
        motor_inertia_[i] = config.joints[i].motor_inertia;
        m_bar_[i] = config.joints[i].m_bar_norm * motor_inertia_[i];
        stiffness_[i] = config.joints[i].stiffness;
        damping_[i] = config.joints[i].damping;
    }
}

bool JointTimeDelayController::UpdateTorqueCommand(const VectorDof& cmd_pos,
                                                   const VectorDof& cmd_vel,
                                                   const VectorDof& feedback_pos,
                                                   const VectorDof& feedback_vel,
                                                   VectorDof& tau_command) {
    if (!initialized_) {
        last_feedback_velocity_ = feedback_vel;
        last_command_velocity_ = cmd_vel;
        initialized_ = true;
        tau_command = output_torque_;
        return true;
    }

    const VectorDof position_error = cmd_pos - feedback_pos;
    const VectorDof velocity_error = cmd_vel - feedback_vel;
    const VectorDof feedback_acceleration = (feedback_vel - last_feedback_velocity_) / cycle_time_;
    VectorDof command_acceleration = (cmd_vel - last_command_velocity_) / cycle_time_;

    last_feedback_velocity_ = feedback_vel;
    last_command_velocity_ = cmd_vel;

    command_acceleration += stiffness_.cwiseProduct(position_error) + damping_.cwiseProduct(velocity_error);
    for (int i = 0; i < ndof_robot; ++i) {
        command_acceleration[i] = std::clamp(command_acceleration[i], -2.0 * M_PI, 2.0 * M_PI);
        const double clamped_feedback_acc = std::clamp(feedback_acceleration[i], -2.0 * M_PI, 2.0 * M_PI);
        output_torque_[i] += m_bar_[i] * (command_acceleration[i] - clamped_feedback_acc);
    }

    tau_command = output_torque_;
    return true;
}

void JointTimeDelayController::Reset() {
    last_feedback_velocity_.setZero();
    last_command_velocity_.setZero();
    output_torque_.setZero();
    initialized_ = false;
}

}  // namespace dynamics
}  // namespace robot_server_lib
