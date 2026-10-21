#pragma once

#include <cmath>
#include <string_view>

namespace robot_server_lib::dynamics {

inline double ComputeDragTorque(double torque_feedforward,
                                double velocity,
                                double damping_d2,
                                double f_dead,
                                double velocity_deadzone,
                                double dq_lowspeed,
                                double omega,
                                double elapsed_seconds,
                                double friction_feedforward,
                                std::string_view friction_model) {
    // dq_lowspeed is a finite positive reference speed, validated when loading YAML.
    const double normalized_velocity = velocity / dq_lowspeed;
    const double damping_torque =
        damping_d2 == 0.0 ? 0.0 : damping_d2 * std::abs(normalized_velocity) * normalized_velocity;
    const bool in_deadzone = std::abs(velocity) < velocity_deadzone;
    if (friction_model == "smooth" || friction_model == "stribeck") {
        // Keep smooth friction and damping at all speeds; excite only inside the deadzone.
        const double deadzone_torque = in_deadzone ? f_dead * std::sin(omega * elapsed_seconds) : 0.0;
        return torque_feedforward + deadzone_torque - damping_torque;
    } else if (in_deadzone) {
        // Classic (sgn) keeps the existing deadzone behavior.
        return torque_feedforward - friction_feedforward + f_dead * std::sin(omega * elapsed_seconds);
    } else {
        // The previous tau_dead term is zero at and above the velocity threshold.
        return torque_feedforward - damping_torque;
    }
}

}  // namespace robot_server_lib::dynamics
