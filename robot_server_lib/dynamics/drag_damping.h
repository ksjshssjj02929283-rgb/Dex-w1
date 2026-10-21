#pragma once

#include <cmath>

namespace robot_server_lib::dynamics {

inline double ComputeDragTorque(double torque_feedforward,
                                double velocity,
                                double damping_d2,
                                double f_dead,
                                double velocity_deadzone,
                                double dq_lowspeed,
                                double omega,
                                double elapsed_seconds,
                                double friction_feedforward) {
    // Keep gravity and other dynamics; replace only friction with the sinusoid.
    if (std::abs(velocity) < velocity_deadzone) {
        return torque_feedforward - friction_feedforward + f_dead * std::sin(omega * elapsed_seconds);
    } else {
        // dq_lowspeed is a finite positive reference speed, validated when loading YAML.
        const double normalized_velocity = velocity / dq_lowspeed;
        const double damping_torque =
            damping_d2 == 0.0 ? 0.0 : damping_d2 * std::abs(normalized_velocity) * normalized_velocity;
        // The previous tau_dead term is zero at and above the velocity threshold.
        return torque_feedforward - damping_torque;
    }
}

}  // namespace robot_server_lib::dynamics
