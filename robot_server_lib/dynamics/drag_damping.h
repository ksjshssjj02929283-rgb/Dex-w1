#pragma once

#include <cmath>

namespace robot_server_lib::dynamics {

inline double ComputeDragTorque(double torque_feedforward,
                                double velocity,
                                double damping_d2,
                                double deadzone_compensation,
                                double velocity_deadzone,
                                double dq_lowspeed) {
    const double deadzone_torque =
        std::abs(velocity) < velocity_deadzone ? deadzone_compensation * ((velocity > 0.0) - (velocity < 0.0)) : 0.0;
    // dq_lowspeed is a finite positive reference speed, validated when loading YAML.
    const double normalized_velocity = velocity / dq_lowspeed;
    const double damping_torque =
        damping_d2 == 0.0 ? 0.0 : damping_d2 * std::abs(normalized_velocity) * normalized_velocity;
    return torque_feedforward - damping_torque + deadzone_torque;
}

}  // namespace robot_server_lib::dynamics
