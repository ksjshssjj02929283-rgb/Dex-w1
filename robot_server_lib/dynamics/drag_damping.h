#pragma once

#include <cmath>

namespace robot_server_lib::dynamics {

inline double ComputeDragTorque(double torque_feedforward,
                                double velocity,
                                double damping_d2,
                                double deadzone_compensation,
                                double velocity_deadzone) {
    const double deadzone_torque =
        std::abs(velocity) < velocity_deadzone ? deadzone_compensation * ((velocity > 0.0) - (velocity < 0.0)) : 0.0;
    return torque_feedforward - damping_d2 * std::abs(velocity) * velocity + deadzone_torque;
}

}  // namespace robot_server_lib::dynamics
