#pragma once

#include <cmath>

namespace robot_server_lib::dynamics {

inline double ComputeDragTorque(double torque_feedforward, double velocity, double damping_d1, double damping_d2) {
    return torque_feedforward - damping_d1 * velocity - damping_d2 * std::abs(velocity) * velocity;
}

}
