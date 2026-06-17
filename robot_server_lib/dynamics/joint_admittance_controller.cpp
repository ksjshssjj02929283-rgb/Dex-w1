#include "joint_admittance_controller.h"

#include <algorithm>
#include <cmath>

namespace robot_server_lib {
namespace dynamics {

namespace {
constexpr double kDefaultTorqueGain = 1.0;
constexpr double kDefaultVirtualInertia = 1.0;
constexpr double kDefaultNaturalFrequencyHz = 1.5;
constexpr double kDefaultDampingRatio = 1.2;
constexpr double kDefaultOffsetLimit = 0.05;
constexpr double kDefaultVelocityLimit = 0.4;
constexpr double kDefaultAccelerationLimit = 5.0;

double PositiveOrDefault(double value, double default_value) {
    return (std::isfinite(value) && value > 0.0) ? value : default_value;
}

double NonNegativeOrDefault(double value, double default_value) {
    return (std::isfinite(value) && value >= 0.0) ? value : default_value;
}
}  // namespace

JointAdmittanceController::JointAdmittanceController() {
    Configure(JointAdmittanceConfig{});
}

JointAdmittanceController::JointAdmittanceController(const JointAdmittanceConfig& config) {
    Configure(config);
}

void JointAdmittanceController::Configure(const JointAdmittanceConfig& config) {
    config_ = config;
    for (int i = 0; i < ndof_robot; ++i) {
        const auto& joint = config_.joints[i];
        torque_gain_[i] = std::isfinite(joint.torque_gain) ? joint.torque_gain : kDefaultTorqueGain;
        virtual_inertia_[i] = PositiveOrDefault(joint.virtual_inertia, kDefaultVirtualInertia);
        natural_frequency_hz_[i] = PositiveOrDefault(joint.natural_frequency_hz, kDefaultNaturalFrequencyHz);
        damping_ratio_[i] = PositiveOrDefault(joint.damping_ratio, kDefaultDampingRatio);
        offset_limit_[i] = NonNegativeOrDefault(joint.offset_limit, kDefaultOffsetLimit);
        velocity_limit_[i] = NonNegativeOrDefault(joint.velocity_limit, kDefaultVelocityLimit);
        acceleration_limit_[i] = NonNegativeOrDefault(joint.acceleration_limit, kDefaultAccelerationLimit);
    }
    RecomputeDerivedParams();
    Reset();
}

void JointAdmittanceController::Reset() {
    q_offset_.setZero();
    dq_offset_.setZero();
    ddq_offset_.setZero();
}

void JointAdmittanceController::DecayToZero(double dt) {
    VectorDof q_unused = VectorDof::Zero();
    VectorDof dq_unused = VectorDof::Zero();
    Update(VectorDof::Zero(), dt, q_unused, dq_unused);
}

bool JointAdmittanceController::Update(const VectorDof& torque_ext,
                                       double dt,
                                       VectorDof& q_offset_out,
                                       VectorDof& dq_offset_out) {
    if (!config_.enabled || dt <= 0.0 || !std::isfinite(dt)) {
        Reset();
        q_offset_out = q_offset_;
        dq_offset_out = dq_offset_;
        return true;
    }

    for (int i = 0; i < ndof_robot; ++i) {
        if (!config_.joints[i].enabled) {
            q_offset_[i] = 0.0;
            dq_offset_[i] = 0.0;
            ddq_offset_[i] = 0.0;
            continue;
        }

        const double external_torque = std::isfinite(torque_ext[i]) ? torque_ext[i] : 0.0;
        const double input_torque = torque_gain_[i] * external_torque;
        ddq_offset_[i] = (input_torque - damping_[i] * dq_offset_[i] - stiffness_[i] * q_offset_[i]) /
                         virtual_inertia_[i];
    }

    ClampState();
    dq_offset_ += ddq_offset_ * dt;
    ClampState();
    q_offset_ += dq_offset_ * dt;
    ClampState();

    q_offset_out = q_offset_;
    dq_offset_out = dq_offset_;
    return true;
}

void JointAdmittanceController::RecomputeDerivedParams() {
    for (int i = 0; i < ndof_robot; ++i) {
        const double omega_n = 2.0 * M_PI * natural_frequency_hz_[i];
        stiffness_[i] = virtual_inertia_[i] * omega_n * omega_n;
        damping_[i] = 2.0 * damping_ratio_[i] * virtual_inertia_[i] * omega_n;
    }
}

void JointAdmittanceController::ClampState() {
    for (int i = 0; i < ndof_robot; ++i) {
        ddq_offset_[i] = std::clamp(ddq_offset_[i], -acceleration_limit_[i], acceleration_limit_[i]);
        dq_offset_[i] = std::clamp(dq_offset_[i], -velocity_limit_[i], velocity_limit_[i]);
        q_offset_[i] = std::clamp(q_offset_[i], -offset_limit_[i], offset_limit_[i]);

        // Keep saturated offset states from integrating farther into the position limit.
        if (q_offset_[i] >= offset_limit_[i]) {
            if (dq_offset_[i] > 0.0) {
                dq_offset_[i] = 0.0;
            }
            if (dq_offset_[i] >= 0.0 && ddq_offset_[i] > 0.0) {
                ddq_offset_[i] = 0.0;
            }
        }
        if (q_offset_[i] <= -offset_limit_[i]) {
            if (dq_offset_[i] < 0.0) {
                dq_offset_[i] = 0.0;
            }
            if (dq_offset_[i] <= 0.0 && ddq_offset_[i] < 0.0) {
                ddq_offset_[i] = 0.0;
            }
        }
    }
}

}  // namespace dynamics
}  // namespace robot_server_lib
