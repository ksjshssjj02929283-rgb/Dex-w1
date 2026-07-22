#include "joint_admittance_controller.h"

#include <algorithm>
#include <cmath>

namespace robot_server_lib {
namespace dynamics {

namespace {
// 以下默认值在配置缺失或非法时使用, 保证控制器始终有合理参数
constexpr double kDefaultNaturalFrequencyHz = 1.5;
constexpr double kDefaultDampingRatio = 1.2;
constexpr double kDefaultOffsetLimit = 0.05;
constexpr double kDefaultVelocityLimit = 0.4;
constexpr double kDefaultAccelerationLimit = 5.0;

// 严格正值校验, 非法值回退默认
double PositiveOrDefault(double value, double default_value) {
    return (std::isfinite(value) && value > 0.0) ? value : default_value;
}

// 非负值校验, 非法值回退默认
double NonNegativeOrDefault(double value, double default_value) {
    return (std::isfinite(value) && value >= 0.0) ? value : default_value;
}
}  // namespace

JointAdmittanceController::JointAdmittanceController(double dt) : dt_(dt) { Configure(JointAdmittanceConfig{}); }

JointAdmittanceController::JointAdmittanceController(const JointAdmittanceConfig& config, double dt) : dt_(dt) {
    Configure(config);
}

void JointAdmittanceController::Configure(const JointAdmittanceConfig& config) {
    config_ = config;
    for (int i = 0; i < ndof_robot; ++i) {
        const auto& joint = config_.joints[i];
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

bool JointAdmittanceController::UpdateAdmittanceOffset(const VectorDof& torque_ext,
                                                       VectorDof& q_offset_out,
                                                       VectorDof& dq_offset_out) {
    if (dt_ <= 0.0 || !std::isfinite(dt_)) {
        Reset();
        q_offset_out = q_offset_;
        dq_offset_out = dq_offset_;
        return true;
    }

    for (int i = 0; i < ndof_robot; ++i) {
        if (!config_.joints[i].enabled) {
            // 未启用的关节保持零偏移, 不参与导纳计算
            q_offset_[i] = 0.0;
            dq_offset_[i] = 0.0;
            ddq_offset_[i] = 0.0;
            continue;
        }

        // 二阶系统动力学 (M=1): ddq = torque_ext - D·dq - K·q
        const double external_torque = std::isfinite(torque_ext[i]) ? torque_ext[i] : 0.0;
        ddq_offset_[i] = external_torque - damping_[i] * dq_offset_[i] - stiffness_[i] * q_offset_[i];
    }

    // 半隐式欧拉积分: 每步积分后限幅, 防止状态越限发散
    ClampState();
    dq_offset_ += ddq_offset_ * dt_;
    ClampState();
    q_offset_ += dq_offset_ * dt_;
    ClampState();

    q_offset_out = q_offset_;
    dq_offset_out = dq_offset_;
    return true;
}

void JointAdmittanceController::RecomputeDerivedParams() {
    // 由自然频率 fn 与阻尼比 ζ 推导二阶系统的刚度 K 与阻尼 D (M=1)
    // ωn = 2π·fn, K = ωn², D = 2·ζ·ωn
    for (int i = 0; i < ndof_robot; ++i) {
        const double omega_n = 2.0 * M_PI * natural_frequency_hz_[i];
        stiffness_[i] = omega_n * omega_n;
        damping_[i] = 2.0 * damping_ratio_[i] * omega_n;
    }
}

void JointAdmittanceController::ClampState() {
    // 对三级状态依次限幅, 并在位置饱和时清零继续外推的速度/加速度, 避免积分累积越限
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
