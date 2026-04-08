#pragma once

#include <array>

#include "utils.h"

namespace robot_server_lib {
namespace dynamics {

class JointImpedanceController {
public:
    struct Output {
        VectorDof torque_joint = VectorDof::Zero();
        VectorDof q_desired_base = VectorDof::Zero();
        VectorDof q_desired_compliant = VectorDof::Zero();
        VectorDof joint_compliance_offset = VectorDof::Zero();
        std::array<int, 2> force_trigger_state{0, 0};
    };

    JointImpedanceController() = default;
    explicit JointImpedanceController(const RobotModel &robot_model);

    Output UpdateTorqueCommand(const VectorDof &feedback_q,
                               const VectorDof &feedback_dq,
                               const VectorDof &cmd_q,
                               const VectorDof &cmd_dq,
                               const Vector12d &sensor_wrench,
                               const VectorDof &joint_stiffness,
                               const VectorDof &joint_damping,
                               double joint_integral_ratio,
                               bool enable_compliance = true);

    VectorDof ComputeHoldTorque(const VectorDof &feedback_q,
                                const VectorDof &feedback_dq,
                                const VectorDof &hold_q,
                                const VectorDof &joint_stiffness,
                                const VectorDof &joint_damping);

    VectorDof ComputeDampingTorque(const VectorDof &feedback_dq, const VectorDof &joint_damping) const;

    void Reset(const VectorDof &reference_q = VectorDof::Zero());

    void SetForceNormThreshold(const std::array<double, 2> &threshold);
    void SetComplianceGain(const std::array<double, 2> &gain);
    void SetComplianceReturnRatio(double return_ratio);
    void SetComplianceOffsetMax(double offset_max);

private:
    VectorDof ApplyIntegralDeadzone(const VectorDof &q_error) const;
    void UpdateComplianceOffset(const VectorDof &feedback_q, const Vector12d &sensor_wrench, bool enable_compliance);
    void UpdateArmComplianceOffset(const Eigen::VectorXd &q_arm,
                                   const Vector6d &wrench_arm,
                                   MotionGroup motion_group,
                                   int arm_index,
                                   bool enable_compliance);
    void DecayArmComplianceOffset(MotionGroup motion_group);

    RobotModel robot_model_;
    VectorDof integral_error_ = VectorDof::Zero();
    VectorDof joint_compliance_offset_ = VectorDof::Zero();
    std::array<int, 2> force_trigger_state_{0, 0};

    double error_integral_max_ = 0.1;
    double integral_deadzone_rad_ = 0.005 / 180.0 * M_PI;

    std::array<double, 2> force_norm_threshold_{8.0, 8.0};
    std::array<double, 2> compliance_gain_{0.0020, 0.0020};
    double compliance_return_ratio_ = 0.05;
    double compliance_offset_max_ = 8.0 / 180.0 * M_PI;
};

}  // namespace dynamics
}  // namespace robot_server_lib
