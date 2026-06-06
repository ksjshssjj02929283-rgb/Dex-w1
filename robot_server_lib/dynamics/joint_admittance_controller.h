#pragma once

#include "data_structure.h"

namespace robot_server_lib {
namespace dynamics {

class JointAdmittanceController {
public:
    JointAdmittanceController();
    explicit JointAdmittanceController(const JointAdmittanceConfig& config);

    void Configure(const JointAdmittanceConfig& config);
    void Reset();
    void DecayToZero(double dt);

    bool Update(const VectorDof& torque_ext, double dt, VectorDof& q_offset_out, VectorDof& dq_offset_out);

    const VectorDof& q_offset() const { return q_offset_; }
    const VectorDof& dq_offset() const { return dq_offset_; }
    const VectorDof& ddq_offset() const { return ddq_offset_; }
    const VectorDof& stiffness() const { return stiffness_; }
    const VectorDof& damping() const { return damping_; }

private:
    void RecomputeDerivedParams();
    void ClampState();

    JointAdmittanceConfig config_{};
    VectorDof torque_gain_{VectorDof::Ones()};
    VectorDof virtual_inertia_{VectorDof::Ones()};
    VectorDof natural_frequency_hz_{VectorDof::Constant(1.5)};
    VectorDof damping_ratio_{VectorDof::Constant(1.2)};
    VectorDof offset_limit_{VectorDof::Constant(0.05)};
    VectorDof velocity_limit_{VectorDof::Constant(0.4)};
    VectorDof acceleration_limit_{VectorDof::Constant(5.0)};
    VectorDof stiffness_{VectorDof::Zero()};
    VectorDof damping_{VectorDof::Zero()};
    VectorDof q_offset_{VectorDof::Zero()};
    VectorDof dq_offset_{VectorDof::Zero()};
    VectorDof ddq_offset_{VectorDof::Zero()};
};

}  // namespace dynamics
}  // namespace robot_server_lib
