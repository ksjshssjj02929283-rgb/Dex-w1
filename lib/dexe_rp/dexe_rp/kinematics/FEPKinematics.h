/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include "dexe_rp/common/CommonHeader.h"
#include "dexe_rp/kinematics/ArmKinematicsBase.h"
#include "dexe_rp/kinematics/FEPKineParams.h"

namespace DexeRP {
namespace Kinematics {

/*
Reference papers:
1. GeoFIK: A Fast and Reliable Geometric Solver for the IK of
    the Franka Arm based on Screw Theory Enabling Multiple Redundancy Parameters
*/
class FEPKinematicsBase : public ArmKinematics<7, FEPConfig> {
public:
    using Params = FEPKineParams;
    using Config = FEPConfig;

    virtual ~FEPKinematicsBase() = default;

    virtual bool SetQposLimits(const JointVec& lower_bound, const JointVec& upper_bound) override;

    virtual Config GetConfig(const JointVec& qpos) const override;
    virtual Mat4d GetFK(const JointVec& qpos) const override;
    virtual StatusType GetAllIK(const Mat4d& pose,
                                const JointVec& qpos_seed,
                                std::vector<JointVec>& qpos_list) const override;
    virtual StatusType GetNearestIK(const Mat4d& pose, const JointVec& qpos_seed, JointVec& qpos) const override;
    // cfg should be get from GetConfig()
    virtual StatusType GetIK(const Mat4d& pose,
                             const Config& cfg,
                             const JointVec& qpos_seed,
                             JointVec& qpos) const override;

    virtual Config GetConfig(const JointVec& qpos, double redundancy_parameter) const;

protected:
    FEPKinematicsBase(Params kine_params);

    // Angle between l_24 and l_46
    double ToVirtualQ4(double q4) const;

    JointVec CalcQposFromJw(const Mat<double, 3, 7>& Jw, double q7) const;

    /// Dirty implementation!
    double CalcCollisionCost(const JointVec& qpos) const;

protected:
    Params params_;
    JointVec model_qpos_lb_, model_qpos_ub_;
    JointVec weights_;

private:
    // precomputed parameters for IK
    double l_24_, l_46_;
    double angle_243_, angle_423_, angle_h46_, angle_h64_;
};

class FEPKinematics : public FEPKinematicsBase {
public:
    FEPKinematics(Params kine_params);

    virtual ~FEPKinematics() = default;

    virtual Config GetConfig(const JointVec& qpos) const override;
    virtual Config GetConfig(const JointVec& qpos, double redundancy_parameter) const override;
    virtual Mat4d GetFK(const JointVec& qpos) const override;
    virtual StatusType GetAllIK(const Mat4d& pose,
                                const JointVec& qpos_seed,
                                std::vector<JointVec>& qpos_list) const override;
    virtual StatusType GetNearestIK(const Mat4d& pose, const JointVec& qpos_seed, JointVec& qpos) const override;
    // cfg should be get from GetConfig()
    virtual StatusType GetIK(const Mat4d& pose,
                             const Config& cfg,
                             const JointVec& qpos_seed,
                             JointVec& qpos) const override;
};

}  // namespace Kinematics
}  // namespace DexeRP
