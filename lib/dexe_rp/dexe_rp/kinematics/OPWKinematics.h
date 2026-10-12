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
#include "dexe_rp/kinematics/OPWKineParams.h"

namespace DexeRP {
namespace Kinematics {

/*
Reference papers:
1. An Analytical Solution of the Inverse Kinematics Problem of
    Industrial Serial Manipulators with an Ortho-parallel Basis
    and a Spherical Wrist" as reference model
*/
class OPWKinematicsBase : public ArmKinematics<6, OPWConfig> {
public:
    using Params = OPWKineParams;
    using Config = OPWConfig;

    virtual ~OPWKinematicsBase() = default;

    virtual bool SetQposLimits(const JointVec& lower_bound, const JointVec& upper_bound) override;

    virtual Config GetConfig(const JointVec& qpos) const override;
    virtual Mat4d GetFK(const JointVec& qpos) const override;
    virtual StatusType GetAllIK(const Mat4d& pose,
                                const JointVec& qpos_seed,
                                std::vector<JointVec>& qpos_list) const override;
    virtual StatusType GetNearestIK(const Mat4d& pose, const JointVec& qpos_seed, JointVec& qpos) const override;
    virtual StatusType GetIK(const Mat4d& pose,
                             const Config& cfg,
                             const JointVec& qpos_seed,
                             JointVec& qpos) const override;

protected:
    OPWKinematicsBase(Params kine_params);

protected:
    Params params_;
    JointVec model_qpos_lb_, model_qpos_ub_;
    JointVec weights_;
};

class OPWKinematics : public OPWKinematicsBase {
public:
    OPWKinematics(Params kine_params);

    virtual ~OPWKinematics() = default;

    virtual Config GetConfig(const JointVec& qpos) const override;
    virtual Mat4d GetFK(const JointVec& qpos) const override;
    virtual StatusType GetAllIK(const Mat4d& pose,
                                const JointVec& qpos_seed,
                                std::vector<JointVec>& qpos_list) const override;
    virtual StatusType GetNearestIK(const Mat4d& pose, const JointVec& qpos_seed, JointVec& qpos) const override;
    virtual StatusType GetIK(const Mat4d& pose,
                             const Config& cfg,
                             const JointVec& qpos_seed,
                             JointVec& qpos) const override;
};

}  // namespace Kinematics
}  // namespace DexeRP
