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
#include "dexe_rp/kinematics/SRSKineParams.h"

namespace DexeRP {
namespace Kinematics {

// TODO: Should support basic type of SRS, not only for DexforceW1
class SRSKinematics : public ArmKinematicsBase {
public:
    SRSKinematics(std::shared_ptr<SRSKineParams> kine_params);

    virtual ~SRSKinematics() { DEXERP_DEBUG("Deconstructing SRSKinematics..."); };

    virtual bool GetFK(const CVecXd &qpos, Mat4d &pose) const override;

    virtual bool GetAllIK(const Mat4d &pose, const CVecXd &qpos_seed, std::vector<CVecXd> &qpos_list) const override {
        (void)pose;
        (void)qpos_seed;
        (void)qpos_list;
        DEXERP_WARN("Not Implemented.");
        return false;
    }

    virtual bool GetIK(const Mat4d &pose,
                       const std::shared_ptr<ArmConfig> &cfg,
                       const CVecXd &qpos_seed,
                       CVecXd &qpos) const override;

    virtual bool GetNearestIK(const Mat4d &pose, const CVecXd &qpos_seed, CVecXd &qpos) const override;

    std::pair<bool, double> CalcArmAngle(const CVec7d &qpos) const;

protected:
    SRSKinematics() = delete;

    CVec3d _CalcRefAngles(const Mat4d &pose, double elbow_cfg) const;

    double _CalcDistCost(const CVec7d &qpos, const CVec7d &ref_qpos) const;

    double _CalcLimitCost(const CVec7d &qpos, double penalty_bound = 0.95) const;

    double _CalcSingularityCost(const CVec7d &qpos, double penalty_bound = Deg2Rad(5.)) const;

    double _CalcCollisionCost(const CVec7d &qpos) const;

    std::shared_ptr<SRSKineParams> kine_params_;
    CVec3d v_0_bs_, v_6_wt_;
};

}  // namespace Kinematics
}  // namespace DexeRP
