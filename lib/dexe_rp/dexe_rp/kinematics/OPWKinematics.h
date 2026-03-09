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

class OPWKinematics : public ArmKinematicsBase {
public:
    OPWKinematics(std::shared_ptr<OPWKineParams> kine_params);

    virtual ~OPWKinematics() { DEXERP_DEBUG("Deconstructing OPWKinematics..."); };
    virtual bool GetFK(const CVecXd &qpos, Mat4d &pose) const override;

    virtual bool GetAllIK(const Mat4d &pose, const CVecXd &qpos_seed, std::vector<CVecXd> &qpos_list) const override;

    virtual bool GetIK(const Mat4d &pose,
                       const std::shared_ptr<ArmConfig> &cfg,
                       const CVecXd &qpos_seed,
                       CVecXd &qpos) const override {
        (void)pose;
        (void)cfg;
        (void)qpos_seed;
        (void)qpos;
        DEXERP_WARN("Not Implemented.");
        return false;
    }

    virtual bool GetNearestIK(const Mat4d &pose, const CVecXd &qpos_seed, CVecXd &qpos) const override;

protected:
    double _CalcDistCost(const CVec7d &qpos, const CVec7d &ref_qpos) const;

    double _CalcLimitCost(const CVec7d &qpos, double penalty_bound = 0.95) const;

protected:
    OPWKinematics() = delete;

    std::shared_ptr<OPWKineParams> kine_params_;
    CVec3d v_0_bs_, v_6_wt_;
};

}  // namespace Kinematics
}  // namespace DexeRP
