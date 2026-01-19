/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include "ArmKinematicsBase.h"
#include "dexe_rp/common/CommonHeader.h"

namespace DexeRP {
namespace Kinematics {

class SRSKinematics : public ArmKinematicsBase {
public:
    SRSKinematics(std::shared_ptr<SRSKineParams> kine_params);

    virtual ~SRSKinematics() { DEXERP_DEBUG("Deconstructing SRSKinematics..."); };

    virtual bool GetFK(const CVecXd &qpos, Mat4d &xpos) const override;

    virtual bool GetAllIK(const Mat4d &xpos, std::vector<CVecXd> &qpos_list) const override {
        DEXERP_WARN("Not Implemented.");
        return false;
    }

    virtual bool GetIK(const Mat4d &xpos, const std::shared_ptr<ArmConfig> &cfg, CVecXd &qpos) const override;

    virtual bool GetNearestIK(const Mat4d &xpos, const CVecXd &qpos_seed, CVecXd &qpos) const override;

    std::pair<bool, double> CalcArmAngle(const CVec7d &qpos) const;

protected:
    SRSKinematics() = delete;

    Mat4d _DhTransform(double a, double alpha, double d, double theta) const;

    CVec3d _CalcRefAngles(const Mat4d &xpos, double elbow_cfg) const;

    double _CalcDistCost(const CVec7d &qpos, const CVec7d &ref_qpos) const;

    double _CalcLimitCost(const CVec7d &qpos, double penalty_bound = 0.95) const;

    double _CalcSingularityCost(const CVec7d &qpos, double penalty_bound = Deg2Rad(5.)) const;

    double _CalcCollisionCost(const CVec7d &qpos) const;

    std::shared_ptr<SRSKineParams> kine_params_;
    CVec3d v_0_bs_, v_6_wt_;
};

}  // namespace Kinematics
}  // namespace DexeRP