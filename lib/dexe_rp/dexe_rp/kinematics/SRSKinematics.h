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

/*
Reference papers:
1. Analytical Inverse Kinematic Computation for 7-DOF Redundant Manipulators
    With Joint Limits and Its Application to Redundancy Resolution

2. Position-based kinematics for 7-DoF serial manipulators with
    global configuration control, joint limit and singularity avoidance
*/
class SRSKinematicsBase : public ArmKinematics<7, SRSConfig> {
public:
    using Params = SRSKineParams;
    using Config = SRSConfig;

    virtual ~SRSKinematicsBase() = default;

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
    virtual double GetArmAngle(const JointVec& qpos) const;
    // cfg should be get from GetConfig()
    virtual Intervals GetFeasibleArmAngles(const Mat4d& pose, const Config& cfg) const;
    // Intervals GetFeasibleArmAngles(const Mat4d& pose, const Config& cfg) const;

protected:
    SRSKinematicsBase(Params kine_params);

    struct _IKProcessVariables {
        double q4;
        Mat3d R_0_07;
        Mat3d As, Bs, Cs, Aw, Bw, Cw;
    };

    std::pair<StatusType, _IKProcessVariables> PreprocessIK(const Mat4d& T_user_tcp, const Config& cfg) const;

    std::pair<StatusType, double> CalcQ4(double d_26, double elbow_cfg) const;

    CVec3d CalcRefQ123(const Mat4d& T_ob_oe, double elbow_cfg) const;

    CVec3d CalcQ123(const Mat3d& R_03, double shoulder_cfg, double q1_seed) const;

    CVec3d CalcQ567(const Mat3d& R_47, double wrist_cfg, double q5_seed) const;

    double CalcCosJoint(double a, double b, double c, double arm_angle, double cfg) const;

    Intervals CalcCosFeasibleArmAngle(double a, double b, double c, double lb, double ub, double cfg) const;

    double CalcTanJoint(
        double an, double bn, double cn, double ad, double bd, double cd, double arm_angle, double cfg) const;

    Intervals CalcTanFeasibleArmAngle(
        double an, double bn, double cn, double ad, double bd, double cd, double lb, double ub, double cfg) const;

    Intervals CalcFeasibleArmAngles(const Mat3d& As,
                                    const Mat3d& Bs,
                                    const Mat3d& Cs,
                                    const Mat3d& Aw,
                                    const Mat3d& Bw,
                                    const Mat3d& Cw,
                                    const Config& cfg) const;
    /// Dirty implementation!
    double CalcCollisionCost(const JointVec& qpos) const;

protected:
    Params params_;
    JointVec model_qpos_lb_, model_qpos_ub_;
    JointVec weights_;

private:
    // precomputed parameters for IK
    CVec3d v_0_02_, v_6_67_;
};

class SRSKinematics : public SRSKinematicsBase {
public:
    SRSKinematics(Params kine_params);

    virtual ~SRSKinematics() = default;

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

    virtual double GetArmAngle(const JointVec& qpos) const override;
    // cfg should be get from GetConfig()
    virtual Intervals GetFeasibleArmAngles(const Mat4d& pose, const Config& cfg) const override;
};

}  // namespace Kinematics
}  // namespace DexeRP
