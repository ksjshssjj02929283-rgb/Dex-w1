/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */
#pragma once

#include <memory>

#include "dexe_rp/common/CommonHeader.h"
#include "dexe_rp/kinematics/Types.h"

namespace DexeRP {
namespace Kinematics {

/// TODO: Rxd = Joint = (q1, q2, q3, q4 ...)
/// TODO: SE3d = Pose = (xyz,wxyz)
/// TODO: R3xSO3Tangent

class ArmKinematicsBase : public std::enable_shared_from_this<ArmKinematicsBase> {
public:
    virtual ~ArmKinematicsBase() { DEXERP_DEBUG("Deconstructing KinematicsBase..."); };

    virtual bool GetFK(const CVecXd &qpos, Mat4d &pose) const = 0;

    virtual bool GetAllIK(const Mat4d &pose, const CVecXd &qpos_seed, std::vector<CVecXd> &qpos_list) const = 0;

    virtual bool GetIK(const Mat4d &pose,
                       const std::shared_ptr<ArmConfig> &cfg,
                       const CVecXd &qpos_seed,
                       CVecXd &qpos) const = 0;

    virtual bool GetNearestIK(const Mat4d &pose, const CVecXd &qpos_seed, CVecXd &qpos) const = 0;

    int GetDOF() const { return dof_; }

    bool SetTCP(const Mat4d &tcp) {
        tcp_ = tcp;
        return true;
    }

    Mat4d GetTCP() const { return tcp_; }

    bool SetUserFrame(const Mat4d &user_frame) {
        user_frame_ = user_frame;
        return true;
    }

    Mat4d GetUserFrame() const { return user_frame_; }

    bool SetHomeJoints(const CVecXd &home_qpos) {
        if (!_CheckDof(home_qpos)) return false;
        home_qpos_ = home_qpos;
        return true;
    };

    CVecXd GetHomeJoints() const { return home_qpos_; };

    bool SetIkNearestWeight(const CVecXd &weight) {
        if (!_CheckDof(weight)) return false;
        ik_nearest_weight_ = weight;
        return true;
    }

    CVecXd GetIkNearestWeight() const { return ik_nearest_weight_; }

    bool SetQposLimits(const CVecXd &lower_bound, const CVecXd &upper_bound) {
        if (!_CheckDof(lower_bound) || !_CheckDof(upper_bound)) {
            return false;
        }

        if ((lower_bound.array() > upper_bound.array()).any()) {
            return false;
        }

        qpos_lb_ = lower_bound;
        qpos_ub_ = upper_bound;
        return true;
    }

    std::pair<CVecXd, CVecXd> GetQposLimits() const { return std::make_pair(qpos_lb_, qpos_ub_); }

    // bool SetUserQposLimits(const CVecXd &lower_bound, const CVecXd &upper_bound);

    // bool GetUserQposLimits(CVecXd &lower_bound, CVecXd &upper_bound) const {
    //     lower_bound = usr_lb_;
    //     upper_bound = usr_ub_;
    //     return true;
    // }

    /// TODO: jacobian

protected:
    ArmKinematicsBase() = default;

    inline bool _CheckDof(const CVecXd &qpos, bool verbose = true) const {
        if (qpos.size() != dof_) {
            if (verbose) {
                DEXERP_ERROR("Dimension qpos({}) != dof({})", qpos.size(), dof_);
            }
            return false;
        }
        return true;
    }

    bool _CheckQposLimits(const CVecXd &qpos, bool verbose = true) const {
        if (!_CheckDof(qpos)) {
            return false;
        }
        if (((qpos.array() >= qpos_lb_.array()) && (qpos.array() <= qpos_ub_.array())).all()) {
            return true;
        }
        if (verbose) {
            DEXERP_ERROR("Out of qpos limits: {}", CVec7d(qpos));
        } else {
            DEXERP_DEBUG("Out of qpos limits: {}", CVec7d(qpos));
        }
        return false;
    }

    // bool initalize_ = false;
    int dof_;
    Mat4d tcp_;         // T_e_tcp
    Mat4d user_frame_;  // T_b_user
    CVecXd home_qpos_;
    CVecXd ik_nearest_weight_;
    CVecXd qpos_lb_, qpos_ub_;  // actual soft limits
    // CVecXd usr_lb_, usr_ub_; // user defined limits within [model_lb,model_ub]
    // CVecXd model_lb_, model_ub_; // read from urdf
};

}  // namespace Kinematics
}  // namespace DexeRP
