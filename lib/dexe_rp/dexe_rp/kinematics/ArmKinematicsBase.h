/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */
#pragma once

#include <memory>
#include <vector>

#include "dexe_rp/common/CommonHeader.h"
#include "dexe_rp/kinematics/Types.h"

namespace DexeRP {
namespace Kinematics {

template <size_t DoF, typename Config>
class ArmKinematics : public std::enable_shared_from_this<ArmKinematics<DoF, Config>> {
public:
    using JointVec = CVec<double, DoF>;

    virtual ~ArmKinematics() = default;

    virtual Config GetConfig(const JointVec& qpos) const = 0;
    virtual Mat4d GetFK(const JointVec& qpos) const = 0;
    virtual StatusType GetAllIK(const Mat4d& pose,
                                const JointVec& qpos_seed,
                                std::vector<JointVec>& qpos_list) const = 0;
    virtual StatusType GetNearestIK(const Mat4d& pose, const JointVec& qpos_seed, JointVec& qpos) const = 0;
    virtual StatusType GetIK(const Mat4d& pose, const Config& cfg, const JointVec& qpos_seed, JointVec& qpos) const = 0;
    /// TODO: GetGeometryJacobian, GetAnalyticalJacobian

    // ------------ common -------------------

    int GetDoF() const { return DoF; }

    void SetTCP(const Mat4d& tcp) { T_e_tcp_ = tcp; }

    Mat4d GetTCP() const { return T_e_tcp_; }

    void SetUserFrame(const Mat4d& user_frame) { T_b_user_ = user_frame; }

    Mat4d GetUserFrame() const { return T_b_user_; }

    virtual bool SetQposLimits(const JointVec& lower_bound, const JointVec& upper_bound) {
        if ((lower_bound.array() > upper_bound.array()).any()) {
            DEXERP_ERROR("Invalid qpos limits: lower_bound {} is greater than upper_bound {}", lower_bound,
                         upper_bound);
            return false;
        }
        qpos_lb_ = lower_bound;
        qpos_ub_ = upper_bound;
        return true;
    }

    std::pair<JointVec, JointVec> GetQposLimits() const { return std::make_pair(qpos_lb_, qpos_ub_); }

    StatusType CalcNearestIKs(const JointVec& start_qpos,
                              const std::vector<Mat4d>& pose_list,
                              std::vector<JointVec>& qpos_list) const;

protected:
    ArmKinematics()
        : T_e_tcp_(Mat4d::Identity()),
          T_b_user_(Mat4d::Identity()),
          qpos_lb_(JointVec::Zero()),
          qpos_ub_(JointVec::Zero()) {}

    inline bool CheckQposLimits(const JointVec& qpos,
                                const JointVec& qpos_lb,
                                const JointVec& qpos_ub,
                                bool verbose = true) const {
        const auto q = qpos.array();
        const auto lb = qpos_lb.array();
        const auto ub = qpos_ub.array();
        bool res = true;
        for (int i = 0; i < q.size(); ++i) {
            if (q[i] < lb[i] || q[i] > ub[i]) {
                res = false;
                if (verbose) {
                    DEXERP_ERROR("Joint {} out of pos limit: value={}, range=[{}, {}]", i + 1, q[i], lb[i], ub[i]);
                }
            }
        }
        return res;
    }

    inline Mat4d ToUserPose(const Mat4d& T_b_e) const { return T_b_user_.inverse() * T_b_e * T_e_tcp_; }

    inline Mat4d ToModelPose(const Mat4d& T_user_tcp) const { return T_b_user_ * T_user_tcp * T_e_tcp_.inverse(); }

    Mat4d T_e_tcp_;
    Mat4d T_b_user_;
    JointVec qpos_lb_, qpos_ub_;  // user defined limits
};

}  // namespace Kinematics
}  // namespace DexeRP
