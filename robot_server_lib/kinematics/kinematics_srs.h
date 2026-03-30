#pragma once

#include "dexe_rp/common/CommonHeader.h"
#include "dexe_rp/kinematics/KinematicsHeader.h"
#include "kinematics_base.h"

namespace robot_server_lib {
namespace kinematics {

class SRSKinematics : public KinematicsBase {
public:
    SRSKinematics(bool is_left, bool is_anthropmorphic);
    ~SRSKinematics() override;
    PoseQuaternion GetFK(const Eigen::VectorXd &q_in,
                         const pinocchio::SE3 &tcp_frame = pinocchio::SE3::Identity()) override;
    bool GetIK(const PoseQuaternion &pose_in,
               Eigen::VectorXd &q_in_and_out,
               const pinocchio::SE3 &tcp_frame = pinocchio::SE3::Identity()) override;

    std::pair<Eigen::VectorXd, Eigen::VectorXd> GetQposLimits() const override;

private:
    std::shared_ptr<DexeRP::Kinematics::SRSKinematics> w1_kine_;
};

}  // namespace kinematics
}  // namespace robot_server_lib
