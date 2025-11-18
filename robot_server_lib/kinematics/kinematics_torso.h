#pragma once
// ----------------------------------------------------------------------------
// Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
// All rights reserved.
// ----------------------------------------------------------------------------

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <pinocchio/spatial/se3.hpp>

#include "kinematics_base.h"

namespace robot_server_lib
{
namespace kinematics
{

class TorsoKinematics final : public KinematicsBase
{
   public:
    TorsoKinematics() = default;

    PoseQuaternion GetFK(const Eigen::VectorXd& q_in, const pinocchio::SE3& tcp_frame = pinocchio::SE3::Identity()) override;

    bool GetIK(const PoseQuaternion& pose, Eigen::VectorXd& q_in_and_out, const pinocchio::SE3& tcp_frame = pinocchio::SE3::Identity()) override;

    PoseQuaternion GetTorsoPose(const Eigen::VectorXd& XZRY);

   private:
    double l0_ = 0.292;   // chassis -> ankle
    double l1_ = 0.34;    // ankle -> knee
    double l2_ = 0.34;    // knee -> buttock
    double l3_ = 0.1578;  // buttock -> waist
    // 关节上下限（度 -> 弧度）
    static constexpr double Deg2Rad = M_PI / 180.0;
    Eigen::Vector3d qmin_ = Eigen::Vector3d(-90.0, -155.0, -110.0) * Deg2Rad;
    Eigen::Vector3d qmax_ = Eigen::Vector3d(90.0, 155.0, 90.0) * Deg2Rad;
    // 膝是否向前
    bool knee_front_{true};
};

}  // namespace kinematics
}  // namespace robot_server_lib