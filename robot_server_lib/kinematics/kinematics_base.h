// ----------------------------------------------------------------------------
// Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
//
// All rights reserved.
// ----------------------------------------------------------------------------
#pragma once
#include "utils.h"

namespace robot_server_lib
{
namespace kinematics
{

class KinematicsBase
{
   public:
    virtual ~KinematicsBase() = default;

    virtual PoseQuaternion GetFK(const Eigen::VectorXd &q_in, const pinocchio::SE3 &tcp_frame = pinocchio::SE3::Identity()) = 0;

    virtual bool GetIK(const PoseQuaternion &pose, Eigen::VectorXd &q_in_and_out, const pinocchio::SE3 &tcp_frame = pinocchio::SE3::Identity()) = 0;
};
}  // namespace kinematics
}  // namespace robot_server_lib