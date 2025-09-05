#pragma once

#include "kinematics_base.h"

namespace robot_server_lib
{
namespace kinematics
{

class NumericalKinematics : public KinematicsBase
{
public:
    NumericalKinematics(const pinocchio::Model &model);
    ~NumericalKinematics() override;

    PoseQuaternion GetFK(const Eigen::VectorXd &q_in, const pinocchio::SE3 &tcp_frame = pinocchio::SE3::Identity()) override;
    bool GetIK(const PoseQuaternion &pose_in, Eigen::VectorXd &q_in_and_out, const pinocchio::SE3 &tcp_frame = pinocchio::SE3::Identity()) override;

private:
    pinocchio::Model model_;
    pinocchio::Data data_;
    int ndof_;  // Number of degrees of freedom
    double eps_ = 1e-5;
    int it_max_ = 1000;
    double dt_ = 1e-1;
    double damp_ = 1e-6;
    double threshold_ = 0.0174532;  // 1 deg
};
}  // namespace kinematics
}  // namespace robot_server_lib