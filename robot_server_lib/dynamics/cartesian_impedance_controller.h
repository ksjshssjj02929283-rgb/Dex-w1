#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include "NanoLog.hpp"
#include "inverse_dynamics.h"

namespace robot_server_lib
{
namespace dynamics
{

class CartesianImpedanceController
{
public:
    CartesianImpedanceController() = default;

    // 初始化参数
    CartesianImpedanceController(const RobotModel &robot_model, MotionGroup motion_group);
    bool UpdateTorqueCommand(const Eigen::VectorXd &feedback_q, const Eigen::VectorXd &feedback_dq, const Eigen::VectorXd &cmd_q, Eigen::VectorXd &tau_command);

    Eigen::VectorXd SaturateTorqueRate(const Eigen::VectorXd& tau_target, const Eigen::VectorXd& tau);

    void SetStiffness(const Vector6d& K);

private:
    RobotModel robot_model_;
    pinocchio::Model pinocchio_model_;
    MotionGroup motion_group_;
    int ndof_;

    // 运行时变量
    Vector6d pose_err_intg_ = Vector6d::Zero();
    Eigen::VectorXd tau_command_;  // 上一周期力矩指令

    // 控制参数
    Vector3d trans_clip_max_ = {0.2, 0.2, 0.2};                 // 位置限幅, 单位m
    Vector3d rot_clip_max_ = {0.4, 0.4, 0.4};                // 旋转限幅, 单位rad
    Vector3d trans_intg_clip_max_ = trans_clip_max_ * 2;
    Vector3d rot_intg_clip_max_ = trans_clip_max_ * 2;


    Vector6d stiffness_vector_ = {150, 150, 150, 1, 1, 1};     // 刚度
    Vector6d damping_vector_;                // 阻尼根据刚度自动适配

    Matrix6d Ki_ = Vector6d{3, 3, 3, 0.3, 0.3, 0.3}.asDiagonal();                      // 积分增益

    Eigen::VectorXd q_nullspace_;                             // 零空间目标位置
    double nullspace_stiff_ = 5;                            // 零空间刚度
    double nullspace_damp_ = 0.0;                                 // 零空间阻尼

    double delta_tau_max_ = 0.5;                              // 最大力矩变化率限制
    double tau_command_max_ = 25.0;                        // 最大力矩限制
};
} // namespace dynamics
} // namespace robot_server_lib