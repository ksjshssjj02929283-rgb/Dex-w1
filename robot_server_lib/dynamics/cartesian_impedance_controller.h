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
    void SetDamping(const Vector6d& D);
    void SetIntegralGain(const Vector6d& Ki);

private:
    // 模型参数
    RobotModel robot_model_;
    pinocchio::Model pinocchio_model_;
    MotionGroup motion_group_;
    int ndof_;

    // 运行时变量
    Vector6d cart_err_ = Vector6d::Zero();
    Vector6d cart_err_intg_ = Vector6d::Zero();
    Eigen::VectorXd tau_command_;  // 上一周期力矩指令

    // 控制参数
    Vector3d trans_clip_max_ = {0.1, 0.1, 0.1};                 // 位置限幅, 单位m
    Vector3d trans_clip_min_ = {-0.1, -0.1, -0.1};              // 位置限幅, 单位m
    Vector3d rot_clip_max_ = {0.05, 0.05, 0.05};                // 旋转限幅, 单位rad
    Vector3d rot_clip_min_ = {-0.05, -0.05, -0.05};             // 旋转限幅, 单位rad
    Matrix6d cart_stiff_ = Vector6d{100, 100, 100, 0, 0, 0}.asDiagonal();     // 刚度
    Matrix6d cart_damp_ = Vector6d{20, 20, 20, 0, 0, 0}.asDiagonal();       // 阻尼
    Matrix6d Ki_ = Vector6d{1, 1, 1, 0, 0, 0}.asDiagonal();                      // 积分增益
    Eigen::VectorXd q_nullspace_;                             // 零空间目标位置
    double nullspace_stiff_ = 5;                       // 零空间刚度
    double delta_tau_max_ = 1;                              // 最大力矩变化率限制

};
} // namespace dynamics
} // namespace robot_server_lib