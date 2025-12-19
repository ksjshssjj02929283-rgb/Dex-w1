#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include "NanoLog.hpp"
#include "dynamics_base.h"

namespace robot_server_lib
{
namespace dynamics
{

class CartesianImpedanceController
{
public:
    CartesianImpedanceController() = default;

    // 初始化参数
    CartesianImpedanceController(const RobotModel &robot_model, MotionGroup motion_group, double control_dt = 0.001)
        : robot_model_(robot_model), motion_group_(motion_group), control_dt_(control_dt)
    {
        // 根据motion_group设置ndof, 并获取对应臂的model
        robot_model.params_ptr->joints_primary.size();
        if (motion_group_ == MotionGroup::Primary)
        {
            pinocchio_model_ = *robot_model.primary_arm_model_ptr;
            ndof_ = pinocchio_model_.nv;
        }
        else if (motion_group_ == MotionGroup::Secondary)
        {
            pinocchio_model_ = *robot_model.secondary_arm_model_ptr;
            ndof_ = pinocchio_model_.nv;
        }
        
    }
    bool UpdateCommand(const JointRawMeasurement &raw_measurement, const PlannerManagerCommand &input_cmd, Eigen::VectorXd &tau_command)
    {
        VectorDof feedback_q = raw_measurement.position;
        VectorDof feedback_dq = raw_measurement.velocity;
        VectorDof cmd_q = input_cmd.cmd.position;
        VectorDof cmd_dq = input_cmd.cmd.velocity;
        // pinocchio计算当前末端位姿, 期望末端位姿和当前的雅可比矩阵
        auto feedback_pose = pinocchio::GetFK(pinocchio_model_, feedback_q);
        auto cmd_pose = pinocchio::GetFK(pinocchio_model_, cmd_q);
        Eigen::MatrixXd J = GetJacobiMatrix(pinocchio_model_, feedback_q);

        // 提取雅可比矩阵
        Eigen::Matrix<double, 6, ndof> jacobian = dynamics::ComputeJacobian(robot_model_, robot_data_, raw_measurement.position);
        Eigen::Matrix<double, ndof, 6> jacobian_transpose = jacobian.transpose();

        // 计算位置误差和积分误差
        // 计算位置误差
        cartesian_error_ = desired_pose - current_pose;

        // 积分误差更新
        cartesian_error_integral_ += cartesian_error_ * control_dt_;

        // 积分限幅
        for (int i = 0; i < 3; ++i)
        {
            cartesian_error_integral_[i] = std::clamp(cartesian_error_integral_[i], translation_min_integral_[i], translation_max_integral_[i]);
            cartesian_error_integral_[i + 3] = std::clamp(cartesian_error_integral_[i + 3], rotation_min_integral_[i], rotation_max_integral_[i]);
        }

        // 计算控制力/力矩
        Vector6d control_wrench = cartesian_stiffness_.asDiagonal() * cartesian_error_
                                 + cartesian_damping_.asDiagonal() * (cartesian_error_ / control_dt_)
                                 + Ki_.asDiagonal() * cartesian_error_integral_;

        // 转换为关节力矩命令
        tau_command = jacobian_transpose * control_wrench;

        return true;
    }
    void SetStiffness(const Vector6d& K)
    {
        cartesian_stiffness_ = K;
        LOG_INFO << "CartesianImpedanceController: Set stiffness to [" << cartesian_stiffness_.transpose() << "]";
    }
    void SetDamping(const Vector6d& D)
    {
        cartesian_damping_ = D;
        LOG_INFO << "CartesianImpedanceController: Set damping to [" << cartesian_damping_.transpose() << "]";
    }
    void SetIntegralGain(const Vector6d& Ki)
    {
        Ki_ = Ki;
        LOG_INFO << "CartesianImpedanceController: Set integral gain to [" << Ki_.transpose() << "]";
    }
private:
    // 模型参数
    RobotModel robot_model_;
    pinocchio::Model pinocchio_model_;
    MotionGroup motion_group_;
    int ndof_;
    // 运行时变量
    Vector6d cartesian_error_ = Vector6d::Zero();
    Vector6d cartesian_error_integral_ = Vector6d::Zero();

    // 控制参数
    double control_dt_ = 0.001;  
                                 // 控制周期 (s)
    Vector3d translation_max_integral_ = {0.05, 0.05, 0.05};  // 最大积分限幅 (m)
    Vector3d translation_min_integral_ = {0.05, 0.05, 0.05};  // 最大积分限幅 (m)
    Vector3d rotation_max_integral_ = {0.05, 0.05, 0.05};     // 最大积分限幅 (m)
    Vector3d rotation_min_integral_ = {0.05, 0.05, 0.05};     // 最大积分限幅 (m)
    Vector6d cartesian_stiffness_ = Vector6d::Zero();         // 刚度
    Vector6d cartesian_damping_ = Vector6d::Zero();           // 阻尼
    Vector6d Ki_ = Vector6d::Zero();                          // 积分增益

    Eigen::Matrix<double, ndof, 1> q_nullspace_;              // 零空间目标位置
    double nullspace_stiffness_ = 20.0;                       // 零空间刚度
    double delta_tau_max_ = 1.0;                              // 最大力矩变化率限制

};
} // namespace dynamics
} // namespace robot_server_lib