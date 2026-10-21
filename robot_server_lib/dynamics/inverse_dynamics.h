#pragma once

#include "NanoLog.hpp"
#include "data_structure.h"
#include "pinocchio_api.h"
#include "utils.h"

// 动力学相关的工具函数
namespace robot_server_lib
{
namespace dynamics
{

// 计算逆动力学；可选输出实际叠加的摩擦前馈，未使用辨识参数的关节为零。
Eigen::VectorXd InverseDynamics(const RobotModel& robot_model, const VectorDof& q, const VectorDof& dq,
                                const VectorDof& ddq, VectorDof* friction_feedforward = nullptr);

// 由符号计算生成的逆动力学计算函数
Eigen::VectorXd InvDynamikBySymbol(const RobotModel& robot_model, const VectorDof& q, const VectorDof& dq,
                                   const VectorDof& ddq, const Vector3d& gravity_acc_armbase, MotionGroup motion_group,
                                   Eigen::VectorXd* friction_feedforward = nullptr);

// 由符号计算生成的质量矩阵计算函数
Eigen::MatrixXd GetMassMatrixBySymbol(const RobotModel& robot_model, const Eigen::VectorXd& q_arm, MotionGroup motion_group);

}  // namespace dynamics
}  // namespace robot_server_lib
