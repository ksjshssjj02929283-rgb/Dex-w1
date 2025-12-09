#pragma once

#include "pinocchio_api.h"
#include "NanoLog.hpp"
#include "data_structure.h"
#include "utils.h"

// 动力学相关的工具函数
namespace robot_server_lib
{
namespace dynamics
{

// 计算逆动力学
Eigen::VectorXd InverseDynamics(const RobotModel& robot_model, pinocchio::Data& data, const VectorDof& q, const VectorDof& dq,
                                const VectorDof& ddq);

// 计算惯性力项的逆动力学
Eigen::VectorXd InvDynamicsInertiaTerm(const RobotModel& robot_model, pinocchio::Data& data, const VectorDof& q, const VectorDof& dq, const VectorDof& ddq, MotionGroup motion_group);

// 计算科氏力项的逆动力学
Eigen::VectorXd InvDynamicsCoriolisTerm(const RobotModel& robot_model, pinocchio::Data& data, const VectorDof& q, const VectorDof& dq, MotionGroup motion_group);

// 计算重力项的逆动力学
Eigen::VectorXd InvDynamicsGravityTerm(const RobotModel& robot_model, pinocchio::Data& data, const VectorDof& q, MotionGroup motion_group);

// 计算正动力学
Eigen::VectorXd ForwardDynamics(const pinocchio::Model& model, pinocchio::Data& data, const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
                                const Eigen::VectorXd& torque_cmd, const std::vector<std::string>& joint_command_sequence_by_name);

// 将数据从joint_command_sequence_by_name顺序映射到URDF顺序
Eigen::VectorXd MapToUrdfOrder(const pinocchio::Model& model, const Eigen::VectorXd& input,
                               const std::vector<std::string>& joint_command_sequence_by_name);

// 将数据从URDF顺序映射到joint_command_sequence_by_name顺序
Eigen::VectorXd MapToParamOrder(const pinocchio::Model& model, const Eigen::VectorXd& input,
                                const std::vector<std::string>& joint_command_sequence_by_name);

Eigen::MatrixXd InvDynamicRegressorPrimary(const Vector3d& gravity_acc, const Eigen::VectorXd& q, const Eigen::VectorXd& dq, const Eigen::VectorXd& ddq);

Eigen::MatrixXd InvDynamicRegressorSecondary(const Vector3d& gravity_acc, const Eigen::VectorXd& q, const Eigen::VectorXd& dq, const Eigen::VectorXd& ddq);
}  // namespace dynamics
}  // namespace robot_server_lib