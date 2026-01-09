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

// 计算逆动力学
Eigen::VectorXd InverseDynamics(const RobotModel& robot_model, const VectorDof& q, const VectorDof& dq, const VectorDof& ddq);

// 计算正动力学
Eigen::VectorXd ForwardDynamics(const pinocchio::Model& model, const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
                                const Eigen::VectorXd& torque_cmd, const std::vector<std::string>& joint_command_sequence_by_name);

// 将数据从joint_command_sequence_by_name顺序映射到URDF顺序
Eigen::VectorXd MapToUrdfOrder(const pinocchio::Model& model, const Eigen::VectorXd& input,
                               const std::vector<std::string>& joint_command_sequence_by_name);

// 将数据从URDF顺序映射到joint_command_sequence_by_name顺序
Eigen::VectorXd MapToParamOrder(const pinocchio::Model& model, const Eigen::VectorXd& input,
                                const std::vector<std::string>& joint_command_sequence_by_name);

// 由符号计算生成的逆动力学计算函数
Eigen::VectorXd InvDynamikBySymbol(const RobotModel& robot_model, const VectorDof& q, const VectorDof& dq,
                                   const VectorDof& ddq, const Vector3d& gravity_acc_armbase, MotionGroup motion_group);

// 由符号计算生成的质量矩阵计算函数
Eigen::MatrixXd GetMassMatrixBySymbol(const RobotModel& robot_model, const VectorDof& q, MotionGroup motion_group);

}  // namespace dynamics
}  // namespace robot_server_lib