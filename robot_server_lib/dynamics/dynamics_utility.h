#pragma once

#include "pinocchio_api.h"

// 动力学相关的工具函数
namespace robot_server_lib
{
namespace dynamics
{

// 计算逆动力学
Eigen::VectorXd InverseDynamics(const pinocchio::Model& model, pinocchio::Data& data, const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
                                const Eigen::VectorXd& ddq, const std::vector<std::string>& joint_command_sequence_by_name);

// 计算正动力学
Eigen::VectorXd ForwardDynamics(const pinocchio::Model& model, pinocchio::Data& data, const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
                                const Eigen::VectorXd& torque_cmd, const std::vector<std::string>& joint_command_sequence_by_name);

// 将数据从joint_command_sequence_by_name顺序映射到URDF顺序
Eigen::VectorXd MapToUrdfOrder(const pinocchio::Model& model, const Eigen::VectorXd& input,
                               const std::vector<std::string>& joint_command_sequence_by_name);

// 将数据从URDF顺序映射到joint_command_sequence_by_name顺序
Eigen::VectorXd MapToParamOrder(const pinocchio::Model& model, const Eigen::VectorXd& input,
                                const std::vector<std::string>& joint_command_sequence_by_name);

}  // namespace dynamics
}  // namespace robot_server_lib