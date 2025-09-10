#pragma once

#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "typedef.h"

namespace pinocchio
{

// 将数据从joint_command_sequence_by_name顺序映射到URDF顺序
Eigen::VectorXd MapToUrdfOrder(const Model& model, const Eigen::VectorXd& input, const std::vector<std::string>& joint_command_sequence_by_name);

// 将数据从URDF顺序映射到joint_command_sequence_by_name顺序
Eigen::VectorXd MapToParamOrder(const Model& model, const Eigen::VectorXd& input, const std::vector<std::string>& joint_command_sequence_by_name);

Eigen::VectorXd InverseDynamics(const Model& model, Data& data, const Eigen::VectorXd& q, const Eigen::VectorXd& dq, const Eigen::VectorXd& ddq,
                                const std::vector<std::string>& joint_command_sequence_by_name);

Eigen::VectorXd ForwardDynamics(const Model& model, Data& data, const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
                                const Eigen::VectorXd& torque_cmd, const std::vector<std::string>& joint_command_sequence_by_name);

}  // namespace pinocchio