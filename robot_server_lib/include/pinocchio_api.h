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
// 计算雅可比矩阵接口
Eigen::MatrixXd GetJacobiMatrix(const Model& model, const Eigen::VectorXd& q);

pinocchio::SE3 GetFkByFrameName(const Model& model, Data& data, const Eigen::VectorXd &q_in, std::string link_name);

}  // namespace pinocchio