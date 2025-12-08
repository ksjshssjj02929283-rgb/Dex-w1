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
// 计算正动力学接口
Eigen::Matrix4d GetFK(const Model& model, const Eigen::VectorXd &q);

// 计算雅可比矩阵接口
Eigen::MatrixXd GetJacobiMatrix(const Model& model, const Eigen::VectorXd& q);

SE3 GetFkByFrameId(const Model& model, Data& data, const Eigen::VectorXd& q_urdf_in, int frame_id);

// 计算矩阵伪逆接口
void PseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped = true);

}  // namespace pinocchio