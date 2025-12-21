#include "cartesian_impedance_controller.h"
#include <iostream>

namespace robot_server_lib {
namespace dynamics {

CartesianImpedanceController::CartesianImpedanceController(const RobotModel &robot_model, MotionGroup motion_group, double control_dt)
	: robot_model_(robot_model), motion_group_(motion_group), control_dt_(control_dt)
{
	this->robot_model_.params_ptr->joints_primary.size();
	if (this->motion_group_ == MotionGroup::Primary)
	{
		this->pinocchio_model_ = *this->robot_model_.primary_arm_model_ptr;
		this->ndof_ = this->pinocchio_model_.nv;
	}
	else if (this->motion_group_ == MotionGroup::Secondary)
	{
		this->pinocchio_model_ = *this->robot_model_.secondary_arm_model_ptr;
		this->ndof_ = this->pinocchio_model_.nv;
	}
	this->q_nullspace_.setZero(this->ndof_);
	this->tau_command_.setZero(this->ndof_);
}

bool CartesianImpedanceController::UpdateTorqueCommand(const Eigen::VectorXd &feedback_q, const Eigen::VectorXd &feedback_dq, const Eigen::VectorXd &cmd_q, Eigen::VectorXd &tau_command)
{
	Eigen::Affine3d feedback_transform(pinocchio::GetFK(this->pinocchio_model_, feedback_q));
	Eigen::Vector3d feedback_trans(feedback_transform.translation());
	Eigen::Quaterniond feedback_quat(feedback_transform.linear());

	Eigen::Affine3d cmd_transform(pinocchio::GetFK(this->pinocchio_model_, cmd_q));
	Eigen::Vector3d cmd_trans(cmd_transform.translation());
	Eigen::Quaterniond cmd_quat(cmd_transform.linear());
	Eigen::MatrixXd jacobian = pinocchio::GetJacobiMatrix(this->pinocchio_model_, feedback_q);

	// std::cout << "feedback_pose trans: " << feedback_trans.transpose() << std::endl;
	// std::cout << "command_pose trans: " << cmd_trans.transpose() << std::endl;
	// std::cout << "feedback_pose rot: " << feedback_quat.coeffs().transpose() << std::endl;
	// std::cout << "command_pose rot: " << cmd_quat.coeffs().transpose() << std::endl;
	// std::cout << "jacobian: \n" << jacobian << std::endl;

	this->cartesian_error_.head<3>() = (cmd_trans - feedback_trans).cwiseMax(this->trans_clip_min_).cwiseMin(this->trans_clip_max_);

	if (cmd_quat.coeffs().dot(feedback_quat.coeffs()) < 0.0)
	{
		feedback_quat.coeffs() << -feedback_quat.coeffs();
	}
	Eigen::Quaterniond error_quaternion(feedback_quat.inverse() * cmd_quat);
	this->cartesian_error_.tail<3>() << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
	this->cartesian_error_.tail<3>() = -feedback_transform.linear() * this->cartesian_error_.tail<3>();
	this->cartesian_error_.tail<3>() = this->cartesian_error_.tail<3>().cwiseMax(this->rot_clip_min_).cwiseMin(this->rot_clip_max_);

	// std::cout << "cartesian_error_: " << cartesian_error_.transpose() << std::endl;

	this->cartesian_error_integral_.head(3) << (this->cartesian_error_integral_.head(3) + this->cartesian_error_.head(3)).cwiseMax(-0.1).cwiseMin(0.1);
	this->cartesian_error_integral_.tail(3) << (this->cartesian_error_integral_.tail(3) + this->cartesian_error_.tail(3)).cwiseMax(-0.3).cwiseMin(0.3);

	// std::cout << "cartesian_error_integral_: " << cartesian_error_integral_.transpose() << std::endl;

	Eigen::VectorXd tau_task(this->ndof_), tau_nullspace(this->ndof_);
	Eigen::MatrixXd jacobian_transpose_pinv;

	tau_task << jacobian.transpose() * (-this->cartesian_stiffness_ * this->cartesian_error_ - this->cartesian_damping_ * (jacobian * feedback_dq) - this->Ki_ * this->cartesian_error_integral_);

	// std::cout << "tau_task: " << tau_task.transpose() << std::endl;

	Eigen::VectorXd q_nullspace_error(this->ndof_), dq_nullspace_error(this->ndof_);
	this->q_nullspace_ = cmd_q;
	q_nullspace_error << this->q_nullspace_ - feedback_q;
	q_nullspace_error.head(1) << q_nullspace_error.head(1) * this->nullspace_stiffness_;
	dq_nullspace_error << feedback_dq;
	dq_nullspace_error.head(1) << dq_nullspace_error.head(1) * 2.0 * sqrt(this->nullspace_stiffness_);
	pinocchio::PseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

	// std::cout << "jacobian_transpose_pinv: \n" << jacobian_transpose_pinv << std::endl;
	tau_nullspace << (Eigen::MatrixXd::Identity(this->ndof_, this->ndof_) - jacobian.transpose() * jacobian_transpose_pinv) *
					(this->nullspace_stiffness_ * q_nullspace_error - (2.0 * sqrt(this->nullspace_stiffness_)) * dq_nullspace_error);
	// std::cout << "tau_nullspace: " << tau_nullspace.transpose() << std::endl;
	this->tau_command_ = this->SaturateTorqueRate(tau_task + tau_nullspace, this->tau_command_);
	tau_command = this->tau_command_;
	// std::cout << "tau_command: " << this->tau_command_.transpose() << std::endl;
	return true;
}

Eigen::VectorXd CartesianImpedanceController::SaturateTorqueRate(const Eigen::VectorXd& tau_target, const Eigen::VectorXd& tau)
{
	Eigen::VectorXd diff = tau_target - tau;
	Eigen::VectorXd clipped = diff.cwiseMax(-this->delta_tau_max_).cwiseMin(this->delta_tau_max_);
	return tau + clipped;
}

void CartesianImpedanceController::SetStiffness(const Vector6d& K)
{
	this->cartesian_stiffness_ = K.asDiagonal();
	LOG_INFO << "CartesianImpedanceController: Set stiffness to [" << this->cartesian_stiffness_.diagonal().transpose() << "]";
}

void CartesianImpedanceController::SetDamping(const Vector6d& D)
{
	this->cartesian_damping_ = D.asDiagonal();
	LOG_INFO << "CartesianImpedanceController: Set damping to [" << this->cartesian_damping_.diagonal().transpose() << "]";
}

void CartesianImpedanceController::SetIntegralGain(const Vector6d& Ki)
{
	this->Ki_ = Ki.asDiagonal();
	LOG_INFO << "CartesianImpedanceController: Set integral gain to [" << this->Ki_.diagonal().transpose() << "]";
}

} // namespace dynamics
} // namespace robot_server_lib
