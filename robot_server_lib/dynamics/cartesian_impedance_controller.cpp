#include "cartesian_impedance_controller.h"
#include <iostream>
#include <iomanip>
#include <sstream>

namespace robot_server_lib {
namespace dynamics {

CartesianImpedanceController::CartesianImpedanceController(const RobotModel &robot_model, MotionGroup motion_group)
	: robot_model_(robot_model), motion_group_(motion_group)
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

	this->cart_err_.head<3>() = (cmd_trans - feedback_trans).cwiseMin(this->trans_clip_max_).cwiseMax(this->trans_clip_min_);

	if (cmd_quat.coeffs().dot(feedback_quat.coeffs()) < 0.0)
	{
		feedback_quat.coeffs() << -feedback_quat.coeffs();
	}
	Eigen::Quaterniond err_quat(feedback_quat.inverse() * cmd_quat);
	this->cart_err_.tail<3>() << err_quat.x(), err_quat.y(), err_quat.z();
	this->cart_err_.tail<3>() = -feedback_transform.linear() * this->cart_err_.tail<3>();
	this->cart_err_.tail<3>() = this->cart_err_.tail<3>().cwiseMin(this->rot_clip_max_).cwiseMax(this->rot_clip_min_);
	this->cart_err_intg_.head(3) << (this->cart_err_intg_.head(3) + this->cart_err_.head(3)).cwiseMin(0.1).cwiseMax(-0.1);
	this->cart_err_intg_.tail(3) << (this->cart_err_intg_.tail(3) + this->cart_err_.tail(3)).cwiseMin(0.3).cwiseMax(-0.3);

	Eigen::VectorXd tau_task(this->ndof_), tau_nullspace(this->ndof_);
	auto cart_stiff = this->cart_stiff_ * this->cart_err_;
	auto cart_damp = this->cart_damp_ * (Vector6d::Zero() - jacobian * feedback_dq);
	tau_task << jacobian.transpose() * (cart_stiff + cart_damp);
	
	Eigen::VectorXd q_nullspace_err(this->ndof_), dq_nullspace_err(this->ndof_);
	this->q_nullspace_ = cmd_q;
	q_nullspace_err << this->q_nullspace_ - feedback_q;
	q_nullspace_err.head(1) << q_nullspace_err.head(1) * this->nullspace_stiff_;
	dq_nullspace_err << Eigen::VectorXd::Zero(this->ndof_) - feedback_dq;
	dq_nullspace_err.head(1) << dq_nullspace_err.head(1) * 2.0 * sqrt(this->nullspace_stiff_);
	Eigen::MatrixXd jacobian_transpose_pinv;
	pinocchio::PseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

	tau_nullspace << (Eigen::MatrixXd::Identity(this->ndof_, this->ndof_) - jacobian.transpose() * jacobian_transpose_pinv) *
									(this->nullspace_stiff_ * q_nullspace_err + (2.0 * sqrt(this->nullspace_stiff_)) * dq_nullspace_err);

	this->tau_command_ = this->SaturateTorqueRate(tau_task + tau_nullspace, this->tau_command_);
	tau_command = this->tau_command_;

	std::stringstream ss_tau_task, ss_cart_stiff, ss_cart_damp, ss_jacobian, ss_tau_nullspace, ss_cart_err, ss_cart_vel_err, ss_q_err;
	ss_cart_stiff << std::fixed << std::setprecision(5) << cart_stiff.transpose();
	ss_cart_damp << std::fixed << std::setprecision(5) << cart_damp.transpose();
	ss_tau_task << std::fixed << std::setprecision(5) << tau_task.transpose();
	ss_tau_nullspace << std::fixed << std::setprecision(5) << tau_nullspace.transpose();
	ss_jacobian << std::fixed << std::setprecision(5) << jacobian;
	ss_q_err << std::fixed << std::setprecision(5) << q_nullspace_err.transpose();
	ss_cart_err << std::fixed << std::setprecision(5) << cart_err_.transpose();
	ss_cart_vel_err << std::fixed << std::setprecision(5) << (-jacobian * feedback_dq).transpose();
	static int print_counter = 0;
	if (++print_counter >= 10) 
	{
    	print_counter = 0;
		LOG_INFO << "tau_task: " << ss_tau_task.str()
				<< ", tau_nullspace: " << ss_tau_nullspace.str()
				<< ", q_err: " << ss_q_err.str()
				<< ", pose_err: " << ss_cart_err.str()
				<< ", stiff_term: " << ss_cart_stiff.str()
				<< ", vel_err: " << ss_cart_vel_err.str()
				<< ", damp_term: " << ss_cart_damp.str();
	}
	return true;
}

Eigen::VectorXd CartesianImpedanceController::SaturateTorqueRate(const Eigen::VectorXd& tau_target, const Eigen::VectorXd& tau)
{
	Eigen::VectorXd diff = tau_target - tau;
	Eigen::VectorXd clipped = diff.cwiseMin(this->delta_tau_max_).cwiseMax(-this->delta_tau_max_);
	return tau + clipped;
}

void CartesianImpedanceController::SetStiffness(const Vector6d& K)
{
	this->cart_stiff_ = K.asDiagonal();
	std::stringstream ss;
	ss << std::fixed << std::setprecision(4) << this->cart_stiff_.diagonal().transpose();
	LOG_INFO << "CartesianImpedanceController: Set stiffness to [" << ss.str() << "]";
}

void CartesianImpedanceController::SetDamping(const Vector6d& D)
{
	this->cart_damp_ = D.asDiagonal();
	std::stringstream ss;
	ss << std::fixed << std::setprecision(4) << this->cart_damp_.diagonal().transpose();
	LOG_INFO << "CartesianImpedanceController: Set damping to [" << ss.str() << "]";
}

void CartesianImpedanceController::SetIntegralGain(const Vector6d& Ki)
{
	this->Ki_ = Ki.asDiagonal();
	std::stringstream ss;
	ss << std::fixed << std::setprecision(4) << this->Ki_.diagonal().transpose();
	LOG_INFO << "CartesianImpedanceController: Set integral gain to [" << ss.str() << "]";
}

} // namespace dynamics
} // namespace robot_server_lib
