#include "cartesian_impedance_controller.h"
#include <iostream>

namespace robot_server_lib {
namespace dynamics {

CartesianImpedanceController::CartesianImpedanceController(const RobotModel &robot_model, MotionGroup motion_group, double control_dt)
	: robot_model_(robot_model), motion_group_(motion_group), control_dt_(control_dt)
{
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
	q_nullspace_.setZero(ndof_);
}

bool CartesianImpedanceController::UpdateCommand(const Eigen::VectorXd &feedback_q, const Eigen::VectorXd &feedback_dq, const Eigen::VectorXd &cmd_q, Eigen::VectorXd &tau_command)
{
	Eigen::Affine3d feedback_transform(pinocchio::GetFK(pinocchio_model_, feedback_q));
	Eigen::Vector3d feedback_trans(feedback_transform.translation());
	Eigen::Quaterniond feedback_quat(feedback_transform.linear());

	Eigen::Affine3d cmd_transform(pinocchio::GetFK(pinocchio_model_, cmd_q));
	Eigen::Vector3d cmd_trans(cmd_transform.translation());
	Eigen::Quaterniond cmd_quat(cmd_transform.linear());
	Eigen::MatrixXd jacobian = pinocchio::GetJacobiMatrix(pinocchio_model_, feedback_q);

	// std::cout << "feedback_pose trans: " << feedback_trans.transpose() << std::endl;
	// std::cout << "command_pose trans: " << cmd_trans.transpose() << std::endl;
	// std::cout << "feedback_pose rot: " << feedback_quat.coeffs().transpose() << std::endl;
	// std::cout << "command_pose rot: " << cmd_quat.coeffs().transpose() << std::endl;
	// std::cout << "jacobian: \n" << jacobian << std::endl;

	cartesian_error_.head<3>() = (cmd_trans - feedback_trans).cwiseMax(trans_clip_min_).cwiseMin(trans_clip_max_);

	if (cmd_quat.coeffs().dot(feedback_quat.coeffs()) < 0.0)
	{
		feedback_quat.coeffs() << -feedback_quat.coeffs();
	}
	Eigen::Quaterniond error_quaternion(feedback_quat.inverse() * cmd_quat);
	cartesian_error_.tail<3>() << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
	cartesian_error_.tail<3>() = -feedback_transform.linear() * cartesian_error_.tail<3>();
	cartesian_error_.tail<3>() = cartesian_error_.tail<3>().cwiseMax(rot_clip_min_).cwiseMin(rot_clip_max_);

	// std::cout << "cartesian_error_: " << cartesian_error_.transpose() << std::endl;

	cartesian_error_integral_.head(3) << (cartesian_error_integral_.head(3) + cartesian_error_.head(3)).cwiseMax(-0.1).cwiseMin(0.1);
	cartesian_error_integral_.tail(3) << (cartesian_error_integral_.tail(3) + cartesian_error_.tail(3)).cwiseMax(-0.3).cwiseMin(0.3);

	// std::cout << "cartesian_error_integral_: " << cartesian_error_integral_.transpose() << std::endl;

	Eigen::VectorXd tau_task(ndof_), tau_nullspace(ndof_);
	Eigen::MatrixXd jacobian_transpose_pinv;

	tau_task << jacobian.transpose() * (-cartesian_stiffness_ * cartesian_error_ - cartesian_damping_ * (jacobian * feedback_dq) - Ki_ * cartesian_error_integral_);

	// std::cout << "tau_task: " << tau_task.transpose() << std::endl;

	Eigen::VectorXd q_nullspace_error(ndof_), dq_nullspace_error(ndof_);
	q_nullspace_ = cmd_q;
	q_nullspace_error << q_nullspace_ - feedback_q;
	q_nullspace_error.head(1) << q_nullspace_error.head(1) * nullspace_stiffness_;
	dq_nullspace_error << feedback_dq;
	dq_nullspace_error.head(1) << dq_nullspace_error.head(1) * 2.0 * sqrt(nullspace_stiffness_);
	pinocchio::PseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

	// std::cout << "jacobian_transpose_pinv: \n" << jacobian_transpose_pinv << std::endl;
	tau_nullspace << (Eigen::MatrixXd::Identity(ndof_, ndof_) - jacobian.transpose() * jacobian_transpose_pinv) *
					(nullspace_stiffness_ * q_nullspace_error - (2.0 * sqrt(nullspace_stiffness_)) * dq_nullspace_error);
	// std::cout << "tau_nullspace: " << tau_nullspace.transpose() << std::endl;
	tau_command = SaturateTorqueRate(tau_task + tau_nullspace, tau_command);
	// std::cout << "tau_command: " << tau_command.transpose() << std::endl;
	return true;
}

Eigen::VectorXd CartesianImpedanceController::SaturateTorqueRate(const Eigen::VectorXd& tau_target, const Eigen::VectorXd& tau)
{
	Eigen::VectorXd diff = tau_target - tau;
	Eigen::VectorXd clipped = diff.cwiseMax(-delta_tau_max_).cwiseMin(delta_tau_max_);
	return tau + clipped;
}

void CartesianImpedanceController::SetStiffness(const Vector6d& K)
{
	cartesian_stiffness_ = K.asDiagonal();
	LOG_INFO << "CartesianImpedanceController: Set stiffness to [" << cartesian_stiffness_.diagonal().transpose() << "]";
}

void CartesianImpedanceController::SetDamping(const Vector6d& D)
{
	cartesian_damping_ = D.asDiagonal();
	LOG_INFO << "CartesianImpedanceController: Set damping to [" << cartesian_damping_.diagonal().transpose() << "]";
}

void CartesianImpedanceController::SetIntegralGain(const Vector6d& Ki)
{
	Ki_ = Ki.asDiagonal();
	LOG_INFO << "CartesianImpedanceController: Set integral gain to [" << Ki_.diagonal().transpose() << "]";
}

} // namespace dynamics
} // namespace robot_server_lib
