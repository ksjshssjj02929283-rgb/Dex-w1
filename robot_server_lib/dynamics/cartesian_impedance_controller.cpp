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
	// damping_ = 2.0 * this->stiffness_.array().sqrt().matrix();
	nullspace_damp_ = 2.0 * sqrt(this->nullspace_stiff_);
}

bool CartesianImpedanceController::UpdateTorqueCommand(const Eigen::VectorXd &feedback_q, const Eigen::VectorXd &feedback_dq, const Eigen::VectorXd &cmd_q, Eigen::VectorXd &tau_command)
{
	Eigen::VectorXd tau_task(this->ndof_), tau_nullspace(this->ndof_);
	Eigen::Affine3d feedback_transform(pinocchio::GetFK(this->pinocchio_model_, feedback_q));
	Vector3d feedback_trans(feedback_transform.translation());
	Eigen::Quaterniond feedback_quat(feedback_transform.linear());

	Eigen::Affine3d cmd_transform(pinocchio::GetFK(this->pinocchio_model_, cmd_q));
	Vector3d cmd_trans(cmd_transform.translation());
	Eigen::Quaterniond cmd_quat(cmd_transform.linear());
	Eigen::MatrixXd jacobian = pinocchio::GetJacobiMatrix(this->pinocchio_model_, feedback_q);

	// tau_task计算
	// pose error计算
	Vector6d pose_err, vel_err;
	pose_err.head<3>() = (cmd_trans - feedback_trans).cwiseMin(this->trans_clip_max_).cwiseMax(-this->trans_clip_max_);
	feedback_quat.coeffs() = (cmd_quat.coeffs().dot(feedback_quat.coeffs()) < 0.0) ? -feedback_quat.coeffs() : feedback_quat.coeffs(); // 保证四元数同号
	
	Eigen::Quaterniond err_quat(feedback_quat.inverse() * cmd_quat);
	pose_err.tail<3>() << err_quat.x(), err_quat.y(), err_quat.z();
	pose_err.tail<3>() = feedback_transform.linear() * pose_err.tail<3>();  // 旋转误差末端坐标系 -> World base
	pose_err.tail<3>() = pose_err.tail<3>().cwiseMin(this->rot_clip_max_).cwiseMax(-this->rot_clip_max_);
	// 积分误差累加与限幅
	this->pose_err_intg_.head(3) << (this->pose_err_intg_.head(3) + pose_err.head(3)).cwiseMin(0.1).cwiseMax(-0.1);
	this->pose_err_intg_.tail(3) << (this->pose_err_intg_.tail(3) + pose_err.tail(3)).cwiseMin(0.3).cwiseMax(-0.3);

	// vel error计算并限制幅度
	vel_err << Eigen::VectorXd::Zero(6) - jacobian * feedback_dq;
	vel_err = vel_err.cwiseMin(this->vel_clip_max_).cwiseMax(-this->vel_clip_max_);

	auto stiffness_term = this->stiffness_ * pose_err;
	auto damping_term = this->damping_ * vel_err;
	tau_task << jacobian.transpose() * (stiffness_term + damping_term + this->Ki_ * this->pose_err_intg_);
	
	// 零空间控制
	Eigen::VectorXd q_nullspace_err(this->ndof_), dq_nullspace_err(this->ndof_);
	this->q_nullspace_ = cmd_q;
	q_nullspace_err << this->q_nullspace_ - feedback_q;
	q_nullspace_err.head(1) << q_nullspace_err.head(1) * this->nullspace_stiff_;
	q_nullspace_err = q_nullspace_err.cwiseMin(this->nullspace_clip_max_).cwiseMax(-this->nullspace_clip_max_);

	dq_nullspace_err << Eigen::VectorXd::Zero(this->ndof_) - feedback_dq;
	dq_nullspace_err.head(1) << dq_nullspace_err.head(1) * this->nullspace_damp_;
	dq_nullspace_err = dq_nullspace_err.cwiseMin(this->nullspace_clip_max_).cwiseMax(-this->nullspace_clip_max_);

	Eigen::MatrixXd jacobian_transpose_pinv;
	pinocchio::PseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

	tau_nullspace << (Eigen::MatrixXd::Identity(this->ndof_, this->ndof_) - jacobian.transpose() * jacobian_transpose_pinv) *
									(this->nullspace_stiff_ * q_nullspace_err + this->nullspace_damp_ * dq_nullspace_err);

	// 力矩变化率限幅与输出
	this->tau_command_ = this->SaturateTorqueRate(tau_task + tau_nullspace, this->tau_command_);
	tau_command = this->tau_command_;

	// debug log
	{
		std::stringstream ss_tau_command, ss_tau_task, ss_stiff_term, ss_cart_damp, ss_jacobian, ss_tau_nullspace, ss_cart_err, ss_cart_vel_err, ss_q_err, ss_dq;
		ss_tau_command << std::fixed << std::setprecision(5) << tau_command.transpose();
		ss_stiff_term << std::fixed << std::setprecision(5) << stiffness_term.transpose();
		ss_cart_damp << std::fixed << std::setprecision(5) << damping_term.transpose();
		ss_tau_task << std::fixed << std::setprecision(5) << tau_task.transpose();
		ss_tau_nullspace << std::fixed << std::setprecision(5) << tau_nullspace.transpose();
		ss_jacobian << std::fixed << std::setprecision(5) << jacobian;
		ss_q_err << std::fixed << std::setprecision(5) << q_nullspace_err.transpose();
		ss_dq << std::fixed << std::setprecision(5) << feedback_dq.transpose();
		ss_cart_err << std::fixed << std::setprecision(5) << pose_err.transpose();
		ss_cart_vel_err << std::fixed << std::setprecision(5) << vel_err.transpose();
		static int print_counter = 0;
		if (++print_counter >= 10) 
		{
			print_counter = 0;
			LOG_INFO << "tau_nullspace: " << ss_tau_nullspace.str()
					<< ", tau_command: " << ss_tau_command.str()
					<< ", q_err: " << ss_q_err.str()
					<< ", dq: " << ss_dq.str()
					<< ", pose_err: " << ss_cart_err.str()
					<< ", stiff_term: " << ss_stiff_term.str()
					<< ", vel_err: " << ss_cart_vel_err.str()
					<< ", damp_term: " << ss_cart_damp.str()
					<< ", tau_task: " << ss_tau_task.str();
		}
	}
	return true;
}

Eigen::VectorXd CartesianImpedanceController::SaturateTorqueRate(const Eigen::VectorXd& tau_target, const Eigen::VectorXd& tau)
{
	Eigen::VectorXd diff = tau_target - tau;
	Eigen::VectorXd clipped = diff.cwiseMin(this->delta_tau_max_).cwiseMax(-this->delta_tau_max_);
	Eigen::VectorXd output_tau = (tau + clipped).cwiseMin(this->tau_command_max_).cwiseMax(-this->tau_command_max_);
	return output_tau;
}

void CartesianImpedanceController::SetStiffness(const Vector6d& K)
{
	this->stiffness_ = K.asDiagonal();
	this->damping_ = 2.0 * this->stiffness_.array().sqrt().matrix();
	std::stringstream ss_stiff, ss_damp;
	ss_stiff << std::fixed << std::setprecision(4) << this->stiffness_.diagonal().transpose();
	ss_damp << std::fixed << std::setprecision(4) << this->damping_.diagonal().transpose();
	LOG_INFO << "CartesianImpedanceController: Set stiffness to [" << ss_stiff.str() << "], damping to [" << ss_damp.str() << "]";
}

} // namespace dynamics
} // namespace robot_server_lib
