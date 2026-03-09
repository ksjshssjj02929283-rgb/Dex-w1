#include "cartesian_impedance_controller.h"

#include <iomanip>
#include <iostream>
#include <sstream>

namespace robot_server_lib {
namespace dynamics {

CartesianImpedanceController::CartesianImpedanceController(const RobotModel &robot_model, MotionGroup motion_group)
    : robot_model_(robot_model), motion_group_(motion_group) {
    this->robot_model_.params_ptr->joints_primary.size();
    if (this->motion_group_ == MotionGroup::Primary) {
        this->pinocchio_model_ = *this->robot_model_.primary_arm_model_ptr;
        this->ndof_ = this->pinocchio_model_.nv;
    } else if (this->motion_group_ == MotionGroup::Secondary) {
        this->pinocchio_model_ = *this->robot_model_.secondary_arm_model_ptr;
        this->ndof_ = this->pinocchio_model_.nv;
    }
    this->q_nullspace_.setZero(this->ndof_);
    this->tau_command_.setZero(this->ndof_);
}

bool CartesianImpedanceController::UpdateTorqueCommand(const Eigen::VectorXd &feedback_q,
                                                       const Eigen::VectorXd &feedback_dq,
                                                       const Eigen::VectorXd &cmd_q,
                                                       const Eigen::VectorXd &cmd_dq,
                                                       Eigen::VectorXd &tau_command,
                                                       Vector6d &cart_impedance_force) {
    Eigen::VectorXd tau_task(this->ndof_), tau_nullspace(this->ndof_);
    tau_task.setZero();
    tau_nullspace.setZero();

    /* ================= FK ================= */
    Eigen::Affine3d T_fb(pinocchio::GetFK(this->pinocchio_model_, feedback_q));
    Eigen::Affine3d T_cmd(pinocchio::GetFK(this->pinocchio_model_, cmd_q));

    Vector3d p_fb = T_fb.translation();
    Vector3d p_cmd = T_cmd.translation();

    Eigen::Quaterniond fb_quat(T_fb.linear());
    Eigen::Quaterniond cmd_quat(T_cmd.linear());

    /* ================= Pose Error ================= */
    Vector3d err_p = (p_cmd - p_fb).cwiseMin(this->trans_clip_max_).cwiseMax(-this->trans_clip_max_);

    // 确保四元数符号一致以选取最短旋转（在计算 err_quat 之前）
    if (cmd_quat.coeffs().dot(fb_quat.coeffs()) < 0.0) fb_quat.coeffs() = -fb_quat.coeffs();
    Eigen::Quaterniond err_quat = fb_quat.inverse() * cmd_quat;
    Eigen::AngleAxisd aa(err_quat);
    Vector3d err_w = aa.axis() * aa.angle();
    err_w = T_fb.linear() * err_w;
    err_w = err_w.cwiseMin(this->rot_clip_max_).cwiseMax(-this->rot_clip_max_);

    /* ================= Jacobian ================= */
    Eigen::MatrixXd J = pinocchio::GetJacobiMatrix(this->pinocchio_model_, feedback_q);
    Eigen::MatrixXd J_cmd = pinocchio::GetJacobiMatrix(this->pinocchio_model_, cmd_q);
    Eigen::MatrixXd J_dot = pinocchio::GetJacobianMatrixDot(this->pinocchio_model_, feedback_q, feedback_dq);
    Vector6d v_cmd = J_cmd * cmd_dq;

    /* ================= Velocity Error ================= */
    Vector6d v_fb = J * feedback_dq;
    Vector3d derr_p = v_cmd.head<3>() - v_fb.head<3>();
    Vector3d derr_w = v_cmd.tail<3>() - v_fb.tail<3>();

    /* ================= Integral (translation only) ================= */
    this->pose_err_intg_.head<3>() = (this->pose_err_intg_.head<3>() + err_p)
                                         .cwiseMin(this->trans_intg_clip_max_)
                                         .cwiseMax(-this->trans_intg_clip_max_);

    /* ================= CartesianImpedance force ================= */
    Vector6d damping_vector = this->damping_ratio_.asDiagonal() * critical_damping_ratio_ *
                              (this->mass_vector_.array() * this->stiffness_vector_.array()).sqrt().matrix();
    Vector3d fp = this->stiffness_vector_.head<3>().asDiagonal() * err_p +
                  damping_vector.head<3>().asDiagonal() * derr_p +
                  this->Ki_vector_.head<3>().asDiagonal() * this->pose_err_intg_.head<3>();

    Vector3d fw =
        this->stiffness_vector_.tail<3>().asDiagonal() * err_w + damping_vector.tail<3>().asDiagonal() * derr_w;

    /* ================= Inertia shaping ================= */
    F_impedance_before_inertia_full_ << fp, fw;
    Vector6d F_impedance;
    Vector6d acc_task = this->mass_vector_.asDiagonal().inverse() * F_impedance_before_inertia_full_;

    this->Mq_ = GetMassMatrixBySymbol(this->robot_model_, feedback_q, this->motion_group_);

    Eigen::MatrixXd lambda;
    pinocchio::PseudoInverse(J * this->Mq_.inverse() * J.transpose(), lambda);  // 笛卡尔空间质量矩阵
    F_impedance = lambda * (acc_task - J_dot * feedback_dq);
    tau_task = J.transpose() * F_impedance;

    /* ================= Nullspace ================= */
    Eigen::MatrixXd J_T_pinv;
    pinocchio::PseudoInverse(J.transpose(), J_T_pinv);

    Eigen::MatrixXd Np = Eigen::MatrixXd::Identity(this->ndof_, this->ndof_) - J.transpose() * J_T_pinv;

    this->q_nullspace_ = cmd_q;
    Eigen::VectorXd q_err = this->q_nullspace_ - feedback_q;
    Eigen::VectorXd dq_err = -feedback_dq;
    q_err.head(1) << q_err.head(1) * this->nullspace_stiff_;
    dq_err.head(1) << dq_err.head(1) * this->nullspace_damp_;

    tau_nullspace = Np * (this->nullspace_stiff_ * q_err + this->nullspace_damp_ * dq_err);

    /* ================= Output ================= */
    this->tau_command_ = this->SaturateTorqueRate(tau_task + tau_nullspace, this->tau_command_);
    tau_command = this->tau_command_;

    // debug log
    static int print_counter = 0;
    if (++print_counter >= 10 && this->motion_group_ == MotionGroup::Primary && false) {
        print_counter = 0;
        std::stringstream ss_tau_command, ss_tau_task, ss_stiff_term, ss_cart_damp, ss_jacobian, ss_tau_nullspace,
            ss_acc_task, ss_lambda, ss_cart_err, ss_cart_vel_err, ss_q_err, ss_dq, ss_pose_err_intg, ss_F_imp,
            ss_acc_Jdot, ss_cmd_trans, ss_v_cmd, ss_acc, ss_Mq;
        ss_q_err << std::fixed << std::setprecision(3) << (cmd_q - feedback_q).transpose();
        ss_tau_command << std::fixed << std::setprecision(3) << tau_command.transpose();
        ss_tau_task << std::fixed << std::setprecision(3) << tau_task.transpose();
        ss_tau_nullspace << std::fixed << std::setprecision(3) << tau_nullspace.transpose();
        ss_dq << std::fixed << std::setprecision(4) << feedback_dq.transpose();
        ss_cart_err << std::fixed << std::setprecision(4) << err_p.transpose() << " " << err_w.transpose();
        ss_v_cmd << std::fixed << std::setprecision(2) << v_cmd.transpose();
        ss_cart_vel_err << std::fixed << std::setprecision(4) << derr_p.transpose() << " " << derr_w.transpose();
        ss_pose_err_intg << std::fixed << std::setprecision(2) << this->pose_err_intg_.transpose();
        ss_acc_task << std::fixed << std::setprecision(4) << acc_task.transpose();
        ss_F_imp << std::fixed << std::setprecision(4) << F_impedance_before_inertia_full_.transpose() << " -> "
                 << F_impedance.transpose();
        ss_Mq << std::fixed << std::setprecision(4) << this->Mq_.diagonal().transpose();
        ss_lambda << std::fixed << std::setprecision(4) << lambda.diagonal().transpose();

        LOG_INFO << "v_cmd: " << ss_v_cmd.str() << ", pose_err: " << ss_cart_err.str()
                 << ", vel_err: " << ss_cart_vel_err.str() << ", acc_task: " << ss_acc_task.str()
                 << ", lambda: " << ss_lambda.str() << ", F_imp: " << ss_F_imp.str()
                 << ", tau_task: " << ss_tau_task.str() << ", tau_nullspace: " << ss_tau_nullspace.str()
                 << ", tau_command: " << ss_tau_command.str() << ", dq: " << ss_dq.str();
    }
    cart_impedance_force = F_impedance_before_inertia_full_;
    return true;
}

Eigen::VectorXd CartesianImpedanceController::SaturateTorqueRate(const Eigen::VectorXd &tau_target,
                                                                 const Eigen::VectorXd &tau) {
    Eigen::VectorXd diff = tau_target - tau;
    Eigen::VectorXd clipped = diff.cwiseMin(this->delta_tau_max_).cwiseMax(-this->delta_tau_max_);
    Eigen::VectorXd output_tau = (tau + clipped).cwiseMin(this->tau_command_max_).cwiseMax(-this->tau_command_max_);
    return output_tau;
}

void CartesianImpedanceController::SetStiffness(const Vector6d &K) {
    this->stiffness_vector_ = K;
    std::stringstream ss_stiff;
    ss_stiff << std::fixed << std::setprecision(4) << this->stiffness_vector_.transpose();
    LOG_INFO << "CartesianImpedanceController: Set stiffness to [" << ss_stiff.str() << "]";
}

void CartesianImpedanceController::SetDampingRatio(const Vector6d &D_ratio) {
    this->damping_ratio_ = D_ratio;
    std::stringstream ss_damp;
    ss_damp << std::fixed << std::setprecision(4) << this->damping_ratio_.transpose();
    LOG_INFO << "CartesianImpedanceController: Set cartesian damping ratio to [" << ss_damp.str() << "]";
}
}  // namespace dynamics
}  // namespace robot_server_lib
