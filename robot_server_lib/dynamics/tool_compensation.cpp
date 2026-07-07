#include "tool_compensation.h"

namespace robot_server_lib {
namespace dynamics {
namespace tool_compensation {

Eigen::VectorXd ComputeToolJointGravityTorque(const pinocchio::Model& arm_model,
                                              const Eigen::VectorXd& feedback_q_arm,
                                              const ToolGravityCompensationConfig& config,
                                              const Vector3d& gravity_armbase,
                                              bool* applied,
                                              std::string* failure_reason) {
    if (applied) *applied = false;
    if (failure_reason) failure_reason->clear();

    if (!config.gravity_enabled) {
        return Eigen::VectorXd::Zero(arm_model.nv);
    }
    if (arm_model.nv <= 0 || feedback_q_arm.size() != arm_model.nq) {
        if (failure_reason) *failure_reason = "arm model or feedback q size is invalid";
        return Eigen::VectorXd::Zero(std::max<int>(arm_model.nv, 0));
    }
    if (!feedback_q_arm.allFinite() || !gravity_armbase.allFinite()) {
        if (failure_reason) *failure_reason = "feedback q or gravity contains non-finite values";
        return Eigen::VectorXd::Zero(arm_model.nv);
    }

    // arm base 系下的工具重力合力 (重力 = m * gravity_armbase, gravity_armbase 已含方向和大小)
    const Vector3d F_armbase = config.mass_kg * gravity_armbase;
    if (!F_armbase.allFinite()) {
        if (failure_reason) *failure_reason = "tool gravity force is non-finite";
        return Eigen::VectorXd::Zero(arm_model.nv);
    }

    // arm-only FK: 取 arm base <- EE 的旋转和 COM 在 arm base 系下的位置
    const Eigen::Matrix4d T_armbase_ee = pinocchio::GetFK(arm_model, feedback_q_arm);
    const Eigen::Matrix3d R_armbase_ee = T_armbase_ee.topLeftCorner<3, 3>();
    const Vector3d r_C_armbase = (R_armbase_ee * config.com_m).eval();  // COM 相对 EE 原点, arm base 系下
    if (!R_armbase_ee.allFinite() || !r_C_armbase.allFinite()) {
        if (failure_reason) *failure_reason = "end-effector FK is non-finite";
        return Eigen::VectorXd::Zero(arm_model.nv);
    }

    // 力矩取在 EE 原点, 表达在 arm base 系: τ_E = r_C × F
    const Vector3d tau_about_ee_armbase = r_C_armbase.cross(F_armbase);
    if (!tau_about_ee_armbase.allFinite()) {
        if (failure_reason) *failure_reason = "tool gravity torque is non-finite";
        return Eigen::VectorXd::Zero(arm_model.nv);
    }

    // wrench 在 arm base 系: [F ; τ_E], 与 LOCAL_WORLD_ALIGNED 雅可比 (arm-only 模型即 arm base 系) 对齐
    Vector6d wrench_armbase;
    wrench_armbase.head<3>() = F_armbase;
    wrench_armbase.tail<3>() = tau_about_ee_armbase;

    // tau_tool = -J^T * wrench
    // 取负: J^T * F_g 是重力对关节产生的力矩 (虚功原理), 逆动力学前馈需取负以抵消重力
    const Eigen::MatrixXd jacobian =
        pinocchio::GetJacobiMatrix(arm_model, feedback_q_arm, pinocchio::LOCAL_WORLD_ALIGNED);
    if (jacobian.rows() != 6 || jacobian.cols() != arm_model.nv || !jacobian.allFinite()) {
        if (failure_reason) *failure_reason = "Jacobian shape invalid or non-finite";
        return Eigen::VectorXd::Zero(arm_model.nv);
    }
    const Eigen::VectorXd tau_force = jacobian.topRows<3>().transpose() * wrench_armbase.head<3>();
    const Eigen::VectorXd tau_moment = jacobian.bottomRows<3>().transpose() * wrench_armbase.tail<3>();
    const Eigen::VectorXd tau_tool = -(tau_force + tau_moment);
    if (!tau_tool.allFinite()) {
        if (failure_reason) *failure_reason = "tool joint torque is non-finite";
        return Eigen::VectorXd::Zero(arm_model.nv);
    }

    if (applied) *applied = true;
    return tau_tool;
}

}  // namespace tool_compensation
}  // namespace dynamics
}  // namespace robot_server_lib
