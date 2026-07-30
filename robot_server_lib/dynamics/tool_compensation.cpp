#include "tool_compensation.h"

namespace robot_server_lib {
namespace dynamics {
namespace tool_compensation {

Eigen::VectorXd ComputeToolJointGravityTorque(const pinocchio::Model& arm_model,
                                              const Eigen::VectorXd& feedback_q_arm,
                                              const ToolGravityCompensationConfig& config,
                                              const Vector3d& gravity_armbase) {
    thread_local const pinocchio::Model* cached_model = nullptr;
    thread_local std::optional<pinocchio::Data> cached_data;
    if (cached_model != &arm_model) {
        cached_data.emplace(arm_model);
        cached_model = &arm_model;
    }
    return ComputeToolJointGravityTorque(arm_model, feedback_q_arm, config, gravity_armbase, *cached_data);
}

Eigen::VectorXd ComputeToolJointGravityTorque(const pinocchio::Model& arm_model,
                                              const Eigen::VectorXd& feedback_q_arm,
                                              const ToolGravityCompensationConfig& config,
                                              const Vector3d& gravity_armbase,
                                              pinocchio::Data& arm_data) {
    if (!config.gravity_enabled) {
        return Eigen::VectorXd::Zero(arm_model.nv);
    }
    if (arm_model.nv <= 0 || feedback_q_arm.size() != arm_model.nq) {
        return Eigen::VectorXd::Zero(std::max<int>(arm_model.nv, 0));
    }
    if (!feedback_q_arm.allFinite() || !gravity_armbase.allFinite()) {
        return Eigen::VectorXd::Zero(arm_model.nv);
    }

    // arm base 系下的工具重力合力 (重力 = m * gravity_armbase, gravity_armbase 已含方向和大小)
    const Vector3d F_armbase = config.mass_kg * gravity_armbase;
    if (!F_armbase.allFinite()) {
        return Eigen::VectorXd::Zero(arm_model.nv);
    }

    // arm-only FK: 取 arm base <- EE 的旋转和 COM 在 arm base 系下的位置
    const Eigen::Matrix4d T_armbase_ee = pinocchio::GetFK(arm_model, feedback_q_arm, arm_data);
    const Eigen::Matrix3d R_armbase_ee = T_armbase_ee.topLeftCorner<3, 3>();
    const Vector3d r_C_armbase = (R_armbase_ee * config.com_m).eval();  // COM 相对 EE 原点, arm base 系下
    if (!R_armbase_ee.allFinite() || !r_C_armbase.allFinite()) {
        return Eigen::VectorXd::Zero(arm_model.nv);
    }

    // 力矩取在 EE 原点, 表达在 arm base 系: τ_E = r_C × F
    const Vector3d tau_about_ee_armbase = r_C_armbase.cross(F_armbase);
    if (!tau_about_ee_armbase.allFinite()) {
        return Eigen::VectorXd::Zero(arm_model.nv);
    }

    // wrench 转换到 EE 坐标系 (LOCAL Jacobian 要求 wrench 与 Jacobian 同系)
    const Eigen::Matrix3d R_armbase_ee_T = R_armbase_ee.transpose();
    const Vector3d F_ee = R_armbase_ee_T * F_armbase;
    const Vector3d tau_ee_ee = R_armbase_ee_T * tau_about_ee_armbase;

    Vector6d wrench_ee;
    wrench_ee.head<3>() = F_ee;
    wrench_ee.tail<3>() = tau_ee_ee;

    const Eigen::MatrixXd jacobian =
        pinocchio::GetJacobiMatrix(arm_model, feedback_q_arm, pinocchio::LOCAL, arm_data);
    if (jacobian.rows() != 6 || jacobian.cols() != arm_model.nv || !jacobian.allFinite()) {
        return Eigen::VectorXd::Zero(arm_model.nv);
    }
    const Eigen::VectorXd tau_force = jacobian.topRows<3>().transpose() * wrench_ee.head<3>();
    const Eigen::VectorXd tau_moment = jacobian.bottomRows<3>().transpose() * wrench_ee.tail<3>();
    const Eigen::VectorXd tau_tool = -(tau_force + tau_moment);
    if (!tau_tool.allFinite()) {
        return Eigen::VectorXd::Zero(arm_model.nv);
    }

    return tau_tool;
}

}  // namespace tool_compensation
}  // namespace dynamics
}  // namespace robot_server_lib
