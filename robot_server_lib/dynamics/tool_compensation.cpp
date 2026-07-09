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
    constexpr int kLogInterval = 200;
    static int64_t call_counter = 0;

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

    // wrench 转换到 EE 坐标系 (LOCAL Jacobian 要求 wrench 与 Jacobian 同系)
    //   F_ee = R^T * F_armbase
    //   τ_EE_ee = R^T * τ_EE_armbase
    const Eigen::Matrix3d R_armbase_ee_T = R_armbase_ee.transpose();
    const Vector3d F_ee = R_armbase_ee_T * F_armbase;
    const Vector3d tau_ee_ee = R_armbase_ee_T * tau_about_ee_armbase;

    Vector6d wrench_ee;
    wrench_ee.head<3>() = F_ee;
    wrench_ee.tail<3>() = tau_ee_ee;

    // tau_tool = -J^T * wrench
    // J 用 LOCAL 坐标系: 线速度和角速度都在 EE 坐标系下表达
    // 取负: J^T * F_g 是重力对关节产生的力矩 (虚功原理), 逆动力学前馈需取负以抵消重力
    const Eigen::MatrixXd jacobian =
        pinocchio::GetJacobiMatrix(arm_model, feedback_q_arm, pinocchio::LOCAL);
    if (jacobian.rows() != 6 || jacobian.cols() != arm_model.nv || !jacobian.allFinite()) {
        if (failure_reason) *failure_reason = "Jacobian shape invalid or non-finite";
        return Eigen::VectorXd::Zero(arm_model.nv);
    }
    const Eigen::VectorXd tau_force = jacobian.topRows<3>().transpose() * wrench_ee.head<3>();
    const Eigen::VectorXd tau_moment = jacobian.bottomRows<3>().transpose() * wrench_ee.tail<3>();
    const Eigen::VectorXd tau_tool = -(tau_force + tau_moment);
    if (!tau_tool.allFinite()) {
        if (failure_reason) *failure_reason = "tool joint torque is non-finite";
        return Eigen::VectorXd::Zero(arm_model.nv);
    }

    // 过程数据打印 (每 kLogInterval 周期)
    call_counter++;
    if (call_counter % kLogInterval == 0) {
        std::cout << "[ToolComp] mass =" << config.mass_kg
                 << "\n gravity_armbase =" << gravity_armbase.transpose()
                 << "\n gravity_armbase_norm=" << gravity_armbase.norm()
                 << "\n com_m=" << config.com_m.transpose()
                 << "\n r_C_armbase=" << r_C_armbase.transpose()
                 << "\n T_armbase_ee =\n" << T_armbase_ee
                 << "\n wrench_armbase_F=" << F_armbase.transpose()
                 << "\n wrench_armbase_tau=" << tau_about_ee_armbase.transpose()
                 << "\n wrench_ee_F=" << wrench_ee.head<3>().transpose()
                 << "\n wrench_ee_tau=" << wrench_ee.tail<3>().transpose()
                 << "\n tau_tool=" << tau_tool.transpose() << std::endl;
    }

    if (applied) *applied = true;
    return tau_tool;
}

}  // namespace tool_compensation
}  // namespace dynamics
}  // namespace robot_server_lib
