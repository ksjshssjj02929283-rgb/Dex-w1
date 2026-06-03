#include <math.h>
#include <stddef.h>
#include <iostream>
#include "inverse_dynamics.h"

// 引入所有生成的版本化函数前向声明
namespace robot_server_lib::dynamics::w1_v03 {
    void left_invdyn_tau(double* tau_out, const double* q, const double* dq, const double* ddq, const double* dyn_base_params, const double* gravity_acc);
    void right_invdyn_tau(double* tau_out, const double* q, const double* dq, const double* ddq, const double* dyn_base_params, const double* gravity_acc);
    void left_mass_term(double* M_term_out, const double* q, const double* ddq, const double* dyn_base_params);
    void right_mass_term(double* M_term_out, const double* q, const double* ddq, const double* dyn_base_params);
}

namespace robot_server_lib
{
namespace dynamics
{

// 调度器：根据构型和版本返回对应的逆动力学函数指针
typedef void (*InvDynFunc)(double*, const double*, const double*, const double*, const double*, const double*);
typedef void (*MassTermFunc)(double*, const double*, const double*, const double*);

struct DynDispatcher {
    static InvDynFunc GetInvDyn(const std::string& robot_key, bool is_left) {
        if (robot_key == "w1_v03") {
            return is_left ? w1_v03::left_invdyn_tau : w1_v03::right_invdyn_tau;
        }
        return nullptr;
    }

    static MassTermFunc GetMassTerm(const std::string& robot_key, bool is_left) {
        if (robot_key == "w1_v03") {
            return is_left ? w1_v03::left_mass_term : w1_v03::right_mass_term;
        }
        return nullptr;
    }
};

Eigen::VectorXd InvDynamikBySymbol(const RobotModel& robot_model, const VectorDof& q, const VectorDof& dq,
                                    const VectorDof& ddq, const Vector3d& gravity_acc_armbase, MotionGroup motion_group)
{
    Eigen::VectorXd dyn_base_params(71), q_offset(7), q_arm(7), dq_arm(7), ddq_arm(7);
    utils::GetJointFromWholeBody(robot_model, q, q_arm, motion_group);
    utils::GetJointFromWholeBody(robot_model, dq, dq_arm, motion_group);
    utils::GetJointFromWholeBody(robot_model, ddq, ddq_arm, motion_group);
    
    bool is_left = (motion_group == MotionGroup::Primary);

    if (is_left)
    {
        assert(robot_model.params_ptr->dyn_base_params_primary.has_value());
        dyn_base_params = robot_model.params_ptr->dyn_base_params_primary.value();
        q_offset << 0.0, 0.0, -M_PI/2.0, 0.0, 0.0, -M_PI/2.0, 0.0;
    }
    else
    {
        assert(robot_model.params_ptr->dyn_base_params_secondary.has_value());
        dyn_base_params = robot_model.params_ptr->dyn_base_params_secondary.value();
        q_offset << 0.0, 0.0, M_PI/2.0, 0.0, 0.0, M_PI/2.0, 0.0;
    }

    const double friction_ratio = 0.6;
    dyn_base_params[3]  *= friction_ratio;
    dyn_base_params[13] *= friction_ratio;
    dyn_base_params[24] *= friction_ratio;
    dyn_base_params[35] *= friction_ratio;
    dyn_base_params[46] *= friction_ratio;
    dyn_base_params[57] *= 0.5;
    dyn_base_params[68] *= 0.5;

    Eigen::VectorXd q_corrected = q_arm + q_offset;
    Eigen::VectorXd tau_out = Eigen::VectorXd::Zero(7);

    auto func = DynDispatcher::GetInvDyn(robot_model.params_ptr->robot_key, is_left);
    if (func) {
        func(tau_out.data(), q_corrected.data(), dq_arm.data(), ddq_arm.data(), dyn_base_params.data(), gravity_acc_armbase.data());
    } else {
        static bool warned = false;
        if (!warned) {
            std::cerr << "[ERROR] No symbolic inverse dynamics for " << robot_model.params_ptr->robot_type << " " << robot_model.params_ptr->robot_version << std::endl;
            warned = true;
        }
    }

    return tau_out;
}

Eigen::MatrixXd GetMassMatrixBySymbol(const RobotModel& robot_model, const Eigen::VectorXd& q_arm, MotionGroup motion_group)
{
    Eigen::VectorXd dyn_base_params = Eigen::VectorXd::Zero(71);
    Eigen::VectorXd q_offset = Eigen::VectorXd::Zero(7);
    bool is_left = (motion_group == MotionGroup::Primary);

    if (is_left)
    {
        assert(robot_model.params_ptr->dyn_base_params_primary.has_value());
        dyn_base_params = robot_model.params_ptr->dyn_base_params_primary.value();
        q_offset << 0.0, 0.0, -M_PI/2.0, 0.0, 0.0, -M_PI/2.0, 0.0;
    }
    else
    {
        assert(robot_model.params_ptr->dyn_base_params_secondary.has_value());
        dyn_base_params = robot_model.params_ptr->dyn_base_params_secondary.value();
        q_offset << 0.0, 0.0, M_PI/2.0, 0.0, 0.0, M_PI/2.0, 0.0;
    }

    dyn_base_params[5] = dyn_base_params[15] = dyn_base_params[26] = dyn_base_params[37] = dyn_base_params[48] = dyn_base_params[59] = dyn_base_params[70] = 0.0;

    auto func = DynDispatcher::GetMassTerm(robot_model.params_ptr->robot_key, is_left);
    Eigen::MatrixXd M_matrix = Eigen::MatrixXd::Zero(7, 7);
    Eigen::VectorXd q_corrected = q_arm + q_offset;

    if (func) {
        for (int col = 0; col < 7; ++col) {
            Eigen::VectorXd ddq_arm = Eigen::VectorXd::Zero(7);
            ddq_arm[col] = 1.0;
            Eigen::VectorXd col_out = Eigen::VectorXd::Zero(7);
            func(col_out.data(), q_corrected.data(), ddq_arm.data(), dyn_base_params.data());
            M_matrix.col(col) = col_out;
        }
    }

    return M_matrix;
}

}  // namespace dynamics
}  // namespace robot_server_lib
