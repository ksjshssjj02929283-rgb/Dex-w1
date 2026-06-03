#include <math.h>
#include <stddef.h>
#include <iostream>
#include <string>
#include <unordered_map>
#include <cstdlib>
#include "inverse_dynamics.h"
#include "inverse_dynamics_symbol_generated/w1_v02/generated_include.h"
#include "inverse_dynamics_symbol_generated/w1_v03/generated_include.h"

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
using RegressionFunc = void (*)(double*, const double*, const double*, const double*, const double*, const double*);
using MassTermFunc = void (*)(double*, const double*, const double*, const double*);

struct RobotDynamics {
    RegressionFunc left_regression_func;
    RegressionFunc right_regression_func;
    MassTermFunc left_mass_term_func;
    MassTermFunc right_mass_term_func;
};

struct RobotDynamics {
    RegressionFunc left_regression_func;
    RegressionFunc right_regression_func;
    MassTermFunc left_mass_term_func;
    MassTermFunc right_mass_term_func;
};

struct DynDispatcher {
    static const std::unordered_map<std::string, RobotDynamics>& GetRegistry() {
        static const std::unordered_map<std::string, RobotDynamics> registry = {
            {"w1_v02", {
                w1_v02::left_regression_func, w1_v02::right_regression_func,
                w1_v02::left_mass_term_func, w1_v02::right_mass_term_func
            }},
            {"w1_v03", {
                w1_v03::left_regression_func, w1_v03::right_regression_func,
                w1_v03::left_mass_term_func, w1_v03::right_mass_term_func
            }}
        };
        return registry;
    }

    static RegressionFunc GetRegressionFunc(const std::string& robot_key, bool is_left) {
        auto it = GetRegistry().find(robot_key);
        if (it != GetRegistry().end()) {
            auto func = is_left ? it->second.left_regression_func : it->second.right_regression_func;
            if (!func) {
                std::cerr << "[ERROR] Null regression func pointer for robot_key: " << robot_key
                          << ", is_left: " << is_left << std::endl;
                LOG_CRIT << "Null regression func pointer for robot_key: " << robot_key
                         << ", is_left: " << is_left;
                std::exit(EXIT_FAILURE);
            }
            return func;
        }
        std::cerr << "[ERROR] No symbolic inverse dynamics for robot_key: " << robot_key << std::endl;
        std::exit(EXIT_FAILURE);
    }

    static MassTermFunc GetMassTermFunc(const std::string& robot_key, bool is_left) {
        auto it = GetRegistry().find(robot_key);
        if (it != GetRegistry().end()) {
            auto func = is_left ? it->second.left_mass_term_func : it->second.right_mass_term_func;
            if (!func) {
                std::cerr << "[ERROR] Null mass term func pointer for robot_key: " << robot_key
                          << ", is_left: " << is_left << std::endl;
                LOG_CRIT << "Null mass term func pointer for robot_key: " << robot_key
                         << ", is_left: " << is_left;
                std::exit(EXIT_FAILURE);
            }
            return func;
        }
        std::cerr << "[ERROR] No symbolic mass term func for robot_key: " << robot_key << std::endl;
        LOG_CRIT << "No symbolic mass term func for robot_key: " << robot_key;
        std::exit(EXIT_FAILURE);
    }
};

Eigen::VectorXd InvDynamikBySymbol(const RobotModel& robot_model, const VectorDof& q, const VectorDof& dq,
                                    const VectorDof& ddq, const Vector3d& gravity_acc_armbase, MotionGroup motion_group)
{
    Eigen::VectorXd dyn_base_params(71), q_arm(7), dq_arm(7), ddq_arm(7);
    utils::GetJointFromWholeBody(robot_model, q, q_arm, motion_group);
    utils::GetJointFromWholeBody(robot_model, dq, dq_arm, motion_group);
    utils::GetJointFromWholeBody(robot_model, ddq, ddq_arm, motion_group);
    
    bool is_left = (motion_group == MotionGroup::Primary);

    if (is_left)
    {
        if (!robot_model.params_ptr->dyn_base_params_primary.has_value()) {
            std::cerr << "[ERROR] dyn_base_params_primary has no value" << std::endl;
            LOG_CRIT << "dyn_base_params_primary has no value";
            std::exit(EXIT_FAILURE);
        }
        dyn_base_params = robot_model.params_ptr->dyn_base_params_primary.value();
    }
    else
    {
        if (!robot_model.params_ptr->dyn_base_params_secondary.has_value()) {
            std::cerr << "[ERROR] dyn_base_params_secondary has no value" << std::endl;
            LOG_CRIT << "dyn_base_params_secondary has no value";
            std::exit(EXIT_FAILURE);
        }
        dyn_base_params = robot_model.params_ptr->dyn_base_params_secondary.value();
    }

    const double friction_ratio = 0.6;
    dyn_base_params[3]  *= friction_ratio;
    dyn_base_params[13] *= friction_ratio;
    dyn_base_params[24] *= friction_ratio;
    dyn_base_params[35] *= friction_ratio;
    dyn_base_params[46] *= friction_ratio;
    dyn_base_params[57] *= 0.5;
    dyn_base_params[68] *= 0.5;

    Eigen::VectorXd tau_out = Eigen::VectorXd::Zero(7);

    auto func = DynDispatcher::GetRegressionFunc(robot_model.params_ptr->robot_key, is_left);
    func(tau_out.data(), q_arm.data(), dq_arm.data(), ddq_arm.data(), dyn_base_params.data(), gravity_acc_armbase.data());

    return tau_out;
}

Eigen::MatrixXd GetMassMatrixBySymbol(const RobotModel& robot_model, const Eigen::VectorXd& q_arm, MotionGroup motion_group)
{
    Eigen::VectorXd dyn_base_params = Eigen::VectorXd::Zero(71);
    bool is_left = (motion_group == MotionGroup::Primary);

    if (is_left)
    {
        if (!robot_model.params_ptr->dyn_base_params_primary.has_value()) {
            std::cerr << "[ERROR] dyn_base_params_primary has no value" << std::endl;
            std::exit(EXIT_FAILURE);
        }
        dyn_base_params = robot_model.params_ptr->dyn_base_params_primary.value();
    }
    else
    {
        if (!robot_model.params_ptr->dyn_base_params_secondary.has_value()) {
            std::cerr << "[ERROR] dyn_base_params_secondary has no value" << std::endl;
            std::exit(EXIT_FAILURE);
        }
        dyn_base_params = robot_model.params_ptr->dyn_base_params_secondary.value();
    }

    dyn_base_params[5] = dyn_base_params[15] = dyn_base_params[26] = dyn_base_params[37] = dyn_base_params[48] = dyn_base_params[59] = dyn_base_params[70] = 0.0;

    auto func = DynDispatcher::GetMassTermFunc(robot_model.params_ptr->robot_key, is_left);
    Eigen::MatrixXd M_matrix = Eigen::MatrixXd::Zero(7, 7);

    for (int col = 0; col < 7; ++col) {
        Eigen::VectorXd ddq_arm = Eigen::VectorXd::Zero(7);
        ddq_arm[col] = 1.0;
        Eigen::VectorXd col_out = Eigen::VectorXd::Zero(7);
        func(col_out.data(), q_arm.data(), ddq_arm.data(), dyn_base_params.data());
        M_matrix.col(col) = col_out;
    }

    Eigen::MatrixXd asym = M_matrix - M_matrix.transpose();
    if (asym.norm() > FloatZeroTol) {
        LOG_CRIT << "Mass matrix is not symmetric! norm: " << asym.norm();
        std::exit(EXIT_FAILURE);
    }

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(M_matrix);
    if (es.info() != Eigen::Success) {
        LOG_CRIT << "Mass matrix eigenvalue decomposition failed!";
        std::exit(EXIT_FAILURE);
    } else {
        double min_eig = es.eigenvalues().minCoeff();
        if (min_eig <= FloatZeroTol) {
            LOG_CRIT << "Mass matrix is not positive definite! min eigenvalue: " << min_eig;
            std::exit(EXIT_FAILURE);
        }
    }

    return M_matrix;
}

}  // namespace dynamics
}  // namespace robot_server_lib
