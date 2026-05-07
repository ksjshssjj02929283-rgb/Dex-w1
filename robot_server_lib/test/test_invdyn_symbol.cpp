#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <chrono>
#include <cmath>

#include "inverse_dynamics.h"
#include "utils.h"

using namespace robot_server_lib;
TEST(InvDynSymbolTest, InvDynSymbol) {
    std::filesystem::path home_dir = std::filesystem::path(std::getenv("HOME"));
    std::filesystem::create_directories(home_dir / "log");
    nanolog::initialize(nanolog::GuaranteedLogger(), (home_dir / "log/").string(), "robot_server", 10,
                        10);                          // 每个日志文件最大20MB，最多保留10个日志文件
    nanolog::set_log_level(nanolog::LogLevel::INFO);  // options: DEBUG, INFO, WARN, CRIT 高频过程数据打印设置为DEBUG

    const auto package_share_path = ament_index_cpp::get_package_share_directory("w1_server");
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot_config.yaml";
    if (!std::filesystem::exists(robot_cfg_path)) {
        std::cerr << "robot_config.yaml not found!" << std::endl;
        return;
    }
    RobotModel robot_model = utils::GetRobotModelFromConfig(robot_cfg_path);
    if (!robot_model.model_ptr) {
        std::cerr << "robot_model.model_ptr is null!" << std::endl;
        return;
    }
    pinocchio::Data data(*robot_model.model_ptr);
    Eigen::VectorXd q(robot_model.model_ptr->nq);
    q << 0.5, -1.0, 0.5, 0.0, 0.0, 0.0, 0.5, -1.2, 0.0, -1.0, 0.0, -0.5, -0.5, -0.5, 1.2, 0.0, 1.0, 0.0, 0.5, 0.5;
    Eigen::VectorXd q_zero = Eigen::VectorXd::Zero(robot_model.model_ptr->nq);

    // 计算重力方向在arm base主/副坐标系下的表示
    auto t0 = std::chrono::high_resolution_clock::now();

    // 主臂
    Vector3d gravity_acc_armbase_primary = Vector3d{-9.81, 0.0, 0.0};

    // 副臂
    Vector3d gravity_acc_armbase_secondary = Vector3d{9.81, 0.0, 0.0};

    Eigen::VectorXd tau_primary = dynamics::InvDynamikBySymbol(robot_model, q, VectorDof::Zero(), VectorDof::Zero(),
                                                               gravity_acc_armbase_primary, MotionGroup::Primary);
    Eigen::VectorXd tau_secondary = dynamics::InvDynamikBySymbol(robot_model, q, VectorDof::Zero(), VectorDof::Zero(),
                                                                 gravity_acc_armbase_secondary, MotionGroup::Secondary);

    Eigen::VectorXd tau_primary_zero = dynamics::InvDynamikBySymbol(
        robot_model, q_zero, VectorDof::Zero(), VectorDof::Zero(), gravity_acc_armbase_primary, MotionGroup::Primary);
    Eigen::VectorXd tau_secondary_zero =
        dynamics::InvDynamikBySymbol(robot_model, q_zero, VectorDof::Zero(), VectorDof::Zero(),
                                     gravity_acc_armbase_secondary, MotionGroup::Secondary);
    auto t1 = std::chrono::high_resolution_clock::now();
    double us = std::chrono::duration_cast<std::chrono::duration<double, std::micro>>(t1 - t0).count();
    std::cout << "InvDynamikBySymbol time: " << us << " us" << std::endl;
    std::cout << "Torque at left ee_link: " << tau_primary.transpose() << std::endl;
    std::cout << "Torque at right ee_link: " << tau_secondary.transpose() << std::endl;
    std::cout << "Torque at left ee_link (zero): " << tau_primary_zero.transpose() << std::endl;
    std::cout << "Torque at right ee_link (zero): " << tau_secondary_zero.transpose() << std::endl;

    Eigen::VectorXd q_primary, q_secondary;
    utils::GetJointFromWholeBody(robot_model, q, q_primary, MotionGroup::Primary);
    utils::GetJointFromWholeBody(robot_model, q, q_secondary, MotionGroup::Secondary);

    Eigen::MatrixXd m_matrix_primary = dynamics::GetMassMatrixBySymbol(robot_model, q_primary, MotionGroup::Primary);
    Eigen::MatrixXd m_matrix_secondary =
        dynamics::GetMassMatrixBySymbol(robot_model, q_secondary, MotionGroup::Secondary);
    std::cout << "Mass matrix at left arm: \n" << m_matrix_primary << std::endl;
    std::cout << "Mass matrix at right arm: \n" << m_matrix_secondary << std::endl;

    // mass 矩阵正定性检测
    Eigen::LLT<Eigen::MatrixXd> lltOfM_primary(m_matrix_primary);  // compute the Cholesky decomposition of M
    if (lltOfM_primary.info() == Eigen::NumericalIssue) {
        std::cerr << "The mass matrix of primary arm is not positive definite!" << std::endl;
    }
    Eigen::LLT<Eigen::MatrixXd> lltOfM_secondary(m_matrix_secondary);  // compute the Cholesky decomposition of M
    if (lltOfM_secondary.info() == Eigen::NumericalIssue) {
        std::cerr << "The mass matrix of secondary arm is not positive definite!" << std::endl;
    }
    return;
}