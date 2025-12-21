#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include "utils.h"
#include "inverse_dynamics.h"
#include <chrono>

using namespace robot_server_lib;
TEST(InvDynSymbolTest, InvDynSymbol)
{
    std::filesystem::path server_data_dir_ = std::filesystem::path(std::getenv("HOME")) / "dexe_robot_server_data";
    std::filesystem::create_directories(server_data_dir_ / "logs");
    nanolog::initialize(nanolog::GuaranteedLogger(), (server_data_dir_ / "logs/").string(), "robot_server", 20, 10);                          // 每个日志文件最大20MB，最多保留10个日志文件
    nanolog::set_log_level(nanolog::LogLevel::INFO);  // options: DEBUG, INFO, WARN, CRIT 高频过程数据打印设置为DEBUG

    const auto package_share_path = ament_index_cpp::get_package_share_directory("w1_server");
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot.cfg";
    if (!std::filesystem::exists(robot_cfg_path))
    {
        std::cerr << "robot.cfg not found!" << std::endl;
        return;
    }
    RobotModel robot_model = utils::GetRobotModelFromConfig(robot_cfg_path);
    if (!robot_model.model_ptr)
    {
        std::cerr << "robot_model.model_ptr is null!" << std::endl;
        return;
    }
    pinocchio::Data data(*robot_model.model_ptr);
    Eigen::VectorXd q_urdf(robot_model.model_ptr->nq);
    q_urdf << 0.5, -1.0, 0.5, 0.0,
         0.5, -1.2, 0.0, -1.0, 0.0, -0.5, -0.5,
         -0.5, 1.2, 0.0, 1.0, 0.0, 0.5, 0.5,
         0.0, 0.0;

    // 计算重力方向在arm base主/副坐标系下的表示
    auto t0 = std::chrono::high_resolution_clock::now();

    // 主臂
    Vector3d gravity_acc_armbase_primary = Vector3d{-9.81, 0.0, 0.0};

    // 副臂
    Vector3d gravity_acc_armbase_secondary = Vector3d{9.81, 0.0, 0.0};

    Eigen::VectorXd tau_primary = dynamics::InvDynamikBySymbol(robot_model, q_urdf, VectorDof::Zero(), VectorDof::Zero(), gravity_acc_armbase_primary, MotionGroup::Primary);
    Eigen::VectorXd tau_secondary = dynamics::InvDynamikBySymbol(robot_model, q_urdf, VectorDof::Zero(), VectorDof::Zero(), gravity_acc_armbase_secondary, MotionGroup::Secondary);

    auto t1 = std::chrono::high_resolution_clock::now();
    double us = std::chrono::duration_cast<std::chrono::duration<double, std::micro>>(t1 - t0).count();
    std::cout << "InvDynamikBySymbol time: " << us << " us" << std::endl;
    std::cout << "Torque at left ee_link: " << tau_primary.transpose() << std::endl;
    std::cout << "Torque at right ee_link: " << tau_secondary.transpose() << std::endl;
    return;
}