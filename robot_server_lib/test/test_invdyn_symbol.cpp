#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include "utils.h"
#include "dynamics_base.h"
#include <chrono>

using namespace robot_server_lib;
TEST(InvDynSymbolTest, InvDynSymbol)
{
    std::filesystem::path server_data_dir_ = std::filesystem::path(std::getenv("HOME")) / "dexe_robot_server_data";
    std::filesystem::create_directories(server_data_dir_ / "logs");
    nanolog::initialize(nanolog::GuaranteedLogger(), (server_data_dir_ / "logs/").string(), "robot_server", 20,
                        10);                          // 每个日志文件最大20MB，最多保留10个日志文件
    nanolog::set_log_level(nanolog::LogLevel::INFO);  // options: DEBUG, INFO, WARN, CRIT 高频过程数据打印设置为DEBUG

    const auto package_share_path = ament_index_cpp::get_package_share_directory("w1_server");
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot.cfg";
    RobotModel robot_model = utils::GetRobotModelFromConfig(robot_cfg_path);
    pinocchio::Data data(*robot_model.model_ptr);

    auto t0 = std::chrono::high_resolution_clock::now();
    Eigen::VectorXd tau = dynamics::InvDynamikBySymbol(robot_model, data, VectorDof{}, VectorDof{}, VectorDof{}, MotionGroup::Primary);
    auto t1 = std::chrono::high_resolution_clock::now();
    double us = std::chrono::duration_cast<std::chrono::duration<double, std::micro>>(t1 - t0).count();
    std::cout << "InvDynamikBySymbol time: " << us << " us" << std::endl;
    std::cout << "Torque at left ee_link (q = 0): " << tau.transpose() << std::endl;
    return;
}