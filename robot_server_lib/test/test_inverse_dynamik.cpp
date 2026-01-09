#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include "utils.h"
#include "inverse_dynamics.h"

using namespace robot_server_lib;
TEST(InverseDynamikTest, InverseDyn)
{
    std::filesystem::path home_dir = std::filesystem::path(std::getenv("HOME"));
    std::filesystem::create_directories(home_dir / "log");
    nanolog::initialize(nanolog::GuaranteedLogger(), (home_dir / "log/").string(), "robot_server", 10,
                        10);                          // 每个日志文件最大20MB，最多保留10个日志文件
    nanolog::set_log_level(nanolog::LogLevel::INFO);  // options: DEBUG, INFO, WARN, CRIT 高频过程数据打印设置为DEBUG
    const auto package_share_path = ament_index_cpp::get_package_share_directory("w1_server");
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot.cfg";
    RobotModel robot_model = utils::GetRobotModelFromConfig(robot_cfg_path);
    Eigen::VectorXd q(robot_model.model_ptr->nq);
    q << 0.5, -1.0, 0.5, 0.0, 0.0, 0.0,
         0.5, -1.2, 0.0, -1.0, 0.0, -0.5, -0.5,
         -0.5, 1.2, 0.0, 1.0, 0.0, 0.5, 0.5;
    VectorDof dq = VectorDof::Zero();
    VectorDof ddq = VectorDof::Zero();
    auto tau = dynamics::InverseDynamics(robot_model, q, dq, ddq);
    std::cout << "Torque output: "<< tau.transpose() << ", when q: "<< q.transpose() << std::endl;
}