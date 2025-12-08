#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include "utils.h"
#include "dynamics_base.h"

using namespace robot_server_lib;
TEST(InverseDynamikTest, InverseDyn)
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
    VectorDof dq = Eigen::VectorXd::Constant(ndof_robot, 0.1);
    auto tau = dynamics::InverseDynamics(robot_model, data, VectorDof{}, dq, VectorDof{});
    std::cout << "Torque at left ee_link (q = 0): " << tau.segment(6, 7).transpose() << std::endl;
}