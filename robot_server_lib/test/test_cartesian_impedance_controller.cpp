#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <chrono>
#include <cmath>

#include "cartesian_impedance_controller.h"
#include "utils.h"

using namespace robot_server_lib;
TEST(CartesianImpedanceControllerTest, TestCartesianImpedanceController) {
    std::filesystem::path home_dir = std::filesystem::path(std::getenv("HOME"));
    std::filesystem::create_directories(home_dir / "log");
    nanolog::initialize(nanolog::GuaranteedLogger(), (home_dir / "log/").string(), "robot_server", 10,
                        10);                          // 每个日志文件最大20MB，最多保留10个日志文件
    nanolog::set_log_level(nanolog::LogLevel::INFO);  // options: DEBUG, INFO, WARN, CRIT 高频过程数据打印设置为DEBUG

    const auto package_share_path = ament_index_cpp::get_package_share_directory("w1_server");
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot.cfg";
    if (!std::filesystem::exists(robot_cfg_path)) {
        std::cerr << "robot.cfg not found!" << std::endl;
        return;
    }
    RobotModel robot_model = utils::GetRobotModelFromConfig(robot_cfg_path);
    if (!robot_model.model_ptr) {
        std::cerr << "robot_model.model_ptr is null!" << std::endl;
        return;
    }
    pinocchio::Data data(*robot_model.model_ptr);

    dynamics::CartesianImpedanceController impedance_controller(robot_model, MotionGroup::Primary);

    Eigen::VectorXd feedback_q(7), feedback_dq(7), cmd_q(7), tau_command(7);
    Vector6d F_imp;
    feedback_q << 0.5, -1.2, 0.0, -1.0, 0.0, -0.5, -0.5;
    feedback_dq << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    cmd_q << 0.51, -1.2, 0.0, -1.0, 0.0, -0.5, -0.5;
    impedance_controller.UpdateTorqueCommand(feedback_q, feedback_dq, cmd_q, Eigen::VectorXd::Zero(7), tau_command,
                                             F_imp);
}