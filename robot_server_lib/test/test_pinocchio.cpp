#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include "utils.h"
#include "NanoLog.hpp"

void PrintModelInfo(const pinocchio::Model& model, const std::string& tag)
{
    std::cout << "==== " << tag << " ====" << std::endl;
    std::cout << "Number of joints: " << model.nv << std::endl;
    for (int idx = 1; idx <= model.nv; ++idx)
    {
        std::cout << "Joint " << idx << ": " << model.names[idx] << std::endl;
    }
}

using namespace robot_server_lib;
TEST(PinocchioModelTest, Pinocchio)
{
    std::filesystem::path server_data_dir_ = std::filesystem::path(std::getenv("HOME")) / "dexe_robot_server_data";
    std::filesystem::create_directories(server_data_dir_ / "logs");
    nanolog::initialize(nanolog::GuaranteedLogger(), (server_data_dir_ / "logs/").string(), "robot_server", 20,
                        10);                          // 每个日志文件最大20MB，最多保留10个日志文件
    nanolog::set_log_level(nanolog::LogLevel::INFO);  // options: DEBUG, INFO, WARN, CRIT 高频过程数据打印设置为DEBUG
    const auto package_share_path = ament_index_cpp::get_package_share_directory("w1_server");
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot.cfg";
    RobotModel robot_model = utils::GetRobotModelFromConfig(robot_cfg_path);

    PrintModelInfo(*robot_model.primary_arm_model_ptr, "Left Arm Model");
    PrintModelInfo(*robot_model.secondary_arm_model_ptr, "Right Arm Model");

    // 计算7个关节全0时的Jacobi矩阵
    Eigen::VectorXd q = Eigen::VectorXd::Zero(robot_model.primary_arm_model_ptr->nv);

    auto J = pinocchio::GetJacobiMatrix(*robot_model.primary_arm_model_ptr, q);
    std::cout << std::fixed << std::setprecision(3) << "Jacobian at left ee_link (q = 0):\n" << J << std::endl;

    Eigen::MatrixXd expected(6, 7);
    expected << 0.000, 0.590, 0.000, -0.000, 0.000, -0.000, 0.066, -0.000, -0.000, -0.000, -0.330, 0.000, -0.066, -0.000, 0.000, 0.000, -0.000,
        -0.000, 0.000, -0.000, 0.000, 0.000, 0.000, -0.000, 1.000, -0.000, 1.000, 0.000, 0.000, 1.000, -0.000, -0.000, -0.000, -0.000, 1.000, 1.000,
        0.000, 1.000, 0.000, 1.000, 0.000, 0.000;

    const double tol = 1e-3;
    double max_abs_diff = (J - expected).cwiseAbs().maxCoeff();
    EXPECT_LE(max_abs_diff, tol) << "Max abs diff = " << max_abs_diff << ", tolerance = " << tol;

    pinocchio::Data data(*robot_model.model_ptr);
    Eigen::VectorXd q1(robot_model.model_ptr->nq);
    q1 << 0.5, -1.0, 0.5, 0.0, 0.5, -1.2, 0.0, -1.0, 0.0, -0.5, -0.5, -0.5, 1.2, 0.0, 1.0, 0.0, 0.5, 0.5, 0.0, 0.0;
    pinocchio::forwardKinematics(*robot_model.model_ptr, data, q1);
    pinocchio::updateFramePlacements(*robot_model.model_ptr, data);

    for (int fid = 0; fid < robot_model.model_ptr->nframes; ++fid)
    {
        const auto &frame = robot_model.model_ptr->frames[fid];
        const auto &pose  = data.oMf[fid];  // world->frame pose
        std::cout << "Frame " << fid << " (" << frame.name << ") pose:\n" << pose << std::endl;

        Eigen::Vector3d g_world(0.0, 0.0, -9.81);
        Eigen::Vector3d g_local = pose.rotation().transpose() * g_world;
        std::cout << "Gravity in frame " << fid << " (" << frame.name << ") : "
                  << g_local.transpose() << std::endl;
    }
}