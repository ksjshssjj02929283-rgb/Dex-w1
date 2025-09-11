#include <gtest/gtest.h>

#include "utils.h"

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
    const auto package_share_path = ament_index_cpp::get_package_share_directory("sim_server");
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot.cfg";
    RobotModel robot_model = utils::GetRobotModelFromConfig(robot_cfg_path);

    PrintModelInfo(*robot_model.left_arm_model_ptr, "Left Arm Model");
    PrintModelInfo(*robot_model.right_arm_model_ptr, "Right Arm Model");

    // 计算7个关节全0时的Jacobi矩阵
    pinocchio::Data data(*robot_model.left_arm_model_ptr);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(robot_model.left_arm_model_ptr->nv);

    auto J = pinocchio::GetJacobiMatrix(*robot_model.left_arm_model_ptr, data, q);
    std::cout << std::fixed << std::setprecision(3) << "Jacobian at left ee_link (q = 0):\n" << J << std::endl;
}