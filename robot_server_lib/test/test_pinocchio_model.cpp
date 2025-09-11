#include <gtest/gtest.h>

#include "utils.h"

using namespace robot_server_lib;
TEST(PinocchioModelTest, Pinocchio)
{
    const auto package_share_path = ament_index_cpp::get_package_share_directory("sim_server");
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot.cfg";
    RobotModel robot_model = utils::GetRobotModelFromConfig(robot_cfg_path);

    auto print_model_info = [](const pinocchio::Model& model, const std::string& tag)
    {
        std::cout << "==== " << tag << " ====" << std::endl;
        std::cout << "Number of joints: " << model.nv << std::endl;
        for (int idx = 1; idx <= model.nv; ++idx)
        {
            std::cout << "Joint " << idx << ": " << model.names[idx];
            if (model.joints[idx].nq() > 0)
            {
                std::cout << ", parent link: " << model.frames[model.joints[idx].idx_q()].name;
            }
            std::cout << std::endl;
        }
    };

    print_model_info(*robot_model.model_ptr, "Full Model");
    print_model_info(*robot_model.left_arm_model_ptr, "Left Arm Model");
    print_model_info(*robot_model.right_arm_model_ptr, "Right Arm Model");
}