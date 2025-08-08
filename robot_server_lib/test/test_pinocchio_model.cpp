#include <gtest/gtest.h>
#include "utilities.h"

using namespace robot_server_lib;
TEST(PinocchioModelTest, Pinocchio)
{
    const auto package_share_path = ament_index_cpp::get_package_share_directory("sim_server");
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot.cfg";
    json config_json;
    if (!std::filesystem::exists(robot_cfg_path))
    {
        assert(false && "Robot configuration file not found");
    }
    std::ifstream ifs(robot_cfg_path);
    if (!ifs.is_open() || !(ifs >> config_json))
    {
        assert(false && "Can't open or parse robot.cfg");
    }
    // 初始化机器人模型
    RobotParams params;
    if (!utilities::GetRobotParamsFromConfig(config_json, params))
    {
        assert(false);
    }

    pinocchio::Model model, left_arm_model, right_arm_model;
    pinocchio::urdf::buildModel(params.urdf_path, model);
    pinocchio::urdf::buildModel(params.left_arm_urdf_path, left_arm_model);
    pinocchio::urdf::buildModel(params.right_arm_urdf_path, right_arm_model);

    auto print_model_info = [](const pinocchio::Model& model, const std::string& tag) {
        std::cout << "==== " << tag << " ====" << std::endl;
        std::cout << "Number of joints: " << model.nv << std::endl;
        for (int idx = 1; idx <= model.nv; ++idx) {
            std::cout << "Joint " << idx << ": " << model.names[idx];
            if (model.joints[idx].nq() > 0) {
                std::cout << ", parent link: " << model.frames[model.joints[idx].idx_q()].name;
            }
            std::cout << std::endl;
        }
    };

    print_model_info(model, "Full Model");
    print_model_info(left_arm_model, "Left Arm Model");
    print_model_info(right_arm_model, "Right Arm Model");
}