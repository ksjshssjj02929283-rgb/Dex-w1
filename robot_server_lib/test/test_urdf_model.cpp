#include <gtest/gtest.h>

#include <iostream>

#include "data_structure.h"
#include "utils.h"

using namespace robot_server_lib;

TEST(ModelAnalysisTest, PrintAllJoints) {
    const auto package_share_path = ament_index_cpp::get_package_share_directory("w1_server");
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot_config.yaml";
    RobotModel robot_model = utils::GetRobotModelFromConfig(robot_cfg_path);

    std::cout << "\n\n========================================" << std::endl;
    std::cout << "=== Model Joint Analysis ===" << std::endl;
    std::cout << "========================================" << std::endl;

    std::cout << "\n1. Pinocchio model.names (URDF order, same as control order):" << std::endl;
    const auto& model_names = robot_model.model_ptr->names;
    for (size_t i = 0; i < model_names.size(); ++i) {
        std::cout << "   [" << i << "] " << model_names[i] << std::endl;
    }

    std::cout << "\n2. joints_primary_names:" << std::endl;
    for (size_t i = 0; i < robot_model.params_ptr->joints_primary_names.size(); ++i) {
        std::cout << "   [" << i << "] " << robot_model.params_ptr->joints_primary_names[i] << std::endl;
    }

    std::cout << "\n3. joints_secondary_names:" << std::endl;
    for (size_t i = 0; i < robot_model.params_ptr->joints_secondary_names.size(); ++i) {
        std::cout << "   [" << i << "] " << robot_model.params_ptr->joints_secondary_names[i] << std::endl;
    }

    std::cout << "\n4. joints_torso_names:" << std::endl;
    for (size_t i = 0; i < robot_model.params_ptr->joints_torso_names.size(); ++i) {
        std::cout << "   [" << i << "] " << robot_model.params_ptr->joints_torso_names[i] << std::endl;
    }

    std::cout << "\n5. joint_names (all joints from model.names, excluding universe):" << std::endl;
    for (size_t i = 0; i < robot_model.params_ptr->joint_names.size(); ++i) {
        std::cout << "   [" << i << "] " << robot_model.params_ptr->joint_names[i] << std::endl;
    }

    std::cout << "\n6. Statistics:" << std::endl;
    std::cout << "   model.nq = " << robot_model.model_ptr->nq << std::endl;
    std::cout << "   model.nv = " << robot_model.model_ptr->nv << std::endl;
    std::cout << "   model.names.size = " << model_names.size() << std::endl;
    std::cout << "   joint_names.size = " << robot_model.params_ptr->joint_names.size() << std::endl;
    std::cout << "   primary + secondary + torso = " << robot_model.params_ptr->joints_primary_names.size() << " + "
              << robot_model.params_ptr->joints_secondary_names.size() << " + "
              << robot_model.params_ptr->joints_torso_names.size() << " = "
              << (robot_model.params_ptr->joints_primary_names.size() +
                  robot_model.params_ptr->joints_secondary_names.size() +
                  robot_model.params_ptr->joints_torso_names.size())
              << std::endl;

    std::cout << "\n========================================\n" << std::endl;

    EXPECT_TRUE(true);
}
