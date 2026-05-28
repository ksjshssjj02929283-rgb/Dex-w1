#include <gtest/gtest.h>

#include <iostream>

#include "NanoLog.hpp"
#include "data_structure.h"
#include "utils.h"

using namespace robot_server_lib;

TEST(ModelAnalysisTest, PrintAllJoints) {
    const auto package_share_path = std::filesystem::path(std::getenv("HOME")) / "dexe_robot_server_data";
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot_config.yaml";
    RobotModel robot_model = utils::GetRobotModelFromConfig(robot_cfg_path);
    const std::string& robot_version = robot_model.params_ptr->robot_version;

    std::cout << "\n\n========================================" << std::endl;
    std::cout << "=== Model Joint Analysis, Version: " << robot_version << " ===" << std::endl;
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

    std::cout << "\n7. Position limit, robot version: " << robot_version << std::endl;
    std::cout << " urdf position lower limit = \n" << robot_model.model_ptr->lowerPositionLimit << std::endl;
    std::cout << " urdf position upper limit = \n" << robot_model.model_ptr->upperPositionLimit << std::endl;
    std::cout << "\n========================================\n" << std::endl;
    Eigen::VectorXd expected_lower_limits(20), expected_upper_limits(20);

    if (robot_version == "v023") {
        expected_lower_limits << -1.5708, -2.70526, -1.91986, -2.96706, -1.5708, -0.785398, -2.96706, -2.0944, -2.96706,
            -2.35619, -2.96706, -0.785398, -1.5708, -2.96706, -1.5708, -2.96706, -1.5708, -2.96706, -0.785398, -1.0472;
        expected_upper_limits << 1.5708, 2.70526, 1.5708, 2.96706, 1.5708, 0.436332, 2.96706, 1.5708, 2.96706, 1.5708,
            2.96706, 0.785398, 1.0472, 2.96706, 2.0944, 2.96706, 2.35619, 2.96706, 0.7854, 1.5708;
    } else if (robot_version == "v024") {
        expected_lower_limits << -1.5708, -2.70526, -1.91986, -3.75246, -1.5708, -0.785398, -2.96706, -2.0944, -2.96706,
            -2.35619, -2.96706, -0.785398, -1.5708, -2.96706, -1.5708, -2.96706, -1.5708, -2.96706, -0.785398, -1.0472;
        expected_upper_limits << 1.5708, 2.70526, 1.5708, 2.1817, 1.5708, 0.436332, 2.96706, 1.5708, 2.96706, 1.5708,
            2.96706, 0.785398, 1.0472, 2.96706, 2.0944, 2.96706, 2.35619, 2.96706, 0.7854, 1.5708;
    } else if (robot_version == "v030") {
        expected_lower_limits << -1.5708, -2.70526, -1.5708, -3.75246, -1.5708, -0.698132, -2.96706, -2.0944, -2.96706,
            -2.58309, -2.96706, -1.65806, -1.74533, -2.96706, -1.5708, -2.96706, -1.5708, -2.96706, -1.65806, -1.74533;
        expected_upper_limits << 1.5708, 2.70526, 1.91986, 2.18166, 1.5708, 0.7854, 2.96706, 1.5708, 2.96706, 1.5708,
            2.96706, 1.65806, 1.74533, 2.96706, 2.0944, 2.96706, 2.58309, 2.96706, 1.65806, 1.74533;
    } else {
        FAIL() << "Unknown robot version: " << robot_version;
    }
    EXPECT_TRUE(robot_model.model_ptr->lowerPositionLimit.isApprox(expected_lower_limits, 1e-4));
    EXPECT_TRUE(robot_model.model_ptr->upperPositionLimit.isApprox(expected_upper_limits, 1e-4));
}

TEST(ModelAnalysisTest, PrimaryArmModelTest) {
    const auto package_share_path = std::filesystem::path(std::getenv("HOME")) / "dexe_robot_server_data";
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot_config.yaml";
    RobotModel robot_model = utils::GetRobotModelFromConfig(robot_cfg_path);

    std::cout << "\n\n========================================" << std::endl;
    std::cout << "=== Primary Arm Model Analysis ===" << std::endl;
    std::cout << "========================================" << std::endl;

    if (robot_model.primary_arm_model_ptr) {
        const auto& model = *robot_model.primary_arm_model_ptr;
        std::cout << "Primary Arm Model Name: " << model.name << std::endl;
        std::cout << "Number of joints (model.nv): " << model.nv << std::endl;
        std::cout << "Number of frames (model.nframes): " << model.nframes << std::endl;

        std::cout << "\nJoint Names:" << std::endl;
        for (int i = 0; i < model.njoints; ++i) {
            std::cout << "   [" << i << "] " << model.names[i] << std::endl;
        }

        std::cout << "\nFrame Names (First and Last):" << std::endl;
        if (model.nframes > 0) {
            std::cout << "   [0] " << model.frames[0].name << std::endl;
            std::cout << "   [" << model.nframes - 1 << "] " << model.frames[model.nframes - 1].name << std::endl;
        }
    } else {
        std::cout << "Primary Arm Model Pointer is NULL!" << std::endl;
    }

    std::cout << "\n========================================\n" << std::endl;
    EXPECT_NE(robot_model.primary_arm_model_ptr, nullptr);
}

int main(int argc, char** argv) {
    const auto home_dir = std::filesystem::path(std::getenv("HOME"));
    nanolog::initialize(nanolog::GuaranteedLogger(), (home_dir / "log/").string(), "test_urdf_model", 1, 1);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
