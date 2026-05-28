/**
 * @file test_joint_mapping.cpp
 * @brief Joint Mapping Functions 单元测试
 * @copyright Copyright (C) 2025 DexForce Technology Co., Ltd. All Rights Reserved.
 */

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <iostream>

#include "data_structure.h"
#include "utils.h"

using namespace robot_server_lib;

TEST(JointMappingTest, MapJointToWholeBody_Primary) {
    std::cout << "\n=== MapJointToWholeBody_Primary Test ===" << std::endl;
    const auto package_share_path = std::filesystem::path(std::getenv("HOME")) / "dexe_robot_server_data";
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot_config.yaml";
    RobotModel robot_model = utils::GetRobotModelFromConfig(robot_cfg_path);

    std::cout << "Robot: " << robot_model.params_ptr->robot_type << std::endl;
    std::cout << "Primary joints: " << robot_model.params_ptr->joints_primary_names.size() << std::endl;
    std::cout << "Joint names: ";
    for (const auto& name : robot_model.params_ptr->joints_primary_names) {
        std::cout << name << " ";
    }
    std::cout << std::endl;

    Eigen::VectorXd q_primary(7);
    q_primary << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
    std::cout << "Input q_primary: " << q_primary.transpose() << std::endl;

    JointPositionDof q_ndof = JointPositionDof::Zero();

    bool result = utils::MapJointToWholeBody(robot_model, q_primary, q_ndof, MotionGroup::Primary);

    EXPECT_TRUE(result);

    auto expected_names = robot_model.params_ptr->joints_primary_names;
    std::cout << "\nMapping results:" << std::endl;
    for (int i = 0; i < 7; ++i) {
        int joint_idx = robot_model.model_ptr->getJointId(expected_names[i]) - 1;
        std::cout << "  " << expected_names[i] << " -> index " << joint_idx << ": " << q_primary[i] << std::endl;
    }

    std::cout << "\nFull q_ndof vector (non-zero only):" << std::endl;
    int non_zero_count = 0;
    for (int i = 0; i < ndof_robot; ++i) {
        if (std::abs(q_ndof[i]) > 1e-6) {
            non_zero_count++;
            std::cout << "  q_ndof[" << i << "] = " << q_ndof[i] << std::endl;
        }
    }

    EXPECT_EQ(non_zero_count, 7) << "Should have exactly 7 non-zero joints for primary arm";

    for (int i = 0; i < 7; ++i) {
        int joint_idx = robot_model.model_ptr->getJointId(expected_names[i]) - 1;
        EXPECT_NEAR(q_ndof[joint_idx], q_primary[i], 1e-6)
            << "Joint " << i << " (" << expected_names[i] << ") value mismatch at index " << joint_idx;
    }

    std::cout << "\n✓ MapJointToWholeBody_Primary test passed!" << std::endl;
}

TEST(JointMappingTest, MapJointToWholeBody_Secondary) {
    std::cout << "\n=== MapJointToWholeBody_Secondary Test ===" << std::endl;
    const auto package_share_path = ament_index_cpp::get_package_share_directory("w1_server");
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot_config.yaml";
    RobotModel robot_model = utils::GetRobotModelFromConfig(robot_cfg_path);

    std::cout << "Robot: " << robot_model.params_ptr->robot_type << std::endl;
    std::cout << "Secondary joints: " << robot_model.params_ptr->joints_secondary_names.size() << std::endl;
    std::cout << "Joint names: ";
    for (const auto& name : robot_model.params_ptr->joints_secondary_names) {
        std::cout << name << " ";
    }
    std::cout << std::endl;

    Eigen::VectorXd q_secondary(7);
    q_secondary << 0.11, 0.22, 0.33, 0.44, 0.55, 0.66, 0.77;
    std::cout << "Input q_secondary: " << q_secondary.transpose() << std::endl;

    JointPositionDof q_ndof = JointPositionDof::Zero();

    bool result = utils::MapJointToWholeBody(robot_model, q_secondary, q_ndof, MotionGroup::Secondary);

    EXPECT_TRUE(result);

    auto expected_names = robot_model.params_ptr->joints_secondary_names;
    std::cout << "\nMapping results:" << std::endl;
    for (int i = 0; i < 7; ++i) {
        int joint_idx = robot_model.model_ptr->getJointId(expected_names[i]) - 1;
        std::cout << "  " << expected_names[i] << " -> index " << joint_idx << ": " << q_secondary[i] << std::endl;
    }

    std::cout << "\nFull q_ndof vector (non-zero only):" << std::endl;
    int non_zero_count = 0;
    for (int i = 0; i < ndof_robot; ++i) {
        if (std::abs(q_ndof[i]) > 1e-6) {
            non_zero_count++;
            std::cout << "  q_ndof[" << i << "] = " << q_ndof[i] << std::endl;
        }
    }

    EXPECT_EQ(non_zero_count, 7) << "Should have exactly 7 non-zero joints for secondary arm";

    for (int i = 0; i < 7; ++i) {
        int joint_idx = robot_model.model_ptr->getJointId(expected_names[i]) - 1;
        EXPECT_NEAR(q_ndof[joint_idx], q_secondary[i], 1e-6)
            << "Joint " << i << " (" << expected_names[i] << ") value mismatch at index " << joint_idx;
    }

    std::cout << "\n✓ MapJointToWholeBody_Secondary test passed!" << std::endl;
}

TEST(JointMappingTest, MapJointToWholeBody_BothArms) {
    std::cout << "\n=== MapJointToWholeBody_BothArms Test ===" << std::endl;
    const auto package_share_path = ament_index_cpp::get_package_share_directory("w1_server");
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot_config.yaml";
    RobotModel robot_model = utils::GetRobotModelFromConfig(robot_cfg_path);

    std::cout << "Robot: " << robot_model.params_ptr->robot_type << std::endl;
    std::cout << "Primary joints: " << robot_model.params_ptr->joints_primary_names.size() << std::endl;
    std::cout << "Secondary joints: " << robot_model.params_ptr->joints_secondary_names.size() << std::endl;

    Eigen::VectorXd q_both(14);
    q_both << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.11, 0.22, 0.33, 0.44, 0.55, 0.66, 0.77;
    std::cout << "Input q_both (Primary 7 + Secondary 7): " << q_both.transpose() << std::endl;

    JointPositionDof q_ndof = JointPositionDof::Zero();

    bool result = utils::MapJointToWholeBody(robot_model, q_both, q_ndof, MotionGroup::BothArms);

    EXPECT_TRUE(result);

    auto primary_names = robot_model.params_ptr->joints_primary_names;
    auto secondary_names = robot_model.params_ptr->joints_secondary_names;

    std::cout << "\nPrimary arm mapping:" << std::endl;
    for (int i = 0; i < 7; ++i) {
        int joint_idx = robot_model.model_ptr->getJointId(primary_names[i]) - 1;
        std::cout << "  " << primary_names[i] << " -> index " << joint_idx << ": " << q_both[i] << std::endl;
    }

    std::cout << "\nSecondary arm mapping:" << std::endl;
    for (int i = 0; i < 7; ++i) {
        int joint_idx = robot_model.model_ptr->getJointId(secondary_names[i]) - 1;
        std::cout << "  " << secondary_names[i] << " -> index " << joint_idx << ": " << q_both[7 + i] << std::endl;
    }

    int non_zero_count = 0;
    for (int i = 0; i < ndof_robot; ++i) {
        if (std::abs(q_ndof[i]) > 1e-6) {
            non_zero_count++;
        }
    }

    EXPECT_EQ(non_zero_count, 14) << "Should have exactly 14 non-zero joints for both arms";

    for (int i = 0; i < 7; ++i) {
        int joint_idx = robot_model.model_ptr->getJointId(primary_names[i]) - 1;
        EXPECT_NEAR(q_ndof[joint_idx], q_both[i], 1e-6) << "Primary Joint " << i << " value mismatch";
    }

    for (int i = 0; i < 7; ++i) {
        int joint_idx = robot_model.model_ptr->getJointId(secondary_names[i]) - 1;
        EXPECT_NEAR(q_ndof[joint_idx], q_both[7 + i], 1e-6) << "Secondary Joint " << i << " value mismatch";
    }

    std::cout << "\n✓ MapJointToWholeBody_BothArms test passed!" << std::endl;
}

TEST(JointMappingTest, MapJointToWholeBody_WholeBody) {
    const auto package_share_path = ament_index_cpp::get_package_share_directory("w1_server");
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot_config.yaml";
    RobotModel robot_model = utils::GetRobotModelFromConfig(robot_cfg_path);

    Eigen::VectorXd q_whole(ndof_robot);
    for (int i = 0; i < ndof_robot; ++i) {
        q_whole[i] = i * 0.1;
    }

    JointPositionDof q_ndof = JointPositionDof::Zero();

    bool result = utils::MapJointToWholeBody(robot_model, q_whole, q_ndof, MotionGroup::WholeBody);

    EXPECT_TRUE(result);

    int non_zero_count = 0;
    for (int i = 0; i < ndof_robot; ++i) {
        if (std::abs(q_ndof[i]) > 1e-6) {
            non_zero_count++;
        }
    }

    EXPECT_EQ(non_zero_count, ndof_robot - 1) << "Should have all joints non-zero for WholeBody (excluding universe)";

    const auto& model_names = robot_model.model_ptr->names;
    int joint_count = 0;
    for (size_t idx = 0; idx < model_names.size(); ++idx) {
        if (model_names[idx] != "universe") {
            EXPECT_NEAR(q_ndof[joint_count], q_whole[joint_count], 1e-6)
                << "WholeBody Joint " << joint_count << " (" << model_names[idx] << ") value mismatch";
            joint_count++;
        }
    }

    std::cout << "MapJointToWholeBody_WholeBody test passed!" << std::endl;
}

TEST(JointMappingTest, GetJointFromWholeBody_Primary) {
    std::cout << "\n=== GetJointFromWholeBody_Primary Test ===" << std::endl;
    const auto package_share_path = ament_index_cpp::get_package_share_directory("w1_server");
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot_config.yaml";
    RobotModel robot_model = utils::GetRobotModelFromConfig(robot_cfg_path);

    std::cout << "Robot: " << robot_model.params_ptr->robot_type << std::endl;

    JointPositionDof q_ndof = JointPositionDof::Zero();
    q_ndof[6] = 0.1;
    q_ndof[7] = 0.2;
    q_ndof[8] = 0.3;
    q_ndof[9] = 0.4;
    q_ndof[10] = 0.5;
    q_ndof[11] = 0.6;
    q_ndof[12] = 0.7;

    std::cout << "Input q_ndof (at expected indices):" << std::endl;
    auto primary_names = robot_model.params_ptr->joints_primary_names;
    for (int i = 0; i < 7; ++i) {
        int joint_idx = robot_model.model_ptr->getJointId(primary_names[i]) - 1;
        std::cout << "  " << primary_names[i] << " at index " << joint_idx << ": " << q_ndof[joint_idx] << std::endl;
    }

    Eigen::VectorXd q_primary;

    bool result = utils::GetJointFromWholeBody(robot_model, q_ndof, q_primary, MotionGroup::Primary);

    EXPECT_TRUE(result);
    EXPECT_EQ(q_primary.size(), 7);

    std::cout << "\nExtracted q_primary: " << q_primary.transpose() << std::endl;

    for (int i = 0; i < 7; ++i) {
        int joint_idx = robot_model.model_ptr->getJointId(primary_names[i]) - 1;
        EXPECT_NEAR(q_primary[i], q_ndof[joint_idx], 1e-6)
            << "Primary Joint " << i << " (" << primary_names[i] << ") at index " << joint_idx << " expected "
            << q_ndof[joint_idx] << " got " << q_primary[i];
    }

    std::cout << "\n✓ GetJointFromWholeBody_Primary test passed!" << std::endl;
}

TEST(JointMappingTest, GetJointFromWholeBody_Secondary) {
    std::cout << "\n=== GetJointFromWholeBody_Secondary Test ===" << std::endl;
    const auto package_share_path = ament_index_cpp::get_package_share_directory("w1_server");
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot_config.yaml";
    RobotModel robot_model = utils::GetRobotModelFromConfig(robot_cfg_path);

    std::cout << "Robot: " << robot_model.params_ptr->robot_type << std::endl;

    JointPositionDof q_ndof = JointPositionDof::Zero();
    q_ndof[13] = 0.11;
    q_ndof[14] = 0.22;
    q_ndof[15] = 0.33;
    q_ndof[16] = 0.44;
    q_ndof[17] = 0.55;
    q_ndof[18] = 0.66;
    q_ndof[19] = 0.77;

    std::cout << "Input q_ndof (at expected indices):" << std::endl;
    auto secondary_names = robot_model.params_ptr->joints_secondary_names;
    for (int i = 0; i < 7; ++i) {
        int joint_idx = robot_model.model_ptr->getJointId(secondary_names[i]) - 1;
        std::cout << "  " << secondary_names[i] << " at index " << joint_idx << ": " << q_ndof[joint_idx] << std::endl;
    }

    Eigen::VectorXd q_secondary;

    bool result = utils::GetJointFromWholeBody(robot_model, q_ndof, q_secondary, MotionGroup::Secondary);

    EXPECT_TRUE(result);
    EXPECT_EQ(q_secondary.size(), 7);

    std::cout << "\nExtracted q_secondary: " << q_secondary.transpose() << std::endl;

    for (int i = 0; i < 7; ++i) {
        int joint_idx = robot_model.model_ptr->getJointId(secondary_names[i]) - 1;
        EXPECT_NEAR(q_secondary[i], q_ndof[joint_idx], 1e-6)
            << "Secondary Joint " << i << " (" << secondary_names[i] << ") at index " << joint_idx << " expected "
            << q_ndof[joint_idx] << " got " << q_secondary[i];
    }

    std::cout << "\n✓ GetJointFromWholeBody_Secondary test passed!" << std::endl;
}

TEST(JointMappingTest, GetJointFromWholeBody_BothArms) {
    const auto package_share_path = ament_index_cpp::get_package_share_directory("w1_server");
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot_config.yaml";
    RobotModel robot_model = utils::GetRobotModelFromConfig(robot_cfg_path);

    JointPositionDof q_ndof = JointPositionDof::Zero();
    for (int i = 6; i < 13; ++i) {
        q_ndof[i] = (i - 6) * 0.1;
    }
    for (int i = 13; i < 20; ++i) {
        q_ndof[i] = (i - 13) * 0.11;
    }

    Eigen::VectorXd q_both;

    bool result = utils::GetJointFromWholeBody(robot_model, q_ndof, q_both, MotionGroup::BothArms);

    EXPECT_TRUE(result);
    EXPECT_EQ(q_both.size(), 14);

    auto primary_names = robot_model.params_ptr->joints_primary_names;
    auto secondary_names = robot_model.params_ptr->joints_secondary_names;

    for (int i = 0; i < 7; ++i) {
        int joint_idx = robot_model.model_ptr->getJointId(primary_names[i]) - 1;
        EXPECT_NEAR(q_both[i], q_ndof[joint_idx], 1e-6) << "Primary Joint " << i << " value mismatch";
    }

    for (int i = 0; i < 7; ++i) {
        int joint_idx = robot_model.model_ptr->getJointId(secondary_names[i]) - 1;
        EXPECT_NEAR(q_both[7 + i], q_ndof[joint_idx], 1e-6) << "Secondary Joint " << i << " value mismatch";
    }

    std::cout << "GetJointFromWholeBody_BothArms test passed!" << std::endl;
}

TEST(JointMappingTest, GetJointFromWholeBody_WholeBody) {
    const auto package_share_path = ament_index_cpp::get_package_share_directory("w1_server");
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot_config.yaml";
    RobotModel robot_model = utils::GetRobotModelFromConfig(robot_cfg_path);

    JointPositionDof q_ndof(ndof_robot);
    for (int i = 0; i < ndof_robot; ++i) {
        q_ndof[i] = i * 0.1;
    }

    Eigen::VectorXd q_whole;

    bool result = utils::GetJointFromWholeBody(robot_model, q_ndof, q_whole, MotionGroup::WholeBody);

    EXPECT_TRUE(result);
    EXPECT_EQ(q_whole.size(), ndof_robot);

    for (int i = 0; i < ndof_robot; ++i) {
        EXPECT_NEAR(q_whole[i], q_ndof[i], 1e-6) << "WholeBody Joint " << i << " value mismatch";
    }

    std::cout << "GetJointFromWholeBody_WholeBody test passed!" << std::endl;
}

TEST(JointMappingTest, RoundTrip_Primary) {
    std::cout << "\n=== RoundTrip_Primary Test ===" << std::endl;
    const auto package_share_path = ament_index_cpp::get_package_share_directory("w1_server");
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot_config.yaml";
    RobotModel robot_model = utils::GetRobotModelFromConfig(robot_cfg_path);

    std::cout << "Robot: " << robot_model.params_ptr->robot_type << std::endl;

    Eigen::VectorXd q_primary_original(7);
    q_primary_original << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
    std::cout << "Original q_primary: " << q_primary_original.transpose() << std::endl;

    JointPositionDof q_ndof = JointPositionDof::Zero();
    bool map_result = utils::MapJointToWholeBody(robot_model, q_primary_original, q_ndof, MotionGroup::Primary);
    EXPECT_TRUE(map_result);
    std::cout << "\n[Map] q_primary -> q_ndof" << std::endl;

    Eigen::VectorXd q_primary_recovered;
    bool get_result = utils::GetJointFromWholeBody(robot_model, q_ndof, q_primary_recovered, MotionGroup::Primary);
    EXPECT_TRUE(get_result);
    std::cout << "[Get] q_ndof -> q_primary" << std::endl;

    EXPECT_EQ(q_primary_recovered.size(), q_primary_original.size());

    std::cout << "\nRoundTrip comparison:" << std::endl;
    auto primary_names = robot_model.params_ptr->joints_primary_names;
    for (int i = 0; i < 7; ++i) {
        int joint_idx = robot_model.model_ptr->getJointId(primary_names[i]) - 1;
        std::cout << "  Joint " << i << " (" << primary_names[i] << "): " << q_primary_original[i] << " -> "
                  << q_primary_recovered[i] << " (diff: " << std::abs(q_primary_recovered[i] - q_primary_original[i])
                  << ")" << std::endl;
        EXPECT_NEAR(q_primary_recovered[i], q_primary_original[i], 1e-6) << "RoundTrip failed for Primary Joint " << i;
    }

    std::cout << "\n✓ RoundTrip_Primary test passed!" << std::endl;
}

TEST(JointMappingTest, RoundTrip_Secondary) {
    std::cout << "\n=== RoundTrip_Secondary Test ===" << std::endl;
    const auto package_share_path = ament_index_cpp::get_package_share_directory("w1_server");
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot_config.yaml";
    RobotModel robot_model = utils::GetRobotModelFromConfig(robot_cfg_path);

    std::cout << "Robot: " << robot_model.params_ptr->robot_type << std::endl;

    Eigen::VectorXd q_secondary_original(7);
    q_secondary_original << 0.11, 0.22, 0.33, 0.44, 0.55, 0.66, 0.77;
    std::cout << "Original q_secondary: " << q_secondary_original.transpose() << std::endl;

    JointPositionDof q_ndof = JointPositionDof::Zero();
    bool map_result = utils::MapJointToWholeBody(robot_model, q_secondary_original, q_ndof, MotionGroup::Secondary);
    EXPECT_TRUE(map_result);
    std::cout << "\n[Map] q_secondary -> q_ndof" << std::endl;

    Eigen::VectorXd q_secondary_recovered;
    bool get_result = utils::GetJointFromWholeBody(robot_model, q_ndof, q_secondary_recovered, MotionGroup::Secondary);
    EXPECT_TRUE(get_result);
    std::cout << "[Get] q_ndof -> q_secondary" << std::endl;

    EXPECT_EQ(q_secondary_recovered.size(), q_secondary_original.size());

    std::cout << "\nRoundTrip comparison:" << std::endl;
    auto secondary_names = robot_model.params_ptr->joints_secondary_names;
    for (int i = 0; i < 7; ++i) {
        int joint_idx = robot_model.model_ptr->getJointId(secondary_names[i]) - 1;
        std::cout << "  Joint " << i << " (" << secondary_names[i] << "): " << q_secondary_original[i] << " -> "
                  << q_secondary_recovered[i]
                  << " (diff: " << std::abs(q_secondary_recovered[i] - q_secondary_original[i]) << ")" << std::endl;
        EXPECT_NEAR(q_secondary_recovered[i], q_secondary_original[i], 1e-6)
            << "RoundTrip failed for Secondary Joint " << i;
    }

    std::cout << "\n✓ RoundTrip_Secondary test passed!" << std::endl;
}

TEST(JointMappingTest, RoundTrip_BothArms) {
    std::cout << "\n=== RoundTrip_BothArms Test ===" << std::endl;
    const auto package_share_path = ament_index_cpp::get_package_share_directory("w1_server");
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot_config.yaml";
    RobotModel robot_model = utils::GetRobotModelFromConfig(robot_cfg_path);

    std::cout << "Robot: " << robot_model.params_ptr->robot_type << std::endl;

    Eigen::VectorXd q_both_original(14);
    q_both_original << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.11, 0.22, 0.33, 0.44, 0.55, 0.66, 0.77;
    std::cout << "Original q_both (Primary 7 + Secondary 7): " << q_both_original.transpose() << std::endl;

    JointPositionDof q_ndof = JointPositionDof::Zero();
    bool map_result = utils::MapJointToWholeBody(robot_model, q_both_original, q_ndof, MotionGroup::BothArms);
    EXPECT_TRUE(map_result);
    std::cout << "\n[Map] q_both -> q_ndof" << std::endl;

    Eigen::VectorXd q_both_recovered;
    bool get_result = utils::GetJointFromWholeBody(robot_model, q_ndof, q_both_recovered, MotionGroup::BothArms);
    EXPECT_TRUE(get_result);
    std::cout << "[Get] q_ndof -> q_both" << std::endl;

    EXPECT_EQ(q_both_recovered.size(), q_both_original.size());

    std::cout << "\nRoundTrip comparison:" << std::endl;
    auto primary_names = robot_model.params_ptr->joints_primary_names;
    auto secondary_names = robot_model.params_ptr->joints_secondary_names;

    std::cout << "\nPrimary arm:" << std::endl;
    for (int i = 0; i < 7; ++i) {
        std::cout << "  Joint " << i << " (" << primary_names[i] << "): " << q_both_original[i] << " -> "
                  << q_both_recovered[i] << " (diff: " << std::abs(q_both_recovered[i] - q_both_original[i]) << ")"
                  << std::endl;
        EXPECT_NEAR(q_both_recovered[i], q_both_original[i], 1e-6) << "RoundTrip failed for BothArms Joint " << i;
    }

    std::cout << "\nSecondary arm:" << std::endl;
    for (int i = 0; i < 7; ++i) {
        std::cout << "  Joint " << i << " (" << secondary_names[i] << "): " << q_both_original[7 + i] << " -> "
                  << q_both_recovered[7 + i] << " (diff: " << std::abs(q_both_recovered[7 + i] - q_both_original[7 + i])
                  << ")" << std::endl;
        EXPECT_NEAR(q_both_recovered[7 + i], q_both_original[7 + i], 1e-6)
            << "RoundTrip failed for BothArms Joint " << (7 + i);
    }

    std::cout << "\n✓ RoundTrip_BothArms test passed!" << std::endl;
}

TEST(JointMappingTest, GetJointCount) {
    const auto package_share_path = ament_index_cpp::get_package_share_directory("w1_server");
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot.yaml";
    RobotModel robot_model = utils::GetRobotModelFromConfig(robot_cfg_path);

    int primary_count = utils::GetJointCount(robot_model, MotionGroup::Primary);
    EXPECT_EQ(primary_count, 7);

    int secondary_count = utils::GetJointCount(robot_model, MotionGroup::Secondary);
    EXPECT_EQ(secondary_count, 7);

    int both_count = utils::GetJointCount(robot_model, MotionGroup::BothArms);
    EXPECT_EQ(both_count, 14);

    int whole_count = utils::GetJointCount(robot_model, MotionGroup::WholeBody);
    EXPECT_EQ(whole_count, ndof_robot);

    std::cout << "Joint counts - Primary: " << primary_count << ", Secondary: " << secondary_count
              << ", Both: " << both_count << ", WholeBody: " << whole_count << std::endl;

    std::cout << "GetJointCount test passed!" << std::endl;
}
