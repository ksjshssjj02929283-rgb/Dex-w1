#include <gtest/gtest.h>
#include <unistd.h>

#include "utils.h"

using namespace robot_server_lib;

class ToolConfigTest : public ::testing::Test {
protected:
    YAML::Node config;
    std::filesystem::path directory, config_path;

    static void SetUpTestSuite() {
        nanolog::initialize(nanolog::GuaranteedLogger(), "/tmp/", "test_tool_config", 1, 1);
    }

    void SetUp() override {
        const auto root = std::filesystem::path(__FILE__).parent_path().parent_path();
        config = YAML::LoadFile((root.parent_path() / "w1_server/robot_config.yaml").string());
        config["file_path"]["robot_urdf"] = (root / "robot_models/w1/robot.urdf").string();
        config["left_hand_type"] = "TestHand";
        config["right_hand_type"] = "TestHand";
        directory = std::filesystem::temp_directory_path() / ("test_tool_config_" + std::to_string(::getpid()));
        std::filesystem::create_directories(directory);
        config_path = directory / "robot_config.yaml";
    }

    void TearDown() override { std::filesystem::remove_all(directory); }

    void WriteHand(const char* name, double mass) {
        std::ofstream out(directory / name);
        out << "<robot name=\"hand\"><link name=\"palm\"><inertial>"
               "<origin xyz=\"0.01 -0.02 0.03\"/><mass value=\"" << mass << "\"/>"
               "<inertia ixx=\"0.001\" ixy=\"0\" ixz=\"0\" iyy=\"0.002\" iyz=\"0\" izz=\"0.002\"/>"
               "</inertial></link></robot>";
    }

    RobotModel Load() {
        {
            std::ofstream out(config_path);
            out << config;
        }
        return utils::GetRobotModelFromConfig(config_path);
    }
};

TEST_F(ToolConfigTest, SelectsSideSpecificUrdfBeforeLegacyMassAndResolvesRelativePaths) {
    WriteHand("left.urdf", 0.7);
    WriteHand("right.urdf", 0.9);
    auto tool = config["tool_dynamics"]["TestHand"];
    tool["left_urdf"] = "left.urdf";
    tool["right_urdf"] = "right.urdf";
    tool["mass_kg"] = 99.0;
    tool["com_m"] = YAML::Load("[0, 0, 0]");
    const auto model = Load();
    const auto& left = model.params_ptr->tool_dynamic_primary;
    const auto& right = model.params_ptr->tool_dynamic_secondary;
    EXPECT_TRUE(left.enabled);
    EXPECT_TRUE(right.enabled);
    EXPECT_DOUBLE_EQ(left.inertia.mass(), 0.7);
    EXPECT_DOUBLE_EQ(right.inertia.mass(), 0.9);
    EXPECT_TRUE(left.inertia.lever().isApprox(Vector3d(0.01, -0.02, 0.03)));
    EXPECT_GT(left.inertia.inertia().matrix().norm(), 0.0);
}

TEST_F(ToolConfigTest, SwitchAndAbsentHandDisableUrdfLoading) {
    auto tool = config["tool_dynamics"]["TestHand"];
    tool["left_urdf"] = "missing.urdf";
    tool["right_urdf"] = "missing.urdf";
    config["tool_dynamics"]["tool_primary_enabled"] = false;
    config["right_hand_type"] = "null";
    const auto model = Load();
    EXPECT_FALSE(model.params_ptr->tool_dynamic_primary.enabled);
    EXPECT_FALSE(model.params_ptr->tool_dynamic_secondary.enabled);
}

TEST_F(ToolConfigTest, UnmatchedHandUsesDefaultPointMass) {
    auto fallback = config["tool_dynamics"]["Default"];
    fallback["mass_kg"] = 0.81;
    fallback["com_m"] = YAML::Load("[0.02, 0.03, 0.04]");
    const auto model = Load();
    for (const auto* tool : {&model.params_ptr->tool_dynamic_primary, &model.params_ptr->tool_dynamic_secondary}) {
        EXPECT_TRUE(tool->enabled);
        EXPECT_DOUBLE_EQ(tool->inertia.mass(), 0.81);
        EXPECT_TRUE(tool->inertia.lever().isApprox(Vector3d(0.02, 0.03, 0.04)));
        EXPECT_TRUE(tool->inertia.inertia().matrix().isZero());
    }
}
