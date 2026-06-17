#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <vector>

#include "tool_compenstaion.h"

using namespace robot_server_lib;

TEST(ToolCompensationTest, JsonFiniteOrZeroReplacesNonFiniteValues) {
    EXPECT_DOUBLE_EQ(dynamics::tool_compensation::JsonFiniteOrZero(1.25), 1.25);
    EXPECT_DOUBLE_EQ(dynamics::tool_compensation::JsonFiniteOrZero(std::numeric_limits<double>::quiet_NaN()), 0.0);
    EXPECT_DOUBLE_EQ(dynamics::tool_compensation::JsonFiniteOrZero(std::numeric_limits<double>::infinity()), 0.0);
}

TEST(ToolCompensationTest, ReadYamlVectorRequiresExpectedSize) {
    const YAML::Node node = YAML::Load("[1.0, 2.0, 3.0]");
    Eigen::VectorXd out;

    ASSERT_TRUE(dynamics::tool_compensation::ReadYamlVector(node, 3, out));
    ASSERT_EQ(out.size(), 3);
    EXPECT_DOUBLE_EQ(out[0], 1.0);
    EXPECT_DOUBLE_EQ(out[1], 2.0);
    EXPECT_DOUBLE_EQ(out[2], 3.0);

    EXPECT_FALSE(dynamics::tool_compensation::ReadYamlVector(node, 2, out));
}

TEST(ToolCompensationTest, ReadYamlMatrix3RequiresThreeByThreeSequence) {
    const YAML::Node node = YAML::Load("[[1.0, 0.1, 0.2], [0.1, 2.0, 0.3], [0.2, 0.3, 3.0]]");
    Eigen::Matrix3d out = Eigen::Matrix3d::Zero();

    ASSERT_TRUE(dynamics::tool_compensation::ReadYamlMatrix3(node, out));
    EXPECT_DOUBLE_EQ(out(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(out(1, 1), 2.0);
    EXPECT_DOUBLE_EQ(out(2, 2), 3.0);
    EXPECT_DOUBLE_EQ(out(0, 1), 0.1);

    const YAML::Node invalid = YAML::Load("[[1.0, 2.0], [3.0, 4.0]]");
    EXPECT_FALSE(dynamics::tool_compensation::ReadYamlMatrix3(invalid, out));
}

TEST(ToolCompensationTest, LoadToolGravityCompensationConfigParsesMassComInertiaAndFilter) {
    const auto cfg_path = std::filesystem::temp_directory_path() / "tool_compensation_test.yaml";
    {
        std::ofstream out(cfg_path);
        out << "tool_gravity_compensation:\n"
            << "  mass_kg: 1.5\n"
            << "  gravity_magnitude: 9.79\n"
            << "  model_sign: -1\n"
            << "  com_mm: [10.0, -20.0, 30.0]\n"
            << "  inertia_com_kg_m2: [[0.1, 0.0, 0.0], [0.0, 0.2, 0.0], [0.0, 0.0, 0.3]]\n"
            << "  inertia_compensation_filter_hz: 8.0\n";
    }

    std::vector<std::string> warnings;
    const auto result = dynamics::tool_compensation::LoadToolGravityCompensationConfig(cfg_path, &warnings);

    EXPECT_TRUE(warnings.empty());
    EXPECT_TRUE(result.config.gravity_enabled);
    EXPECT_TRUE(result.config.inertia_enabled);
    EXPECT_DOUBLE_EQ(result.config.mass_kg, 1.5);
    EXPECT_DOUBLE_EQ(result.config.gravity_magnitude, 9.79);
    EXPECT_EQ(result.config.model_sign, -1);
    EXPECT_TRUE(result.config.com_m.isApprox(Vector3d(0.01, -0.02, 0.03)));
    EXPECT_DOUBLE_EQ(result.config.inertia_com(0, 0), 0.1);
    EXPECT_DOUBLE_EQ(result.config.inertia_com(1, 1), 0.2);
    EXPECT_DOUBLE_EQ(result.config.inertia_com(2, 2), 0.3);
    EXPECT_DOUBLE_EQ(result.config.inertia_filter_cutoff_hz, 8.0);
}

TEST(ToolCompensationTest, ApplyToolGravityCompensationSubtractsExpectedWrench) {
    dynamics::tool_compensation::ToolCompensationConfig config;
    config.gravity_enabled = true;
    config.mass_kg = 2.0;
    config.gravity_magnitude = 10.0;
    config.model_sign = 1;
    config.com_m = Vector3d(0.1, 0.0, 0.0);

    Vector6d wrench = Vector6d::Zero();
    bool applied = false;
    const Vector6d compensated = dynamics::tool_compensation::ApplyToolGravityCompensation(
        wrench, config, Eigen::Matrix3d::Identity(), &applied);

    ASSERT_TRUE(applied);
    EXPECT_DOUBLE_EQ(compensated[0], 0.0);
    EXPECT_DOUBLE_EQ(compensated[1], 0.0);
    EXPECT_DOUBLE_EQ(compensated[2], 20.0);
    EXPECT_DOUBLE_EQ(compensated[3], 0.0);
    EXPECT_DOUBLE_EQ(compensated[4], -2.0);
    EXPECT_DOUBLE_EQ(compensated[5], 0.0);
}
