#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <fstream>

#include "torque_controller_inverse_dynamic.h"

using namespace robot_server_lib;

namespace {
class TorqueExtDeadzoneTest : public ::testing::Test {
protected:
    YAML::Node config;
    std::filesystem::path config_path;
    std::filesystem::path default_path;

    static void SetUpTestSuite() {
        nanolog::initialize(nanolog::GuaranteedLogger(), std::filesystem::temp_directory_path().string() + "/",
                            "test_torque_ext_deadzone_" + std::to_string(::getpid()), 1, 1);
    }

    void SetUp() override {
        const auto source = std::filesystem::path(__FILE__).parent_path().parent_path();
        config = YAML::LoadFile((source.parent_path() / "w1_server/robot_config.yaml").string());
        config["file_path"]["robot_urdf"] = (source / "robot_models/w1/robot.urdf").string();
        config_path = std::filesystem::temp_directory_path() /
                      ("test_torque_ext_deadzone_" + std::to_string(::getpid()) + ".yaml");
        default_path = config_path.string() + ".default";
    }

    void TearDown() override {
        std::filesystem::remove(config_path);
        std::filesystem::remove(default_path);
    }

    static void Write(const std::filesystem::path& path, const YAML::Node& node) {
        std::ofstream out(path);
        out << node;
    }

    RobotModel Load(const std::string& version = "v024") {
        config["version"] = version;
        std::ofstream out(config_path);
        out << config;
        out.close();
        return utils::GetRobotModelFromConfig(config_path);
    }
};

TEST_F(TorqueExtDeadzoneTest, FirstStartupCreatesConfigWithArmDeadzones) {
    Write(default_path, config);
    utils::CheckConfig(default_path, config_path);
    const auto created = YAML::LoadFile(config_path.string());
    for (const auto* key : {"w1_v02", "w1_v03"}) {
        for (int i = 0; i < ndof_robot; ++i) {
            const auto field = created[key]["joints"][i]["torque_ext_deadzone"];
            if (i < 6) {
                EXPECT_FALSE(field.IsDefined());
            } else {
                EXPECT_DOUBLE_EQ(field.as<double>(), 0.5);
            }
        }
    }
}

TEST_F(TorqueExtDeadzoneTest, SameVersionStartupFillsMissingJointFieldsAndPreservesTuning) {
    Write(default_path, config);
    for (const auto* key : {"w1_v02", "w1_v03"}) {
        auto joints = config[key]["joints"];
        for (auto joint : joints) {
            joint.remove("torque_ext_deadzone");
        }
        joints[10]["torque_ext_deadzone"] = 0.2;
        joints[17]["torque_ext_deadzone"] = 0.0;
        joints[6]["joint_pd_control"]["stiffness"] = 123.0;
    }
    Write(config_path, config);
    for (int startup = 0; startup < 2; ++startup) {
        utils::CheckConfig(default_path, config_path);
        const auto merged = YAML::LoadFile(config_path.string());
        EXPECT_EQ(merged["config_version"].as<std::string>(), config["config_version"].as<std::string>());
        for (const auto* key : {"w1_v02", "w1_v03"}) {
            const auto joints = merged[key]["joints"];
            EXPECT_DOUBLE_EQ(joints[6]["joint_pd_control"]["stiffness"].as<double>(), 123.0);
            for (int i = 6; i < ndof_robot; ++i) {
                EXPECT_DOUBLE_EQ(joints[i]["torque_ext_deadzone"].as<double>(),
                                 i == 10 ? 0.2 : (i == 17 ? 0.0 : 0.5));
            }
        }
    }
}

TEST_F(TorqueExtDeadzoneTest, NewerDefaultVersionStillReplacesUserConfig) {
    Write(default_path, config);
    config["config_version"] = "v00";
    config["w1_v02"]["joints"][10]["torque_ext_deadzone"] = 0.2;
    Write(config_path, config);
    utils::CheckConfig(default_path, config_path);
    const auto replaced = YAML::LoadFile(config_path.string());
    EXPECT_EQ(replaced["config_version"].as<std::string>(), "v06");
    EXPECT_DOUBLE_EQ(replaced["w1_v02"]["joints"][10]["torque_ext_deadzone"].as<double>(), 0.5);
}

TEST_F(TorqueExtDeadzoneTest, ShippedAndOldConfigsKeepOriginalThreshold) {
    for (const auto* version : {"v024", "v031"}) {
        const std::string key = std::string("w1_") + std::string(version).substr(0, 3);
        EXPECT_TRUE(Load(version).params_ptr->joint_torque_ext_deadzone.isConstant(0.5));
        for (auto joint : config[key]["joints"]) {
            joint.remove("torque_ext_deadzone");
        }
        EXPECT_TRUE(Load(version).params_ptr->joint_torque_ext_deadzone.isConstant(0.5));
    }
}

TEST_F(TorqueExtDeadzoneTest, EachArmJointIsIndependentAndNonArmOverridesAreIgnored) {
    for (const auto* version : {"v024", "v031"}) {
        const std::string key = std::string("w1_") + std::string(version).substr(0, 3);
        auto joints = config[key]["joints"];
        ASSERT_EQ(joints.size(), ndof_robot);
        for (int i = 0; i < ndof_robot; ++i) {
            joints[i]["torque_ext_deadzone"] = 0.1 * i;
        }
        const auto model = Load(version);
        for (int i = 0; i < ndof_robot; ++i) {
            EXPECT_DOUBLE_EQ(model.params_ptr->joint_torque_ext_deadzone[i], i < 6 ? 0.5 : 0.1 * i);
        }
    }
}

TEST_F(TorqueExtDeadzoneTest, InvalidValuesFallBackAndZeroIsAllowed) {
    auto joints = config["w1_v02"]["joints"];
    const char* invalid[] = {"-1", ".nan", ".inf", "-.inf", "bad", "null", "[0.2]"};
    for (int i = 0; i < 7; ++i) {
        joints[6 + i]["torque_ext_deadzone"] = YAML::Load(invalid[i]);
    }
    joints[13]["torque_ext_deadzone"] = 0.0;
    const auto model = Load();
    for (int i = 6; i < 13; ++i) {
        EXPECT_DOUBLE_EQ(model.params_ptr->joint_torque_ext_deadzone[i], 0.5);
    }
    EXPECT_DOUBLE_EQ(model.params_ptr->joint_torque_ext_deadzone[13], 0.0);
}

TEST_F(TorqueExtDeadzoneTest, OperatingControllerUsesConfiguredDeadzone) {
    // Exercise positive/negative torques, equality at the threshold and zero deadzone on both arms.
    auto joints = config["w1_v02"]["joints"];
    for (int i = 6; i < ndof_robot; ++i) {
        joints[i]["torque_ext_deadzone"] = (i % 3) * 0.5;
    }
    const auto model = Load();
    TorqueControllerInvDynamic controller(model);
    JointRawMeasurement measurement{};
    measurement.position.setZero();
    measurement.velocity.setZero();
    const VectorDof feedforward = dynamics::InverseDynamics(
        model, measurement.position, measurement.velocity, VectorDof::Zero());
    VectorDof expected_ext = VectorDof::Zero();
    for (int i = 0; i < ndof_robot; ++i) {
        const double residual = (i % 2 == 0 ? 0.5 : -0.5);
        measurement.torque[i] = feedforward[i] - residual;
        const double threshold = i < 6 ? 0.5 : (i % 3) * 0.5;
        expected_ext[i] = std::copysign(std::max(0.0, std::abs(residual) - threshold), residual);
    }
    PlannerManagerCommand input{};
    input.cmd.position.setZero();
    input.cmd.velocity.setZero();
    input.cmd.acceleration.setZero();
    input.cmd.torque_feedforward.setZero();
    input.cmd.joint_stiffness.setOnes();
    input.cmd.joint_damping.setZero();
    input.cmd.joint_integral_ratio.setZero();
    controller.ResetForNewEpisode(measurement, TorqueControllerState::Operating);
    dynamics::JointAdmittanceController expected(model.robot_property_default.joint_admittance);
    for (int step = 0; step < 10; ++step) {
        VectorDof q_offset, dq_offset;
        ASSERT_TRUE(expected.UpdateAdmittanceOffset(expected_ext, q_offset, dq_offset));
        TorqueCtrlCommand output{};
        ASSERT_TRUE(controller.ComputeCommand(measurement, input, output));
        EXPECT_TRUE((output.torque - feedforward).isApprox(q_offset, 1e-7));
    }
}
}  // namespace
