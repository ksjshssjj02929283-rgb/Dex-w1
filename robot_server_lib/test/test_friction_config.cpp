#include <gtest/gtest.h>
#include <unistd.h>

#include "drag_damping.h"
#include "inverse_dynamics.h"
#include "utils.h"

using namespace robot_server_lib;

class FrictionConfigTest : public ::testing::Test {
protected:
    YAML::Node config;
    std::filesystem::path user_path, default_path;
    static void SetUpTestSuite() {
        nanolog::initialize(nanolog::GuaranteedLogger(), "/tmp/", "test_friction_config", 1, 1);
    }
    void SetUp() override {
        auto root = std::filesystem::path(__FILE__).parent_path().parent_path();
        config = YAML::LoadFile((root.parent_path() / "w1_server/robot_config.yaml").string());
        config["file_path"]["robot_urdf"] = (root / "robot_models/w1/robot.urdf").string();
        user_path = "/tmp/test_friction_" + std::to_string(::getpid()) + ".yaml";
        default_path = user_path.string() + ".default";
    }
    void TearDown() override {
        std::filesystem::remove(user_path);
        std::filesystem::remove(default_path);
    }
    static void Write(const std::filesystem::path& path, const YAML::Node& node) {
        std::ofstream out(path);
        out << node;
    }
    RobotModel Load() {
        Write(user_path, config);
        return utils::GetRobotModelFromConfig(user_path);
    }
};

TEST_F(FrictionConfigTest, StartupCreatesRepairsAndPreservesOrderedArrays) {
    Write(default_path, config);
    utils::CheckConfig(default_path, user_path);
    auto user = YAML::LoadFile(user_path.string());
    for (const auto* version : {"w1_v02", "w1_v03"}) {
        ASSERT_EQ(user[version]["fc_ratio_secondary"].size(), 7u);
        user[version].remove("fv_ratio_primary");
        user[version].remove("fc_ratio_secondary");
        user[version]["fc_ratio_primary"][2] = 0.73;
    }
    Write(user_path, user);
    for (int restart = 0; restart < 2; ++restart) {
        utils::CheckConfig(default_path, user_path);
        user = YAML::LoadFile(user_path.string());
        for (const auto* version : {"w1_v02", "w1_v03"}) {
            EXPECT_DOUBLE_EQ(user[version]["fc_ratio_primary"][2].as<double>(), 0.73);
            EXPECT_DOUBLE_EQ(user[version]["fv_ratio_primary"][6].as<double>(), 0.5);
            EXPECT_DOUBLE_EQ(user[version]["fc_ratio_secondary"][6].as<double>(), 0.2);
            std::vector<std::string> keys;
            for (const auto& field : user[version]) keys.push_back(field.first.as<std::string>());
            auto pos = std::find(keys.begin(), keys.end(), "dyn_base_params_secondary");
            ASSERT_GE(std::distance(pos, keys.end()), 5);
            EXPECT_EQ(*(pos + 1), "fv_ratio_primary");
            EXPECT_EQ(*(pos + 2), "fv_ratio_secondary");
            EXPECT_EQ(*(pos + 3), "fc_ratio_primary");
            EXPECT_EQ(*(pos + 4), "fc_ratio_secondary");
        }
    }
}

TEST_F(FrictionConfigTest, EveryJointAndArmScalesExactlyOnce) {
    const std::array<int, 7> fv{3, 13, 24, 35, 46, 57, 68};
    for (const auto* version : {"v024", "v031"}) {
        config["version"] = version;
        const std::string key = std::string("w1_") + std::string(version).substr(0, 3);
        for (int j = 0; j < 7; ++j) {
            config[key]["fv_ratio_primary"][j] = 0.1 * j;
            config[key]["fv_ratio_secondary"][j] = 0.2 * j;
            config[key]["fc_ratio_primary"][j] = 0.3 * j;
            config[key]["fc_ratio_secondary"][j] = 0.4 * j;
        }
        auto model = Load();
        Eigen::VectorXd params = Eigen::VectorXd::Zero(71);
        for (int j = 0; j < 7; ++j) {
            params[fv[j]] = 2.0;
            params[fv[j] + 1] = 3.0;
            params[fv[j] + 2] = 0.4;
        }
        auto tuned_params = std::make_shared<RobotParams>(*model.params_ptr);
        tuned_params->dyn_base_params_primary = params;
        tuned_params->dyn_base_params_secondary = params;
        model.params_ptr = tuned_params;
        for (auto group : {MotionGroup::Primary, MotionGroup::Secondary}) {
            const double k = group == MotionGroup::Primary ? 1.0 : 2.0;
            const double c = group == MotionGroup::Primary ? 0.3 : 0.4;
            for (double speed : {-2.0, -0.000872664626, 0.0, 0.000872664626, 2.0}) {
                const auto tau = dynamics::InvDynamikBySymbol(model, VectorDof::Zero(), VectorDof::Constant(speed),
                                                              VectorDof::Zero(), Vector3d::Zero(), group);
                for (int j = 0; j < 7; ++j) {
                    const double sign = speed > 0.000872664626 ? 1.0 : (speed < -0.000872664626 ? -1.0 : 0.0);
                    EXPECT_NEAR(tau[j], 2.0 * speed * 0.1 * j * k + 3.0 * sign * c * j + 0.4, 1e-12);
                }
            }
        }
        EXPECT_TRUE(model.params_ptr->dyn_base_params_primary->isApprox(params));
        EXPECT_TRUE(model.params_ptr->dyn_base_params_secondary->isApprox(params));
    }
}

TEST_F(FrictionConfigTest, RejectsInvalidRatiosAndSupportsOldConfigDefaults) {
    config["version"] = "v024";
    for (auto value : {"[1, 2]", "[0, 0, 0, -1, 0, 0, 0]", "[0, 0, .nan, 0, 0, 0, 0]"}) {
        config["w1_v02"]["fc_ratio_secondary"] = YAML::Load(value);
        EXPECT_THROW(Load(), std::runtime_error);
    }
    config["w1_v02"].remove("fc_ratio_secondary");
    EXPECT_DOUBLE_EQ(Load().params_ptr->fc_ratio_secondary[6], 0.2);
}

TEST(DragDampingTest, OpposesVelocityAndPreservesFeedforwardAtZero) {
    for (double velocity : {-2.0, -0.1, 0.0, 0.1, 2.0}) {
        const double feedforward = 3.7;
        const double deadzone = 0.2;
        const double expected_dead = std::abs(velocity) < deadzone ? 0.3 * ((velocity > 0.0) - (velocity < 0.0)) : 0.0;
        const double torque = dynamics::ComputeDragTorque(feedforward, velocity, 0.4, 0.6, 0.3, deadzone);
        EXPECT_DOUBLE_EQ(torque, feedforward - 0.4 * velocity - 0.6 * std::abs(velocity) * velocity + expected_dead);
        const double damped_torque = dynamics::ComputeDragTorque(feedforward, velocity, 0.4, 0.6, 0.0, deadzone);
        EXPECT_LE((damped_torque - feedforward) * velocity, 0.0);
        EXPECT_DOUBLE_EQ(dynamics::ComputeDragTorque(feedforward, velocity, 0.0, 0.0, 0.0, deadzone), feedforward);
        EXPECT_DOUBLE_EQ(dynamics::ComputeDragTorque(feedforward, velocity, 0.4, 0.0, 0.0, deadzone),
                         feedforward - 0.4 * velocity);
        EXPECT_DOUBLE_EQ(dynamics::ComputeDragTorque(feedforward, velocity, 0.0, 0.6, 0.0, deadzone),
                         feedforward - 0.6 * std::abs(velocity) * velocity);
        EXPECT_DOUBLE_EQ(dynamics::ComputeDragTorque(feedforward, 0.1, 0.0, 0.0, 0.3, 0.2), feedforward + 0.3);
        EXPECT_DOUBLE_EQ(dynamics::ComputeDragTorque(feedforward, -0.1, 0.0, 0.0, 0.3, 0.2), feedforward - 0.3);
        EXPECT_DOUBLE_EQ(dynamics::ComputeDragTorque(feedforward, 0.0, 0.0, 0.0, 0.3, 0.2), feedforward);
        EXPECT_DOUBLE_EQ(dynamics::ComputeDragTorque(feedforward, 0.2, 0.0, 0.0, 0.3, 0.2), feedforward);
    }
}

TEST_F(FrictionConfigTest, DragDampingMapsEachArmJointAndLeavesOtherJointsUnchanged) {
    for (const auto* version : {"v024", "v031"}) {
        config["version"] = version;
        const std::string key = std::string("w1_") + std::string(version).substr(0, 3);
        for (size_t joint = 0; joint < 7; ++joint) {
            config[key]["drag_damping_d1_primary"][joint] = 0.1 * (joint + 1);
            config[key]["drag_damping_d2_primary"][joint] = 0.2 * (joint + 1);
            config[key]["drag_damping_d1_secondary"][joint] = 0.3 * (joint + 1);
            config[key]["drag_damping_d2_secondary"][joint] = 0.4 * (joint + 1);
            config[key]["drag_dead_compensation_primary"][joint] = 0.5 * (joint + 1);
            config[key]["drag_dead_compensation_secondary"][joint] = 0.6 * (joint + 1);
        }
        const auto model = Load();
        const auto& params = *model.params_ptr;
        for (size_t joint = 0; joint < params.joint_names.size(); ++joint) {
            const auto primary = std::find(params.joints_primary_names.begin(), params.joints_primary_names.end(),
                                           params.joint_names[joint]);
            const auto secondary = std::find(params.joints_secondary_names.begin(), params.joints_secondary_names.end(),
                                             params.joint_names[joint]);
            if (primary != params.joints_primary_names.end()) {
                const double number = std::distance(params.joints_primary_names.begin(), primary) + 1;
                EXPECT_DOUBLE_EQ(params.drag_damping_d1[joint], 0.1 * number);
                EXPECT_DOUBLE_EQ(params.drag_damping_d2[joint], 0.2 * number);
                EXPECT_DOUBLE_EQ(params.drag_dead_compensation[joint], 0.5 * number);
            } else if (secondary != params.joints_secondary_names.end()) {
                const double number = std::distance(params.joints_secondary_names.begin(), secondary) + 1;
                EXPECT_DOUBLE_EQ(params.drag_damping_d1[joint], 0.3 * number);
                EXPECT_DOUBLE_EQ(params.drag_damping_d2[joint], 0.4 * number);
                EXPECT_DOUBLE_EQ(params.drag_dead_compensation[joint], 0.6 * number);
            } else {
                EXPECT_DOUBLE_EQ(params.drag_damping_d1[joint], 0.0);
                EXPECT_DOUBLE_EQ(params.drag_damping_d2[joint], 0.0);
                EXPECT_DOUBLE_EQ(params.drag_dead_compensation[joint], 0.0);
            }
        }
    }
}

TEST_F(FrictionConfigTest, DragDampingRejectsInvalidCoefficientsAndDefaultsToZero) {
    for (const auto* version : {"v024", "v031"}) {
        config["version"] = version;
        const std::string key = std::string("w1_") + std::string(version).substr(0, 3);
        for (const auto* field :
             {"drag_damping_d1_primary", "drag_damping_d2_primary", "drag_damping_d1_secondary",
              "drag_damping_d2_secondary", "drag_dead_compensation_primary", "drag_dead_compensation_secondary"}) {
            for (const auto* value :
                 {"1", "[1, 2]", "[0, 0, 0, 0, 0, 0, 0, 0]", "[0, 0, 0, -1, 0, 0, 0]", "[0, 0, .nan, 0, 0, 0, 0]",
                  "[0, 0, .inf, 0, 0, 0, 0]", "[0, 0, bad, 0, 0, 0, 0]"}) {
                config[key][field] = YAML::Load(value);
                EXPECT_ANY_THROW(Load()) << field << ": " << value;
            }
            config[key].remove(field);
        }
        const auto model = Load();
        EXPECT_TRUE(model.params_ptr->drag_damping_d1.isZero());
        EXPECT_TRUE(model.params_ptr->drag_damping_d2.isZero());
        EXPECT_TRUE(model.params_ptr->drag_dead_compensation.isZero());
        EXPECT_DOUBLE_EQ(model.params_ptr->drag_dq_deadzone, 0.1);
        for (const auto* value : {"-0.1", ".nan", ".inf", "bad"}) {
            config[key]["drag_dq_deadzone"] = YAML::Load(value);
            EXPECT_ANY_THROW(Load()) << value;
        }
        config[key].remove("drag_dq_deadzone");
    }
}

TEST_F(FrictionConfigTest, DragDampingStartupRepairsMissingFieldsAndPreservesTuning) {
    Write(default_path, config);
    for (const auto* version : {"w1_v02", "w1_v03"}) {
        config[version].remove("drag_damping_d2_primary");
        config[version].remove("drag_damping_d1_secondary");
        config[version].remove("drag_dead_compensation_primary");
        config[version]["drag_damping_d1_primary"][2] = 0.73;
    }
    Write(user_path, config);
    utils::CheckConfig(default_path, user_path);
    const auto repaired = YAML::LoadFile(user_path.string());
    for (const auto* version : {"w1_v02", "w1_v03"}) {
        EXPECT_DOUBLE_EQ(repaired[version]["drag_damping_d1_primary"][2].as<double>(), 0.73);
        EXPECT_EQ(repaired[version]["drag_damping_d2_primary"].size(), 7u);
        EXPECT_EQ(repaired[version]["drag_damping_d1_secondary"].size(), 7u);
        EXPECT_EQ(repaired[version]["drag_dead_compensation_primary"].size(), 7u);
    }
}
