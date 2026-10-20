#include <gtest/gtest.h>
#include <unistd.h>

#include <cmath>

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
    constexpr double velocity_scale = 0.1;
    for (const auto* version : {"v024", "v031"}) {
        config["version"] = version;
        const std::string key = std::string("w1_") + std::string(version).substr(0, 3);
        // Pin the model so this scaling test does not depend on deployment defaults.
        config[key]["friction_model"] = "smooth";
        config[key]["friction_velocity_scale"] = velocity_scale;
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
            for (double speed : {-2.0, -0.1, -0.000872664626, 0.0, 0.000872664626, 0.1, 2.0}) {
                const auto tau = dynamics::InvDynamikBySymbol(model, VectorDof::Zero(), VectorDof::Constant(speed),
                                                              VectorDof::Zero(), Vector3d::Zero(), group);
                for (int j = 0; j < 7; ++j) {
                    const double direction = std::tanh(speed / velocity_scale);
                    EXPECT_NEAR(tau[j], 2.0 * speed * 0.1 * j * k + 3.0 * direction * c * j + 0.4, 1e-12);
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

TEST(DragDampingTest, DeadzoneUsesSinusoidIncludingAtRest) {
    const double pi = std::acos(-1.0);
    const double omega = 2.0 * pi;
    for (double velocity : {-0.199, -0.1, 0.0, 0.1, 0.199}) {
        for (double reference_speed : {0.1, 1.0, 2.0}) {
            // Nonzero feedforward and damping must both be replaced inside the deadzone.
            EXPECT_DOUBLE_EQ(dynamics::ComputeDragTorque(3.7, velocity, 0.6, 0.3, 0.2, reference_speed, omega, 0.0),
                             0.0);
            EXPECT_NEAR(dynamics::ComputeDragTorque(3.7, velocity, 0.6, 0.3, 0.2, reference_speed, omega, 0.25), 0.3,
                        1e-12);
            EXPECT_NEAR(dynamics::ComputeDragTorque(3.7, velocity, 0.6, 0.3, 0.2, reference_speed, omega, 0.75), -0.3,
                        1e-12);
            EXPECT_DOUBLE_EQ(dynamics::ComputeDragTorque(3.7, velocity, 0.6, 0.0, 0.2, reference_speed, omega, 0.25),
                             0.0);
            EXPECT_DOUBLE_EQ(dynamics::ComputeDragTorque(3.7, velocity, 0.6, 0.3, 0.2, reference_speed, 0.0, 0.25),
                             0.0);
        }
    }
    EXPECT_NEAR(dynamics::ComputeDragTorque(3.7, 0.0, 0.6, 0.3, 0.2, 1.0, 4.0 * pi, 0.125), 0.3, 1e-12);
}

TEST(DragDampingTest, AtAndAboveThresholdUsesFeedforwardAndQuadraticDamping) {
    const double feedforward = 3.7;
    const double d2 = 0.6;
    for (double reference_speed : {0.1, 0.5, 1.0, 2.0}) {
        for (double velocity : {-2.0, -0.2, 0.2, 2.0}) {
            for (double time : {0.0, 0.25, 0.75}) {
                const double torque = dynamics::ComputeDragTorque(feedforward, velocity, d2, 0.3, 0.2, reference_speed,
                                                                  2.0 * std::acos(-1.0), time);
                EXPECT_NEAR(torque,
                            feedforward - d2 * std::abs(velocity) * velocity / (reference_speed * reference_speed),
                            1e-12);
                EXPECT_LE((torque - feedforward) * velocity, 0.0);
            }
        }
        // A zero threshold disables the sinusoidal branch, including at zero velocity.
        EXPECT_DOUBLE_EQ(dynamics::ComputeDragTorque(feedforward, 0.0, d2, 0.3, 0.0, reference_speed, 1.0, 0.25),
                         feedforward);
        EXPECT_DOUBLE_EQ(dynamics::ComputeDragTorque(0.0, reference_speed, d2, 0.3, 0.0, reference_speed, 1.0, 0.25),
                         -d2);
        EXPECT_DOUBLE_EQ(dynamics::ComputeDragTorque(0.0, -reference_speed, d2, 0.3, 0.0, reference_speed, 1.0, 0.25),
                         d2);
    }
}

TEST_F(FrictionConfigTest, DragReferenceSpeedLoadsValidatesAndDefaults) {
    for (const auto* version : {"v024", "v031"}) {
        config["version"] = version;
        const std::string key = std::string("w1_") + std::string(version).substr(0, 3);
        for (const auto* field : {"drag_dq_lowspeed_primary", "drag_dq_lowspeed_secondary"}) {
            for (const auto* value :
                 {"1", "[1, 2]", "[1, 1, 1, 1, 1, 1, 1, 1]", "[1, 1, 0, 1, 1, 1, 1]", "[1, 1, -0.1, 1, 1, 1, 1]",
                  "[1, 1, .nan, 1, 1, 1, 1]", "[1, 1, .inf, 1, 1, 1, 1]", "[1, 1, bad, 1, 1, 1, 1]"}) {
                config[key][field] = YAML::Load(value);
                EXPECT_ANY_THROW(Load()) << field << ": " << value;
            }
            config[key].remove(field);
        }
        EXPECT_TRUE(Load().params_ptr->drag_dq_lowspeed.isOnes());
    }
}

TEST_F(FrictionConfigTest, DragDampingMapsEachArmJointAndLeavesOtherJointsUnchanged) {
    for (const auto* version : {"v024", "v031"}) {
        config["version"] = version;
        const std::string key = std::string("w1_") + std::string(version).substr(0, 3);
        for (size_t joint = 0; joint < 7; ++joint) {
            config[key]["drag_dq_lowspeed_primary"][joint] = 0.1 * (joint + 1);
            config[key]["drag_dq_lowspeed_secondary"][joint] = 0.3 * (joint + 1);
            config[key]["drag_damping_d2_primary"][joint] = 0.2 * (joint + 1);
            config[key]["drag_damping_d2_secondary"][joint] = 0.4 * (joint + 1);
            config[key]["f_dead_primary"][joint] = 0.5 * (joint + 1);
            config[key]["f_dead_secondary"][joint] = 0.6 * (joint + 1);
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
                EXPECT_DOUBLE_EQ(params.drag_dq_lowspeed[joint], 0.1 * number);
                EXPECT_DOUBLE_EQ(params.drag_damping_d2[joint], 0.2 * number);
                EXPECT_DOUBLE_EQ(params.drag_f_dead[joint], 0.5 * number);
            } else if (secondary != params.joints_secondary_names.end()) {
                const double number = std::distance(params.joints_secondary_names.begin(), secondary) + 1;
                EXPECT_DOUBLE_EQ(params.drag_dq_lowspeed[joint], 0.3 * number);
                EXPECT_DOUBLE_EQ(params.drag_damping_d2[joint], 0.4 * number);
                EXPECT_DOUBLE_EQ(params.drag_f_dead[joint], 0.6 * number);
            } else {
                EXPECT_DOUBLE_EQ(params.drag_dq_lowspeed[joint], 1.0);
                EXPECT_DOUBLE_EQ(params.drag_damping_d2[joint], 0.0);
                EXPECT_DOUBLE_EQ(params.drag_f_dead[joint], 0.0);
            }
        }
    }
}

TEST_F(FrictionConfigTest, DragDampingRejectsInvalidCoefficientsAndDefaultsToZero) {
    for (const auto* version : {"v024", "v031"}) {
        config["version"] = version;
        const std::string key = std::string("w1_") + std::string(version).substr(0, 3);
        for (const auto* field :
             {"drag_damping_d2_primary", "drag_damping_d2_secondary", "f_dead_primary", "f_dead_secondary"}) {
            for (const auto* value :
                 {"1", "[1, 2]", "[0, 0, 0, 0, 0, 0, 0, 0]", "[0, 0, 0, -1, 0, 0, 0]", "[0, 0, .nan, 0, 0, 0, 0]",
                  "[0, 0, .inf, 0, 0, 0, 0]", "[0, 0, bad, 0, 0, 0, 0]"}) {
                config[key][field] = YAML::Load(value);
                EXPECT_ANY_THROW(Load()) << field << ": " << value;
            }
            config[key].remove(field);
        }
        const auto model = Load();
        EXPECT_TRUE(model.params_ptr->drag_damping_d2.isZero());
        EXPECT_TRUE(model.params_ptr->drag_f_dead.isZero());
        EXPECT_DOUBLE_EQ(model.params_ptr->drag_v_dead, 0.1);
        for (const auto* value : {"-0.1", ".nan", ".inf", "bad"}) {
            config[key]["v_dead"] = YAML::Load(value);
            EXPECT_ANY_THROW(Load()) << value;
        }
        config[key].remove("v_dead");
    }
}

TEST_F(FrictionConfigTest, DragDampingStartupRepairsMissingFieldsAndPreservesTuning) {
    Write(default_path, config);
    for (const auto* version : {"w1_v02", "w1_v03"}) {
        config[version].remove("drag_damping_d2_secondary");
        config[version].remove("f_dead_primary");
        config[version].remove("v_dead");
        config[version].remove("omega");
        config[version].remove("drag_dq_lowspeed_primary");
        config[version].remove("drag_dq_lowspeed_secondary");
        config[version]["drag_damping_d2_primary"][2] = 0.73;
    }
    Write(user_path, config);
    utils::CheckConfig(default_path, user_path);
    const auto repaired = YAML::LoadFile(user_path.string());
    for (const auto* version : {"w1_v02", "w1_v03"}) {
        EXPECT_DOUBLE_EQ(repaired[version]["drag_damping_d2_primary"][2].as<double>(), 0.73);
        EXPECT_EQ(repaired[version]["drag_damping_d2_primary"].size(), 7u);
        EXPECT_EQ(repaired[version]["drag_damping_d2_secondary"].size(), 7u);
        EXPECT_EQ(repaired[version]["f_dead_primary"].size(), 7u);
        EXPECT_DOUBLE_EQ(repaired[version]["v_dead"].as<double>(), 0.1);
        EXPECT_DOUBLE_EQ(repaired[version]["omega"].as<double>(), 6.283185307179586);
        for (const auto* field : {"drag_dq_lowspeed_primary", "drag_dq_lowspeed_secondary"}) {
            ASSERT_EQ(repaired[version][field].size(), 7u);
            for (size_t joint = 0; joint < 7; ++joint) {
                EXPECT_DOUBLE_EQ(repaired[version][field][joint].as<double>(), 1.0);
            }
        }
    }
    config = repaired;
    for (const auto* version : {"w1_v02", "w1_v03"}) {
        config[version]["drag_dq_lowspeed_primary"][2] = 0.25;
        config[version]["drag_dq_lowspeed_secondary"][4] = 0.75;
        config[version]["f_dead_primary"][2] = 0.4;
        config[version]["f_dead_secondary"][4] = 0.8;
        config[version]["v_dead"] = 0.15;
        config[version]["omega"] = 12.0;
    }
    Write(user_path, config);
    utils::CheckConfig(default_path, user_path);
    const auto tuned = YAML::LoadFile(user_path.string());
    for (const auto* version : {"w1_v02", "w1_v03"}) {
        EXPECT_DOUBLE_EQ(tuned[version]["drag_dq_lowspeed_primary"][2].as<double>(), 0.25);
        EXPECT_DOUBLE_EQ(tuned[version]["drag_dq_lowspeed_secondary"][4].as<double>(), 0.75);
        EXPECT_DOUBLE_EQ(tuned[version]["f_dead_primary"][2].as<double>(), 0.4);
        EXPECT_DOUBLE_EQ(tuned[version]["f_dead_secondary"][4].as<double>(), 0.8);
        EXPECT_DOUBLE_EQ(tuned[version]["v_dead"].as<double>(), 0.15);
        EXPECT_DOUBLE_EQ(tuned[version]["omega"].as<double>(), 12.0);
    }
}

TEST_F(FrictionConfigTest, DragSinusoidScalarsLoadValidateAndDefault) {
    for (const auto* version : {"v024", "v031"}) {
        config["version"] = version;
        const std::string key = std::string("w1_") + std::string(version).substr(0, 3);
        for (const auto* field : {"v_dead", "omega"}) {
            for (const auto* value : {"-0.1", ".nan", ".inf", "bad", "[1, 2]"}) {
                config[key][field] = YAML::Load(value);
                EXPECT_ANY_THROW(Load()) << field << ": " << value;
            }
            config[key].remove(field);
        }
        auto model = Load();
        EXPECT_DOUBLE_EQ(model.params_ptr->drag_v_dead, 0.1);
        EXPECT_DOUBLE_EQ(model.params_ptr->drag_omega, 6.283185307179586);
        for (double value : {0.0, 0.25, 12.0}) {
            config[key]["v_dead"] = value;
            config[key]["omega"] = value;
            model = Load();
            EXPECT_DOUBLE_EQ(model.params_ptr->drag_v_dead, value);
            EXPECT_DOUBLE_EQ(model.params_ptr->drag_omega, value);
        }
    }
}
