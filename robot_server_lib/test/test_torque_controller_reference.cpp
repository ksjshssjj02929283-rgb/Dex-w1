#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <memory>

#include "torque_controller_manager.h"

using namespace robot_server_lib;

namespace {
class TorqueControllerReferenceTest : public ::testing::Test {
protected:
    RobotModel model;
    PlannerManagerBuffers manager_buffers;
    TorqueControlBuffer robot_buffers;
    std::unique_ptr<TorqueControllerManager> manager;
    JointRawMeasurement measurement{};
    PlannerManagerCommand planner_command{};
    JointCommand control_reference{};
    std::filesystem::path config_path;

    static void SetUpTestSuite() {
        nanolog::initialize(nanolog::GuaranteedLogger(), std::filesystem::temp_directory_path().string() + "/",
                            "test_torque_controller_reference_" + std::to_string(::getpid()), 1, 1);
    }

    void SetUp() override {
        const auto source = std::filesystem::path(__FILE__).parent_path().parent_path();
        auto config = YAML::LoadFile((source.parent_path() / "w1_server/robot_config.yaml").string());
        config["version"] = "v024";
        config["file_path"]["robot_urdf"] = (source / "robot_models/w1/robot.urdf").string();
        config_path = std::filesystem::temp_directory_path() /
                      ("test_torque_controller_reference_" + std::to_string(::getpid()) + ".yaml");
        {
            std::ofstream out(config_path);
            out << config;
        }
        model = utils::GetRobotModelFromConfig(config_path);
        for (auto& joint : model.robot_property_default.joint_admittance.joints) {
            joint.enabled = false;
        }
        manager_buffers.Initialize();
        robot_buffers.Initialize();
        manager = std::make_unique<TorqueControllerManager>(model, manager_buffers, robot_buffers);

        measurement.position.setZero();
        measurement.velocity.setZero();
        PublishMeasurement();
        control_reference.position.setZero();
        control_reference.position[6] = 0.15;
        control_reference.velocity.setZero();
        control_reference.velocity[6] = 0.25;
        control_reference.acceleration.setZero();
        control_reference.acceleration[6] = 0.5;
        control_reference.torque_feedforward.setZero();
        control_reference.torque_feedforward[6] = 0.7;
        control_reference.joint_stiffness.setConstant(12.0);
        control_reference.joint_damping.setConstant(3.0);
        control_reference.joint_integral_ratio.setZero();
        planner_command.sequence_number = 0;
        planner_command.cmd = control_reference;
        planner_command.cmd.position[6] = 0.8;
        planner_command.cmd.velocity[6] = 0.9;
        planner_command.cmd.acceleration[6] = -0.4;
        planner_command.cmd.torque_feedforward[6] = -0.8;
        planner_command.cmd.joint_stiffness.setConstant(70.0);
        planner_command.cmd.joint_damping.setConstant(30.0);

        PlannerManagerCommand buffered_command = planner_command;
        buffered_command.sequence_number = 100;
        buffered_command.cmd.position[6] = -1.2;
        manager_buffers.planner_manager_command->AppendNewElement(buffered_command);
    }

    void TearDown() override { std::filesystem::remove(config_path); }

    void PublishMeasurement() {
        measurement.time_point = TimePointSystem::Now();
        measurement.torque =
            dynamics::InverseDynamics(model, measurement.position, measurement.velocity, VectorDof::Zero());
        robot_buffers.joint_raw_measurement->AppendNewElement(measurement);
    }

    void StartOperating() {
        manager->LoopOnce(&planner_command, control_reference);
        ++planner_command.sequence_number;
        manager->LoopOnce(&planner_command, control_reference);
    }
};

TEST_F(TorqueControllerReferenceTest, UsesExplicitReferenceForPositionVelocityAndAcceleration) {
    StartOperating();
    TorqueControllerInvDynamic expected_controller(model);
    expected_controller.ResetForNewEpisode(measurement, TorqueControllerState::Operating);
    TorqueCtrlCommand previous{};
    for (int channel = 0; channel < 4; ++channel) {
        SCOPED_TRACE(channel);
        if (channel == 1) {
            control_reference.position[6] += 0.1;
        } else if (channel == 2) {
            control_reference.velocity[6] += 0.2;
        } else if (channel == 3) {
            control_reference.acceleration[6] += 0.3;
        }
        ++planner_command.sequence_number;
        manager->LoopOnce(&planner_command, control_reference);
        TorqueCtrlCommand actual{};
        ASSERT_TRUE(robot_buffers.torque_controller_command->PeekLatest(&actual));
        PlannerManagerCommand expected_command = planner_command;
        expected_command.cmd = control_reference;
        TorqueCtrlCommand expected{};
        ASSERT_TRUE(expected_controller.ComputeCommand(measurement, expected_command, expected));
        EXPECT_TRUE(actual.q_desired.isApprox(control_reference.position, 1e-12));
        EXPECT_TRUE(actual.torque.isApprox(expected.torque, 1e-10));
        EXPECT_TRUE(actual.torque_feedforward.isApprox(expected.torque_feedforward, 1e-10));
        EXPECT_TRUE(actual.torque_impedance.isApprox(expected.torque_impedance, 1e-10));
        if (channel == 1 || channel == 2) {
            EXPECT_GT(actual.torque[6] - previous.torque[6], 0.1);
            EXPECT_TRUE(actual.torque_feedforward.isApprox(previous.torque_feedforward, 1e-10));
        } else if (channel == 3) {
            EXPECT_GT((actual.torque_feedforward - previous.torque_feedforward).norm(), 1e-4);
        }
        JointMeasurement published{};
        ASSERT_TRUE(manager_buffers.joint_measurement->PeekLatest(&published));
        EXPECT_TRUE(published.q_desired_pre.isApprox(planner_command.cmd.position, 1e-12));
        previous = actual;
    }
}

TEST_F(TorqueControllerReferenceTest, RetainsReferenceWhileStoppingAndRecoversAfterMissingMeasurement) {
    StartOperating();
    measurement.position[6] = 0.12;
    measurement.velocity[6] = 0.2;
    PublishMeasurement();
    JointCommand unused_reference = control_reference;
    unused_reference.joint_damping.setConstant(50.0);
    manager->LoopOnce(nullptr, unused_reference);

    TorqueControllerInvDynamic expected_controller(model);
    expected_controller.ResetForNewEpisode(measurement, TorqueControllerState::Decelerating);
    PlannerManagerCommand expected_command = planner_command;
    expected_command.cmd = control_reference;
    TorqueCtrlCommand expected{};
    ASSERT_TRUE(expected_controller.ComputeCommand(measurement, expected_command, expected));
    TorqueCtrlCommand actual{};
    ASSERT_TRUE(robot_buffers.torque_controller_command->PeekLatest(&actual));
    EXPECT_TRUE(actual.torque.isApprox(expected.torque, 1e-10));
    JointMeasurement published{};
    ASSERT_TRUE(manager_buffers.joint_measurement->PeekLatest(&published));
    EXPECT_TRUE(published.q_desired_pre.isApprox(measurement.position, 1e-12));

    robot_buffers.joint_raw_measurement->Reset();
    manager->LoopOnce(nullptr, unused_reference);
    EXPECT_FALSE(robot_buffers.torque_controller_command->PeekLatest(&actual));
    EXPECT_FALSE(manager_buffers.joint_measurement->PeekLatest(&published));

    measurement.position[6] = 0.3;
    measurement.velocity.setZero();
    PublishMeasurement();
    control_reference.position[6] = 0.4;
    ++planner_command.sequence_number;
    manager->LoopOnce(&planner_command, control_reference);
    ASSERT_TRUE(robot_buffers.torque_controller_command->PeekLatest(&actual));
    EXPECT_TRUE(actual.q_desired.isApprox(measurement.position, 1e-12));
    ASSERT_TRUE(manager_buffers.joint_measurement->PeekLatest(&published));
    EXPECT_TRUE(published.q_desired_pre.isApprox(measurement.position, 1e-12));

    ++planner_command.sequence_number;
    manager->LoopOnce(&planner_command, control_reference);
    expected_controller.ResetForNewEpisode(measurement, TorqueControllerState::Operating);
    expected_command = planner_command;
    expected_command.cmd = control_reference;
    ASSERT_TRUE(expected_controller.ComputeCommand(measurement, expected_command, expected));
    ASSERT_TRUE(robot_buffers.torque_controller_command->PeekLatest(&actual));
    EXPECT_TRUE(actual.q_desired.isApprox(control_reference.position, 1e-12));
    EXPECT_TRUE(actual.torque.isApprox(expected.torque, 1e-10));
    ASSERT_TRUE(manager_buffers.joint_measurement->PeekLatest(&published));
    EXPECT_TRUE(published.q_desired_pre.isApprox(planner_command.cmd.position, 1e-12));
}
}  // namespace
