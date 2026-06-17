#include <gtest/gtest.h>

#include <filesystem>

#include "torque_controller_inverse_dynamic.h"
#include "utils.h"

using namespace robot_server_lib;

namespace {
RobotModel LoadW1RobotModel() {
    const auto package_share_path = ament_index_cpp::get_package_share_directory("w1_server");
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot_config.yaml";
    return utils::GetRobotModelFromConfig(robot_cfg_path);
}

void EnablePrimaryJointAdmittanceForTest(RobotModel& robot_model) {
    auto& config = robot_model.robot_property_default.joint_admittance;
    config.enabled = true;
    config.torque_ext_direct_gain = 0.0;
    for (int i = 6; i < 13; ++i) {
        config.joints[i].enabled = true;
        config.joints[i].torque_gain = 1.0;
        config.joints[i].virtual_inertia = 1.0;
        config.joints[i].natural_frequency_hz = 1.5;
        config.joints[i].damping_ratio = 1.2;
        config.joints[i].offset_limit = 0.05;
        config.joints[i].velocity_limit = 0.4;
        config.joints[i].acceleration_limit = 5.0;
    }
}

JointRawMeasurement MakeRawMeasurementWithPrimaryWrench() {
    JointRawMeasurement raw{};
    raw.position.setZero();
    raw.velocity.setZero();
    raw.effort.setZero();
    raw.sensor_wrench.setZero();
    raw.sensor_wrench[0] = 20.0;
    raw.sensor_wrench[2] = 10.0;
    return raw;
}

PlannerManagerCommand MakeOperatingCommand() {
    PlannerManagerCommand input{};
    input.sequence_number = 1;
    input.cmd.position.setZero();
    input.cmd.velocity.setZero();
    input.cmd.acceleration.setZero();
    input.cmd.torque_feedforward.setZero();
    input.cmd.joint_stiffness = VectorDof::Constant(100.0);
    input.cmd.joint_damping = VectorDof::Constant(10.0);
    input.cmd.joint_integral_ratio = VectorDof::Constant(0.0);
    return input;
}
}  // namespace

TEST(TorqueControllerJointAdmittanceTest, RawTorqueExtMovesAdmittanceOffset) {
    RobotModel robot_model = LoadW1RobotModel();
    ASSERT_TRUE(robot_model.model_ptr != nullptr);
    EnablePrimaryJointAdmittanceForTest(robot_model);

    TorqueControllerInvDynamic controller(robot_model);
    JointRawMeasurement raw = MakeRawMeasurementWithPrimaryWrench();
    PlannerManagerCommand input = MakeOperatingCommand();

    ASSERT_TRUE(controller.ResetForNewEpisode(raw, TorqueControllerState::Operating));

    TorqueCtrlCommand output{};
    for (int i = 0; i < 50; ++i) {
        ASSERT_TRUE(controller.ComputeCommand(raw, input, output));
    }

    EXPECT_GT(output.torque_ext.segment<7>(6).norm(), 0.0);
    EXPECT_GT(output.joint_admittance_q_offset.segment<7>(6).norm(), 0.0);
    EXPECT_GT(output.joint_position_ref_admittance.segment<7>(6).norm(), 0.0);
}

TEST(TorqueControllerJointAdmittanceTest, DirectTorqueExtIsNotAddedWhenDirectGainIsZero) {
    RobotModel robot_model = LoadW1RobotModel();
    ASSERT_TRUE(robot_model.model_ptr != nullptr);
    EnablePrimaryJointAdmittanceForTest(robot_model);
    ASSERT_TRUE(robot_model.robot_property_default.joint_admittance.enabled);
    ASSERT_DOUBLE_EQ(robot_model.robot_property_default.joint_admittance.torque_ext_direct_gain, 0.0);

    TorqueControllerInvDynamic controller(robot_model);
    JointRawMeasurement raw = MakeRawMeasurementWithPrimaryWrench();
    PlannerManagerCommand input = MakeOperatingCommand();
    input.cmd.joint_stiffness.setZero();
    input.cmd.joint_damping.setZero();

    ASSERT_TRUE(controller.ResetForNewEpisode(raw, TorqueControllerState::Operating));

    TorqueCtrlCommand output{};
    ASSERT_TRUE(controller.ComputeCommand(raw, input, output));

    ASSERT_GT(output.torque_ext.segment<7>(6).norm(), 0.0);
    EXPECT_TRUE((output.torque - output.torque_feedforward).isZero(1e-9));
}

TEST(TorqueControllerJointAdmittanceTest, DeceleratingEpisodeKeepsAdmittanceOffsetForDecay) {
    RobotModel robot_model = LoadW1RobotModel();
    ASSERT_TRUE(robot_model.model_ptr != nullptr);
    EnablePrimaryJointAdmittanceForTest(robot_model);

    TorqueControllerInvDynamic controller(robot_model);
    JointRawMeasurement raw = MakeRawMeasurementWithPrimaryWrench();
    PlannerManagerCommand input = MakeOperatingCommand();

    ASSERT_TRUE(controller.ResetForNewEpisode(raw, TorqueControllerState::Operating));

    TorqueCtrlCommand output{};
    for (int i = 0; i < 50; ++i) {
        ASSERT_TRUE(controller.ComputeCommand(raw, input, output));
    }
    const double operating_offset_norm = output.joint_admittance_q_offset.segment<7>(6).norm();
    ASSERT_GT(operating_offset_norm, 0.0);

    raw.sensor_wrench.setZero();
    ASSERT_TRUE(controller.ResetForNewEpisode(raw, TorqueControllerState::Decelerating));
    ASSERT_TRUE(controller.ComputeCommand(raw, input, output));

    EXPECT_GT(output.joint_admittance_q_offset.segment<7>(6).norm(), 0.0);
}

TEST(TorqueControllerJointAdmittanceTest, DisabledAdmittancePreservesDirectExternalTorque) {
    RobotModel robot_model = LoadW1RobotModel();
    ASSERT_TRUE(robot_model.model_ptr != nullptr);
    robot_model.robot_property_default.joint_admittance.enabled = false;

    TorqueControllerInvDynamic controller(robot_model);
    JointRawMeasurement raw = MakeRawMeasurementWithPrimaryWrench();
    PlannerManagerCommand input = MakeOperatingCommand();
    input.cmd.joint_stiffness.setZero();
    input.cmd.joint_damping.setZero();

    ASSERT_TRUE(controller.ResetForNewEpisode(raw, TorqueControllerState::Operating));

    TorqueCtrlCommand output{};
    ASSERT_TRUE(controller.ComputeCommand(raw, input, output));

    ASSERT_GT(output.torque_ext.segment<7>(6).norm(), 0.0);
    EXPECT_TRUE((output.torque - output.torque_feedforward - output.torque_ext).isZero(1e-9));
    EXPECT_TRUE(output.joint_admittance_q_offset.isZero(1e-12));
    EXPECT_TRUE(output.joint_admittance_dq_offset.isZero(1e-12));
}
