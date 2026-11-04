#include <gtest/gtest.h>

#include "joint_reference_processor.h"

using namespace robot_server_lib;

namespace {

RobotProperty Properties(bool filter_enabled = true, bool shaper_enabled = true) {
    RobotProperty properties{};
    for (int i = 0; i < ndof_robot; ++i) {
        properties.enable_butterworth_filter[i] = filter_enabled;
        properties.enable_input_shaper[i] = shaper_enabled;
    }
    return properties;
}

JointCommand Command(double position = 0.0) {
    JointCommand command{};
    command.position.setConstant(position);
    command.velocity.setConstant(19.0);
    command.acceleration.setConstant(-31.0);
    command.torque_feedforward.setConstant(2.0);
    command.joint_stiffness.setConstant(300.0);
    command.joint_damping.setConstant(25.0);
    command.joint_integral_ratio.setConstant(0.02);
    return command;
}

TEST(JointReferenceProcessorTest, RetainsExistingPositionChainAndCommandParameters) {
    auto properties = Properties();
    properties.butterworth_filter_frequency[0] = 7.0;
    properties.input_shaper_frequency[0] = 11.0;
    JointReferenceProcessor processor(properties);
    utils::LinearInterpolator interpolator(manager_dt_micro / torque_ctrl_dt_micro);
    ButterworthFilter filter(properties.butterworth_filter_frequency[0]);
    EIInputShaper shaper(EIInputShaper::EI, properties.input_shaper_frequency[0], torque_ctrl_dt, 10 * M_PI);
    auto command = Command(0.4);
    for (int step = 0; step < 600; ++step) {
        if (step % 4 == 0) {
            command.position.setConstant(step < 300 ? 0.4 + step * 0.01 : -0.2);
        }
        const auto reference = processor.Process(command, properties);
        const auto interpolated = interpolator.Interpolate(command.position);
        const double filtered = filter.Apply(interpolated[0]);
        EXPECT_DOUBLE_EQ(processor.PositionBeforeFilter()[0], interpolated[0]);
        EXPECT_DOUBLE_EQ(processor.PositionAfterFilter()[0], filtered);
        EXPECT_DOUBLE_EQ(reference.position[0], shaper.Apply(filtered));
        EXPECT_TRUE(reference.torque_feedforward.isApprox(command.torque_feedforward));
        EXPECT_TRUE(reference.joint_stiffness.isApprox(command.joint_stiffness));
        EXPECT_TRUE(reference.joint_damping.isApprox(command.joint_damping));
        EXPECT_TRUE(reference.joint_integral_ratio.isApprox(command.joint_integral_ratio));
    }
}

TEST(JointReferenceProcessorTest, DerivativesTrackProcessedPositionThroughMotionAndStop) {
    const auto properties = Properties();
    JointReferenceProcessor processor(properties);
    auto command = Command(0.3);
    auto previous = processor.Process(command, properties);
    for (int step = 1; step < 1600; ++step) {
        if (step % 4 == 0 && step < 800) {
            command.position.setConstant(0.3 + step * 0.0004);
        }
        const auto reference = processor.Process(command, properties);
        EXPECT_LT((reference.position - previous.position - torque_ctrl_dt * reference.velocity).norm(), 1e-12);
        EXPECT_LT((reference.velocity - previous.velocity - torque_ctrl_dt * reference.acceleration).norm(), 1e-12);
        if (step > 600 && step < 780) {
            EXPECT_NEAR(reference.velocity[0], 0.4, 0.01);
            EXPECT_NEAR(reference.acceleration[0], 0.0, 0.1);
        }
        previous = reference;
    }
    EXPECT_NEAR(previous.velocity[0], 0.0, 1e-7);
    EXPECT_NEAR(previous.acceleration[0], 0.0, 1e-5);
}

TEST(JointReferenceProcessorTest, FirstFrameAndResetDiscardDerivativeAndFilterHistory) {
    const auto properties = Properties();
    JointReferenceProcessor processor(properties);
    auto command = Command(0.7);
    auto reference = processor.Process(command, properties);
    EXPECT_TRUE(reference.position.isApprox(command.position));
    EXPECT_TRUE(reference.velocity.isZero());
    EXPECT_TRUE(reference.acceleration.isZero());
    for (int step = 1; step < 100; ++step) {
        command.position.setConstant(0.7 + step * 0.002);
        processor.Process(command, properties);
    }
    processor.Reset();
    command.position.setConstant(-0.8);
    reference = processor.Process(command, properties);
    EXPECT_TRUE(reference.position.isApprox(command.position));
    EXPECT_TRUE(reference.velocity.isZero());
    EXPECT_TRUE(reference.acceleration.isZero());
    reference = processor.Process(command, properties);
    EXPECT_LT(reference.velocity.norm(), 1e-10);
    EXPECT_LT(reference.acceleration.norm(), 1e-7);
}

TEST(JointReferenceProcessorTest, JointSwitchesBypassAndResetTheirOwnHistory) {
    auto properties = Properties();
    properties.enable_butterworth_filter[1] = false;
    properties.enable_input_shaper[1] = false;
    JointReferenceProcessor processor(properties);
    auto command = Command();
    processor.Process(command, properties);
    command.position.setOnes();
    JointCommand reference{};
    for (int step = 0; step < 4; ++step) {
        reference = processor.Process(command, properties);
    }
    EXPECT_DOUBLE_EQ(reference.position[1], 1.0);
    EXPECT_LT(reference.position[0], 0.1);

    properties.enable_butterworth_filter[0] = false;
    properties.enable_input_shaper[0] = false;
    reference = processor.Process(command, properties);
    EXPECT_DOUBLE_EQ(reference.position[0], 1.0);
    properties.enable_butterworth_filter[0] = true;
    properties.enable_input_shaper[0] = true;
    reference = processor.Process(command, properties);
    EXPECT_DOUBLE_EQ(reference.position[0], 1.0);

    properties.enable_butterworth_filter[1] = true;
    properties.enable_input_shaper[1] = true;
    reference = processor.Process(command, properties);
    EXPECT_DOUBLE_EQ(reference.position[1], 1.0);
}

TEST(JointReferenceProcessorTest, ResetInterpolationRestartsAtTargetWithoutClearingOutputHistory) {
    const auto properties = Properties(false, false);
    JointReferenceProcessor processor(properties);
    auto command = Command();
    processor.Process(command, properties);
    command.position.setConstant(0.004);
    auto reference = processor.Process(command, properties);
    EXPECT_NEAR(reference.position[0], 0.001, 1e-15);
    processor.ResetInterpolation();
    reference = processor.Process(command, properties);
    EXPECT_DOUBLE_EQ(reference.position[0], 0.004);
    EXPECT_NEAR(reference.velocity[0], 3.0, 1e-12);
}

}  // namespace
