#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "joint_admittance_controller.h"

using namespace robot_server_lib;

TEST(JointAdmittanceControllerTest, DerivesStiffnessAndDampingFromFrequency) {
    JointAdmittanceConfig config;
    config.enabled = true;
    config.joints[6].enabled = true;
    config.joints[6].virtual_inertia = 2.0;
    config.joints[6].natural_frequency_hz = 1.5;
    config.joints[6].damping_ratio = 1.2;

    dynamics::JointAdmittanceController controller(config);

    const double omega = 2.0 * M_PI * 1.5;
    EXPECT_NEAR(controller.stiffness()[6], 2.0 * omega * omega, 1e-9);
    EXPECT_NEAR(controller.damping()[6], 2.0 * 1.2 * 2.0 * omega, 1e-9);
}

TEST(JointAdmittanceControllerTest, PositiveTorqueProducesPositiveOffset) {
    JointAdmittanceConfig config;
    config.enabled = true;
    config.joints[6].enabled = true;
    config.joints[6].torque_gain = 1.0;
    config.joints[6].virtual_inertia = 1.0;
    config.joints[6].natural_frequency_hz = 1.0;
    config.joints[6].damping_ratio = 1.0;
    config.joints[6].offset_limit = 1.0;
    config.joints[6].velocity_limit = 10.0;
    config.joints[6].acceleration_limit = 100.0;

    dynamics::JointAdmittanceController controller(config);
    VectorDof torque_ext = VectorDof::Zero();
    torque_ext[6] = 1.0;
    VectorDof q_offset = VectorDof::Zero();
    VectorDof dq_offset = VectorDof::Zero();

    ASSERT_TRUE(controller.Update(torque_ext, 0.001, q_offset, dq_offset));
    EXPECT_GT(q_offset[6], 0.0);
    EXPECT_GT(dq_offset[6], 0.0);
}

TEST(JointAdmittanceControllerTest, ClampsOffsetVelocityAndAcceleration) {
    JointAdmittanceConfig config;
    config.enabled = true;
    config.joints[6].enabled = true;
    config.joints[6].virtual_inertia = 1.0;
    config.joints[6].natural_frequency_hz = 1.0;
    config.joints[6].damping_ratio = 0.1;
    config.joints[6].offset_limit = 0.001;
    config.joints[6].velocity_limit = 0.01;
    config.joints[6].acceleration_limit = 0.1;

    dynamics::JointAdmittanceController controller(config);
    VectorDof torque_ext = VectorDof::Zero();
    torque_ext[6] = 1000.0;
    VectorDof q_offset = VectorDof::Zero();
    VectorDof dq_offset = VectorDof::Zero();

    ASSERT_TRUE(controller.Update(torque_ext, 0.1, q_offset, dq_offset));
    EXPECT_LE(std::abs(controller.ddq_offset()[6]), 0.1 + 1e-12);
    EXPECT_LE(std::abs(dq_offset[6]), 0.01 + 1e-12);
    EXPECT_LE(std::abs(q_offset[6]), 0.001 + 1e-12);
}

TEST(JointAdmittanceControllerTest, OffsetLimitClearsOutwardVelocityAndAcceleration) {
    JointAdmittanceConfig config;
    config.enabled = true;
    config.joints[6].enabled = true;
    config.joints[6].torque_gain = 1.0;
    config.joints[6].virtual_inertia = 1.0;
    config.joints[6].natural_frequency_hz = 1.0;
    config.joints[6].damping_ratio = 0.1;
    config.joints[6].offset_limit = 0.001;
    config.joints[6].velocity_limit = 100.0;
    config.joints[6].acceleration_limit = 1000.0;

    dynamics::JointAdmittanceController controller(config);
    VectorDof torque_ext = VectorDof::Zero();
    torque_ext[6] = 1000.0;
    VectorDof q_offset = VectorDof::Zero();
    VectorDof dq_offset = VectorDof::Zero();

    ASSERT_TRUE(controller.Update(torque_ext, 0.01, q_offset, dq_offset));
    EXPECT_NEAR(q_offset[6], config.joints[6].offset_limit, 1e-12);
    EXPECT_NEAR(dq_offset[6], 0.0, 1e-12);
    EXPECT_NEAR(controller.ddq_offset()[6], 0.0, 1e-12);

    ASSERT_TRUE(controller.Update(torque_ext, 0.01, q_offset, dq_offset));
    EXPECT_NEAR(q_offset[6], config.joints[6].offset_limit, 1e-12);
    EXPECT_NEAR(dq_offset[6], 0.0, 1e-12);
    EXPECT_NEAR(controller.ddq_offset()[6], 0.0, 1e-12);
}

TEST(JointAdmittanceControllerTest, DisabledControllerOutputsZeroOffsets) {
    JointAdmittanceConfig config;
    config.enabled = false;
    config.joints[6].enabled = true;

    dynamics::JointAdmittanceController controller(config);
    VectorDof torque_ext = VectorDof::Constant(10.0);
    VectorDof q_offset = VectorDof::Constant(1.0);
    VectorDof dq_offset = VectorDof::Constant(1.0);

    ASSERT_TRUE(controller.Update(torque_ext, 0.001, q_offset, dq_offset));
    EXPECT_TRUE(q_offset.isZero(1e-12));
    EXPECT_TRUE(dq_offset.isZero(1e-12));
}

TEST(JointAdmittanceControllerTest, DecayToZeroReducesOffsetMagnitude) {
    JointAdmittanceConfig config;
    config.enabled = true;
    config.joints[6].enabled = true;
    config.joints[6].virtual_inertia = 1.0;
    config.joints[6].natural_frequency_hz = 1.0;
    config.joints[6].damping_ratio = 1.5;
    config.joints[6].offset_limit = 1.0;
    config.joints[6].velocity_limit = 10.0;
    config.joints[6].acceleration_limit = 100.0;

    dynamics::JointAdmittanceController controller(config);
    VectorDof torque_ext = VectorDof::Zero();
    torque_ext[6] = 5.0;
    VectorDof q_offset = VectorDof::Zero();
    VectorDof dq_offset = VectorDof::Zero();
    for (int i = 0; i < 100; ++i) {
        controller.Update(torque_ext, 0.001, q_offset, dq_offset);
    }
    const double before = std::abs(q_offset[6]);
    torque_ext.setZero();
    for (int i = 0; i < 1000; ++i) {
        controller.DecayToZero(0.001);
    }
    EXPECT_LT(std::abs(controller.q_offset()[6]), before);
}

TEST(JointAdmittanceControllerTest, InvalidConfigFallsBackToSafeDefaults) {
    JointAdmittanceConfig config;
    config.enabled = true;
    config.joints[6].enabled = true;
    config.joints[6].virtual_inertia = -1.0;
    config.joints[6].natural_frequency_hz = 0.0;
    config.joints[6].damping_ratio = -2.0;
    config.joints[6].offset_limit = -0.1;
    config.joints[6].velocity_limit = -0.2;
    config.joints[6].acceleration_limit = -0.3;

    dynamics::JointAdmittanceController controller(config);
    VectorDof torque_ext = VectorDof::Constant(100.0);
    VectorDof q_offset = VectorDof::Ones();
    VectorDof dq_offset = VectorDof::Ones();

    ASSERT_TRUE(controller.Update(torque_ext, 0.001, q_offset, dq_offset));
    EXPECT_TRUE(q_offset.allFinite());
    EXPECT_TRUE(dq_offset.allFinite());
    EXPECT_TRUE(controller.ddq_offset().allFinite());
    EXPECT_GE(controller.stiffness()[6], 0.0);
    EXPECT_GE(controller.damping()[6], 0.0);
}

TEST(JointAdmittanceControllerTest, NonFiniteTorqueInputDoesNotPolluteState) {
    JointAdmittanceConfig config;
    config.enabled = true;
    config.joints[6].enabled = true;

    dynamics::JointAdmittanceController controller(config);
    VectorDof torque_ext = VectorDof::Zero();
    torque_ext[6] = std::numeric_limits<double>::quiet_NaN();
    VectorDof q_offset = VectorDof::Ones();
    VectorDof dq_offset = VectorDof::Ones();

    ASSERT_TRUE(controller.Update(torque_ext, 0.001, q_offset, dq_offset));
    EXPECT_TRUE(q_offset.allFinite());
    EXPECT_TRUE(dq_offset.allFinite());
    EXPECT_TRUE(controller.ddq_offset().allFinite());
}
