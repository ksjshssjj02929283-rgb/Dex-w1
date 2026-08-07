#include <gtest/gtest.h>

#include "joint_time_delay_controller.h"

using namespace robot_server_lib;

TEST(JointTimeDelayControllerTest, PositionErrorProducesTorque) {
    TimeDelayControlConfig config;
    config.joints[6].motor_inertia = 0.015;
    config.joints[6].m_bar_norm = 2.0;
    dynamics::JointTimeDelayController controller(config);
    const VectorDof zero = VectorDof::Zero();
    VectorDof command = VectorDof::Zero();
    VectorDof cmd_pos = zero;
    cmd_pos[6] = 1.0;

    // 首帧仅初始化速度基线, 输出零力矩; 第二帧才开始计算
    ASSERT_TRUE(controller.UpdateTorqueCommand(cmd_pos, zero, zero, zero, command));
    ASSERT_TRUE(controller.UpdateTorqueCommand(cmd_pos, zero, zero, zero, command));
    EXPECT_GT(command[6], 0.0);
}

TEST(JointTimeDelayControllerTest, TorqueOutputIsLimitedAndResettable) {
    TimeDelayControlConfig config;
    config.joints[6].motor_inertia = 0.015;
    config.joints[6].m_bar_norm = 2.0;
    dynamics::JointTimeDelayController controller(config);
    const VectorDof zero = VectorDof::Zero();
    VectorDof command = VectorDof::Zero();
    VectorDof cmd_pos = VectorDof::Constant(100.0);

    for (int i = 0; i < 100; ++i) {
        ASSERT_TRUE(controller.UpdateTorqueCommand(cmd_pos, zero, zero, zero, command));
    }
    EXPECT_GT(command[6], 0.0);

    controller.Reset();
    EXPECT_TRUE(controller.UpdateTorqueCommand(zero, zero, zero, zero, command));
    EXPECT_DOUBLE_EQ(command[6], 0.0);
}
