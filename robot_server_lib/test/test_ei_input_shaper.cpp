/**
 * @file test_ei_input_shaper.cpp
 * @brief EI Input Shaper 单元测试
 *
 * 测试内容：
 * 1. 单位阶跃响应
 * 2. 零输入响应
 * 3. 斜坡响应
 * 4. 重置功能
 * 5. 双峰 EI 算法
 * 6. Ramp 功能（单通道）
 * 7. Ramp 功能（双通道）
 * 8. Ramp 方向变化
 */

#include <gtest/gtest.h>

#include <cmath>

#include "../utils/ei_input_shaper.h"

// 测试 1: 单位阶跃响应 - 验证稳态增益为 1
TEST(EIInputShaperTest, UnitStepResponse) {
    const double natural_frequency_hz = 1.67;
    const double damping_ratio = 0.05;
    const double vibration_tolerance = 0.05;

    EIInputShaper shaper(EIInputShaper::EI, natural_frequency_hz, 0.001, 0.0, vibration_tolerance);

    double final_output = 0.0;
    for (int i = 0; i < 100; ++i) {
        final_output = shaper.Apply(1.0);
    }
    EXPECT_NEAR(final_output, 1.0, 1e-10);
}

// 测试 2: 零输入响应 - 验证零输入时输出为零
TEST(EIInputShaperTest, ZeroInputResponse) {
    const double natural_frequency_hz = 1.67;
    const double damping_ratio = 0.05;
    const double vibration_tolerance = 0.05;

    EIInputShaper shaper(EIInputShaper::EI, natural_frequency_hz, 0.001, 0.0, vibration_tolerance);

    for (int i = 0; i < 100; ++i) {
        double output = shaper.Apply(0.0);
        EXPECT_NEAR(output, 0.0, 1e-10);
    }
}

// 测试 3: 斜坡响应 - 验证恒定延迟特性
TEST(EIInputShaperTest, RampResponse) {
    const double natural_frequency_hz = 1.67;
    const double damping_ratio = 0.05;
    const double vibration_tolerance = 0.05;

    EIInputShaper shaper(EIInputShaper::EI, natural_frequency_hz, 0.001, 0.0, vibration_tolerance);

    double slope = 0.01;
    double steady_state_error = 0.0;

    for (int i = 0; i < 100; ++i) {
        double input = i * slope;
        double output = shaper.Apply(input);

        if (i > 50) {
            double error = output - input;
            if (i == 51) {
                steady_state_error = error;
            } else {
                EXPECT_NEAR(error, steady_state_error, 1e-8);
            }
        }
    }
}

// 测试 4: 重置功能 - 验证 Reset 后状态正确恢复
TEST(EIInputShaperTest, ResetFunctionality) {
    const double natural_frequency_hz = 1.67;
    const double damping_ratio = 0.05;
    const double vibration_tolerance = 0.05;

    EIInputShaper shaper(EIInputShaper::EI, natural_frequency_hz, 0.001, 0.0, vibration_tolerance);

    for (int i = 0; i < 50; ++i) {
        shaper.Apply(1.0);
    }

    shaper.Reset();

    double first_output = shaper.Apply(2.0);
    EXPECT_NEAR(first_output, 2.0, 1e-10);
}

// 测试 5: 双峰 EI 算法 - 验证 Two-hump EI 稳态增益为 1
TEST(EIInputShaperTest, TwoHumpEI) {
    const double natural_frequency_hz = 1.67;
    const double damping_ratio = 0.05;
    const double vibration_tolerance = 0.05;

    EIInputShaper shaper(EIInputShaper::TWO_HUMP_EI, natural_frequency_hz, 0.001, 0.0, vibration_tolerance);

    double final_output = 0.0;
    for (int i = 0; i < 200; ++i) {
        final_output = shaper.Apply(1.0);
    }
    EXPECT_NEAR(final_output, 1.0, 1e-10);
}

// 测试 6: Ramp 功能（单通道）- 验证单通道斜坡限制功能
TEST(EIInputShaperTest, RampFunction) {
    const double natural_frequency_hz = 1.67;
    const double damping_ratio = 0.05;
    const double vibration_tolerance = 0.05;

    EIInputShaper shaper(EIInputShaper::EI, natural_frequency_hz, 0.001, 0.0, vibration_tolerance);

    double output1 = shaper.Apply(1.0);
    EXPECT_NEAR(output1, 0.01, 1e-10);

    double output2 = shaper.Apply(1.0);
    EXPECT_NEAR(output2, 0.02, 1e-10);

    double output3 = shaper.Apply(1.0);
    EXPECT_NEAR(output3, 0.03, 1e-10);

    for (int i = 0; i < 100; ++i) {
        shaper.Apply(1.0);
    }
    double final_output = shaper.Apply(1.0);
    EXPECT_NEAR(final_output, 1.0, 1e-10);
}

// 测试 6: Ramp 功能（双通道）- 验证双通道斜坡限制功能
TEST(EIInputShaperTest, RampFunctionDualChannel) {
    const double natural_frequency_hz = 1.67;
    const double damping_ratio = 0.05;
    const double vibration_tolerance = 0.05;

    EIInputShaper shaper(EIInputShaper::EI, natural_frequency_hz, 0.001, 0.0, vibration_tolerance);

    auto [output_r1, output_l1] = shaper.Apply(1.0, 2.0);
    EXPECT_NEAR(output_r1, 0.01, 1e-10);
    EXPECT_NEAR(output_l1, 0.01, 1e-10);

    auto [output_r2, output_l2] = shaper.Apply(1.0, 2.0);
    EXPECT_NEAR(output_r2, 0.02, 1e-10);
    EXPECT_NEAR(output_l2, 0.02, 1e-10);

    for (int i = 0; i < 200; ++i) {
        shaper.Apply(1.0, 2.0);
    }
    auto [final_r, final_l] = shaper.Apply(1.0, 2.0);
    EXPECT_NEAR(final_r, 1.0, 1e-10);
    EXPECT_NEAR(final_l, 2.0, 1e-10);
}

// 测试 7: Ramp 方向变化 - 验证 Ramp 能够正确处理方向反转
TEST(EIInputShaperTest, RampDirectionChange) {
    const double natural_frequency_hz = 1.67;
    const double damping_ratio = 0.05;
    const double vibration_tolerance = 0.05;

    EIInputShaper shaper(EIInputShaper::EI, natural_frequency_hz, 0.001, 0.0, vibration_tolerance);

    for (int i = 0; i < 100; ++i) {
        shaper.Apply(1.0);
    }
    double output1 = shaper.Apply(1.0);
    EXPECT_NEAR(output1, 1.0, 1e-2);

    double output2 = shaper.Apply(0.0);
    EXPECT_NEAR(output2, 0.99, 1e-2);

    double output3 = shaper.Apply(0.0);
    EXPECT_NEAR(output3, 0.98, 1e-2);
}

// 测试 8: NONE类型 - 不滤波但支持ramp
TEST(EIInputShaperTest, NoneTypeNoFilterWithRamp) {
    const double natural_frequency_hz = 1.67;
    const double vibration_tolerance = 0.05;

    EIInputShaper shaper(EIInputShaper::NONE, natural_frequency_hz, 0.001, 0.0, vibration_tolerance);

    // 测试不滤波 - 直接透传
    double output1 = shaper.Apply(5.0);
    EXPECT_NEAR(output1, 5.0, 1e-10);

    double output2 = shaper.Apply(10.0);
    EXPECT_NEAR(output2, 10.0, 1e-10);

    // 测试双通道不滤波
    auto [output_r1, output_l1] = shaper.Apply(3.0, 7.0);
    EXPECT_NEAR(output_r1, 3.0, 1e-10);
    EXPECT_NEAR(output_l1, 7.0, 1e-10);

    // 启用ramp - 验证NONE类型支持ramp功能
    shaper.Reset();

    // 第一次调用设为输入值
    double ramp_output1 = shaper.Apply(1.0);
    EXPECT_DOUBLE_EQ(ramp_output1, 1.0);

    // 改变目标值，验证ramp会逐步接近新目标
    double prev = ramp_output1;
    for (int i = 0; i < 15; ++i) {
        double curr = shaper.Apply(2.0);
        // ramp应该逐步增加或保持不变（达到目标后）
        EXPECT_GE(curr, prev);
        EXPECT_LE(curr, 2.0);
        prev = curr;
    }
    // 最终应该达到或接近目标值2.0
    EXPECT_NEAR(prev, 2.0, 0.15);
}
