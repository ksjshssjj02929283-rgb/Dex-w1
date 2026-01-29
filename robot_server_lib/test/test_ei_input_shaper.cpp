/**
 * @file test_ei_input_shaper.cpp
 * @brief EI Input Shaper 单元测试
 *
 * 测试内容：
 * 1. 系数计算验证（与 MATLAB 参考代码对比）
 * 2. 延迟计算验证
 * 3. 单位阶跃响应测试
 * 4. 零输入响应测试
 * 5. 斜坡响应测试
 * 6. 重置功能测试
 * 7. 重新配置测试
 * 8. 与 ZVD 对比测试
 */

#include <gtest/gtest.h>

#include <cmath>

#include "../utils/ei_input_shaper.h"

// 测试夹具
class EIInputShaperTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 默认测试参数（与 MATLAB 参考代码一致）
        natural_frequency_hz_ = 5.0;
        damping_ratio_ = 0.05;
        sampling_time_sec_ = 0.01;
        vibration_tolerance_ = 0.05;
    }

    double natural_frequency_hz_;
    double damping_ratio_;
    double sampling_time_sec_;
    double vibration_tolerance_;
};

// 测试 1: 系数计算验证（与 MATLAB 参考值对比）
TEST_F(EIInputShaperTest, CoefficientCalculation) {
    EIInputShaper shaper(natural_frequency_hz_, damping_ratio_, sampling_time_sec_, vibration_tolerance_);

    double A1, A2, A3;
    int T2, T3;
    shaper.GetCoefficients(A1, A2, A3, T2, T3);

    // MATLAB 参考值（来自 EI.m 第 32-37 行）
    // 使用相同参数：freq=5Hz, damping=0.05, vibrTol=0.05, samplingTime=0.01
    double A1_expected = 0.24968 + 0.24961 * 0.05 + 0.80008 * 0.05 + 1.23328 * 0.05 * 0.05 + 0.49599 * 0.05 * 0.05 +
                         3.17316 * 0.05 * 0.05 * 0.05;
    double A3_expected = 0.25149 + 0.21474 * 0.05 - 0.83249 * 0.05 + 1.41498 * 0.05 * 0.05 + 0.85181 * 0.05 * 0.05 -
                         4.90094 * 0.05 * 0.05 * 0.05;
    double A2_expected = 1.0 - A1_expected - A3_expected;

    // 验证系数
    EXPECT_NEAR(A1, A1_expected, 1e-5);
    EXPECT_NEAR(A2, A2_expected, 1e-5);
    EXPECT_NEAR(A3, A3_expected, 1e-5);

    // 验证归一化
    EXPECT_NEAR(A1 + A2 + A3, 1.0, 1e-10);
}

// 测试 2: 延迟计算验证
TEST_F(EIInputShaperTest, DelayCalculation) {
    EIInputShaper shaper(natural_frequency_hz_, damping_ratio_, sampling_time_sec_, vibration_tolerance_);

    // 振动周期 = 1 / 5 = 0.2 秒
    // T_d = round(0.2 / 0.01) = 20 采样点
    EXPECT_EQ(shaper.GetDelaySamples(), 20);
    EXPECT_NEAR(shaper.GetDelaySeconds(), 0.2, 1e-10);
}

// 测试 3: T2 时间计算验证
TEST_F(EIInputShaperTest, T2Calculation) {
    EIInputShaper shaper(natural_frequency_hz_, damping_ratio_, sampling_time_sec_, vibration_tolerance_);

    double A1, A2, A3;
    int T2, T3;
    shaper.GetCoefficients(A1, A2, A3, T2, T3);

    // MATLAB 参考计算（EI.m 第 35-36 行）
    double vibration_period = 1.0 / natural_frequency_hz_;
    double zeta = damping_ratio_;
    double V_tol = vibration_tolerance_;
    double T2_time_expected =
        vibration_period * (0.4999 + 0.46159 * V_tol * zeta + 4.26169 * V_tol * zeta * zeta +
                            1.75601 * V_tol * zeta * zeta * zeta + 8.57843 * V_tol * V_tol * zeta -
                            108.644 * V_tol * V_tol * zeta * zeta + 336.989 * V_tol * V_tol * zeta * zeta * zeta);
    int T2_expected = static_cast<int>(std::round(T2_time_expected / sampling_time_sec_));

    EXPECT_EQ(T2, T2_expected);
    EXPECT_EQ(T3, 20);  // T3 = T_d
}

// 测试 4: 单位阶跃响应
TEST_F(EIInputShaperTest, UnitStepResponse) {
    EIInputShaper shaper(natural_frequency_hz_, damping_ratio_, sampling_time_sec_, vibration_tolerance_);

    // 输入足够多的样本以达到稳态
    double final_output = 0.0;
    for (int i = 0; i < 100; ++i) {
        final_output = shaper.Apply(1.0);
    }

    // 稳态输出应该等于输入（无稳态误差）
    EXPECT_NEAR(final_output, 1.0, 1e-10);
}

// 测试 5: 零输入响应
TEST_F(EIInputShaperTest, ZeroInputResponse) {
    EIInputShaper shaper(natural_frequency_hz_, damping_ratio_, sampling_time_sec_, vibration_tolerance_);

    // 所有输入为零，输出也应该为零
    for (int i = 0; i < 100; ++i) {
        double output = shaper.Apply(0.0);
        EXPECT_NEAR(output, 0.0, 1e-10);
    }
}

// 测试 6: 斜坡响应（恒定延迟）
TEST_F(EIInputShaperTest, RampResponse) {
    EIInputShaper shaper(natural_frequency_hz_, damping_ratio_, sampling_time_sec_, vibration_tolerance_);

    // 输入斜坡信号
    double slope = 0.01;
    double steady_state_error = 0.0;

    for (int i = 0; i < 100; ++i) {
        double input = i * slope;
        double output = shaper.Apply(input);

        // 在稳态后（i > 50），误差应该恒定
        if (i > 50) {
            double error = output - input;
            if (i == 51) {
                steady_state_error = error;
            } else {
                // 误差应该保持恒定（恒定延迟）
                EXPECT_NEAR(error, steady_state_error, 1e-8);
            }
        }
    }
}

// 测试 7: Reset 功能
TEST_F(EIInputShaperTest, ResetFunctionality) {
    EIInputShaper shaper(natural_frequency_hz_, damping_ratio_, sampling_time_sec_, vibration_tolerance_);

    // 应用一些输入
    for (int i = 0; i < 50; ++i) {
        shaper.Apply(1.0);
    }

    // 重置
    shaper.Reset();
    EXPECT_FALSE(shaper.IsInitialized());

    // 重置后第一个输出应该等于输入
    double first_output = shaper.Apply(2.0);
    EXPECT_NEAR(first_output, 2.0, 1e-10);
    EXPECT_TRUE(shaper.IsInitialized());
}

// 测试 8: 重新配置
TEST_F(EIInputShaperTest, Reconfiguration) {
    EIInputShaper shaper(natural_frequency_hz_, damping_ratio_, sampling_time_sec_, vibration_tolerance_);

    // 初始延迟
    int initial_delay = shaper.GetDelaySamples();
    EXPECT_EQ(initial_delay, 20);

    // 重新配置为 10 Hz
    shaper.Configure(10.0, 0.1);

    // 新延迟应该是 10 采样点（周期 0.1s）
    EXPECT_EQ(shaper.GetDelaySamples(), 10);
    EXPECT_NEAR(shaper.GetDelaySeconds(), 0.1, 1e-10);

    // 重新配置后应该未初始化
    EXPECT_FALSE(shaper.IsInitialized());
}

// 测试 9: 不同频率参数
TEST_F(EIInputShaperTest, DifferentFrequencies) {
    // 测试 10 Hz
    EIInputShaper shaper_10hz(10.0, 0.05, 0.01, 0.05);
    EXPECT_EQ(shaper_10hz.GetDelaySamples(), 10);

    // 测试 3 Hz
    EIInputShaper shaper_3hz(3.0, 0.05, 0.01, 0.05);
    EXPECT_EQ(shaper_3hz.GetDelaySamples(), 33);  // round(1/3 / 0.01)

    // 验证稳态响应
    double output_10hz = 0.0;
    double output_3hz = 0.0;
    for (int i = 0; i < 100; ++i) {
        output_10hz = shaper_10hz.Apply(1.0);
        output_3hz = shaper_3hz.Apply(1.0);
    }
    EXPECT_NEAR(output_10hz, 1.0, 1e-10);
    EXPECT_NEAR(output_3hz, 1.0, 1e-10);
}

// 测试 10: 初始化状态
TEST_F(EIInputShaperTest, InitializationState) {
    EIInputShaper shaper(natural_frequency_hz_, damping_ratio_, sampling_time_sec_, vibration_tolerance_);

    // 初始状态应该未初始化
    EXPECT_FALSE(shaper.IsInitialized());

    // 第一次 Apply 后应该初始化
    shaper.Apply(1.0);
    EXPECT_TRUE(shaper.IsInitialized());
}

// 测试 11: 振动容忍度影响
TEST_F(EIInputShaperTest, VibrationToleranceEffect) {
    // 创建不同振动容忍度的 shaper
    EIInputShaper shaper_low(5.0, 0.05, 0.01, 0.01);   // 1% 容忍度
    EIInputShaper shaper_high(5.0, 0.05, 0.01, 0.10);  // 10% 容忍度

    double A1_low, A2_low, A3_low;
    double A1_high, A2_high, A3_high;
    int T2_low, T3_low, T2_high, T3_high;

    shaper_low.GetCoefficients(A1_low, A2_low, A3_low, T2_low, T3_low);
    shaper_high.GetCoefficients(A1_high, A2_high, A3_high, T2_high, T3_high);

    // 系数应该不同
    EXPECT_NE(A1_low, A1_high);
    EXPECT_NE(A2_low, A2_high);
    EXPECT_NE(A3_low, A3_high);

    // 但都应该归一化
    EXPECT_NEAR(A1_low + A2_low + A3_low, 1.0, 1e-10);
    EXPECT_NEAR(A1_high + A2_high + A3_high, 1.0, 1e-10);
}

// 测试 12: Configure 保持振动容忍度
TEST_F(EIInputShaperTest, ConfigureKeepsVibrationTolerance) {
    EIInputShaper shaper(5.0, 0.05, 0.01, 0.08);

    double A1_before, A2_before, A3_before;
    int T2_before, T3_before;
    shaper.GetCoefficients(A1_before, A2_before, A3_before, T2_before, T3_before);

    // 重新配置但不改变振动容忍度
    shaper.Configure(5.0, 0.05, -1.0);

    double A1_after, A2_after, A3_after;
    int T2_after, T3_after;
    shaper.GetCoefficients(A1_after, A2_after, A3_after, T2_after, T3_after);

    // 系数应该相同（因为参数没变）
    EXPECT_NEAR(A1_before, A1_after, 1e-10);
    EXPECT_NEAR(A2_before, A2_after, 1e-10);
    EXPECT_NEAR(A3_before, A3_after, 1e-10);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
