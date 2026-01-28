/**
 * @file test_zvd_input_shaper.cpp
 * @brief ZVD Input Shaper 单元测试
 *
 * 测试内容：
 * 1. 系数计算验证
 * 2. 延迟计算验证
 * 3. 单位阶跃响应测试
 * 4. 斜坡响应测试
 * 5. 重置功能测试
 * 6. 重新配置测试
 */

#include "../utils/zvd_input_shaper.h"

#include <gtest/gtest.h>

#include <cmath>

// 测试夹具
class ZVDInputShaperTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 默认测试参数
        natural_frequency_hz_ = 5.0;
        damping_ratio_ = 0.05;
        sampling_time_sec_ = 0.001;
    }

    double natural_frequency_hz_;
    double damping_ratio_;
    double sampling_time_sec_;
};

// 测试 1: 系数计算验证
TEST_F(ZVDInputShaperTest, CoefficientCalculation) {
    ZVDInputShaper shaper(natural_frequency_hz_, damping_ratio_, sampling_time_sec_);

    // 手动计算预期系数
    double sqrt_term = std::sqrt(1.0 - damping_ratio_ * damping_ratio_);
    double K = std::exp(-damping_ratio_ * M_PI / sqrt_term);
    double J_ZVD = 1.0 + 2.0 * K + K * K;
    double A1_expected = 1.0 / J_ZVD;
    double A2_expected = 2.0 * K / J_ZVD;
    double A3_expected = K * K / J_ZVD;

    // 验证系数归一化
    EXPECT_NEAR(A1_expected + A2_expected + A3_expected, 1.0, 1e-10);

    // 预期值（根据 MATLAB 参考代码）
    EXPECT_NEAR(K, 0.854468, 1e-5);
    EXPECT_NEAR(A1_expected, 0.290778, 1e-5);
    EXPECT_NEAR(A2_expected, 0.496921, 1e-5);
    EXPECT_NEAR(A3_expected, 0.212301, 1e-5);
}

// 测试 2: 延迟计算验证
TEST_F(ZVDInputShaperTest, DelayCalculation) {
    ZVDInputShaper shaper(natural_frequency_hz_, damping_ratio_, sampling_time_sec_);

    // 振动周期 = 1 / 自然频率 = 1 / 5 = 0.2 秒
    // T_d = round(0.2 / 0.001) = 200 采样点
    EXPECT_EQ(shaper.GetDelaySamples(), 200);
    EXPECT_NEAR(shaper.GetDelaySeconds(), 0.2, 1e-10);
}

// 测试 3: 单位阶跃响应
TEST_F(ZVDInputShaperTest, UnitStepResponse) {
    ZVDInputShaper shaper(natural_frequency_hz_, damping_ratio_, sampling_time_sec_);

    // 输入足够多的样本以达到稳态
    double final_output = 0.0;
    for (int i = 0; i < 500; ++i) {
        final_output = shaper.Apply(1.0);
    }

    // 稳态输出应该等于输入（无稳态误差）
    EXPECT_NEAR(final_output, 1.0, 1e-10);
}

// 测试 4: 零输入响应
TEST_F(ZVDInputShaperTest, ZeroInputResponse) {
    ZVDInputShaper shaper(natural_frequency_hz_, damping_ratio_, sampling_time_sec_);

    // 所有输入为零，输出也应该为零
    for (int i = 0; i < 500; ++i) {
        double output = shaper.Apply(0.0);
        EXPECT_NEAR(output, 0.0, 1e-10);
    }
}

// 测试 5: 斜坡响应（恒定延迟）
TEST_F(ZVDInputShaperTest, RampResponse) {
    ZVDInputShaper shaper(natural_frequency_hz_, damping_ratio_, sampling_time_sec_);

    // 输入斜坡信号
    double slope = 0.01;
    double steady_state_error = 0.0;

    for (int i = 0; i < 500; ++i) {
        double input = i * slope;
        double output = shaper.Apply(input);

        // 在稳态后（i > 250），误差应该恒定
        if (i > 250) {
            double error = output - input;
            if (i == 251) {
                steady_state_error = error;
            } else {
                // 误差应该保持恒定（恒定延迟）
                EXPECT_NEAR(error, steady_state_error, 1e-8);
            }
        }
    }
}

// 测试 6: Reset 功能
TEST_F(ZVDInputShaperTest, ResetFunctionality) {
    ZVDInputShaper shaper(natural_frequency_hz_, damping_ratio_, sampling_time_sec_);

    // 应用一些输入
    for (int i = 0; i < 100; ++i) {
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

// 测试 7: 重新配置
TEST_F(ZVDInputShaperTest, Reconfiguration) {
    ZVDInputShaper shaper(natural_frequency_hz_, damping_ratio_, sampling_time_sec_);

    // 初始延迟
    int initial_delay = shaper.GetDelaySamples();
    EXPECT_EQ(initial_delay, 200);

    // 重新配置为 10 Hz
    shaper.Configure(10.0, 0.1);

    // 新延迟应该是 100 采样点（周期 0.1s）
    EXPECT_EQ(shaper.GetDelaySamples(), 100);
    EXPECT_NEAR(shaper.GetDelaySeconds(), 0.1, 1e-10);

    // 重新配置后应该未初始化
    EXPECT_FALSE(shaper.IsInitialized());
}

// 测试 8: 不同频率参数
TEST_F(ZVDInputShaperTest, DifferentFrequencies) {
    // 测试 10 Hz
    ZVDInputShaper shaper_10hz(10.0, 0.05, 0.001);
    EXPECT_EQ(shaper_10hz.GetDelaySamples(), 100);

    // 测试 3 Hz
    ZVDInputShaper shaper_3hz(3.0, 0.05, 0.001);
    EXPECT_EQ(shaper_3hz.GetDelaySamples(), 333);  // round(1/3 / 0.001)

    // 验证稳态响应
    double output_10hz = 0.0;
    double output_3hz = 0.0;
    for (int i = 0; i < 500; ++i) {
        output_10hz = shaper_10hz.Apply(1.0);
        output_3hz = shaper_3hz.Apply(1.0);
    }
    EXPECT_NEAR(output_10hz, 1.0, 1e-10);
    EXPECT_NEAR(output_3hz, 1.0, 1e-10);
}

// 测试 9: 初始化状态
TEST_F(ZVDInputShaperTest, InitializationState) {
    ZVDInputShaper shaper(natural_frequency_hz_, damping_ratio_, sampling_time_sec_);

    // 初始状态应该未初始化
    EXPECT_FALSE(shaper.IsInitialized());

    // 第一次 Apply 后应该初始化
    shaper.Apply(1.0);
    EXPECT_TRUE(shaper.IsInitialized());
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
