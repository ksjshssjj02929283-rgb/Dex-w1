/**
 * @file test_butterworth_filter.cpp
 * @brief Butterworth Filter 单元测试
 *
 * 测试内容：
 * 1. 默认构造函数测试
 * 2. 带参构造函数测试
 * 3. 直流信号响应（稳态增益）
 * 4. 零输入响应
 * 5. 单位阶跃响应
 * 6. 阶跃响应特性（临界阻尼无超调）
 * 7. 重置功能
 * 8. 高频噪声抑制
 * 9. 低频信号通过
 * 10. 输出连续性
 * 11. 不同截止频率对比
 */

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "../utils/butterworth_filter.h"

// 测试 1: 默认构造函数测试 - 验证默认截止频率3Hz
TEST(ButterworthFilterTest, DefaultConstructor) {
    ButterworthFilter filter;
    double input = 5.0;
    double output = filter.Apply(input);
    EXPECT_NEAR(output, input, 1e-10);
}

// 测试 2: 带参构造函数测试 - 验证自定义截止频率
TEST(ButterworthFilterTest, ParameterizedConstructor) {
    ButterworthFilter filter(10.0);  // 10Hz截止频率
    double input = 5.0;
    double output = filter.Apply(input);
    EXPECT_NEAR(output, input, 1e-10);
}

// 测试 3: 直流信号响应 - 验证稳态增益为 1
TEST(ButterworthFilterTest, DCSignalResponse) {
    ButterworthFilter filter;
    double dc_value = 10.0;
    double output = 0.0;

    for (int i = 0; i < 1000; ++i) {
        output = filter.Apply(dc_value);
    }
    EXPECT_NEAR(output, dc_value, 1e-6);
}

// 测试 4: 零输入响应 - 验证零输入时输出为零
TEST(ButterworthFilterTest, ZeroInputResponse) {
    ButterworthFilter filter;
    for (int i = 0; i < 100; ++i) {
        double output = filter.Apply(0.0);
        EXPECT_NEAR(output, 0.0, 1e-10);
    }
}

// 测试 5: 单位阶跃响应 - 验证稳态输出接近 1.0
TEST(ButterworthFilterTest, UnitStepResponse) {
    ButterworthFilter filter;
    double output = 0.0;

    for (int i = 0; i < 1000; ++i) {
        output = filter.Apply(1.0);
    }
    EXPECT_NEAR(output, 1.0, 1e-6);
}

// 测试 6: 阶跃响应特性 - 验证临界阻尼无超调（单调上升）
TEST(ButterworthFilterTest, StepResponseNoOvershoot) {
    ButterworthFilter filter(5.0);  // 5Hz截止频率
    std::vector<double> outputs;

    // 从0阶跃到1
    for (int i = 0; i < 1000; ++i) {
        double input = (i < 100) ? 0.0 : 1.0;
        double output = filter.Apply(input);
        outputs.push_back(output);
    }

    // 验证初始状态
    EXPECT_NEAR(outputs[50], 0.0, 1e-10);

    // 验证临界阻尼无超调：输出始终不超过最终值
    double final_value = outputs[999];
    for (size_t i = 100; i < outputs.size(); ++i) {
        EXPECT_LE(outputs[i], final_value + 1e-10);  // 无超调
    }

    // 验证最终收敛
    EXPECT_NEAR(outputs[999], 1.0, 1e-3);
}

// 测试 7: 重置功能 - 验证 Reset 后状态正确恢复
TEST(ButterworthFilterTest, ResetFunctionality) {
    ButterworthFilter filter;

    for (int i = 0; i < 100; ++i) {
        filter.Apply(5.0);
    }

    filter.Reset();

    double first_output = filter.Apply(10.0);
    EXPECT_NEAR(first_output, 10.0, 1e-10);
}

// 测试 8: 高频噪声抑制 - 验证 50Hz 信号被显著衰减（采样率1kHz）
TEST(ButterworthFilterTest, HighFrequencyAttenuation) {
    ButterworthFilter filter;  // 默认3Hz截止频率
    constexpr double kSamplingFreq = 1000.0;
    double signal_freq = 50.0;
    double dt = 1.0 / kSamplingFreq;

    std::vector<double> inputs;
    std::vector<double> outputs;

    for (int i = 0; i < 1000; ++i) {
        double t = i * dt;
        double input = std::sin(2.0 * M_PI * signal_freq * t);
        double output = filter.Apply(input);
        inputs.push_back(input);
        outputs.push_back(output);
    }

    double input_amplitude = 0.0;
    double output_amplitude = 0.0;
    for (size_t i = 500; i < 1000; ++i) {
        input_amplitude = std::max(input_amplitude, std::abs(inputs[i]));
        output_amplitude = std::max(output_amplitude, std::abs(outputs[i]));
    }

    // 50Hz远高于3Hz截止频率，应被显著衰减
    EXPECT_LT(output_amplitude, 0.1 * input_amplitude);
}

// 测试 9: 低频信号通过 - 验证 1Hz 信号幅值接近 1.0（采样率1kHz）
TEST(ButterworthFilterTest, LowFrequencyPassthrough) {
    ButterworthFilter filter;
    constexpr double kSamplingFreq = 1000.0;
    double signal_freq = 1.0;
    double dt = 1.0 / kSamplingFreq;

    std::vector<double> outputs;

    for (int i = 0; i < 3000; ++i) {
        double t = i * dt;
        double input = std::sin(2.0 * M_PI * signal_freq * t);
        double output = filter.Apply(input);
        outputs.push_back(output);
    }

    double output_amplitude = 0.0;
    for (size_t i = 2000; i < 3000; ++i) {
        output_amplitude = std::max(output_amplitude, std::abs(outputs[i]));
    }

    // 1Hz低于3Hz截止频率，应能通过
    EXPECT_NEAR(output_amplitude, 1.0, 0.2);
}

// 测试 10: 输出连续性 - 验证相邻样本差值小于阈值
TEST(ButterworthFilterTest, OutputContinuity) {
    ButterworthFilter filter;
    std::vector<double> outputs;

    for (int i = 0; i < 500; ++i) {
        double input = i * 0.01;
        double output = filter.Apply(input);
        outputs.push_back(output);
    }

    for (size_t i = 1; i < outputs.size(); ++i) {
        double diff = std::abs(outputs[i] - outputs[i - 1]);
        EXPECT_LT(diff, 0.1);
    }
}

// 测试 11: 不同截止频率对比 - 验证不同截止频率产生不同滤波器系数
TEST(ButterworthFilterTest, DifferentCutoffFrequencies) {
    // 这个测试验证不同截止频率产生不同的响应特性
    ButterworthFilter filter_low(2.0);   // 2Hz截止频率
    ButterworthFilter filter_high(20.0);  // 20Hz截止频率

    // 记录前几次迭代的输出
    double output_low_prev = 0.0;
    double output_high_prev = 0.0;

    for (int i = 0; i < 10; ++i) {
        output_low_prev = filter_low.Apply(1.0);
        output_high_prev = filter_high.Apply(1.0);
    }

    // 验证两者都处于上升过程中（未完全收敛）
    // 高截止频率应该更快接近稳态值1.0
    // 由于临界阻尼的特性，两者都应该在0到1之间
    EXPECT_GE(output_low_prev, 0.0);
    EXPECT_LE(output_low_prev, 1.0);
    EXPECT_GE(output_high_prev, 0.0);
    EXPECT_LE(output_high_prev, 1.0);

    // 高截止频率在相同迭代次数下应该更接近1.0（响应更快）
    EXPECT_GE(output_high_prev, output_low_prev);
}
