/**
 * @file test_butterworth_filter.cpp
 * @brief Butterworth Filter 单元测试
 *
 * 测试内容：
 * 1. 初始化测试
 * 2. 直流信号响应（稳态增益）
 * 3. 零输入响应
 * 4. 单位阶跃响应
 * 5. 阶跃响应特性
 * 6. 重置功能
 * 7. 高频噪声抑制
 * 8. 低频信号通过
 * 9. 输出连续性
 */

#include "../utils/butterworth_filter.h"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "../utils/butterworth_filter.h"

// 测试夹具
class ButterworthFilterTest : public ::testing::Test {
protected:
    void SetUp() override { filter_ = std::make_unique<ButterworthFilter>(); }

    std::unique_ptr<ButterworthFilter> filter_;
};

// 测试 1: 初始化测试
TEST_F(ButterworthFilterTest, Initialization) {
    // 第一次调用 Apply，输出应该等于输入（初始化行为）
    double input = 5.0;
    double output = filter.Apply(input);
    EXPECT_NEAR(output, input, 1e-10);
}

// 测试 2: 直流信号响应 - 验证稳态增益为 1
TEST(ButterworthFilterTest, DCSignalResponse) {
    ButterworthFilter filter;
    double dc_value = 10.0;
    double output = 0.0;

    for (int i = 0; i < 1000; ++i) {
        output = filter.Apply(dc_value);
    }
    EXPECT_NEAR(output, dc_value, 1e-6);
}

// 测试 3: 零输入响应 - 验证零输入时输出为零
TEST(ButterworthFilterTest, ZeroInputResponse) {
    ButterworthFilter filter;
    for (int i = 0; i < 100; ++i) {
        double output = filter.Apply(0.0);
        EXPECT_NEAR(output, 0.0, 1e-10);
    }
}

// 测试 4: 单位阶跃响应 - 验证稳态输出接近 1.0
TEST(ButterworthFilterTest, UnitStepResponse) {
    ButterworthFilter filter;
    double output = 0.0;

    for (int i = 0; i < 1000; ++i) {
        output = filter.Apply(1.0);
    }
    EXPECT_NEAR(output, 1.0, 1e-6);
}

// 测试 5: 阶跃响应特性 - 验证低通滤波器单调上升特性
TEST(ButterworthFilterTest, StepResponseCharacteristics) {
    ButterworthFilter filter;
    std::vector<double> outputs;

    for (int i = 0; i < 500; ++i) {
        double input = (i < 100) ? 0.0 : 1.0;
        double output = filter.Apply(input);
        outputs.push_back(output);
    }

    EXPECT_NEAR(outputs[50], 0.0, 1e-10);

    for (size_t i = 101; i < 200; ++i) {
        EXPECT_GE(outputs[i], outputs[i - 1] - 1e-10);
    }

    EXPECT_NEAR(outputs[499], 1.0, 5e-3);
}

// 测试 6: 重置功能 - 验证 Reset 后状态正确恢复
TEST(ButterworthFilterTest, ResetFunctionality) {
    ButterworthFilter filter;

    for (int i = 0; i < 100; ++i) {
        filter.Apply(5.0);
    }

    filter.Reset();

    double first_output = filter.Apply(10.0);
    EXPECT_NEAR(first_output, 10.0, 1e-10);
}

// 测试 7: 高频噪声抑制 - 验证 50Hz 信号被显著衰减
TEST(ButterworthFilterTest, HighFrequencyAttenuation) {
    ButterworthFilter filter;
    double sampling_freq = 1000.0;
    double signal_freq = 50.0;
    double dt = 1.0 / sampling_freq;

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

    EXPECT_LT(output_amplitude, 0.1 * input_amplitude);
}

// 测试 8: 低频信号通过 - 验证 1Hz 信号幅值接近 1.0
TEST(ButterworthFilterTest, LowFrequencyPassthrough) {
    ButterworthFilter filter;
    double sampling_freq = 1000.0;
    double signal_freq = 1.0;
    double dt = 1.0 / sampling_freq;

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

    EXPECT_NEAR(output_amplitude, 1.0, 0.2);
}

// 测试 9: 输出连续性 - 验证相邻样本差值小于阈值
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
