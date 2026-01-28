/**
 * @file test_butterworth_filter.cpp
 * @brief Butterworth Filter 单元测试
 *
 * 测试内容：
 * 1. 滤波器初始化测试
 * 2. 直流信号测试（稳态增益）
 * 3. 阶跃响应测试
 * 4. 重置功能测试
 * 5. 高频噪声抑制测试
 * 6. 相位延迟测试
 */

#include "../utils/butterworth_filter.h"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

// 测试夹具
class ButterworthFilterTest : public ::testing::Test {
protected:
    void SetUp() override {
        filter_ = std::make_unique<ButterworthFilter>();
    }

    std::unique_ptr<ButterworthFilter> filter_;
};

// 测试 1: 初始化测试
TEST_F(ButterworthFilterTest, Initialization) {
    // 第一次调用 Apply，输出应该等于输入（初始化行为）
    double input = 5.0;
    double output = filter_->Apply(input);
    EXPECT_NEAR(output, input, 1e-10);
}

// 测试 2: 直流信号测试（稳态增益应为 1）
TEST_F(ButterworthFilterTest, DCSignalResponse) {
    // 输入恒定的直流信号
    double dc_value = 10.0;
    double output = 0.0;

    // 应用足够多的样本以达到稳态
    for (int i = 0; i < 1000; ++i) {
        output = filter_->Apply(dc_value);
    }

    // 稳态输出应该等于输入（直流增益 = 1）
    EXPECT_NEAR(output, dc_value, 1e-6);
}

// 测试 3: 零输入响应
TEST_F(ButterworthFilterTest, ZeroInputResponse) {
    // 所有输入为零，输出也应该为零
    for (int i = 0; i < 100; ++i) {
        double output = filter_->Apply(0.0);
        EXPECT_NEAR(output, 0.0, 1e-10);
    }
}

// 测试 4: 单位阶跃响应
TEST_F(ButterworthFilterTest, UnitStepResponse) {
    double output = 0.0;

    // 输入单位阶跃信号
    for (int i = 0; i < 1000; ++i) {
        output = filter_->Apply(1.0);
    }

    // 稳态输出应该接近 1.0
    EXPECT_NEAR(output, 1.0, 1e-6);
}

// 测试 5: 阶跃响应特性
TEST_F(ButterworthFilterTest, StepResponseCharacteristics) {
    std::vector<double> outputs;

    // 输入阶跃信号并记录输出
    for (int i = 0; i < 500; ++i) {
        double input = (i < 100) ? 0.0 : 1.0;
        double output = filter_->Apply(input);
        outputs.push_back(output);
    }

    // 验证阶跃前输出为 0
    EXPECT_NEAR(outputs[50], 0.0, 1e-10);

    // 验证阶跃后输出单调上升（低通滤波器特性）
    for (size_t i = 101; i < 200; ++i) {
        EXPECT_GE(outputs[i], outputs[i - 1] - 1e-10);  // 允许小的数值误差
    }

    // 验证最终稳态值接近 1.0（允许轻微超调）
    EXPECT_NEAR(outputs[499], 1.0, 5e-3);  // 放宽到 0.005 容限
}

// 测试 6: Reset 功能
TEST_F(ButterworthFilterTest, ResetFunctionality) {
    // 应用一些输入
    for (int i = 0; i < 100; ++i) {
        filter_->Apply(5.0);
    }

    // 重置滤波器
    filter_->Reset();

    // 重置后第一个输出应该等于输入（重新初始化）
    double first_output = filter_->Apply(10.0);
    EXPECT_NEAR(first_output, 10.0, 1e-10);
}

// 测试 7: 高频噪声抑制
TEST_F(ButterworthFilterTest, HighFrequencyAttenuation) {
    // 3Hz Butterworth 滤波器应该显著衰减高频信号
    // 测试 50Hz 正弦波（远高于截止频率）

    double sampling_freq = 1000.0;  // 1kHz 采样率
    double signal_freq = 50.0;      // 50Hz 信号频率
    double dt = 1.0 / sampling_freq;

    std::vector<double> inputs;
    std::vector<double> outputs;

    // 生成 50Hz 正弦波并通过滤波器
    for (int i = 0; i < 1000; ++i) {
        double t = i * dt;
        double input = std::sin(2.0 * M_PI * signal_freq * t);
        double output = filter_->Apply(input);
        inputs.push_back(input);
        outputs.push_back(output);
    }

    // 计算输入和输出的幅值（使用后半部分数据，避免瞬态）
    double input_amplitude = 0.0;
    double output_amplitude = 0.0;
    for (size_t i = 500; i < 1000; ++i) {
        input_amplitude = std::max(input_amplitude, std::abs(inputs[i]));
        output_amplitude = std::max(output_amplitude, std::abs(outputs[i]));
    }

    // 输出幅值应该显著小于输入幅值（高频衰减）
    EXPECT_LT(output_amplitude, 0.1 * input_amplitude);
}

// 测试 8: 低频信号通过
TEST_F(ButterworthFilterTest, LowFrequencyPassthrough) {
    // 3Hz Butterworth 滤波器应该让低频信号通过
    // 测试 1Hz 正弦波（低于截止频率）

    double sampling_freq = 1000.0;  // 1kHz 采样率
    double signal_freq = 1.0;       // 1Hz 信号频率
    double dt = 1.0 / sampling_freq;

    std::vector<double> outputs;

    // 生成 1Hz 正弦波并通过滤波器
    for (int i = 0; i < 3000; ++i) {  // 3 秒数据
        double t = i * dt;
        double input = std::sin(2.0 * M_PI * signal_freq * t);
        double output = filter_->Apply(input);
        outputs.push_back(output);
    }

    // 计算输出幅值（使用后半部分数据，避免瞬态）
    double output_amplitude = 0.0;
    for (size_t i = 2000; i < 3000; ++i) {
        output_amplitude = std::max(output_amplitude, std::abs(outputs[i]));
    }

    // 输出幅值应该接近输入幅值 1.0（低频通过）
    EXPECT_NEAR(output_amplitude, 1.0, 0.2);  // 允许 20% 误差
}

// 测试 9: 连续性测试
TEST_F(ButterworthFilterTest, OutputContinuity) {
    std::vector<double> outputs;

    // 输入斜坡信号
    for (int i = 0; i < 500; ++i) {
        double input = i * 0.01;
        double output = filter_->Apply(input);
        outputs.push_back(output);
    }

    // 验证输出连续（相邻样本差值不应过大）
    for (size_t i = 1; i < outputs.size(); ++i) {
        double diff = std::abs(outputs[i] - outputs[i - 1]);
        EXPECT_LT(diff, 0.1);  // 相邻样本差值应该小于 0.1
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}