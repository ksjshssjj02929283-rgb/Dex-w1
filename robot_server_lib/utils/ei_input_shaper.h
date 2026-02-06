#pragma once

#include <cmath>
#include <deque>
#include <utility>
#include <vector>

class EIInputShaper {
public:
    enum Type { NONE, EI, TWO_HUMP_EI };  // EI整形器类型枚举，NONE只进行ramp不滤波

    EIInputShaper(Type type,
                  double natural_freq,
                  double sampling_time,
                  double max_ramp_rate = 0.0,
                  double vibration_tol = 0.05);  // 构造函数，初始化EI整形器参数

    double Apply(double input);                                           // 应用EI输入整形
    std::pair<double, double> Apply(double input_ch1, double input_ch2);  // 双通道应用EI输入整形

    void Reset();  // 重置状态

private:
    void CalculateCoefficients();         // 计算EI系数
    void CalculateTwoHumpCoefficients();  // 计算双峰EI系数

    double ApplyImpl(std::deque<double>& buffer, double input);       // 应用整形器实现
    double RampImpl(double target, double current, double max_rate);  // 斜坡限制实现

    Type type_ = Type::TWO_HUMP_EI;
    double natural_freq_ = 2.0;
    double damping_ratio_ = 0.05;
    double sampling_time_ = 0.001;
    double vibration_tol_ = 0.05;

    double A1_ = 0.0;
    double A2_ = 0.0;
    double A3_ = 0.0;
    double A4_ = 0.0;
    int T2_ = 0;
    int T3_ = 0;
    int T4_ = 0;

    std::deque<double> buffer_;
    std::deque<double> buffer_ch1_;
    std::deque<double> buffer_ch2_;

    bool initialized_ = false;
    double max_ramp_rate_per_sample_ = 0.0;  // 最大斜坡变化率，单位：输入单位/周期
    double ramp_output_ = 0.0;
    double ramp_output_ch1_ = 0.0;
    double ramp_output_ch2_ = 0.0;
    bool ramp_initialized_ = false;
};
