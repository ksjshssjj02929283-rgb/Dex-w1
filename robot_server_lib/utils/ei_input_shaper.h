#pragma once

#include <cmath>
#include <deque>
#include <utility>
#include <vector>

class EIInputShaper {
public:
    enum Type { EI, TWO_HUMP_EI };  // EI整形器类型枚举

    EIInputShaper(Type type, double natural_freq, double vibration_tol = 0.05);  // 构造函数，初始化EI整形器参数

    double Apply(double input);                                       // 应用EI输入整形
    std::pair<double, double> Apply(double input_r, double input_l);  // 双通道应用EI输入整形

    void DecelStop(double current_velocity, int interpolation_length);               // 减速停止，单通道
    void DecelStop(double velocity_r, double velocity_l, int interpolation_length);  // 减速停止，双通道

    void Reset();                         // 重置状态
    void Configure(double natural_freq);  // 配置频率
    void SetType(Type type);              // 设置整形器类型

    void EnableRamp(bool enable);                  // 使能斜坡限制
    void SetRampRate(double max_rate_per_sample);  // 设置斜坡最大变化率

    bool DetectDecelStart(bool current_decel_flag);  // 检测减速开始时刻

private:
    void CalculateCoefficients();         // 计算EI系数
    void CalculateTwoHumpCoefficients();  // 计算双峰EI系数

    double ApplyImpl(std::deque<double>& buffer, double input);  // 应用整形器实现
    void DecelStopImpl(std::deque<double>& buffer, double current_velocity, int interpolation_length);  // 减速停止实现
    double RampImpl(double target, double current, double max_rate);  // 斜坡限制实现

    Type type_ = Type::EI;
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
    std::deque<double> buffer_r_;
    std::deque<double> buffer_l_;

    std::deque<double> decel_buffer_;
    std::deque<double> decel_buffer_r_;
    std::deque<double> decel_buffer_l_;

    bool decel_mode_ = false;
    bool initialized_ = false;
    int decel_counter_ = 0;
    int decel_total_steps_ = 0;
    bool previous_decel_flag_ = false;

    bool ramp_enabled_ = false;
    double max_ramp_rate_ = 0.0;
    double ramp_output_ = 0.0;
    double ramp_output_r_ = 0.0;
    double ramp_output_l_ = 0.0;
    bool ramp_initialized_ = false;
};
