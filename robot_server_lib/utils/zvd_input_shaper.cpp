#include "zvd_input_shaper.h"

#include <algorithm>
#include <cmath>

ZVDInputShaper::ZVDInputShaper(double natural_frequency_hz,
                               double damping_ratio,
                               double sampling_time_sec)
    : natural_frequency_hz_(natural_frequency_hz),
      damping_ratio_(damping_ratio),
      sampling_time_(sampling_time_sec),
      A1_(0.0),
      A2_(0.0),
      A3_(0.0),
      T2_(0),
      T3_(0),
      T_d_(0),
      K_(0.0),
      initialized_(false) {
    CalculateCoefficients();
}

void ZVDInputShaper::CalculateCoefficients() {
    // 计算 K 因子：K = exp(-ζπ/√(1-ζ²))
    double sqrt_term = std::sqrt(1.0 - damping_ratio_ * damping_ratio_);
    K_ = std::exp(-damping_ratio_ * M_PI / sqrt_term);

    // 计算归一化因子：J_ZVD = 1 + 2K + K²
    double J_ZVD = 1.0 + 2.0 * K_ + K_ * K_;

    // 计算脉冲幅值
    A1_ = 1.0 / J_ZVD;
    A2_ = 2.0 * K_ / J_ZVD;
    A3_ = K_ * K_ / J_ZVD;

    // 计算时间延迟
    // 振动周期 = 1 / 自然频率
    double vibration_period = 1.0 / natural_frequency_hz_;
    // T_d = round(振动周期 / 采样周期)
    T_d_ = static_cast<int>(std::round(vibration_period / sampling_time_));
    T2_ = T_d_ / 2;
    T3_ = T_d_;

    // 调整缓冲区大小（需要存储 T3+1 个样本）
    input_history_.resize(T3_ + 1, 0.0);
}

double ZVDInputShaper::Apply(double input) {
    // 首次调用：用输入值填充缓冲区
    if (!initialized_) {
        std::fill(input_history_.begin(), input_history_.end(), input);
        initialized_ = true;
        return input;  // 第一个样本不整形
    }

    // 更新 FIFO 缓冲区
    input_history_.pop_front();
    input_history_.push_back(input);

    // 应用 ZVD 整形公式：
    // output = A1·input(t) + A2·input(t-T2) + A3·input(t-T3)
    // 注意：input_history_[T3_] 是最新的输入（当前时刻）
    //       input_history_[T3_ - T2_] 是延迟 T2 的输入
    //       input_history_[0] 是延迟 T3 的输入
    double output = A1_ * input_history_[T3_] +           // 当前输入
                    A2_ * input_history_[T3_ - T2_] +     // 延迟 T2
                    A3_ * input_history_[0];              // 延迟 T3

    return output;
}

void ZVDInputShaper::Reset() {
    if (!initialized_) return;
    initialized_ = false;
    std::fill(input_history_.begin(), input_history_.end(), 0.0);
}

void ZVDInputShaper::Configure(double natural_frequency_hz, double damping_ratio) {
    natural_frequency_hz_ = natural_frequency_hz;
    damping_ratio_ = damping_ratio;
    CalculateCoefficients();
    // 重置状态，下次 Apply 时重新初始化
    initialized_ = false;
}

int ZVDInputShaper::GetDelaySamples() const {
    return T_d_;
}

double ZVDInputShaper::GetDelaySeconds() const {
    return T_d_ * sampling_time_;
}

bool ZVDInputShaper::IsInitialized() const {
    return initialized_;
}
