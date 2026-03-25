#include "butterworth_filter.h"
#include <cmath>

ButterworthFilter::ButterworthFilter() : ButterworthFilter(3.0) {}

ButterworthFilter::ButterworthFilter(double cutoff_freq) {
    // 临界阻尼二阶巴特沃斯滤波器 (ζ=1.0，无超调)
    // 采样率固定为1kHz（力控线程频率）
    constexpr double kSampleRate = 1000.0;
    constexpr double kDampingRatio = 1.0;  // 临界阻尼，无超调
    constexpr double kSqrt2 = std::sqrt(2.0);

    double T = 1.0 / kSampleRate;
    double wc = 2.0 * M_PI * cutoff_freq;

    // 预畸变校正
    double wc_prewarped = (2.0 / T) * std::tan(wc * T / 2.0);

    // 临界阻尼二阶系统: H(s) = wc^2 / (s^2 + 2*wc*s + wc^2)
    double k = wc_prewarped;
    double k2 = k * k;
    double T2 = T * T;

    // 分母: 1 + 2*k*T + k2*T2
    double denom = 1.0 + 2.0 * k * kDampingRatio * T + k2 * T2;

    // 分子系数
    filter_.b[0] = k2 * T2 / denom;
    filter_.b[1] = 2.0 * filter_.b[0];
    filter_.b[2] = filter_.b[0];

    // 分母系数
    filter_.a[0] = 1.0;
    filter_.a[1] = 2.0 * (k2 * T2 - 1.0) / denom;
    filter_.a[2] = (1.0 - 2.0 * k * kDampingRatio * T + k2 * T2) / denom;
}

double ButterworthFilter::Apply(double input) {
    if (!initialized_) {
        for (int i = 0; i <= FILTER_ORDER; ++i) {
            inputHistory_[i] = input;
            outputHistory_[i] = input;
        }
        initialized_ = true;
        return input;
    }

    for (int i = FILTER_ORDER; i > 0; --i) {
        inputHistory_[i] = inputHistory_[i - 1];
        outputHistory_[i] = outputHistory_[i - 1];
    }
    inputHistory_[0] = input;

    double y = 0.0;
    for (int i = 0; i <= FILTER_ORDER; ++i) {
        y += filter_.b[i] * inputHistory_[i];
    }
    for (int i = 1; i <= FILTER_ORDER; ++i) {
        y -= filter_.a[i] * outputHistory_[i];
    }
    outputHistory_[0] = y;
    return y;
}

void ButterworthFilter::Reset() {
    if (!initialized_) return;
    initialized_ = false;
    inputHistory_.fill(0.0);
    outputHistory_.fill(0.0);
}