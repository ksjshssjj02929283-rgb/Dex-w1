#pragma once
#include <array>
#include <cstddef>

#define FILTER_ORDER 2

struct BiquadFilter {
    std::array<double, FILTER_ORDER + 1> b;
    std::array<double, FILTER_ORDER + 1> a;
};

class ButterworthFilter {
public:
    /// 默认构造函数，使用3Hz截止频率
    ButterworthFilter();
    /// 构造函数，指定截止频率
    /// @param cutoff_freq 截止频率 (Hz)，默认3Hz
    explicit ButterworthFilter(double cutoff_freq);
    double Apply(double input);
    void Reset();

private:
    BiquadFilter filter_;
    std::array<double, FILTER_ORDER + 1> inputHistory_{};
    std::array<double, FILTER_ORDER + 1> outputHistory_{};
    bool initialized_{false};
};