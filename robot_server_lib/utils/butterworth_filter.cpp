#include "butterworth_filter.h"

ButterworthFilter::ButterworthFilter() {
    // 固定参数：3Hz 二阶临界阻尼滤波器（ζ=1.0，无过冲）
    filter_.b = {8.7175477092027442e-05, 1.7435095418405488e-04, 8.7175477092027442e-05};
    filter_.a = {1.0, -1.9626528765033522, 0.9630015784117203};
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