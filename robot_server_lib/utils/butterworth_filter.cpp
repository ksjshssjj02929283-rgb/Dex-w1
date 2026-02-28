#include "butterworth_filter.h"

ButterworthFilter::ButterworthFilter() {
    // 固定参数：3Hz 二阶临界阻尼低通滤波器 (Fs=250Hz, ζ=1.0)
    filter_.b = {0.0013487119483563, 0.0026974238967127, 0.0013487119483563};
    filter_.a = {1.0, -1.8934641463618269, 0.8988589941552524};
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