#include "butterworth_filter.h"

ButterworthFilter::ButterworthFilter() {
    // 一阶低通滤波器：fc=3Hz, fs=250Hz, 无超调
    // alpha = 2*pi*fc / (fs + 2*pi*fc) ≈ 0.0753
    filter_.b = {7.5303293549481168e-02, 7.5303293549481168e-02};
    filter_.a = {1.0, -8.4939670251051883e-01};
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