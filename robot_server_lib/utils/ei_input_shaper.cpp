#include "ei_input_shaper.h"

#include <algorithm>
#include <iostream>
#include <numeric>

// EI输入整形器实现 - 用于抑制机械系统振动
EIInputShaper::EIInputShaper(Type type, double natural_freq, double vibration_tol)
    : type_(type), natural_freq_(natural_freq), vibration_tol_(vibration_tol) {
    CalculateCoefficients();
}

void EIInputShaper::CalculateCoefficients() {
    double vib_period = 1.0 / natural_freq_;
    int T_d = static_cast<int>(std::round(vib_period / sampling_time_));
    T3_ = T_d;

    if (type_ == EI) {
        double V = vibration_tol_;
        double z = damping_ratio_;
        double z2 = z * z;
        double V2 = V * V;

        A1_ = 0.24968 + 0.24961 * V + 0.80008 * z + 1.23328 * V * z + 0.49599 * z2 + 3.17316 * V * z2;
        A3_ = 0.25149 + 0.21474 * V - 0.83249 * z + 1.41498 * V * z + 0.85181 * z2 - 4.90094 * V * z2;
        A2_ = 1.0 - A1_ - A3_;

        double T2_time = vib_period * (0.4999 + 0.46159 * V * z + 4.26169 * V * z2 + 1.75601 * V * z * z2 +
                                       8.57843 * V2 * z - 108.644 * V2 * z2 + 336.989 * V2 * z * z2);
        T2_ = static_cast<int>(std::round(T2_time / sampling_time_));
        if (T2_ < 0) T2_ = 0;
        if (T2_ > T3_) T2_ = T3_;

    } else if (type_ == TWO_HUMP_EI) {
        CalculateTwoHumpCoefficients();
    }

    int buffer_size = T3_ + 1;
    buffer_.resize(buffer_size, 0.0);
    buffer_r_.resize(buffer_size, 0.0);
    buffer_l_.resize(buffer_size, 0.0);
    initialized_ = false;
}

void EIInputShaper::CalculateTwoHumpCoefficients() {
    static const std::vector<std::vector<double>> M = {
        {0.49890, 0.16270, -0.54262, 6.16180},  {0.99748, 0.18382, -1.58270, 8.17120},
        {1.49920, -0.09297, -0.28338, 1.85710}, {0.16054, 0.76699, 2.26560, -1.22750},
        {0.33911, 0.45081, -2.58080, 1.73650},  {0.34089, -0.61533, -0.68765, 0.42261},
        {0.15997, -0.60246, 1.00280, -0.93145}};

    double z = damping_ratio_;
    double z2 = z * z;
    double z3 = z2 * z;
    std::vector<double> ZETA = {1.0, z, z2, z3};

    double vib_period = 1.0 / natural_freq_;

    A1_ = std::inner_product(M[3].begin(), M[3].end(), ZETA.begin(), 0.0);
    A2_ = std::inner_product(M[4].begin(), M[4].end(), ZETA.begin(), 0.0);
    A3_ = std::inner_product(M[5].begin(), M[5].end(), ZETA.begin(), 0.0);
    A4_ = std::inner_product(M[6].begin(), M[6].end(), ZETA.begin(), 0.0);

    T2_ = static_cast<int>(
        std::round(std::inner_product(M[0].begin(), M[0].end(), ZETA.begin(), 0.0) * vib_period / sampling_time_));
    T3_ = static_cast<int>(
        std::round(std::inner_product(M[1].begin(), M[1].end(), ZETA.begin(), 0.0) * vib_period / sampling_time_));
    T4_ = static_cast<int>(
        std::round(std::inner_product(M[2].begin(), M[2].end(), ZETA.begin(), 0.0) * vib_period / sampling_time_));
}

double EIInputShaper::RampImpl(double target, double current, double max_rate) {
    double diff = target - current;

    if (std::abs(diff) <= max_rate) {
        return target;
    }

    if (diff > 0) {
        return current + max_rate;
    } else {
        return current - max_rate;
    }
}

double EIInputShaper::Apply(double input) {
    if (decel_mode_) {
        return ApplyImpl(decel_buffer_, 0.0);
    }

    double ramped_input = input;
    if (ramp_enabled_) {
        if (!ramp_initialized_) {
            ramp_output_ = input;
            ramp_initialized_ = true;
        } else {
            ramp_output_ = RampImpl(input, ramp_output_, max_ramp_rate_);
        }
        ramped_input = ramp_output_;
    }

    return ApplyImpl(buffer_, ramped_input);
}

std::pair<double, double> EIInputShaper::Apply(double input_r, double input_l) {
    if (decel_mode_) {
        return {ApplyImpl(decel_buffer_r_, 0.0), ApplyImpl(decel_buffer_l_, 0.0)};
    }

    double ramped_input_r = input_r;
    double ramped_input_l = input_l;

    if (ramp_enabled_) {
        if (!ramp_initialized_) {
            ramp_output_r_ = input_r;
            ramp_output_l_ = input_l;
            ramp_initialized_ = true;
        } else {
            ramp_output_r_ = RampImpl(input_r, ramp_output_r_, max_ramp_rate_);
            ramp_output_l_ = RampImpl(input_l, ramp_output_l_, max_ramp_rate_);
        }
        ramped_input_r = ramp_output_r_;
        ramped_input_l = ramp_output_l_;
    }

    return {ApplyImpl(buffer_r_, ramped_input_r), ApplyImpl(buffer_l_, ramped_input_l)};
}

double EIInputShaper::ApplyImpl(std::deque<double>& buffer, double input) {
    if (!initialized_) {
        std::fill(buffer.begin(), buffer.end(), input);
        initialized_ = true;
        return input;
    }

    buffer.pop_front();
    buffer.push_back(input);

    double output = A1_ * buffer.back();

    if (type_ == EI) {
        if (static_cast<int>(buffer.size()) > T2_) {
            output += A2_ * buffer[buffer.size() - 1 - T2_];
        }
        if (static_cast<int>(buffer.size()) > T3_) {
            output += A3_ * buffer[buffer.size() - 1 - T3_];
        }
    } else if (type_ == TWO_HUMP_EI) {
        if (static_cast<int>(buffer.size()) > T2_) {
            output += A2_ * buffer[buffer.size() - 1 - T2_];
        }
        if (static_cast<int>(buffer.size()) > T3_) {
            output += A3_ * buffer[buffer.size() - 1 - T3_];
        }
        if (static_cast<int>(buffer.size()) > T4_) {
            output += A4_ * buffer[buffer.size() - 1 - T4_];
        }
    }

    return output;
}

void EIInputShaper::DecelStop(double current_velocity, int interpolation_length) {
    DecelStopImpl(decel_buffer_, current_velocity, interpolation_length);
    decel_counter_ = static_cast<int>(decel_buffer_.size());
    decel_total_steps_ = decel_counter_;
    decel_mode_ = true;
}

void EIInputShaper::DecelStop(double velocity_r, double velocity_l, int interpolation_length) {
    DecelStopImpl(decel_buffer_r_, velocity_r, interpolation_length);
    DecelStopImpl(decel_buffer_l_, velocity_l, interpolation_length);
    decel_counter_ = static_cast<int>(decel_buffer_r_.size());
    decel_total_steps_ = decel_counter_;
    decel_mode_ = true;
}

void EIInputShaper::DecelStopImpl(std::deque<double>& buffer, double current_velocity, int interpolation_length) {
    if (buffer.empty()) {
        buffer.resize(T3_ + 1, 0.0);
    }

    int buffer_size = static_cast<int>(buffer.size());
    double ramp_rate = current_velocity / static_cast<double>(interpolation_length);

    for (int i = 0; i < buffer_size - interpolation_length; ++i) {
        buffer.pop_front();
        buffer.push_back(current_velocity);
    }

    for (int i = 0; i < interpolation_length; ++i) {
        buffer.pop_front();
        buffer.push_back(current_velocity - ramp_rate * i);
    }
}

void EIInputShaper::Reset() {
    std::fill(buffer_.begin(), buffer_.end(), 0.0);
    std::fill(buffer_r_.begin(), buffer_r_.end(), 0.0);
    std::fill(buffer_l_.begin(), buffer_l_.end(), 0.0);
    decel_buffer_.clear();
    decel_buffer_r_.clear();
    decel_buffer_l_.clear();
    initialized_ = false;
    decel_mode_ = false;
    decel_counter_ = 0;
    decel_total_steps_ = 0;
    previous_decel_flag_ = false;
    ramp_output_ = 0.0;
    ramp_output_r_ = 0.0;
    ramp_output_l_ = 0.0;
    ramp_initialized_ = false;
}

void EIInputShaper::Configure(double natural_freq) {
    natural_freq_ = natural_freq;
    CalculateCoefficients();
    Reset();
}

void EIInputShaper::SetType(Type type) {
    type_ = type;
    CalculateCoefficients();
    Reset();
}

void EIInputShaper::EnableRamp(bool enable) { ramp_enabled_ = enable; }

void EIInputShaper::SetRampRate(double max_rate_per_sample) { max_ramp_rate_ = max_rate_per_sample; }

bool EIInputShaper::DetectDecelStart(bool current_decel_flag) {
    bool decel_start = (current_decel_flag && !previous_decel_flag_);
    previous_decel_flag_ = current_decel_flag;
    return decel_start;
}
