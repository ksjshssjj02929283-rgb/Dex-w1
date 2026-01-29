#include "ei_input_shaper.h"

#include <algorithm>
#include <cmath>

EIInputShaper::EIInputShaper(double natural_frequency_hz,
                             double damping_ratio,
                             double sampling_time_sec,
                             double vibration_tolerance)
    : natural_frequency_hz_(natural_frequency_hz),
      damping_ratio_(damping_ratio),
      sampling_time_(sampling_time_sec),
      vibration_tolerance_(vibration_tolerance),
      A1_(0.0),
      A2_(0.0),
      A3_(0.0),
      T2_(0),
      T3_(0),
      T_d_(0),
      initialized_(false) {
    CalculateCoefficients();
}

void EIInputShaper::CalculateCoefficients() {
    // 计算振动周期
    double freq_hz = natural_frequency_hz_;
    double vibration_period = 1.0 / freq_hz;  // 单位：秒

    // 计算离散化振动周期
    T_d_ = static_cast<int>(std::round(vibration_period / sampling_time_));
    T3_ = T_d_;

    // 简化变量名
    double V_tol = vibration_tolerance_;
    double zeta = damping_ratio_;
    double zeta2 = zeta * zeta;
    double zeta3 = zeta2 * zeta;
    double V_tol2 = V_tol * V_tol;

    // 使用 EI 多项式公式计算 A1
    A1_ =
        0.24968 + 0.24961 * V_tol + 0.80008 * zeta + 1.23328 * V_tol * zeta + 0.49599 * zeta2 + 3.17316 * V_tol * zeta2;

    // 使用 EI 多项式公式计算 A3
    A3_ =
        0.25149 + 0.21474 * V_tol - 0.83249 * zeta + 1.41498 * V_tol * zeta + 0.85181 * zeta2 - 4.90094 * V_tol * zeta2;

    // 归一化约束：A1 + A2 + A3 = 1
    A2_ = 1.0 - A1_ - A3_;

    // 使用 EI 多项式公式计算 T2 时间
    double T2_time =
        vibration_period * (0.4999 + 0.46159 * V_tol * zeta + 4.26169 * V_tol * zeta2 + 1.75601 * V_tol * zeta3 +
                            8.57843 * V_tol2 * zeta - 108.644 * V_tol2 * zeta2 + 336.989 * V_tol2 * zeta3);

    // 离散化 T2
    T2_ = static_cast<int>(std::round(T2_time / sampling_time_));

    // 确保 T2 在有效范围内 [0, T3]
    if (T2_ < 0) T2_ = 0;
    if (T2_ > T3_) T2_ = T3_;

    // 调整缓冲区大小（需要存储 T3+1 个样本）
    input_history_.resize(T3_ + 1, 0.0);
}

double EIInputShaper::Apply(double input) {
    // 首次调用：用输入值填充缓冲区
    if (!initialized_) {
        std::fill(input_history_.begin(), input_history_.end(), input);
        initialized_ = true;
        return input;  // 第一个样本不整形
    }

    // 更新 FIFO 缓冲区
    input_history_.pop_front();
    input_history_.push_back(input);

    // 应用 EI 整形公式：
    // output = A1·input(t) + A2·input(t-T2) + A3·input(t-T3)
    // 注意：input_history_[T3_] 是最新的输入（当前时刻）
    //       input_history_[T3_ - T2_] 是延迟 T2 的输入
    //       input_history_[0] 是延迟 T3 的输入
    double output = A1_ * input_history_[T3_] +        // 当前输入
                    A2_ * input_history_[T3_ - T2_] +  // 延迟 T2
                    A3_ * input_history_[0];           // 延迟 T3

    return output;
}

void EIInputShaper::Reset() {
    if (!initialized_) return;
    initialized_ = false;
    std::fill(input_history_.begin(), input_history_.end(), 0.0);
}

void EIInputShaper::Configure(double natural_frequency_hz, double damping_ratio, double vibration_tolerance) {
    natural_frequency_hz_ = natural_frequency_hz;
    damping_ratio_ = damping_ratio;

    // 如果提供了新的振动容忍度，则更新
    if (vibration_tolerance >= 0.0) {
        vibration_tolerance_ = vibration_tolerance;
    }

    CalculateCoefficients();
    // 重置状态，下次 Apply 时重新初始化
    initialized_ = false;
}

int EIInputShaper::GetDelaySamples() const { return T_d_; }

double EIInputShaper::GetDelaySeconds() const { return T_d_ * sampling_time_; }

bool EIInputShaper::IsInitialized() const { return initialized_; }

void EIInputShaper::GetCoefficients(double& A1, double& A2, double& A3, int& T2, int& T3) const {
    A1 = A1_;
    A2 = A2_;
    A3 = A3_;
    T2 = T2_;
    T3 = T3_;
}
