#pragma once
// IMU 健康状态机：把「IMU 能不能用」从单一的超时判据扩成五类，并带恢复迟滞。
// 不依赖 ROS / Eigen，可在 gtest 中单独测试；节点侧负责加锁与日志/diagnostics 外发。
//
// 失效类别（严重度从高到低，同时满足取最严重）：
//   Invalid       非有限值（NaN/Inf）连续超过 invalid_frames；或时间戳与节点时钟疑似跨源
//   Frozen        六轴原始值连续 frozen_frames 帧完全相同（传感器卡死 / 驱动重复发）
//   Timeout       最近一帧 stamp 距现在超过 timeout_sec
//   Noisy         陀螺 z 帧间差分在滑窗内的标准差超过 gyro_noise_max（振动 / 器件异常）
//   Uninitialized 静止初始化尚未完成（开机前几秒属正常；超过 init_timeout_sec 视为异常）
//   Ok
//
// 为什么噪声判据用「帧间差分」而不是原始值的标准差：机器人真实转动时陀螺 z 在 1 s 内
// 可以从 0 变到 0.5 rad/s，原始值标准差会把正常机动误判成噪声；而 100 Hz 下真实转动的
// 帧间差分只有 ω̇·dt ≈ 0.01 rad/s 量级且平滑，白噪声 / 振动的差分则是随机跳变，两者可分。
//
// 恢复迟滞：Timeout / Frozen / Noisy / Invalid 解除后，需连续 recover_frames 帧正常才回 Ok，
// 避免抖动的 IMU 让 EKF 在「用 IMU / 不用 IMU」之间逐帧翻转。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <string>

namespace chassis_node {

enum class ImuHealth { Ok, Uninitialized, Noisy, Timeout, Frozen, Invalid };

inline const char* ImuHealthName(ImuHealth h) {
    switch (h) {
        case ImuHealth::Ok:            return "ok";
        case ImuHealth::Uninitialized: return "uninitialized";
        case ImuHealth::Noisy:         return "noisy";
        case ImuHealth::Timeout:       return "timeout";
        case ImuHealth::Frozen:        return "frozen";
        case ImuHealth::Invalid:       return "invalid";
    }
    return "unknown";
}

struct ImuHealthParams {
    double timeout_sec        = 0.1;     // 超时（与 chassis_node 的 imu_timeout 同源）
    int    invalid_frames     = 3;       // 连续非有限值帧数 → Invalid
    int    frozen_frames      = 20;      // 连续完全相同帧数 → Frozen（0.2 s @100 Hz）
    double gyro_noise_max     = 0.02;    // 陀螺 z 帧间差分标准差上限 (rad/s)
    double noise_window_sec   = 1.0;     // 差分标准差滑窗长度
    int    noise_min_samples  = 20;      // 滑窗内至少这么多差分才判噪声
    int    recover_frames     = 20;      // 失效解除后需连续正常帧数才回 Ok
    double init_timeout_sec   = 30.0;    // 未初始化超过此时长视为异常（init_overdue）
    double cross_clock_sec    = 1.0e6;   // age 超过此值判跨时钟源（跨源偏差是 10^9 s 量级）
    double long_outage_sec    = 60.0;    // 超过此时长的超时在 reason 里提示查驱动
};

struct ImuHealthReport {
    ImuHealth   state = ImuHealth::Uninitialized;
    std::string reason;                    // 英文，直接进日志 / diagnostics
    double      age_sec = NAN;             // now − 最近一帧 stamp
    double      gyro_diff_std = NAN;       // 当前滑窗差分标准差
    bool        init_overdue = false;      // 未初始化已超 init_timeout_sec
    bool        changed = false;           // 本次 Evaluate 状态是否变化
    double      degraded_since_sec = NAN;  // 进入非 Ok 的时刻（节点时钟秒）
    uint32_t    degraded_count = 0;        // Ok → 非 Ok 的次数
    double      degraded_total_sec = 0.0;  // 累计非 Ok 时长（含 Uninitialized）
    int         recover_progress = 0;      // 恢复迟滞已累计的正常帧数
};

class ImuHealthMonitor {
public:
    explicit ImuHealthMonitor(const ImuHealthParams& p = ImuHealthParams()) : p_(p) {}

    void SetParams(const ImuHealthParams& p) { p_ = p; }
    const ImuHealthParams& params() const { return p_; }

    /// 每帧原始数据到达时调用（去零偏前的原始值）。返回 false 表示该帧含非有限值，调用方应丢弃。
    bool OnSample(double stamp_sec, double ax, double ay, double az, double gx, double gy, double gz) {
        const bool finite = std::isfinite(ax) && std::isfinite(ay) && std::isfinite(az) &&
                            std::isfinite(gx) && std::isfinite(gy) && std::isfinite(gz) && std::isfinite(stamp_sec);
        if (!finite) {
            ++invalid_streak_;
            healthy_streak_ = 0;
            return false;
        }
        invalid_streak_ = 0;

        // 冻结：六轴与上一帧完全相同（浮点精确相等——真实传感器连续两帧六轴全同的概率为零）。
        // frozen_streak_ 计的是「当前这串相同帧的帧数」（含首帧），与参数 frozen_frames 同口径。
        if (has_sample_ && ax == last_[0] && ay == last_[1] && az == last_[2] &&
            gx == last_[3] && gy == last_[4] && gz == last_[5]) {
            ++frozen_streak_;
        } else {
            frozen_streak_ = 1;
        }

        // 噪声：陀螺 z 帧间差分的滑窗标准差
        if (has_sample_) {
            const double d = gz - last_[5];
            PushDiff(stamp_sec, d);
        }
        last_[0] = ax; last_[1] = ay; last_[2] = az; last_[3] = gx; last_[4] = gy; last_[5] = gz;
        last_stamp_ = stamp_sec;
        if (!has_sample_) first_stamp_ = stamp_sec;
        has_sample_ = true;

        const bool frame_ok = frozen_streak_ < p_.frozen_frames && !NoisyNow();
        healthy_streak_ = frame_ok ? healthy_streak_ + 1 : 0;
        return true;
    }

    /// 周期评估。now_sec 用节点时钟（与 stamp 同源）；inited 为静止初始化是否完成。
    const ImuHealthReport& Evaluate(double now_sec, bool inited) {
        if (std::isnan(eval_start_)) eval_start_ = now_sec;
        const ImuHealth prev = r_.state;
        r_.changed = false;
        r_.reason.clear();
        r_.init_overdue = false;
        r_.gyro_diff_std = DiffStd();
        r_.recover_progress = healthy_streak_;

        ImuHealth s = ImuHealth::Ok;
        if (!has_sample_) {
            // 从未收到过样本 ≠ 数据中断：驱动可能比本节点晚起。按 init_timeout_sec 给宽限，
            // 宽限内报 Uninitialized（WARN 级），超过才升 Timeout（ERROR 级）。
            r_.age_sec = now_sec - eval_start_;
            if (r_.age_sec > p_.init_timeout_sec) {
                s = ImuHealth::Timeout;
                r_.reason = "no IMU sample received for " + Fmt(r_.age_sec, 1) + " s since start";
            } else {
                s = ImuHealth::Uninitialized;
                r_.reason = "waiting for first IMU sample (" + Fmt(r_.age_sec, 1) + " s)";
            }
        } else {
            r_.age_sec = now_sec - last_stamp_;
            if (r_.age_sec < -1.0 || r_.age_sec > p_.cross_clock_sec) {
                s = ImuHealth::Invalid;
                r_.reason = "stamp and node clock appear to be different sources (age=" + Fmt(r_.age_sec, 3) +
                            " s); check whether the publisher uses rclcpp::Clock()";
            } else if (invalid_streak_ >= p_.invalid_frames) {
                s = ImuHealth::Invalid;
                r_.reason = "non-finite values in " + std::to_string(invalid_streak_) + " consecutive samples";
            } else if (frozen_streak_ >= p_.frozen_frames) {
                s = ImuHealth::Frozen;
                r_.reason = "identical readings for " + std::to_string(frozen_streak_) +
                            " consecutive samples (sensor stuck or driver repeating)";
            } else if (r_.age_sec > p_.timeout_sec) {
                s = ImuHealth::Timeout;
                r_.reason = (r_.age_sec > p_.long_outage_sec)
                                ? "no IMU data for " + Fmt(r_.age_sec, 0) + " s, check whether the driver is disconnected"
                                : "IMU timeout (" + Fmt(r_.age_sec, 3) + " s > " + Fmt(p_.timeout_sec, 3) + " s)";
            } else if (NoisyNow()) {
                s = ImuHealth::Noisy;
                r_.reason = "gyro z frame-to-frame std " + Fmt(r_.gyro_diff_std, 4) + " rad/s > " +
                            Fmt(p_.gyro_noise_max, 4);
            } else if (!inited) {
                s = ImuHealth::Uninitialized;
                const double waited = now_sec - first_stamp_;
                r_.init_overdue = waited > p_.init_timeout_sec;
                r_.reason = "static initialization pending for " + Fmt(waited, 1) + " s" +
                            (r_.init_overdue ? " (overdue, check that the robot is still and thresholds fit the device)" : "");
            }
        }

        // 恢复迟滞：从数据类失效（非 Uninitialized）解除后，连续正常帧数不够时维持原失效态
        if (s == ImuHealth::Ok && IsDataFault(prev) && healthy_streak_ < p_.recover_frames) {
            s = prev;
            r_.reason = std::string("recovering from ") + ImuHealthName(prev) + " (" +
                        std::to_string(healthy_streak_) + "/" + std::to_string(p_.recover_frames) + " good samples)";
        }

        // 统计与迁移
        if (prev == ImuHealth::Ok && s != ImuHealth::Ok) {
            r_.degraded_since_sec = now_sec;
            ++r_.degraded_count;
        } else if (prev != ImuHealth::Ok && s == ImuHealth::Ok) {
            if (!std::isnan(r_.degraded_since_sec)) r_.degraded_total_sec += now_sec - r_.degraded_since_sec;
            r_.degraded_since_sec = NAN;
        }
        if (!seen_first_eval_) {
            // 首次评估：起始态就是非 Ok（未初始化），从这里开始计时但不计入 count
            seen_first_eval_ = true;
            if (s != ImuHealth::Ok) r_.degraded_since_sec = now_sec;
        }
        r_.changed = (s != prev);
        r_.state = s;
        return r_;
    }

    const ImuHealthReport& report() const { return r_; }

    void Reset() {
        *this = ImuHealthMonitor(p_);
    }

    static bool IsDataFault(ImuHealth h) {
        return h == ImuHealth::Timeout || h == ImuHealth::Frozen || h == ImuHealth::Noisy || h == ImuHealth::Invalid;
    }

private:
    struct Diff { double t; double d; };

    void PushDiff(double t, double d) {
        diffs_.push_back({t, d});
        sum_ += d;
        sumsq_ += d * d;
        while (!diffs_.empty() && t - diffs_.front().t > p_.noise_window_sec) {
            sum_ -= diffs_.front().d;
            sumsq_ -= diffs_.front().d * diffs_.front().d;
            diffs_.pop_front();
        }
    }
    double DiffStd() const {
        const size_t n = diffs_.size();
        if (n < 2) return NAN;
        const double mean = sum_ / static_cast<double>(n);
        const double var = std::max(0.0, sumsq_ / static_cast<double>(n) - mean * mean);
        return std::sqrt(var);
    }
    bool NoisyNow() const {
        if (static_cast<int>(diffs_.size()) < p_.noise_min_samples) return false;
        const double s = DiffStd();
        return std::isfinite(s) && s > p_.gyro_noise_max;
    }
    static std::string Fmt(double v, int digits) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.*f", digits, v);
        return buf;
    }

    ImuHealthParams p_;
    ImuHealthReport r_;
    bool   has_sample_ = false;
    bool   seen_first_eval_ = false;
    double last_[6] = {0, 0, 0, 0, 0, 0};
    double last_stamp_ = NAN;
    double first_stamp_ = NAN;
    double eval_start_ = NAN;
    int    invalid_streak_ = 0;
    int    frozen_streak_ = 0;
    int    healthy_streak_ = 0;
    std::deque<Diff> diffs_;
    double sum_ = 0.0;
    double sumsq_ = 0.0;
};

}  // namespace chassis_node
