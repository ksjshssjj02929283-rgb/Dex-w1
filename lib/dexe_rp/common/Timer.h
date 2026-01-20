/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include <chrono>

#include "dexe_rp/common/Logger.h"

namespace DexeRP {

struct TimeDuration;

struct TimeStamp {
public:
    TimeStamp();

    /**
     * @brief Construct a new TimeStamp by giving sec and nsec
     *
     * @param sec [in] seconds
     * @param nsec [in] nano seconds
     */
    TimeStamp(uint32_t sec, uint32_t nsec);

    /**
     * @brief Construct a newTime Stamp by giving seconds
     *
     * @param sec [in] time in seconds
     */
    explicit TimeStamp(double sec);  // explicit 防止编译器隐式类型转换

    ~TimeStamp() = default;

    /**
     * @brief Static function to get current time stamp
     *
     * @return TimeStamp
     */
    static TimeStamp Now();

    /**
     * @brief Update TimeStamp by giving time in seconds
     *
     * @param t [in] time in seconds
     * @return TimeStamp&
     */
    TimeStamp &FromSec(double t);

    /**
     * @brief Update TimeStamp by giving time in nano seconds
     *
     * @param t [in] time in nano seconds
     * @return TimeStamp&
     */
    TimeStamp &FromNanoSec(int64_t t);

    /**
     * @brief Get time in seconds
     *
     * double range is not enough to hold nano seconds, if we return a double,
     * there will be precision lost.
     *
     * @return double
     */
    double ToSec() const;

    /**
     * @brief Get time in nano seconds
     *
     * @return int64_t
     */
    int64_t ToNanoSec() const;

    bool operator==(const TimeStamp &o) const;
    bool operator!=(const TimeStamp &o) const;
    bool operator>(const TimeStamp &o) const;
    bool operator<(const TimeStamp &o) const;
    bool operator>=(const TimeStamp &o) const;
    bool operator<=(const TimeStamp &o) const;

    TimeDuration operator-(const TimeStamp &other) const;
    TimeStamp operator-(const TimeDuration &duration) const;
    TimeStamp &operator-=(const TimeDuration &duration);
    TimeStamp operator+(const TimeDuration &duration) const;
    TimeStamp &operator+=(const TimeDuration &duration);

public:
    uint32_t sec;
    uint32_t nsec;
};

struct TimeDuration {
public:
    TimeDuration();
    TimeDuration(int32_t sec, int32_t nsec);
    explicit TimeDuration(double sec);

    ~TimeDuration() = default;

    TimeDuration &FromSec(double t);
    TimeDuration &FromMilliSec(double t);
    TimeDuration &FromMicroSec(double t);
    TimeDuration &FromNanoSec(int64_t t);

    double ToSec() const;
    double ToMilliSec() const;
    double ToMicroSec() const;
    int64_t ToNanoSec() const;

    bool operator==(const TimeDuration &o) const;
    bool operator!=(const TimeDuration &o) const;
    bool operator>(const TimeDuration &o) const;
    bool operator<(const TimeDuration &o) const;
    bool operator>=(const TimeDuration &o) const;
    bool operator<=(const TimeDuration &o) const;

    TimeDuration operator+(const TimeDuration &other) const;
    TimeDuration &operator+=(const TimeDuration &other);
    TimeDuration operator-() const;
    TimeDuration operator-(const TimeDuration &other) const;
    TimeDuration &operator-=(const TimeDuration &other);
    TimeDuration operator*(double scale) const;
    TimeDuration &operator*=(double scale);
    TimeDuration operator/(double scale) const;
    TimeDuration &operator/=(double scale);

    /**
     * @brief Sleep for the amount of time specified by this TimeDuration.
     */
    void Sleep() const;

public:
    int32_t sec;   ///< seconds
    int32_t nsec;  ///< nano seconds
};

class Timer {
public:
    Timer() : start_(0), stop_(0), duration_(0), is_timing_(false) {}

    ~Timer() = default;

    inline void Start() {
        if (is_timing_) {
            DEXERP_WARN("A running timer cannot be re-started, do nothing.");
            return;
        }
        is_timing_ = true;
        stop_ = start_ = TimeStamp::Now();
    }

    inline void Stop() {
        if (!is_timing_) {
            DEXERP_WARN("A stopped timer cannot be re-stopped, do nothing.");
            return;
        }
        is_timing_ = false;
        stop_ = TimeStamp::Now();
        duration_ += stop_ - start_;
    }

    inline void Reset() {
        if (is_timing_) {
            DEXERP_WARN("A running timer cannot be reset, do nothing.");
            return;
        }
        is_timing_ = false;
        stop_ = start_ = TimeStamp::Now();
        duration_ = TimeDuration(0);
    }

    inline void ResetAndStart() {
        Reset();
        Start();
    }

    inline TimeDuration Duration() const {
        if (is_timing_) {
            return TimeStamp::Now() - start_ + duration_;
        } else {
            return duration_;
        }
    }

private:
    TimeStamp start_, stop_;
    TimeDuration duration_;
    bool is_timing_;
};

}  // namespace DexeRP