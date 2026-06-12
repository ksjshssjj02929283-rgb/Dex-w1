#pragma once

#include <atomic>
#include <cassert>
#include <chrono>

namespace robot_server_lib {

// Time point utility
class TimePointBase {
public:
    TimePointBase() = default;
    ~TimePointBase() = default;

    static bool IsValid(const TimePointBase& timepoint) { return timepoint.time_in_ns_ >= 0; }
    static bool IsInvalid(const TimePointBase& timepoint) { return timepoint.time_in_ns_ < 0; }

    // raw repr
    [[nodiscard]] std::int64_t time_in_ns() const { return time_in_ns_; }
    [[nodiscard]] bool is_valid() const { return IsValid(*this); }
    [[nodiscard]] bool is_invalid() const { return IsInvalid(*this); }

protected:
    explicit TimePointBase(std::int64_t time_in_ns) : time_in_ns_(time_in_ns) {};
    std::int64_t time_in_ns_{-1};
};

class TimePointSystem : public TimePointBase {
public:
    TimePointSystem() = default;
    ~TimePointSystem() = default;

    // utility
    static TimePointSystem Now() {
        const auto now = std::chrono::system_clock::now();
        return TimePointSystem{std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count()};
    }
    static const TimePointSystem& Invalid() {
        static const TimePointSystem invalid;
        assert(invalid.is_invalid());
        return invalid;
    }

private:
    explicit TimePointSystem(std::int64_t time_in_ns) : TimePointBase(time_in_ns) {};
};

class TimePointManual : public TimePointBase {
public:
    TimePointManual() = default;
    ~TimePointManual() = default;

    // utility
    static TimePointManual Now() {
        const auto now = shared_time_in_ns.load(std::memory_order_relaxed);
        return TimePointManual{now};
    }
    static const TimePointManual& Invalid() {
        static const TimePointManual invalid(-1);
        assert(invalid.is_invalid());
        return invalid;
    }
    static void IncreaseTimer(std::int64_t increase_time_in_ns) {
        shared_time_in_ns.fetch_add(increase_time_in_ns, std::memory_order_relaxed);
    }

private:
    static std::atomic_int64_t shared_time_in_ns;
    explicit TimePointManual(std::int64_t time_in_ns) : TimePointBase(time_in_ns) {};
};

inline std::atomic_int64_t TimePointManual::shared_time_in_ns{0};

};  // namespace robot_server_lib