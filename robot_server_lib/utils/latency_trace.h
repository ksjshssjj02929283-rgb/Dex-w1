#pragma once

#include <cstdint>
#include <cstdio>
#include <ctime>
#include <string>

namespace robot_server_lib::utils {

// 在测量点同时采集两种时钟，允许延后输出日志；mono_ns 用于计算耗时，wall 用于对照物理时间。
struct LatencyTraceTime {
    int64_t mono_ns{0};
    timespec wall{};

    static LatencyTraceTime Now() {
        LatencyTraceTime stamp;
        timespec mono{};
        clock_gettime(CLOCK_MONOTONIC, &mono);
        clock_gettime(CLOCK_REALTIME, &stamp.wall);
        stamp.mono_ns = static_cast<int64_t>(mono.tv_sec) * 1000000000LL + mono.tv_nsec;
        return stamp;
    }

    std::string WallTime() const {
        tm local{};
        localtime_r(&wall.tv_sec, &local);
        char date[32]{};
        strftime(date, sizeof(date), "%Y-%m-%d %H:%M:%S", &local);
        char formatted[64]{};
        snprintf(formatted, sizeof(formatted), "%s.%09ld", date, wall.tv_nsec);
        return formatted;
    }
};

}  // namespace robot_server_lib::utils
