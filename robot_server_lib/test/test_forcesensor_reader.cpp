#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "force_sensor_reader.h"

using namespace robot_server_lib;

static std::string FindForceSensorPort() {
    const char* env_port = std::getenv("FORCE_SENSOR_PORT");
    if (env_port && access(env_port, F_OK) == 0) return std::string(env_port);

    // Try common serial device names
    std::vector<std::string> candidates = {"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"};
    for (const auto& p : candidates) {
        if (access(p.c_str(), F_OK) == 0) return p;
    }
    return {};
}

TEST(ForceSensorReaderTest, ReadForTenSeconds) {
    std::string port = FindForceSensorPort();
    if (port.empty()) {
        GTEST_SKIP() << "No force sensor serial device found. Set FORCE_SENSOR_PORT or plug sensor.";
    }

    ForceSensorReader reader(port);
    Vector6d data;

    // Warm-up: wait up to 2s for sensor to become ready (zero calibration thread)
    auto warm_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    bool ready = false;
    while (std::chrono::steady_clock::now() < warm_deadline) {
        if (reader.Read(data)) {
            ready = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (!ready) {
        // Continue anyway; measurement window may still see readiness.
    }

    // Measure for 10 seconds
    const auto duration_s = std::chrono::seconds(10);
    auto start = std::chrono::steady_clock::now();
    auto end = start + duration_s;
    size_t success_count = 0;
    size_t total_attempts = 0;

    while (std::chrono::steady_clock::now() < end) {
        Vector6d v;
        if (reader.Read(v)) {
            ++success_count;
        }
        ++total_attempts;
        // Avoid busy loop; sensor read is non-blocking
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    double rate = success_count / elapsed;

    std::cout << "[ForceSensorReaderTest] Port: " << port << ", success_count: " << success_count
              << ", attempts: " << total_attempts << ", elapsed: " << elapsed << " s"
              << ", success_rate: " << rate << " Hz" << std::endl;

    // Expect at least one successful reading in 10s if sensor is present
    EXPECT_GE(success_count, static_cast<size_t>(1)) << "No successful reads in 10 seconds on " << port;
}