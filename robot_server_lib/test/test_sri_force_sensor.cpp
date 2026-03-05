#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <atomic>

#include "sri_force_sensor.h"


static std::string FindSRISensorPort() {
    const char* env_port = std::getenv("SRI_SENSOR_PORT");
    if (env_port && access(env_port, F_OK) == 0) return std::string(env_port);

    // Try common serial device names
    std::vector<std::string> candidates = {"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"};
    for (const auto& p : candidates) {
        if (access(p.c_str(), F_OK) == 0) return p;
    }
    return {};
}

TEST(SRISerialTest, OpenPortAndReadData) {
    std::string port = FindSRISensorPort();
    if (port.empty()) {
        GTEST_SKIP() << "No SRI sensor serial device found. Set SRI_SENSOR_PORT or plug sensor.";
    }

    SerialManeger_linux serial;
    CSRICommM8218Parser parser;

    // Open serial port
    ASSERT_EQ(serial.OpenSerialPort(port.c_str()), 0) << "Failed to open port " << port;

    // Set callback to count received data
    std::atomic<int> callback_count{0};
    SRICommM8218CallbackFunction m8218Callback = [&callback_count](float fx, float fy, float fz, float mx, float my, float mz) {
        callback_count++;
        if(callback_count % 10 == 0){
            printf("Data: %f, %f, %f,   %f, %f, %f\n", fx, fy, fz, mx, my, mz);
        }
        return true;
    };
    parser.SetM8218CallbackFunction(m8218Callback);
    serial.AddCommParser(&parser);

    // Start reading - this runs in background thread
    int ret = serial.SerialRun(460800);
    ASSERT_EQ(ret, 0) << "SerialRun failed with code " << ret;

    // Measure for 10 seconds
    const auto duration_s = std::chrono::seconds(10);
    auto start = std::chrono::steady_clock::now();
    auto end = start + duration_s;

    while (std::chrono::steady_clock::now() < end) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    int count = callback_count.load();

    std::cout << "[SRISerialTest] Port: " << port << ", callback_count: " << count
              << ", elapsed: " << elapsed << " s, rate: " << (count / elapsed) << " Hz" << std::endl;

    // Close port
    serial.ClosePort();

    // Expect at least some successful readings in 5s if sensor is present
    // M8218 typically outputs at 100Hz, so expect ~500 readings
    EXPECT_GE(count, 10) << "Too few callbacks received in 5 seconds on " << port;
}
