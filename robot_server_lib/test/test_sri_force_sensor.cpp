#include <gtest/gtest.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "sri_force_sensor.h"
#include "utils.h"

TEST(SRISerialTest, OpenPortAndReadData) {
    std::filesystem::path home_dir = std::filesystem::path(std::getenv("HOME"));
    std::filesystem::create_directories(home_dir / "log");
    nanolog::initialize(nanolog::GuaranteedLogger(), (home_dir / "log/").string(), "robot_server", 10,
                        10);                           // 每个日志文件最大20MB，最多保留10个日志文件
    nanolog::set_log_level(nanolog::LogLevel::DEBUG);  // options: DEBUG, INFO, WARN, CRIT 高频过程数据打印设置为DEBUG
    std::cout << "log recorded to: " << ((home_dir / "log").string() + "/robot_server.txt") << std::endl;
    SriForceSensor sensor;

    // 初始化传感器（自动查找串口、校准）
    if (!sensor.Init(460800)) {
        GTEST_SKIP() << "No SRI sensor serial device found or init failed.";
    }

    // 测量 5 秒
    const auto duration_s = std::chrono::seconds(5);
    auto start = std::chrono::steady_clock::now();
    auto end = start + duration_s;

    std::atomic<int> read_count{0};
    robot_server_lib::Vector6d data;

    while (std::chrono::steady_clock::now() < end) {
        if (sensor.PeekLatest(&data)) {
            read_count++;
            LOG_INFO << "Read data: " << data[0] << ", " << data[1] << ", " << data[2] << ", " << data[3] << ", "
                     << data[4] << ", " << data[5];
        }
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }

    double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    int count = read_count.load();

    std::cout << "[SRISerialTest] read_count: " << count << ", elapsed: " << elapsed
              << " s, rate: " << (count / elapsed) << " Hz" << std::endl;

    // 关闭传感器
    sensor.Close();

    // 期望至少读到一些数据（2000Hz * 5s = 10000，容错取 5000）
    EXPECT_GE(count, 5000) << "Too few reads received in 5 seconds";
}

#include <Eigen/Geometry>
#include <optional>

namespace {
// Feed n calibration frames with the given raw float channels.
void FeedCalibrationFrames(SriForceSensor& s, int n, float fx, float fy, float fz, float mx, float my, float mz) {
    for (int i = 0; i < n; ++i) {
        s.ProcessCalibrationDataForTest(fx, fy, fz, mx, my, mz);
    }
}
}  // namespace

TEST(SriForceSensorBias, FinalizeRequiresEnoughSamples) {
    SriForceSensor s;
    s.BeginCalibrationForTest();
    s.SetToolGravityCompensationForZeroCalibration(
        false /*enabled=false → no gravity needed*/, 0.0, robot_server_lib::Vector3d::Zero(), 9.79, 1,
        []() -> std::optional<robot_server_lib::Matrix3d> { return std::nullopt; });

    EXPECT_FALSE(s.TryFinalizeBias());  // zero samples
    FeedCalibrationFrames(s, 100, 1.0f, 2.0f, 3.0f, 0.1f, 0.2f, 0.3f);
    EXPECT_FALSE(s.TryFinalizeBias());  // below kMinCalibrationSamples
    EXPECT_TRUE(s.IsCalibrating());

    FeedCalibrationFrames(s, 500, 1.0f, 2.0f, 3.0f, 0.1f, 0.2f, 0.3f);
    EXPECT_TRUE(s.TryFinalizeBias());
    EXPECT_TRUE(s.HasValidZeroOffset());
    EXPECT_FALSE(s.IsCalibrating());
    EXPECT_GE(s.GetZeroOffsetSampleCount(), 500);
}

TEST(SriForceSensorBias, WaitsForValidRotation) {
    SriForceSensor s;
    s.BeginCalibrationForTest();
    bool r_valid = false;
    s.SetToolGravityCompensationForZeroCalibration(true, 1.0, robot_server_lib::Vector3d::Zero(), 9.79, 1,
                                                   [&]() -> std::optional<robot_server_lib::Matrix3d> {
                                                       if (!r_valid) return std::nullopt;
                                                       return robot_server_lib::Matrix3d::Identity();
                                                   });

    FeedCalibrationFrames(s, 1000, 0.0f, 0.0f, -9.79f, 0.0f, 0.0f, 0.0f);
    EXPECT_FALSE(s.TryFinalizeBias());  // R still nullopt
    EXPECT_FALSE(s.HasValidZeroOffset());
    EXPECT_TRUE(s.IsCalibrating());

    r_valid = true;
    EXPECT_TRUE(s.TryFinalizeBias());
    EXPECT_TRUE(s.HasValidZeroOffset());
}

TEST(SriForceSensorBias, GravityDisabledFinalizesOnSamplesAlone) {
    SriForceSensor s;
    s.BeginCalibrationForTest();
    s.SetToolGravityCompensationForZeroCalibration(
        false, 0.0, robot_server_lib::Vector3d::Zero(), 9.79, 1,
        []() -> std::optional<robot_server_lib::Matrix3d> { return std::nullopt; });

    FeedCalibrationFrames(s, 500, 1.0f, 2.0f, 3.0f, 0.1f, 0.2f, 0.3f);
    EXPECT_TRUE(s.TryFinalizeBias());
    EXPECT_TRUE(s.HasValidZeroOffset());
}

TEST(SriForceSensorBias, PeekLatestBlockedUntilFinalized) {
    SriForceSensor s;
    s.BeginCalibrationForTest();
    robot_server_lib::Vector6d out;
    EXPECT_FALSE(s.PeekLatest(&out));
}

TEST(SriForceSensorBias, TryFinalizeReturnsTrueOnceOnly) {
    SriForceSensor s;
    s.BeginCalibrationForTest();
    s.SetToolGravityCompensationForZeroCalibration(
        false, 0.0, robot_server_lib::Vector3d::Zero(), 9.79, 1,
        []() -> std::optional<robot_server_lib::Matrix3d> { return std::nullopt; });

    FeedCalibrationFrames(s, 500, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    EXPECT_TRUE(s.TryFinalizeBias());
    EXPECT_FALSE(s.TryFinalizeBias());  // already finalized → false
    EXPECT_FALSE(s.TryFinalizeBias());  // still false on subsequent calls
}
