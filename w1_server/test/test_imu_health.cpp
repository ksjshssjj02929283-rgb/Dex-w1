// imu_health 单元测试：五类失效判据 + 恢复迟滞 + 统计
//
// 不依赖 ROS，直接喂 100 Hz 合成样本。

#include <gtest/gtest.h>

#include <cmath>

#include "imu_health.h"

namespace chassis_node {
namespace {

constexpr double kDt = 0.01;  // 100 Hz
constexpr double kG = 9.7887;

ImuHealthParams P() { return ImuHealthParams{}; }

/// 喂 n 帧静止样本（带微小白噪声，避免被判冻结），返回最后一帧 stamp
double FeedStatic(ImuHealthMonitor& m, double t0, int n, double gz_noise = 0.001) {
    double t = t0;
    for (int i = 0; i < n; ++i) {
        t = t0 + i * kDt;
        const double n1 = ((i * 7919) % 13 - 6) * 1e-4;            // 确定性伪随机 ±6e-4
        const double n2 = ((i * 104729) % 11 - 5) * gz_noise / 5;  // gz 噪声幅度可控
        m.OnSample(t, 0.01 + n1, -0.02 - n1, kG + n1, 0.001 + n2, -0.0005, 0.0002 + n2);
    }
    return t;
}

TEST(ImuHealth, StaticInitedIsOk) {
    ImuHealthMonitor m(P());
    const double t = FeedStatic(m, 0.0, 100);
    const auto& r = m.Evaluate(t, /*inited=*/true);
    EXPECT_EQ(r.state, ImuHealth::Ok) << r.reason;
    EXPECT_LT(r.age_sec, 0.02);
    EXPECT_LT(r.gyro_diff_std, P().gyro_noise_max);
}

TEST(ImuHealth, UninitializedBeforeStaticInitAndOverdueAfterTimeout) {
    ImuHealthMonitor m(P());
    double t = FeedStatic(m, 0.0, 100);
    auto r = m.Evaluate(t, /*inited=*/false);
    EXPECT_EQ(r.state, ImuHealth::Uninitialized);
    EXPECT_FALSE(r.init_overdue);
    // 40 s 仍未初始化 → overdue
    t = FeedStatic(m, t, 4000);
    r = m.Evaluate(t, false);
    EXPECT_EQ(r.state, ImuHealth::Uninitialized);
    EXPECT_TRUE(r.init_overdue);
    // 初始化完成 → 立即 Ok（不走数据类失效的迟滞）
    r = m.Evaluate(t, true);
    EXPECT_EQ(r.state, ImuHealth::Ok);
    EXPECT_TRUE(r.changed);
}

TEST(ImuHealth, NanSamplesAreRejectedAndBecomeInvalidAfterThreeFrames) {
    ImuHealthMonitor m(P());
    double t = FeedStatic(m, 0.0, 50);
    m.Evaluate(t, true);
    const double nan = std::nan("");
    EXPECT_FALSE(m.OnSample(t + kDt, nan, 0, kG, 0, 0, 0));
    EXPECT_FALSE(m.OnSample(t + 2 * kDt, 0, nan, kG, 0, 0, 0));
    EXPECT_EQ(m.Evaluate(t + 2 * kDt, true).state, ImuHealth::Ok);  // 2 帧还不判
    EXPECT_FALSE(m.OnSample(t + 3 * kDt, 0, 0, kG, 0, 0, nan));
    const auto& r = m.Evaluate(t + 3 * kDt, true);
    EXPECT_EQ(r.state, ImuHealth::Invalid) << r.reason;
    EXPECT_EQ(r.degraded_count, 1u);
}

TEST(ImuHealth, RecoveryNeedsHysteresis) {
    ImuHealthMonitor m(P());
    double t = FeedStatic(m, 0.0, 50);
    m.Evaluate(t, true);
    for (int i = 1; i <= 3; ++i) m.OnSample(t + i * kDt, std::nan(""), 0, kG, 0, 0, 0);
    ASSERT_EQ(m.Evaluate(t + 3 * kDt, true).state, ImuHealth::Invalid);
    // 19 帧正常：仍报 Invalid（recovering）
    t = FeedStatic(m, t + 4 * kDt, 19);
    auto r = m.Evaluate(t, true);
    EXPECT_EQ(r.state, ImuHealth::Invalid);
    EXPECT_NE(r.reason.find("recovering"), std::string::npos);
    // 第 20 帧 → Ok
    t = FeedStatic(m, t + kDt, 1);
    r = m.Evaluate(t, true);
    EXPECT_EQ(r.state, ImuHealth::Ok) << r.reason;
    EXPECT_TRUE(r.changed);
    EXPECT_GT(r.degraded_total_sec, 0.0);
}

TEST(ImuHealth, FrozenReadingsDetected) {
    ImuHealthMonitor m(P());
    double t = FeedStatic(m, 0.0, 50);
    for (int i = 1; i <= 19; ++i) m.OnSample(t + i * kDt, 0.01, -0.02, kG, 0.001, -0.0005, 0.0002);
    EXPECT_EQ(m.Evaluate(t + 19 * kDt, true).state, ImuHealth::Ok);  // 19 帧相同还不判
    m.OnSample(t + 20 * kDt, 0.01, -0.02, kG, 0.001, -0.0005, 0.0002);
    const auto& r = m.Evaluate(t + 20 * kDt, true);
    EXPECT_EQ(r.state, ImuHealth::Frozen) << r.reason;
}

TEST(ImuHealth, TimeoutAndLongOutageReason) {
    ImuHealthMonitor m(P());
    const double t = FeedStatic(m, 0.0, 50);
    EXPECT_EQ(m.Evaluate(t + 0.05, true).state, ImuHealth::Ok);
    auto r = m.Evaluate(t + 0.3, true);
    EXPECT_EQ(r.state, ImuHealth::Timeout) << r.reason;
    EXPECT_NE(r.reason.find("timeout"), std::string::npos);
    r = m.Evaluate(t + 120.0, true);
    EXPECT_EQ(r.state, ImuHealth::Timeout);
    EXPECT_NE(r.reason.find("check whether the driver"), std::string::npos);
}

TEST(ImuHealth, CrossClockSourceIsInvalid) {
    ImuHealthMonitor m(P());
    const double t = FeedStatic(m, 0.0, 50);
    EXPECT_EQ(m.Evaluate(t - 5.0, true).state, ImuHealth::Invalid);    // 未来时间戳
    EXPECT_EQ(m.Evaluate(t + 1.8e9, true).state, ImuHealth::Invalid);  // 1.8e9 s：MONOTONIC vs REALTIME
}

TEST(ImuHealth, NoisyGyroDetectedButSmoothRotationIsNot) {
    // 噪声突增：gz 逐帧 ±0.1 跳变 → 差分 std ≈ 0.2 ≫ 0.02
    ImuHealthMonitor m(P());
    double t = FeedStatic(m, 0.0, 50);
    for (int i = 1; i <= 100; ++i) {
        m.OnSample(t + i * kDt, 0.01, -0.02, kG, 0.001, -0.0005, (i % 2 ? 0.1 : -0.1));
    }
    auto r = m.Evaluate(t + 100 * kDt, true);
    EXPECT_EQ(r.state, ImuHealth::Noisy) << r.reason;
    EXPECT_GT(r.gyro_diff_std, P().gyro_noise_max);

    // 真实转动：gz 1 s 内从 0 平滑升到 1 rad/s（差分恒 0.01）→ 不是噪声
    ImuHealthMonitor m2(P());
    t = FeedStatic(m2, 0.0, 50);
    for (int i = 1; i <= 100; ++i) {
        m2.OnSample(t + i * kDt, 0.01, -0.02, kG, 0.001, -0.0005, 0.01 * i);
    }
    r = m2.Evaluate(t + 100 * kDt, true);
    EXPECT_EQ(r.state, ImuHealth::Ok) << r.reason;
    EXPECT_LT(r.gyro_diff_std, 0.005);  // 滑窗尾部还含 1~2 个静止段差分，略高于 0，远小于阈值 0.02
}

TEST(ImuHealth, SeverityOrderFrozenBeatsUninitialized) {
    ImuHealthMonitor m(P());
    const double t = 0.0;
    for (int i = 0; i < 25; ++i) m.OnSample(t + i * kDt, 0.01, -0.02, kG, 0.001, -0.0005, 0.0002);
    const auto& r = m.Evaluate(t + 25 * kDt, /*inited=*/false);
    EXPECT_EQ(r.state, ImuHealth::Frozen);
}

TEST(ImuHealth, NoSampleEverBecomesTimeoutOnlyAfterInitTimeout) {
    ImuHealthMonitor m(P());
    EXPECT_EQ(m.Evaluate(100.0, false).state, ImuHealth::Uninitialized);  // 刚启动，等首帧
    EXPECT_EQ(m.Evaluate(110.0, false).state, ImuHealth::Uninitialized);  // 驱动晚起 10 s 仍属正常
    const auto& r = m.Evaluate(131.0, false);                             // 超过 init_timeout 30 s
    EXPECT_EQ(r.state, ImuHealth::Timeout);
    EXPECT_NE(r.reason.find("since start"), std::string::npos);
}

TEST(ImuHealth, DegradedCountOnlyCountsOkToFaultTransitions) {
    ImuHealthMonitor m(P());
    double t = FeedStatic(m, 0.0, 50);
    m.Evaluate(t, true);  // Uninitialized(初态) → Ok，不计数
    EXPECT_EQ(m.report().degraded_count, 0u);
    m.Evaluate(t + 0.5, true);  // Ok → Timeout
    EXPECT_EQ(m.report().degraded_count, 1u);
    m.Evaluate(t + 1.0, true);  // Timeout 持续，不重复计数
    EXPECT_EQ(m.report().degraded_count, 1u);
    t = FeedStatic(m, t + 1.0, 30);
    m.Evaluate(t, true);  // 恢复
    EXPECT_EQ(m.report().state, ImuHealth::Ok);
    m.Evaluate(t + 0.5, true);  // 再次超时
    EXPECT_EQ(m.report().degraded_count, 2u);
}

}  // namespace
}  // namespace chassis_node
