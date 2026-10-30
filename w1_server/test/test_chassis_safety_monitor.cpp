// chassis_safety_monitor 单元测试：超速 / 反馈失效 / 驱动器故障的确认与锁存、人工复位、复位后放行门。
// 差速输入经 MakeDiffSafetyInput 从共享内存反馈换算，换算本身一并覆盖。

#include <gtest/gtest.h>

#include <cmath>

#include "chassis_kinematics.h"
#include "chassis_safety_monitor.h"

namespace chassis_node {
namespace {

constexpr int64_t kMs = 1'000'000;
/// 从出现到确认的样本数（每 1 ms 一个，含首尾）
constexpr int kOverspeedConfirmSamples = static_cast<int>(kChassisOverspeedConfirmNs / kMs) + 1;
constexpr int kDriverFaultConfirmSamples = static_cast<int>(kChassisDriverFaultConfirmNs / kMs) + 1;

/// 差速反馈：左右轮线速度 → 共享内存里的 rpm → 监测输入
ChassisSafetyInput WheelSpeeds(double left_mps, double right_mps) {
    const ChassisKinematics kin;
    ChassisFeedback fb;
    fb.left_velocity = kin.DiffSpeedToRpm(left_mps);
    fb.right_velocity = kin.DiffSpeedToRpm(right_mps);
    return MakeDiffSafetyInput(true, fb, kin);
}

ChassisSafetyInput InvalidFeedback() { return MakeDiffSafetyInput(false, ChassisFeedback{}, ChassisKinematics{}); }

/// 从 start_ms 起每 1 ms 喂一次，共 count 次；返回下一个时刻（ms）
int64_t Feed(ChassisSafetyMonitor& m, const ChassisSafetyInput& in, int64_t start_ms, int count) {
    for (int i = 0; i < count; ++i) m.Update(in, (start_ms + i) * kMs);
    return start_ms + count;
}

// ============================================================ 差速反馈换算

TEST(ChassisSafetyInput, DiffFeedbackConvertsToWheelSpeedAndYawRate) {
    const ChassisSafetyInput in = WheelSpeeds(-0.3, 0.3);
    EXPECT_TRUE(in.feedback_valid);
    EXPECT_NEAR(in.wheel_speed_mps[0], -0.3, 1e-9);
    EXPECT_NEAR(in.wheel_speed_mps[1], 0.3, 1e-9);
    EXPECT_NEAR(in.wz_rps, 0.6 / ChassisGeometry{}.diff_track, 1e-9) << "右轮快为正向（逆时针）转动";
    EXPECT_FALSE(in.driver_fault);
}

TEST(ChassisSafetyInput, DiffLinearSpeedIsWheelAverage) {
    EXPECT_NEAR(WheelSpeeds(0.4, 0.6).linear_speed_mps, 0.5, 1e-9);
    EXPECT_NEAR(WheelSpeeds(-0.4, -0.6).linear_speed_mps, 0.5, 1e-9) << "取绝对值，倒车同样受限";
}

TEST(ChassisSafetyInput, AnyDiffErrorCodeIsDriverFault) {
    const ChassisKinematics kin;
    ChassisFeedback fb;
    fb.error_code_left = 0x0001;
    EXPECT_TRUE(MakeDiffSafetyInput(true, fb, kin).driver_fault);
    fb.error_code_left = 0;
    fb.error_code_right = 0x0010;
    EXPECT_TRUE(MakeDiffSafetyInput(true, fb, kin).driver_fault);
}

// ============================================================ 判定与锁存

TEST(ChassisSafetyMonitor, NormalSpeedNeverLatches) {
    ChassisSafetyMonitor m;
    Feed(m, WheelSpeeds(0.7, 0.7), 0, 1000);
    EXPECT_FALSE(m.latched());
    EXPECT_EQ(m.fault(), ChassisSafetyFault::None);
}

TEST(ChassisSafetyMonitor, LinearOverspeedLatchesAfterConfirmTime) {
    ChassisSafetyMonitor m;
    Feed(m, WheelSpeeds(0.9, 0.9), 0, kOverspeedConfirmSamples);  // 直行 0.9 m/s，持续到确认时间
    EXPECT_TRUE(m.latched());
    EXPECT_EQ(m.fault(), ChassisSafetyFault::LinearOverspeed);
}

TEST(ChassisSafetyMonitor, WheelOnlyOverspeedLatchesAsWheel) {
    // 右 1.0 / 左 0.6：v = 0.8、ω ≈ 0.91 都在 10% 余量内，只有单轮超
    ChassisSafetyMonitor m;
    Feed(m, WheelSpeeds(0.6, 1.0), 0, kOverspeedConfirmSamples);
    EXPECT_TRUE(m.latched());
    EXPECT_EQ(m.fault(), ChassisSafetyFault::WheelOverspeed);
}

TEST(ChassisSafetyMonitor, OverspeedConfirmTimeIs100ms) {
    // 100 ms（约 10 个 10 ms 反馈周期）：滤掉加减速瞬态与单帧毛刺
    ChassisSafetyMonitor m;
    const int64_t t = Feed(m, WheelSpeeds(0.9, 0.9), 0, 100);  // 0～99 ms
    EXPECT_FALSE(m.latched());
    Feed(m, WheelSpeeds(0.9, 0.9), t, 1);  // 第 100 ms
    EXPECT_TRUE(m.latched());
}

TEST(ChassisSafetyMonitor, DriverFaultConfirmTimeIs20ms) {
    // 驱动器自己报的故障不存在加减速瞬态问题，确认时间更短
    ChassisSafetyMonitor m;
    const ChassisKinematics kin;
    ChassisFeedback fb;
    fb.error_code_left = 0x0001;
    const int64_t t = Feed(m, MakeDiffSafetyInput(true, fb, kin), 0, 20);  // 0～19 ms
    EXPECT_FALSE(m.latched());
    Feed(m, MakeDiffSafetyInput(true, fb, kin), t, 1);  // 第 20 ms
    EXPECT_TRUE(m.latched());
    EXPECT_EQ(m.fault(), ChassisSafetyFault::DriverFault);
}

TEST(ChassisSafetyMonitor, ShortOverspeedDoesNotLatch) {
    ChassisSafetyMonitor m;
    const int64_t t = Feed(m, WheelSpeeds(0.9, 0.9), 0, 90);
    Feed(m, WheelSpeeds(0.5, 0.5), t, 100);
    EXPECT_FALSE(m.latched());
}

TEST(ChassisSafetyMonitor, SpeedInsideMarginDoesNotLatch) {
    ChassisSafetyMonitor m;
    const double inside = kChassisSafeWheelSpeedMps * kChassisOverspeedMargin - 0.01;
    Feed(m, WheelSpeeds(inside, inside), 0, 200);
    EXPECT_FALSE(m.latched());
}

TEST(ChassisSafetyMonitor, AngularOverspeedLatches) {
    // 两轮 ±0.3 m/s 都不超单轮上限，但 ω = 0.6 / 0.439 ≈ 1.37 rad/s 超角速度上限
    ChassisSafetyMonitor m;
    Feed(m, WheelSpeeds(-0.3, 0.3), 0, kOverspeedConfirmSamples);
    EXPECT_TRUE(m.latched());
    EXPECT_EQ(m.fault(), ChassisSafetyFault::AngularOverspeed);
}

TEST(ChassisSafetyMonitor, PeakOfAllWheelsIsChecked) {
    // 输入与底盘类型无关：任一轮超限即超速（全向四轮接入时同样适用）
    ChassisSafetyMonitor m;
    ChassisSafetyInput in;
    in.feedback_valid = true;
    in.wheel_speed_mps = {0.2, 0.2, 0.2, -0.9};
    Feed(m, in, 0, kOverspeedConfirmSamples);
    EXPECT_TRUE(m.latched());
    EXPECT_EQ(m.fault(), ChassisSafetyFault::WheelOverspeed);
}

TEST(ChassisSafetyMonitor, BodyLinearSpeedIsCheckedIndependently) {
    // 与底盘类型无关：车体线速度超限即报，不依赖各轮（全向斜行时轮速可小于车体速度）
    ChassisSafetyMonitor m;
    ChassisSafetyInput in;
    in.feedback_valid = true;
    in.linear_speed_mps = 0.9;
    in.wheel_speed_mps = {0.5, 0.5, 0.5, 0.5};
    Feed(m, in, 0, kOverspeedConfirmSamples);
    EXPECT_TRUE(m.latched());
    EXPECT_EQ(m.fault(), ChassisSafetyFault::LinearOverspeed);
}

TEST(ChassisSafetyMonitor, FeedbackLostAfterValidLatchesAt100ms) {
    ChassisSafetyMonitor m;
    int64_t t = Feed(m, WheelSpeeds(0.2, 0.2), 0, 10);
    t = Feed(m, InvalidFeedback(), t, 100);  // 失效持续 0～99 ms
    EXPECT_FALSE(m.latched());
    Feed(m, InvalidFeedback(), t, 1);  // 第 100 ms
    EXPECT_TRUE(m.latched());
    EXPECT_EQ(m.fault(), ChassisSafetyFault::FeedbackLost);
}

TEST(ChassisSafetyMonitor, NoFeedbackSinceStartupDoesNotLatch) {
    ChassisSafetyMonitor m;
    Feed(m, InvalidFeedback(), 0, 1000);
    EXPECT_FALSE(m.latched());
}

TEST(ChassisSafetyMonitor, NonFiniteFeedbackCountsAsLost) {
    ChassisSafetyMonitor m;
    const int64_t t = Feed(m, WheelSpeeds(0.2, 0.2), 0, 10);
    const ChassisKinematics kin;
    ChassisFeedback bad;
    bad.left_velocity = std::nan("");
    bad.right_velocity = kin.DiffSpeedToRpm(0.2);
    Feed(m, MakeDiffSafetyInput(true, bad, kin), t, 101);
    EXPECT_TRUE(m.latched());
    EXPECT_EQ(m.fault(), ChassisSafetyFault::FeedbackLost);
}

TEST(ChassisSafetyMonitor, DriverFaultLatchesAfterConfirmTime) {
    ChassisSafetyMonitor m;
    const ChassisKinematics kin;
    ChassisFeedback fb;
    fb.error_code_right = 0x0010;
    Feed(m, MakeDiffSafetyInput(true, fb, kin), 0, kDriverFaultConfirmSamples);
    EXPECT_TRUE(m.latched());
    EXPECT_EQ(m.fault(), ChassisSafetyFault::DriverFault);
}

TEST(ChassisSafetyMonitor, LatchHoldsAfterConditionClears) {
    ChassisSafetyMonitor m;
    const int64_t t = Feed(m, WheelSpeeds(0.9, 0.9), 0, kOverspeedConfirmSamples);
    Feed(m, WheelSpeeds(0.0, 0.0), t, 1000);
    EXPECT_TRUE(m.latched()) << "不自动恢复";
    EXPECT_EQ(m.fault(), ChassisSafetyFault::LinearOverspeed);
}

// ============================================================ 人工复位

TEST(ChassisSafetyMonitor, ResetRejectedWhileConditionPresent) {
    ChassisSafetyMonitor m;
    Feed(m, WheelSpeeds(0.9, 0.9), 0, kOverspeedConfirmSamples);
    EXPECT_TRUE(m.fault_present());
    EXPECT_FALSE(m.TryReset());
    EXPECT_TRUE(m.latched());
}

TEST(ChassisSafetyMonitor, ResetClearsLatchOnceConditionGone) {
    ChassisSafetyMonitor m;
    const int64_t t = Feed(m, WheelSpeeds(0.9, 0.9), 0, kOverspeedConfirmSamples);
    Feed(m, WheelSpeeds(0.0, 0.0), t, 1);
    EXPECT_FALSE(m.fault_present());
    EXPECT_TRUE(m.TryReset());
    EXPECT_FALSE(m.latched());
    EXPECT_EQ(m.fault(), ChassisSafetyFault::None);
}

TEST(ChassisSafetyMonitor, ResetRejectedWhileFeedbackLost) {
    ChassisSafetyMonitor m;
    const int64_t t = Feed(m, WheelSpeeds(0.2, 0.2), 0, 10);
    Feed(m, InvalidFeedback(), t, 101);
    EXPECT_FALSE(m.TryReset()) << "反馈未恢复前不能复位";
}

// ============================================================ 复位后放行门

/// 超速锁存 → 异常消失 → 复位成功，返回处于"复位后待零速"状态的监测器
ChassisSafetyMonitor ResetAfterOverspeed() {
    ChassisSafetyMonitor m;
    const int64_t t = Feed(m, WheelSpeeds(0.9, 0.9), 0, kOverspeedConfirmSamples);
    Feed(m, WheelSpeeds(0.0, 0.0), t, 1);
    EXPECT_TRUE(m.TryReset());
    return m;
}

TEST(ChassisSafetyMonitor, NoRestartHoldWithoutReset) {
    ChassisSafetyMonitor m;
    EXPECT_FALSE(m.HoldAfterReset(true, 0.5, 0.5));
    EXPECT_FALSE(m.awaiting_zero_command());
}

TEST(ChassisSafetyMonitor, NonZeroCommandAfterResetIsHeld) {
    ChassisSafetyMonitor m = ResetAfterOverspeed();
    EXPECT_TRUE(m.awaiting_zero_command());
    EXPECT_TRUE(m.HoldAfterReset(true, 0.3, 0.0));
    EXPECT_TRUE(m.HoldAfterReset(true, 0.0, 0.5)) << "非零指令持续多久都不放行";
    EXPECT_TRUE(m.awaiting_zero_command());
}

TEST(ChassisSafetyMonitor, ZeroCommandAfterResetReleases) {
    ChassisSafetyMonitor m = ResetAfterOverspeed();
    EXPECT_FALSE(m.HoldAfterReset(true, 0.0, 0.0));
    EXPECT_FALSE(m.awaiting_zero_command());
    EXPECT_FALSE(m.HoldAfterReset(true, 0.3, 0.0)) << "放行后正常运行";
}

TEST(ChassisSafetyMonitor, CommandStreamStoppedAfterResetReleases) {
    // 任务层停止任务 → 指令停发超时 → 放行，下一个任务的新指令正常执行
    ChassisSafetyMonitor m = ResetAfterOverspeed();
    EXPECT_FALSE(m.HoldAfterReset(false, 0.3, 0.0));
    EXPECT_FALSE(m.HoldAfterReset(true, 0.3, 0.0));
}

TEST(ChassisSafetyMonitor, NonFiniteCommandAfterResetIsHeld) {
    ChassisSafetyMonitor m = ResetAfterOverspeed();
    EXPECT_TRUE(m.HoldAfterReset(true, std::nan(""), 0.0));
}


// ============================================================ 上半身异常与一起清错

ChassisSafetyInput StillWithUpperBodyFault() {
    ChassisSafetyInput in = WheelSpeeds(0.0, 0.0);
    in.upper_body_fault = true;
    return in;
}

ChassisSafetyInput StillWithDriverFault() {
    const ChassisKinematics kin;
    ChassisFeedback fb;
    fb.error_code_left = 0x0400;
    return MakeDiffSafetyInput(true, fb, kin);
}

/// 喂到锁存为止，返回下一个时刻（ms）
int64_t LatchWith(ChassisSafetyMonitor& m, const ChassisSafetyInput& in) {
    const int64_t t = Feed(m, in, 0, kDriverFaultConfirmSamples);
    EXPECT_TRUE(m.latched());
    return t;
}

TEST(ChassisSafetyMonitor, UpperBodyFaultName) {
    EXPECT_STREQ(ChassisSafetyFaultName(ChassisSafetyFault::UpperBodyFault), "upper_body_fault");
}

TEST(ChassisSafetyMonitor, UpperBodyFaultLatchesAfterConfirm) {
    ChassisSafetyMonitor m;
    Feed(m, StillWithUpperBodyFault(), 0, kDriverFaultConfirmSamples - 1);
    EXPECT_FALSE(m.latched()) << "与驱动器故障同样滤 20 ms";
    Feed(m, StillWithUpperBodyFault(), kDriverFaultConfirmSamples - 1, 1);
    EXPECT_TRUE(m.latched());
    EXPECT_EQ(m.fault(), ChassisSafetyFault::UpperBodyFault);
}

TEST(ChassisSafetyMonitor, UpperBodyFaultBlocksReset) {
    ChassisSafetyMonitor m;
    LatchWith(m, StillWithUpperBodyFault());
    EXPECT_TRUE(m.fault_present());
    EXPECT_EQ(m.present_fault(), ChassisSafetyFault::UpperBodyFault);
    EXPECT_FALSE(m.TryReset());
}

TEST(ChassisSafetyMonitor, ResetWindowIs2s) { EXPECT_EQ(kChassisResetWindowNs, 2000 * kMs); }

TEST(ChassisSafetyMonitor, RequestResetIgnoredWhenNotLatched) {
    ChassisSafetyMonitor m;
    const int64_t t = Feed(m, WheelSpeeds(0.0, 0.0), 0, 10);
    m.RequestReset(t * kMs);
    EXPECT_FALSE(m.reset_pending());
    Feed(m, WheelSpeeds(0.0, 0.0), t, 1);
    EXPECT_EQ(m.TakeResetResult(), ChassisResetResult::None);
}

TEST(ChassisSafetyMonitor, RequestResetWaitsUntilUpperBodyCleared) {
    ChassisSafetyMonitor m;
    int64_t t = LatchWith(m, StillWithUpperBodyFault());
    m.RequestReset(t * kMs);
    EXPECT_TRUE(m.reset_pending());
    t = Feed(m, StillWithUpperBodyFault(), t, 300);  // 上半身还在清错
    EXPECT_TRUE(m.latched());
    EXPECT_EQ(m.TakeResetResult(), ChassisResetResult::None);
    Feed(m, WheelSpeeds(0.0, 0.0), t, 1);
    EXPECT_FALSE(m.latched()) << "上半身清完即一起解除，不用再按一次";
    EXPECT_TRUE(m.awaiting_zero_command()) << "解除后仍先零速才放行";
    EXPECT_FALSE(m.reset_pending());
    EXPECT_EQ(m.TakeResetResult(), ChassisResetResult::Accepted);
    EXPECT_EQ(m.TakeResetResult(), ChassisResetResult::None) << "结果只报一次";
}

TEST(ChassisSafetyMonitor, RequestResetWaitsUntilDriverReenabled) {
    ChassisSafetyMonitor m;
    int64_t t = LatchWith(m, StillWithDriverFault());
    m.RequestReset(t * kMs);
    t = Feed(m, StillWithDriverFault(), t, 900);  // 驱动器复位后约 1 s 内重新使能、错误码归零
    EXPECT_TRUE(m.latched());
    Feed(m, WheelSpeeds(0.0, 0.0), t, 1);
    EXPECT_FALSE(m.latched());
    EXPECT_EQ(m.TakeResetResult(), ChassisResetResult::Accepted);
}

TEST(ChassisSafetyMonitor, RequestResetRejectedAfterWindow) {
    ChassisSafetyMonitor m;
    int64_t t = LatchWith(m, StillWithDriverFault());
    m.RequestReset(t * kMs);
    t = Feed(m, StillWithDriverFault(), t, 2000);
    EXPECT_TRUE(m.reset_pending()) << "窗口内一直等";
    EXPECT_EQ(m.TakeResetResult(), ChassisResetResult::None);
    Feed(m, StillWithDriverFault(), t, 1);
    EXPECT_FALSE(m.reset_pending());
    EXPECT_TRUE(m.latched());
    EXPECT_EQ(m.TakeResetResult(), ChassisResetResult::Rejected);
    EXPECT_EQ(m.present_fault(), ChassisSafetyFault::DriverFault) << "拒绝原因取当前仍在的异常";
}

TEST(ChassisSafetyMonitor, PresentFaultIsNoneWhenClear) {
    ChassisSafetyMonitor m;
    Feed(m, WheelSpeeds(0.3, 0.3), 0, 5);
    EXPECT_EQ(m.present_fault(), ChassisSafetyFault::None);
}

}  // namespace
}  // namespace chassis_node
