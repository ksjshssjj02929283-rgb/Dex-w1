// omni_chassis_ekf 单元测试
//
// 全向 EKF 与差速 EKF 只差 vy 一维。这里锁住三件事：
//   1. 运动学：纯侧移 / 斜向 / 侧移 + 旋转 的位置积分正确，θ 不被侧移污染
//   2. 退化性：vy ≡ 0 时 x / θ / vx 的协方差与差速 EKF 逐位一致
//   3. 继承的设计决策仍成立：Q 每秒量纲、ZUPT、检测/更新分离（3 维检测、IMU 模式只更新 vx vy）

#include <gtest/gtest.h>

#include <cmath>

#include "chassis_ekf.h"
#include "omni_chassis_ekf.h"

namespace chassis_node {
namespace {

constexpr double kRad2Deg = 57.29577951308232;
constexpr double kDt = 0.01;
constexpr double kGyroBiasResid = 1e-4;

OmniChassisEkf::Params DefaultParams() { return OmniChassisEkf::Params{}; }

void RunStatic(OmniChassisEkf& ekf, double seconds, double dt = kDt) {
    const int n = static_cast<int>(seconds / dt);
    for (int i = 0; i < n; ++i) ekf.Step(dt, 0.0, 0.0, 0.0, kGyroBiasResid, true, /*is_stopped=*/true);
}

/// 匀速运动：编码器与 IMU 一致（ω_enc = ω_imu = w）
void RunMove(OmniChassisEkf& ekf, double seconds, double vx, double vy, double w, double dt = kDt, bool imu = true) {
    const int n = static_cast<int>(seconds / dt);
    for (int i = 0; i < n; ++i) ekf.Step(dt, vx, vy, w, w, imu, false);
}

// ============================================================ 运动学

TEST(OmniEkfBasic, ResetGivesZeroStateAndInitialCovariance) {
    OmniChassisEkf ekf(DefaultParams());
    EXPECT_DOUBLE_EQ(ekf.GetX(), 0.0);
    EXPECT_DOUBLE_EQ(ekf.GetVy(), 0.0);
    const auto& P = ekf.GetCovariance();
    EXPECT_DOUBLE_EQ(P(0, 0), DefaultParams().p0_xy);
    EXPECT_DOUBLE_EQ(P(4, 4), DefaultParams().p0_vy);
    EXPECT_DOUBLE_EQ(P(5, 5), DefaultParams().p0_omega);
}

TEST(OmniEkfBasic, PureLateralMotionMovesAlongBodyYAndKeepsHeading) {
    // 差速 EKF 没有 vy，侧移只能被解释成航向误差；全向 EKF 里它是独立状态
    OmniChassisEkf ekf(DefaultParams());
    RunMove(ekf, 10.0, 0.0, 0.3, 0.0);
    EXPECT_NEAR(ekf.GetY(), 3.0, 0.05);
    EXPECT_NEAR(ekf.GetX(), 0.0, 1e-6);
    EXPECT_NEAR(ekf.GetTheta(), 0.0, 1e-9);
    EXPECT_NEAR(ekf.GetVy(), 0.3, 1e-3);
}

TEST(OmniEkfBasic, DiagonalMotionIsVectorSum) {
    OmniChassisEkf ekf(DefaultParams());
    RunMove(ekf, 10.0, 0.3, 0.4, 0.0);
    EXPECT_NEAR(ekf.GetX(), 3.0, 0.05);
    EXPECT_NEAR(ekf.GetY(), 4.0, 0.05);
}

TEST(OmniEkfBasic, LateralMotionWhileRotatingTracesCircle) {
    // 机体系恒定 (0, vy) 且以 ω 自转 ⇒ odom 系轨迹是半径 vy/ω 的圆：
    //   x(t) = (vy/ω)(cos ωt − 1)，y(t) = (vy/ω) sin ωt（起步向 +y，圆心在 −x 侧）
    // 步数取整后按实际时长 t_n 对照解析解，避免 T/dt 非整数带来的假残差。
    OmniChassisEkf ekf(DefaultParams());
    const double vy = 0.3, w = 0.5;
    const int n = static_cast<int>(2.0 * M_PI / w / kDt);
    const double tn = n * kDt;
    for (int i = 0; i < n; ++i) ekf.Step(kDt, 0.0, vy, w, w, true, false);
    const double x_exp = (vy / w) * (std::cos(w * tn) - 1.0);
    const double y_exp = (vy / w) * std::sin(w * tn);
    EXPECT_NEAR(ekf.GetX(), x_exp, 0.02) << "闭合误差应在厘米级（起步速度收敛 + 中点法截断）";
    EXPECT_NEAR(ekf.GetY(), y_exp, 0.02);
    EXPECT_NEAR(ekf.GetTheta(), std::atan2(std::sin(w * tn), std::cos(w * tn)), 1e-9);
    // 四分之一圈时应在 (−r, +r)：锁住旋转方向与侧移方向的符号
    OmniChassisEkf q(DefaultParams());
    const int nq = n / 4;
    for (int i = 0; i < nq; ++i) q.Step(kDt, 0.0, vy, w, w, true, false);
    EXPECT_NEAR(q.GetX(), (vy / w) * (std::cos(w * nq * kDt) - 1.0), 0.02);
    EXPECT_NEAR(q.GetY(), (vy / w) * std::sin(w * nq * kDt), 0.02);
}

TEST(OmniEkfBasic, ForwardMotionWhileRotatingMatchesDiffDriveCircle) {
    // vx + ω 的圆与差速 EKF 同一条：两个滤波器给出同一位姿
    ChassisEkf diff(ChassisEkf::Params{});
    OmniChassisEkf omni(DefaultParams());
    const double v = 0.5, w = 0.5, T = 2.0 * M_PI / w;
    const int n = static_cast<int>(T / kDt);
    for (int i = 0; i < n; ++i) {
        diff.Step(kDt, v, w, w, true, false);
        omni.Step(kDt, v, 0.0, w, w, true, false);
    }
    EXPECT_NEAR(omni.GetX(), diff.GetX(), 1e-9);
    EXPECT_NEAR(omni.GetY(), diff.GetY(), 1e-9);
    EXPECT_NEAR(omni.GetTheta(), diff.GetTheta(), 1e-12);
}

// ============================================================ 退化性

TEST(OmniEkf, CovarianceMatchesDiffDriveWhenNoLateralMotion) {
    // vy ≡ 0 且 θ=0 时 vy 与 x/θ/vx 解耦（F(0,4) = −sinθ·dt = 0），这三项协方差应逐位一致；
    // y 会多出 vy 不确定度的贡献（F(1,4) = cosθ·dt），故不比较 P(1,1)。
    ChassisEkf diff(ChassisEkf::Params{});
    OmniChassisEkf omni(DefaultParams());
    for (int i = 0; i < 6000; ++i) {
        diff.Step(kDt, 0.5, 0.0, 0.0, true, false);
        omni.Step(kDt, 0.5, 0.0, 0.0, 0.0, true, false);
    }
    const auto& Pd = diff.GetCovariance();
    const auto& Po = omni.GetCovariance();
    EXPECT_NEAR(Po(0, 0), Pd(0, 0), 1e-12);
    EXPECT_NEAR(Po(2, 2), Pd(2, 2), 1e-15);
    EXPECT_NEAR(Po(3, 3), Pd(3, 3), 1e-12);
    EXPECT_NEAR(Po(5, 5), Pd(4, 4), 1e-15);
    EXPECT_GT(Po(1, 1), Pd(1, 1)) << "全向的 y 多了 vy 不确定度，理应更大";
}

// ============================================================ Q 量纲 / ZUPT

TEST(OmniEkfProcessNoise, YawCovarianceIsFrequencyIndependent) {
    double sigma[3];
    const double rates[3] = {50.0, 100.0, 200.0};
    for (int i = 0; i < 3; ++i) {
        OmniChassisEkf ekf(DefaultParams());
        RunMove(ekf, 60.0, 0.3, 0.2, 0.0, 1.0 / rates[i]);
        sigma[i] = std::sqrt(ekf.GetCovariance()(2, 2));
    }
    EXPECT_NEAR(sigma[0], sigma[1], 1e-9);
    EXPECT_NEAR(sigma[1], sigma[2], 1e-9);
}

TEST(OmniEkfZupt, HeadingAndVelocitiesHoldWhileStopped) {
    OmniChassisEkf ekf(DefaultParams());
    RunMove(ekf, 2.0, 0.2, 0.2, 0.3);
    RunStatic(ekf, 3600.0);
    EXPECT_NEAR(ekf.GetVx(), 0.0, 1e-6);
    EXPECT_NEAR(ekf.GetVy(), 0.0, 1e-6);
    EXPECT_NEAR(ekf.GetOmega(), 0.0, 1e-6);
    const double theta_after = ekf.GetTheta();
    RunStatic(ekf, 3600.0);
    EXPECT_NEAR(ekf.GetTheta(), theta_after, 1e-9) << "静止一小时航向不应被零偏残差带走";
    EXPECT_FALSE(ekf.IsAerial());
}

// ============================================================ 检测 / 更新分离

TEST(OmniEkfDetectUpdateSplit, ImuOmegaIsNotDraggedByEncoder) {
    OmniChassisEkf ekf(DefaultParams());
    for (int i = 0; i < 500; ++i) ekf.Step(kDt, 0.3, 0.0, /*omega_enc=*/0.3, /*omega_imu=*/0.5, true, false);
    EXPECT_NEAR(ekf.GetOmega(), 0.5, 1e-9) << "IMU 模式下 ω 应等于 IMU，而非与编码器折中";
}

TEST(OmniEkfDetectUpdateSplit, LateralSlipIsDetectedAndNotTrusted) {
    // 侧向编码器速度突跳而预测 vy≈0：3 维检测第一步即报 aerial，且 vy 状态不被拉走。
    // 稳态 P44 ≈ 0.028（q_vy·dt 与 r_vy 的 Riccati 解），S ≈ 0.078，Δvy = 1 ⇒ D² ≈ 12.8 > 11.34。
    OmniChassisEkf ekf(DefaultParams());
    RunMove(ekf, 5.0, 0.3, 0.0, 0.0);
    EXPECT_TRUE(ekf.Step(kDt, 0.3, /*vy_enc=*/1.0, 0.0, 0.0, true, false));
    EXPECT_LT(std::abs(ekf.GetVy()), 0.01) << "r_vy_aerial 放大后编码器侧向速度应几乎不进状态";

    // 持续大幅侧滑（3 m/s）：连续 5 步都判 aerial，vy 仍不被拉走。
    // 注意语义与差速一致：aerial 期 R 放大、P 只增不减，**中等**分歧（如 1 m/s）几十毫秒后会跌到阈值内
    // 被接受——"编码器长期自洽 ⇒ 不再视为打滑"是设计，不是缺陷；本用例只锁大幅分歧。
    OmniChassisEkf big(DefaultParams());
    RunMove(big, 5.0, 0.3, 0.0, 0.0);
    for (int i = 0; i < 5; ++i) {
        EXPECT_TRUE(big.Step(kDt, 0.3, /*vy_enc=*/3.0, 0.0, 0.0, true, false)) << "step " << i;
    }
    EXPECT_LT(std::abs(big.GetVy()), 0.05);
}

TEST(OmniEkfDetectUpdateSplit, EncoderOnlyModeUsesEncoderOmega) {
    OmniChassisEkf ekf(DefaultParams());
    RunMove(ekf, 5.0, 0.0, 0.0, 0.4, kDt, /*imu=*/false);
    EXPECT_NEAR(ekf.GetOmega(), 0.4, 1e-3);
    EXPECT_NEAR(ekf.GetTheta(), 2.0, 0.05) << "无 IMU 时 θ 由编码器 ω 积分（含起步收敛）";
}

TEST(OmniEkfDetectUpdateSplit, NoAerialFlagWhileStopped) {
    OmniChassisEkf ekf(DefaultParams());
    for (int i = 0; i < 100; ++i) {
        EXPECT_FALSE(ekf.Step(kDt, 0.0, 0.0, 0.0, 0.3, true, /*is_stopped=*/true));
    }
}

// ============================================================ SetState / SetParams

TEST(OmniEkfSetState, SeedsStateAndNoPoseJumpOnNextStep) {
    OmniChassisEkf ekf(DefaultParams());
    ekf.SetState(1.0, 2.0, 0.5, 0.3, -0.1, 0.2);
    EXPECT_DOUBLE_EQ(ekf.GetVy(), -0.1);
    EXPECT_DOUBLE_EQ(ekf.GetCovariance()(0, 0), DefaultParams().p0_xy);
    ekf.Step(kDt, 0.3, -0.1, 0.2, 0.2, true, false);
    EXPECT_NEAR(ekf.GetX(), 1.0 + (0.3 * std::cos(0.501) + 0.1 * std::sin(0.501)) * kDt, 1e-6);
    EXPECT_NEAR(ekf.GetTheta(), 0.502, 1e-9);
}

TEST(OmniEkfParams, SetParamsDoesNotResetState) {
    OmniChassisEkf ekf(DefaultParams());
    RunMove(ekf, 3.0, 0.2, 0.1, 0.1);
    const double x = ekf.GetX(), p = ekf.GetCovariance()(0, 0);
    auto q = DefaultParams();
    q.r_vy = 0.5;
    ekf.SetParams(q);
    EXPECT_DOUBLE_EQ(ekf.GetX(), x);
    EXPECT_DOUBLE_EQ(ekf.GetCovariance()(0, 0), p);
}

TEST(DeadReckoningCovOmni, SeedFromSixDimCovariance) {
    OmniChassisEkf ekf(DefaultParams());
    RunMove(ekf, 10.0, 0.2, 0.2, 0.1);
    DeadReckoningCov dr;
    dr.SeedFrom(ekf.GetCovariance());
    const auto& P = ekf.GetCovariance();
    EXPECT_DOUBLE_EQ(dr.var_theta, P(2, 2));
    EXPECT_DOUBLE_EQ(dr.var_xy, std::max(P(0, 0), P(1, 1)));
    EXPECT_DOUBLE_EQ(dr.TwistVarVy(DefaultParams()), DefaultParams().r_vy);
}

}  // namespace
}  // namespace chassis_node
