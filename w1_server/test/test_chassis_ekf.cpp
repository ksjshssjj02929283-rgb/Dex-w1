// chassis_ekf 单元测试
//
// 覆盖 M4 的验收标志与三条设计决策：
//   DEC-005  过程噪声 Q 改为「每秒」量纲并随 dt 缩放；q_theta 按陀螺零偏残差量级取值
//   DEC-006  静止时以零速量测（ZUPT）而非硬置零，保持状态与协方差一致
//   DEC-007  「打滑检测」与「状态更新」分离：检测始终用 2 维新息，更新按模式选行
//
// chassis_ekf 只依赖 Eigen，不牵扯 ROS 运行时，故可脱离节点单独测。

#include "chassis_ekf.h"

#include <gtest/gtest.h>

#include <cmath>

namespace chassis_node {
namespace {

constexpr double kRad2Deg = 57.29577951308232;
constexpr double kDt = 0.01;          // 100 Hz
constexpr double kGyroBiasResid = 1e-4;  // rad/s，M2 实测的初始化后零偏残差量级

ChassisEkf::Params DefaultParams() { return ChassisEkf::Params{}; }

/// 静止：编码器 v=ω=0，IMU 只剩零偏残差
void RunStatic(ChassisEkf& ekf, double seconds, double dt = kDt,
               double gyro_bias_resid = kGyroBiasResid) {
    const int n = static_cast<int>(seconds / dt);
    for (int i = 0; i < n; ++i) {
        ekf.Step(dt, 0.0, 0.0, gyro_bias_resid, /*imu_available=*/true, /*is_stopped=*/true);
    }
}

/// 直行：编码器与 IMU 一致
void RunStraight(ChassisEkf& ekf, double seconds, double v = 0.5, double dt = kDt) {
    const int n = static_cast<int>(seconds / dt);
    for (int i = 0; i < n; ++i) {
        ekf.Step(dt, v, 0.0, 0.0, /*imu_available=*/true, /*is_stopped=*/false);
    }
}

double MaxAsymmetry(const ChassisEkf::Mat5& P) {
    return (P - P.transpose()).cwiseAbs().maxCoeff();
}

double MinEigenvalue(const ChassisEkf::Mat5& P) {
    Eigen::SelfAdjointEigenSolver<ChassisEkf::Mat5> es(P);
    return es.eigenvalues().minCoeff();
}

// ============================================================ 基础行为

TEST(ChassisEkfBasic, ResetGivesZeroStateAndInitialCovariance) {
    const auto p = DefaultParams();
    ChassisEkf ekf(p);

    EXPECT_DOUBLE_EQ(ekf.GetX(), 0.0);
    EXPECT_DOUBLE_EQ(ekf.GetY(), 0.0);
    EXPECT_DOUBLE_EQ(ekf.GetTheta(), 0.0);
    EXPECT_DOUBLE_EQ(ekf.GetV(), 0.0);
    EXPECT_DOUBLE_EQ(ekf.GetOmega(), 0.0);
    EXPECT_FALSE(ekf.IsAerial());

    const auto& P = ekf.GetCovariance();
    EXPECT_DOUBLE_EQ(P(0, 0), p.p0_xy);
    EXPECT_DOUBLE_EQ(P(2, 2), p.p0_theta);
    EXPECT_DOUBLE_EQ(P(4, 4), p.p0_omega);
}

TEST(ChassisEkfBasic, StraightLineIntegratesDistance) {
    ChassisEkf ekf(DefaultParams());
    constexpr double kV = 0.5;
    RunStraight(ekf, 10.0, kV);

    // 速度收敛到编码器读数；位移接近 v*t（起步阶段 v 从 0 收敛，故留 5% 容差）
    EXPECT_NEAR(ekf.GetV(), kV, 1e-3);
    EXPECT_NEAR(ekf.GetX(), kV * 10.0, kV * 10.0 * 0.05);
    EXPECT_NEAR(ekf.GetY(), 0.0, 1e-6);
    EXPECT_NEAR(ekf.GetTheta(), 0.0, 1e-6);
}

TEST(ChassisEkfBasic, HeadingIsNormalizedToPi) {
    ChassisEkf ekf(DefaultParams());
    // 持续转动足够长时间，跨越 ±π 若干次
    for (int i = 0; i < 5000; ++i) {
        ekf.Step(kDt, 0.0, 2.0, 2.0, true, false);
    }
    EXPECT_LE(ekf.GetTheta(), M_PI);
    EXPECT_GE(ekf.GetTheta(), -M_PI);
}

// ============================================================ 协方差性质（M4 验收）

TEST(ChassisEkfCovariance, StaysSymmetricAndPositiveDefiniteOverLongRun) {
    ChassisEkf ekf(DefaultParams());
    // 行驶与静止交替 100 轮，共 200 s
    for (int i = 0; i < 100; ++i) {
        RunStraight(ekf, 1.0);
        RunStatic(ekf, 1.0);
    }
    const auto& P = ekf.GetCovariance();
    EXPECT_LT(MaxAsymmetry(P), 1e-12) << "Joseph 形式应保持 P 对称";
    EXPECT_GT(MinEigenvalue(P), 0.0) << "P 必须保持正定";
}

// ============================================================ DEC-005：Q 量纲与量级

TEST(ChassisEkfProcessNoise, YawCovarianceIsFrequencyIndependent) {
    // θ 无量测、纯积分 ⇒ P(2,2) = P0 + q_theta·T 只与物理时长有关。
    // 若 Q 按「每步」定义（未乘 dt），此处会随频率线性变化。
    double sigma[3];
    const double rates[3] = {50.0, 100.0, 200.0};
    for (int i = 0; i < 3; ++i) {
        ChassisEkf ekf(DefaultParams());
        RunStraight(ekf, 60.0, 0.5, 1.0 / rates[i]);
        sigma[i] = std::sqrt(ekf.GetCovariance()(2, 2));
    }
    EXPECT_NEAR(sigma[0], sigma[1], 1e-9);
    EXPECT_NEAR(sigma[1], sigma[2], 1e-9);
}

TEST(ChassisEkfProcessNoise, VelocityCovarianceImprovesWithHigherRate) {
    // v 有量测：采样率越高信息越多，稳态方差应越小。
    // 若 Q 不随 dt 缩放，三个频率会得到完全相同的值（反而是异常）。
    ChassisEkf slow(DefaultParams());
    ChassisEkf fast(DefaultParams());
    RunStraight(slow, 60.0, 0.5, 1.0 / 50.0);
    RunStraight(fast, 60.0, 0.5, 1.0 / 200.0);
    EXPECT_LT(fast.GetCovariance()(3, 3), slow.GetCovariance()(3, 3));
}

TEST(ChassisEkfProcessNoise, YawUncertaintyStaysMeaningfulAfterLongRun) {
    // q_theta 量级须由陀螺零偏残差决定。取 1e-3 rad²/step（旧值）时
    // 100 s 后 σ_yaw 将达 180° 量级，协方差彻底失去意义。
    ChassisEkf ekf(DefaultParams());
    RunStatic(ekf, 100.0);
    const double sigma_deg = std::sqrt(ekf.GetCovariance()(2, 2)) * kRad2Deg;
    EXPECT_LT(sigma_deg, 10.0) << "静止 100 s 后航向不确定度不应达到十度量级";
}

// ============================================================ DEC-006：ZUPT

TEST(ChassisEkfZupt, HeadingDoesNotDriftWhileStopped) {
    // 不加 ZUPT 时，IMU 零偏残差会被逐帧积进 θ 且永不修正
    //（1e-4 rad/s 持续 1 h 约 20°）。
    ChassisEkf ekf(DefaultParams());
    RunStatic(ekf, 3600.0);
    EXPECT_NEAR(ekf.GetTheta() * kRad2Deg, 0.0, 1e-9);
}

TEST(ChassisEkfZupt, VelocityStatesCollapseToZeroWhileStopped) {
    ChassisEkf ekf(DefaultParams());
    RunStraight(ekf, 5.0, 0.8);   // 先跑起来
    ASSERT_GT(ekf.GetV(), 0.5);
    RunStatic(ekf, 2.0);          // 再停下
    EXPECT_NEAR(ekf.GetV(), 0.0, 1e-6);
    EXPECT_NEAR(ekf.GetOmega(), 0.0, 1e-6);
}

TEST(ChassisEkfZupt, PositionCovarianceGrowsSlowlyWhileStopped) {
    // 静止模型近乎完美 ⇒ q_scale_stopped 抑制过程噪声，
    // 位置协方差不应在长时间停车中一路膨胀。
    ChassisEkf ekf(DefaultParams());
    RunStatic(ekf, 3600.0);
    const double sigma_pos = std::sqrt(ekf.GetCovariance()(0, 0));
    EXPECT_LT(sigma_pos, 0.5) << "静止 1 h 的位置不确定度不应达到米级";
}

TEST(ChassisEkfZupt, NoAerialFlagWhileStopped) {
    // 静止时编码器与 IMU 都指向「没在动」，一致性判定无意义，不应触发。
    ChassisEkf ekf(DefaultParams());
    const bool aerial = ekf.Step(kDt, 0.0, 0.0, kGyroBiasResid, true, /*is_stopped=*/true);
    EXPECT_FALSE(aerial);
    EXPECT_FALSE(ekf.IsAerial());
}

// ============================================================ DEC-007：检测/更新分离

TEST(ChassisEkfDetectUpdateSplit, ImuOmegaIsNotDraggedByEncoder) {
    // IMU 模式下 ω 由 IMU 直接给定；用编码器 ω 去「修正」它会把高带宽低噪声的
    // IMU 往编码器方向拉（原实现 K≈0.5，输出成了两者对半折中）。
    constexpr double kOmegaTrue = 0.5;
    for (double omega_enc : {0.5, 0.7, 1.0, 1.5}) {
        ChassisEkf ekf(DefaultParams());
        for (int i = 0; i < 300; ++i) {
            ekf.Step(kDt, 0.5, omega_enc, kOmegaTrue, /*imu_available=*/true, false);
        }
        EXPECT_NEAR(ekf.GetOmega(), kOmegaTrue, 1e-6)
            << "omega_enc = " << omega_enc << " 时输出被编码器拉偏";
    }
}

TEST(ChassisEkfDetectUpdateSplit, PublishedOmegaEqualsHeadingDerivative) {
    // θ 由 Predict 用纯 IMU 的 ω 积分。若更新步用编码器 ω 改写状态 ω，
    // 发布的角速度就不再是发布航向的导数，上层做微分一致性检查会发现矛盾。
    constexpr double kOmegaTrue = 0.5;
    ChassisEkf ekf(DefaultParams());
    for (int i = 0; i < 100; ++i) {
        ekf.Step(kDt, 0.5, 0.7, kOmegaTrue, true, false);
    }
    const double th0 = ekf.GetTheta();
    for (int i = 0; i < 200; ++i) {
        ekf.Step(kDt, 0.5, 0.7, kOmegaTrue, true, false);
    }
    const double dtheta_dt = (ekf.GetTheta() - th0) / 2.0;
    EXPECT_NEAR(dtheta_dt, ekf.GetOmega(), 1e-6);
}

TEST(ChassisEkfDetectUpdateSplit, SlipDetectionSurvivesTheSplit) {
    // 关键回归点：若为了不让编码器修正 ω 而直接把 H 降为 1 维，
    // Δω 不再进入新息，转向打滑这条唯一有效的检测通道就没了。
    constexpr double kOmegaTrue = 0.5;
    auto run = [](double omega_enc) {
        ChassisEkf ekf(DefaultParams());
        bool aerial = false;
        for (int i = 0; i < 300; ++i) {
            aerial = ekf.Step(kDt, 0.5, omega_enc, kOmegaTrue, true, false);
        }
        return aerial;
    };
    EXPECT_FALSE(run(0.50)) << "无分歧不应误报";
    EXPECT_TRUE(run(1.00)) << "Δω=0.5 rad/s 应被检出";
}

TEST(ChassisEkfDetectUpdateSplit, DetectionThresholdMatchesChiSquare) {
    // 最小可检 Δω = √(χ²·(P44 + r_omega))。该门限决定 M6 的灵敏度上限，
    // 若 P44 因 q_omega_imu 取值不当而偏大，门限会显著抬高。
    const auto p = DefaultParams();
    const double p44 = p.q_omega_imu * kDt;
    const double dw_min = std::sqrt(p.aerial_chi2_threshold * (p44 + p.r_omega));
    EXPECT_NEAR(dw_min, 0.303, 0.01);

    constexpr double kOmegaTrue = 0.5;
    auto detect = [&](double delta) {
        ChassisEkf ekf(DefaultParams());
        bool aerial = false;
        for (int i = 0; i < 300; ++i) {
            aerial = ekf.Step(kDt, 0.5, kOmegaTrue + delta, kOmegaTrue, true, false);
        }
        return aerial;
    };
    EXPECT_FALSE(detect(dw_min * 0.6)) << "门限以下不应触发";
    EXPECT_TRUE(detect(dw_min * 1.5)) << "门限以上应触发";
}

TEST(ChassisEkfDetectUpdateSplit, EncoderOnlyModeUsesEncoderOmega) {
    // 无 IMU 时 ω 退化为被估计量，由编码器量测更新。
    ChassisEkf ekf(DefaultParams());
    constexpr double kOmegaEnc = 0.4;
    for (int i = 0; i < 300; ++i) {
        ekf.Step(kDt, 0.5, kOmegaEnc, 0.0, /*imu_available=*/false, false);
    }
    EXPECT_NEAR(ekf.GetOmega(), kOmegaEnc, 1e-2);
}

// ============================================================ 参数热更新（R10）

TEST(ChassisEkfParams, SetParamsDoesNotResetCovarianceOrState) {
    // p0_* 的语义是「滤波器复位时的初值」，只应在 Reset() 生效。
    // 早期实现会在 SetParams 里把 P 打回 p0_*，导致运行中调一个噪声参数
    // 就让协方差突然跳变、滤波器不连续。
    ChassisEkf ekf(DefaultParams());
    RunStraight(ekf, 10.0);

    const ChassisEkf::Mat5 P_before = ekf.GetCovariance();
    const double x_before = ekf.GetX();

    auto p = DefaultParams();
    p.r_v = 0.2;  // 改一个与 p0 无关的量测噪声
    ekf.SetParams(p);

    EXPECT_DOUBLE_EQ(ekf.GetX(), x_before);
    EXPECT_TRUE(ekf.GetCovariance().isApprox(P_before))
        << "SetParams 不应重置协方差";
}

TEST(ChassisEkfParams, ResetRestoresInitialCovariance) {
    ChassisEkf ekf(DefaultParams());
    RunStraight(ekf, 10.0);
    ASSERT_GT(ekf.GetX(), 1.0);

    ekf.Reset();
    EXPECT_DOUBLE_EQ(ekf.GetX(), 0.0);
    EXPECT_DOUBLE_EQ(ekf.GetCovariance()(0, 0), DefaultParams().p0_xy);
}

}  // namespace
}  // namespace chassis_node
