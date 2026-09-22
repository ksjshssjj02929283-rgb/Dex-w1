// chassis_kinematics / chassis_command 单元测试
//
// 最重要的一组是 LegacyEquivalence：把切分前 `ChassisVelocityControlCallback` 的实现原样复刻，
// 与新的纯函数在随机输入上逐点比对。本次切分的承诺是"只搬位置不改行为"，这组测试就是证据。

#include <gtest/gtest.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <type_traits>

#include "chassis_command.h"
#include "chassis_kinematics.h"

namespace chassis_node {
namespace {

constexpr int N = ChassisKinematics::kOmniWheels;
constexpr double kL = 0.460;
constexpr double kW = 0.345;
constexpr double kD = 0.107;

ChassisCommandConfig DefaultConfig() {
    ChassisCommandConfig c;
    c.chassis_type = ChassisType::Omni;
    return c;
}

ChassisTarget MakeTarget(double vx, double vy, double wz) {
    ChassisTarget t;
    t.source = ChassisCommandSource::VelocityCommand;
    t.twist = {vx, vy, wz};
    t.config = DefaultConfig();
    return t;
}

WheelFeedback MakeFeedback(const std::array<double, N>& ang, const std::array<double, N>& rpm, bool valid = true) {
    WheelFeedback fb;
    fb.valid = valid;
    fb.angle_rad = ang;
    fb.speed_rpm = rpm;
    return fb;
}

// ============================================================ 运动学

TEST(ChassisKinematics, WheelPositionsMatchTheDocumentedConvention) {
    ChassisKinematics k{ChassisGeometry{}};
    EXPECT_DOUBLE_EQ(k.OmniWheelPosition(0).x, kL / 2);  // 前左
    EXPECT_DOUBLE_EQ(k.OmniWheelPosition(0).y, kW / 2);
    EXPECT_DOUBLE_EQ(k.OmniWheelPosition(1).y, -kW / 2);  // 前右
    EXPECT_DOUBLE_EQ(k.OmniWheelPosition(2).x, -kL / 2);  // 后左
    EXPECT_DOUBLE_EQ(k.OmniWheelPosition(3).y, -kW / 2);  // 后右
}

TEST(ChassisKinematics, OmniInverseMatchesRigidBodyVelocityField) {
    ChassisKinematics k{ChassisGeometry{}};
    std::array<Vec2, N> v{};
    k.OmniInverse({0.4, -0.25, 0.35}, v);
    for (int i = 0; i < N; ++i) {
        const Vec2 r = k.OmniWheelPosition(i);
        EXPECT_NEAR(v[i].x, 0.4 - 0.35 * r.y, 1e-15);
        EXPECT_NEAR(v[i].y, -0.25 + 0.35 * r.x, 1e-15);
    }
}

TEST(ChassisKinematics, OmniForwardInvertsOmniInverse) {
    ChassisKinematics k{ChassisGeometry{}};
    for (const Twist2D t : {Twist2D{0.5, 0, 0}, Twist2D{0, 0.3, 0}, Twist2D{0, 0, 0.6}, Twist2D{0.4, -0.25, 0.35}}) {
        std::array<Vec2, N> v{};
        k.OmniInverse(t, v);
        std::array<double, N> speed{};
        std::array<double, N> angle{};
        for (int i = 0; i < N; ++i) {
            speed[i] = std::hypot(v[i].x, v[i].y);
            angle[i] = std::atan2(v[i].y, v[i].x);
        }
        const Twist2D back = k.OmniForward(speed, angle);
        EXPECT_NEAR(back.vx, t.vx, 1e-12);
        EXPECT_NEAR(back.vy, t.vy, 1e-12);
        EXPECT_NEAR(back.wz, t.wz, 1e-12);
    }
}

TEST(ChassisKinematics, OmniForwardMatchesLegacyOdometryFormula) {
    // 与切分前 IntegrateOmniOdomOnce 的手写式子逐点比对
    ChassisKinematics k{ChassisGeometry{}};
    const std::array<double, N> rpm{120.0, 118.0, 121.0, 119.0};
    const std::array<double, N> ang{0.10, -0.08, 0.05, 0.02};
    const double odom_factor = 1.03;

    std::array<double, N> speed{};
    for (int i = 0; i < N; ++i) speed[i] = k.OmniRpmToSpeed(rpm[i]) * odom_factor;
    const Twist2D got = k.OmniForward(speed, ang);

    double vx[N];
    double vy[N];
    for (int i = 0; i < N; ++i) {
        const double v = rpm[i] * 0.001 * 107.0 * M_PI / 60.0 * odom_factor;
        vx[i] = v * std::cos(ang[i]);
        vy[i] = v * std::sin(ang[i]);
    }
    const double legacy_vx = 0.25 * (vx[0] + vx[1] + vx[2] + vx[3]);
    const double legacy_vy = 0.25 * (vy[0] + vy[1] + vy[2] + vy[3]);
    const double vw_x = (vx[1] + vx[3] - vx[0] - vx[2]) / (2.0 * kW);
    const double vw_y = (vy[0] + vy[1] - vy[2] - vy[3]) / (2.0 * kL);
    EXPECT_NEAR(got.vx, legacy_vx, 1e-12);
    EXPECT_NEAR(got.vy, legacy_vy, 1e-12);
    EXPECT_NEAR(got.wz, 0.5 * (vw_x + vw_y), 1e-12);
}

TEST(ChassisKinematics, DiffInverseMatchesLegacyFormula) {
    ChassisKinematics k{ChassisGeometry{}};
    double l = 0;
    double r = 0;
    k.DiffInverse({0.6, 0.0, 0.4}, &l, &r);
    const double legacy_r = (0.6 + 0.4 * 0.4390 * 0.5) * 60 / (M_PI * 0.1730);
    const double legacy_l = (0.6 - 0.4 * 0.4390 * 0.5) * 60 / (M_PI * 0.1730);
    EXPECT_NEAR(r, legacy_r, 1e-9);
    EXPECT_NEAR(l, legacy_l, 1e-9);
}

TEST(ChassisKinematics, GeometryIsOneSetSharedByForwardAndInverse) {
    // 一套参数：改轴距，正解与逆解必须同时变，不允许一侧用新值另一侧用旧值
    ChassisGeometry g;
    g.diff_track = 0.5000;
    ChassisKinematics k{g};
    const ChassisKinematics k0{ChassisGeometry{}};
    double l = 0;
    double r = 0;
    double l0 = 0;
    double r0 = 0;
    k.DiffInverse({0.0, 0.0, 0.4}, &l, &r);
    k0.DiffInverse({0.0, 0.0, 0.4}, &l0, &r0);
    EXPECT_NE(r, r0) << "逆解必须跟着轴距变";
    EXPECT_NE(k.DiffForward(0.5, 0.7).wz, k0.DiffForward(0.5, 0.7).wz) << "正解必须跟着轴距变";
}

TEST(ChassisKinematics, OdomFactorAppliesSymmetricallyToBothDirections) {
    // 标定量对正逆解对称：正解 ×factor（报真实走了多少），逆解 ÷factor（让实际跑出期望速度）
    ChassisCalibration c;
    c.odom_factor = 1.05;
    const ChassisKinematics k{ChassisGeometry{}, c};
    const ChassisKinematics k0{ChassisGeometry{}};

    // 正解：同样的 rpm，标定后上报速度大 5%
    EXPECT_NEAR(k.OmniRpmToSpeed(100.0), k0.OmniRpmToSpeed(100.0) * 1.05, 1e-12);
    EXPECT_NEAR(k.DiffRpmToSpeed(100.0), k0.DiffRpmToSpeed(100.0) * 1.05, 1e-12);
    // 逆解：同样的期望速度，标定后下发 rpm 小 5%
    EXPECT_NEAR(k.OmniSpeedToRpm(0.5), k0.OmniSpeedToRpm(0.5) / 1.05, 1e-12);
    EXPECT_NEAR(k.DiffSpeedToRpm(0.5), k0.DiffSpeedToRpm(0.5) / 1.05, 1e-12);
    // 往返自洽：标定量不改变 rpm→速度→rpm 的恒等性
    for (double rpm : {-137.0, 0.0, 42.5, 300.0}) {
        EXPECT_NEAR(k.OmniSpeedToRpm(k.OmniRpmToSpeed(rpm)), rpm, 1e-9);
        EXPECT_NEAR(k.DiffSpeedToRpm(k.DiffRpmToSpeed(rpm)), rpm, 1e-9);
    }
    // 默认 1.0 时与改前完全一致
    EXPECT_DOUBLE_EQ(ChassisCalibration{}.odom_factor, 1.0);
    // 标定量与名义几何是两个结构：只改标定不动几何，几何读数不变
    EXPECT_DOUBLE_EQ(k.geometry().diff_wheel_radius, k0.geometry().diff_wheel_radius);
    EXPECT_DOUBLE_EQ(k.geometry().omni_wheel_d, k0.geometry().omni_wheel_d);
    EXPECT_NE(k.calibration().odom_factor, k0.calibration().odom_factor);
}

TEST(ChassisKinematics, SteerZeroAppliesSymmetricallyToBothDirections) {
    // 舵角零位对正逆解对称、方向相反：正解 +zero（编码器→真实朝向），逆解 −zero（理想朝向→编码器目标）。
    // 只补一侧会造成"控制对了里程计错"，两边各自自洽、不易察觉——与 odom_factor 同构的失败模式。
    ChassisCalibration c;
    c.omni_steer_zero_rad = {0.01, -0.02, 0.03, -0.04};
    const ChassisKinematics k{ChassisGeometry{}, c};
    const ChassisKinematics k0{ChassisGeometry{}};

    for (int i = 0; i < N; ++i) {
        const double zero = c.omni_steer_zero_rad[i];
        // 方向相反
        EXPECT_NEAR(k.OmniSteerTrueFromFb(i, 0.5), 0.5 + zero, 1e-12);
        EXPECT_NEAR(k.OmniSteerCmdFromTrue(i, 0.5), 0.5 - zero, 1e-12);
        // 互逆：下发再读回等于原值
        for (double a : {-1.3, 0.0, 0.7}) {
            EXPECT_NEAR(k.OmniSteerTrueFromFb(i, k.OmniSteerCmdFromTrue(i, a)), a, 1e-12);
        }
        // 默认全 0 时是恒等映射 ⇒ 与引入本参数前逐位一致
        EXPECT_DOUBLE_EQ(k0.OmniSteerTrueFromFb(i, 0.5), 0.5);
        EXPECT_DOUBLE_EQ(k0.OmniSteerCmdFromTrue(i, 0.5), 0.5);
        EXPECT_DOUBLE_EQ(ChassisCalibration{}.omni_steer_zero_rad[i], 0.0);
    }
    // 标定量与名义几何是两个结构：只改标定不动几何
    EXPECT_DOUBLE_EQ(k.geometry().omni_wheelbase, k0.geometry().omni_wheelbase);
}

TEST(ChassisKinematics, SteerZeroRestoresTrueOmegaOnStraightDrive) {
    // 场景：直行指令 ⇒ 舵角编码器读数恒为 0；零位为纯"前后差"模态 (d,d,−d,−d)。
    // 四轮实际朝向互不平行 ⇒ 底盘被合力矩推着以 Δω = 2·a·v/L 旋转（a = d）。
    //
    // 不补偿时 sin(0)=0 让 vy 全为 0 ⇒ w_base 支路整条失效 ⇒ 合成 ω 恰好只剩真值的**一半**，
    // 单看轮式里程计就看不出问题有多严重。补偿后两条基线都还原出 Δω。
    const double d = 0.01;  // rad ≈ 0.57°
    const double v = 0.5;   // m/s
    const ChassisGeometry g{};
    const double domega = 2.0 * d * v / g.omni_wheelbase;  // 物理上真实发生的角速度

    // 轮 i 的接地点速度沿其实际朝向的分量（一阶）：s_i = v − Δω·y_i
    std::array<double, N> speed{};
    std::array<double, N> enc_angle{};  // 编码器读数：直行指令下恒为 0
    for (int i = 0; i < N; ++i) {
        const Vec2 r = ChassisKinematics{g}.OmniWheelPosition(i);
        speed[i] = v - domega * r.y;
        enc_angle[i] = 0.0;
    }

    ChassisCalibration c;
    c.omni_steer_zero_rad = {d, d, -d, -d};
    const Twist2D compensated = ChassisKinematics{g, c}.OmniForward(speed, enc_angle);
    const Twist2D uncompensated = ChassisKinematics{g}.OmniForward(speed, enc_angle);

    EXPECT_NEAR(uncompensated.wz, 0.5 * domega, 1e-12) << "未补偿时应恰好低估一半";
    EXPECT_NEAR(compensated.wz, domega, 1e-5) << "补偿后应还原出真实角速度";
    // 纯前后差模态不含共模，故无侧移；纵向速度不受影响
    EXPECT_NEAR(compensated.vy, 0.0, 1e-9);
    EXPECT_NEAR(compensated.vx, v, 1e-4);
}

// ============================================================ 指令生成

TEST(ChassisCommand, ZeroCommandHoldsSteeringAngleAndStopsWheels) {
    const std::array<double, N> ang{0.3, -0.2, 0.1, 0.05};
    ChassisControl out{};
    GenerateOmniCommand(MakeTarget(0, 0, 0), MakeFeedback(ang, {10, 10, 10, 10}), /*expired=*/false, &out);
    for (int i = 0; i < N; ++i) {
        EXPECT_DOUBLE_EQ(out.chassis_motor_direction[i], ang[i]);
        EXPECT_DOUBLE_EQ(out.chassis_motor_velocity[i], 0.0);
    }
}

TEST(ChassisCommand, ExpiredTargetKeepsAngleAndZeroesSpeed) {
    const std::array<double, N> ang{0.3, -0.2, 0.1, 0.05};
    ChassisControl out{};
    GenerateOmniCommand(MakeTarget(0.5, 0, 0), MakeFeedback(ang, {50, 50, 50, 50}), /*expired=*/true, &out);
    for (int i = 0; i < N; ++i) {
        EXPECT_DOUBLE_EQ(out.chassis_motor_direction[i], ang[i]) << "idle 必须保留驻车角度";
        EXPECT_DOUBLE_EQ(out.chassis_motor_velocity[i], 0.0);
    }
}

TEST(ChassisCommand, AlignmentLimitsSpeedPerWheelNotAllOrNothing) {
    // 只有 0 号轮偏 90°：它降到爬行，其余三轮照常走——不再一轮拖累四轮
    ChassisControl out{};
    const auto t = MakeTarget(0.5, 0, 0);
    GenerateOmniCommand(t, MakeFeedback({M_PI / 2, 0, 0, 0}, {0, 0, 0, 0}), false, &out);
    const ChassisKinematics k{t.config.geometry, t.config.calibration};
    const double full_rpm = k.OmniSpeedToRpm(0.5) * 0.8;  // speed_ramp 从实测 0 出发
    EXPECT_NEAR(out.chassis_motor_velocity[0], full_rpm * 0.08, 1e-6) << "偏 90° 的轮只剩爬行";
    for (int i = 1; i < N; ++i) {
        EXPECT_NEAR(out.chassis_motor_velocity[i], full_rpm, 1e-6) << "已对齐的轮不该被拖累，wheel " << i;
    }
    EXPECT_NEAR(out.chassis_motor_direction[0], 0.0, 1e-12) << "舵角目标照常下发";
}

TEST(ChassisCommand, AlignSpeedScaleIsContinuousAndMonotonic) {
    OmniCommandParams p;
    EXPECT_DOUBLE_EQ(AlignSpeedScale(0.0, p), 1.0);
    EXPECT_DOUBLE_EQ(AlignSpeedScale(p.align_full_rad, p), 1.0);
    EXPECT_DOUBLE_EQ(AlignSpeedScale(p.align_crawl_rad, p), p.align_min_scale);
    EXPECT_DOUBLE_EQ(AlignSpeedScale(3.0, p), p.align_min_scale);
    // 中点线性
    const double mid = 0.5 * (p.align_full_rad + p.align_crawl_rad);
    EXPECT_NEAR(AlignSpeedScale(mid, p), 0.5 * (1.0 + p.align_min_scale), 1e-12);
    // 单调不增且连续
    double prev = 1.0;
    for (double e = 0.0; e <= 1.2; e += 0.01) {
        const double s = AlignSpeedScale(e, p);
        EXPECT_LE(s, prev + 1e-12);
        EXPECT_GE(s, p.align_min_scale - 1e-12);
        prev = s;
    }
}

TEST(ChassisCommand, SteeringCostUsesUnwrappedTravelWithinTheLimitInterval) {
    // 舵角可行域是单区间 [−0.8π, 0.8π]，+140° → −140° 的真实行程是 280° 而非环绕的 80°。
    // 当前角 +140°，命令后退：候选 a = 180°（超限），候选 b = 0°。
    // 用不环绕代价时 b 的行程 140° 明显更短，必须选 b。
    ChassisControl out{};
    const double cur = 140.0 * M_PI / 180.0;
    GenerateOmniCommand(MakeTarget(-0.5, 0, 0), MakeFeedback({cur, cur, cur, cur}, {0, 0, 0, 0}), false, &out);
    for (int i = 0; i < N; ++i) {
        EXPECT_NEAR(out.chassis_motor_direction[i], 0.0, 1e-12);
        EXPECT_LE(std::abs(out.chassis_motor_direction[i]), 0.8 * M_PI) << "选中的解必须在限位内";
    }
}

TEST(ChassisCommand, SaturationScalesAllWheelsUniformlyAndKeepsTwistRatio) {
    // 大速度 + 大 yaw 使部分轮超限：四轮同比缩放，正解还原出的 ω/|v| 不变
    auto t = MakeTarget(1.2, 0.0, 1.2);
    t.config.omni.max_wheel_speed_mps = 0.8;
    t.config.omni.align_full_rad = 10.0;  // 排除对齐限速干扰
    ChassisControl out{};
    GenerateOmniCommand(t, MakeFeedback({0, 0, 0, 0}, {0, 0, 0, 0}), false, &out);

    const ChassisKinematics k{t.config.geometry, t.config.calibration};
    std::array<double, N> sp{};
    std::array<double, N> ang{};
    for (int i = 0; i < N; ++i) {
        // 反解出被下发的轮速（speed_ramp 从实测 0 出发，故命令 = 0.8×目标）
        sp[i] = k.OmniRpmToSpeed(out.chassis_motor_velocity[i] / 0.8);
        ang[i] = out.chassis_motor_direction[i];
        EXPECT_LE(std::abs(sp[i]), 0.8 + 1e-9) << "wheel " << i << " 必须在轮速上限内";
    }
    const Twist2D got = k.OmniForward(sp, ang);
    const double ratio_cmd = 1.2 / std::hypot(1.2, 0.0);
    const double ratio_got = got.wz / std::hypot(got.vx, got.vy);
    EXPECT_NEAR(ratio_got, ratio_cmd, 1e-6) << "同比缩放必须保持 ω/|v|（瞬心不动）";
}

TEST(ChassisCommand, SaturationDisabledWhenLimitNotPositive) {
    auto t = MakeTarget(1.2, 0.0, 1.2);
    t.config.omni.max_wheel_speed_mps = 0.0;  // ≤0 表示不限制
    t.config.omni.align_full_rad = 10.0;
    ChassisControl out{};
    GenerateOmniCommand(t, MakeFeedback({0, 0, 0, 0}, {0, 0, 0, 0}), false, &out);
    const ChassisKinematics k{t.config.geometry, t.config.calibration};
    double peak = 0.0;
    for (int i = 0; i < N; ++i) peak = std::max(peak, std::abs(k.OmniRpmToSpeed(out.chassis_motor_velocity[i] / 0.8)));
    // 该 twist 的理论峰值轮速 = hypot(1.2 + 1.2·W/2, 1.2·L/2) ≈ 1.434 m/s
    const double expect_peak = std::hypot(1.2 + 1.2 * kW * 0.5, 1.2 * kL * 0.5);
    EXPECT_NEAR(peak, expect_peak, 1e-6) << "关闭上限后不应被削";
    EXPECT_GT(peak, 0.8) << "确实超过了上一条测试用的 0.8 上限";
}

TEST(ChassisCommand, WheelVectorDeadbandHoldsAngleNearInstantCenter) {
    // 让 3 号轮恰好落在瞬心上：v_i ≈ 0，方向无定义，必须保持当前舵角而不是跳到 atan2 噪声值
    auto t = MakeTarget(0.0, 0.0, 0.0);
    const ChassisKinematics k{t.config.geometry, t.config.calibration};
    const Vec2 r = k.OmniWheelPosition(3);
    // 瞬心在 3 号轮位置 ⇒ vx = ω·y_3，vy = −ω·x_3
    const double w = 0.5;
    t.twist = {w * r.y, -w * r.x, w};
    const double held = 1.234;
    std::array<double, N> ang{0.0, 0.0, 0.0, held};
    ChassisControl out{};
    GenerateOmniCommand(t, MakeFeedback(ang, {0, 0, 0, 0}), false, &out);
    EXPECT_DOUBLE_EQ(out.chassis_motor_direction[3], held) << "瞬心上的轮保持原舵角";
    EXPECT_DOUBLE_EQ(out.chassis_motor_velocity[3], 0.0);
    EXPECT_NE(out.chassis_motor_direction[0], held) << "其余轮照常解算";
}

TEST(ChassisCommand, StaleFeedbackStopsWheels) {
    ChassisControl out{};
    GenerateOmniCommand(MakeTarget(0.5, 0, 0), MakeFeedback({0, 0, 0, 0}, {0, 0, 0, 0}, /*valid=*/false), false, &out);
    for (int i = 0; i < N; ++i) EXPECT_DOUBLE_EQ(out.chassis_motor_velocity[i], 0.0);
}

TEST(ChassisCommand, AlignedWheelsRampTowardTargetFromMeasuredSpeed) {
    ChassisControl out{};
    const auto t = MakeTarget(0.5, 0, 0);
    GenerateOmniCommand(t, MakeFeedback({0, 0, 0, 0}, {10, 10, 10, 10}), false, &out);
    const ChassisKinematics k{t.config.geometry};
    const double target_rpm = k.OmniSpeedToRpm(0.5);
    for (int i = 0; i < N; ++i) {
        EXPECT_NEAR(out.chassis_motor_velocity[i], 10.0 + (target_rpm - 10.0) * 0.8, 1e-9);
    }
}

TEST(ChassisCommand, ReverseSolutionChosenWhenCloserToCurrentAngle) {
    // 目标方向 +180°（后退）：舵角 π 超限（0.8π），必须选反向解 0 并给负轮速
    ChassisControl out{};
    GenerateOmniCommand(MakeTarget(-0.5, 0, 0), MakeFeedback({0, 0, 0, 0}, {0, 0, 0, 0}), false, &out);
    for (int i = 0; i < N; ++i) {
        EXPECT_NEAR(out.chassis_motor_direction[i], 0.0, 1e-12);
        EXPECT_LT(out.chassis_motor_velocity[i], 0.0);
    }
}

TEST(ChassisCommand, SteerZeroShiftsTargetIntoEncoderFrame) {
    // 逆解侧：理想朝向在 atan2 之后立即换到编码器坐标系（减去零位），
    // 使得下发的编码器目标与 fb.angle_rad 同系——限位 / 择近解 / 对齐限速因此无需改动。
    const std::array<double, N> zero{0.01, -0.02, 0.03, -0.04};
    const std::array<double, N> fb_ang{0.0, 0.0, 0.0, 0.0};
    const std::array<double, N> fb_rpm{0.0, 0.0, 0.0, 0.0};

    ChassisTarget t = MakeTarget(0.4, 0.0, 0.0);  // 纯直行 ⇒ 理想朝向全为 0
    t.config.calibration.omni_steer_zero_rad = zero;

    ChassisControl out{};
    GenerateOmniCommand(t, MakeFeedback(fb_ang, fb_rpm), false, &out);

    for (int i = 0; i < N; ++i) {
        // 编码器目标 = 理想朝向(0) − zero
        EXPECT_NEAR(out.chassis_motor_direction[i], -zero[i], 1e-12);
    }

    // 零位全 0 时与引入本参数前一致：直行的编码器目标就是 0
    ChassisTarget t0 = MakeTarget(0.4, 0.0, 0.0);
    ChassisControl out0{};
    GenerateOmniCommand(t0, MakeFeedback(fb_ang, fb_rpm), false, &out0);
    for (int i = 0; i < N; ++i) {
        EXPECT_NEAR(out0.chassis_motor_direction[i], 0.0, 1e-12);
    }
}

TEST(ChassisCommand, MotorLevelTargetBypassesInverseKinematics) {
    ChassisTarget t;
    t.source = ChassisCommandSource::ParkingAction;
    t.config = DefaultConfig();
    t.motor_level = true;
    t.motor_angle_rad = {0.5, -0.5, 0.25, -0.25};
    t.motor_speed_rpm = {5, -5, 5, -5};
    t.estop = 1;
    t.neutral = 1;
    ChassisControl out{};
    GenerateOmniCommand(t, MakeFeedback({0, 0, 0, 0}, {0, 0, 0, 0}), false, &out);
    for (int i = 0; i < N; ++i) {
        EXPECT_DOUBLE_EQ(out.chassis_motor_direction[i], t.motor_angle_rad[i]);
        EXPECT_DOUBLE_EQ(out.chassis_motor_velocity[i], t.motor_speed_rpm[i]);
    }
    EXPECT_EQ(out.chassis_estop, 1);
    EXPECT_EQ(out.chassis_neutral, 1);
}

TEST(ChassisCommand, ChassisTypeEnumMatchesSharedMemoryWireValues) {
    // 枚举值必须与 ChassisControl::chassis_mode 的线上取值一致，否则 static_cast 会悄悄发错模式
    EXPECT_EQ(static_cast<int>(ChassisType::DiffDrive), 1);
    EXPECT_EQ(static_cast<int>(ChassisType::Omni), 2);
    EXPECT_STREQ(ChassisTypeName(ChassisType::DiffDrive), "diff_drive");
    EXPECT_STREQ(ChassisTypeName(ChassisType::Omni), "omni");
    // 全工程只有一个底盘形态枚举：配置层与命令层是同一个类型
    static_assert(std::is_same<ChassisType, robot_server_lib::utils::ChassisType>::value, "命令层不应再造一个同义枚举");
    ChassisTarget t = MakeTarget(0.3, 0, 0);
    t.config.chassis_type = ChassisType::Omni;
    ChassisControl out{};
    GenerateChassisCommand(t, WheelFeedback{}, false, &out);
    EXPECT_EQ(out.chassis_mode, 2);
}

TEST(ChassisCommand, DiffModeDispatchesToDifferentialInverse) {
    ChassisTarget t = MakeTarget(0.6, 0.0, 0.4);
    t.config.chassis_type = ChassisType::DiffDrive;
    ChassisControl out{};
    GenerateChassisCommand(t, WheelFeedback{}, false, &out);
    EXPECT_EQ(out.chassis_mode, static_cast<int>(ChassisType::DiffDrive));
    EXPECT_GT(out.chassis_right_velocity, out.chassis_left_velocity);
}

// ============================================================ 与切分前实现的等价性

namespace legacy {
constexpr double kSteerLimit = M_PI * 0.8;
constexpr double kAngleEps = 1.0;
constexpr double kSpeedRamp = 0.8;
double Wrap(double a) {
    while (a > M_PI) a -= 2 * M_PI;
    while (a <= -M_PI) a += 2 * M_PI;
    return a;
}
double Diff(double a, double b) { return Wrap(a - b); }

/// 切分前 ChassisVelocityControlCallback 的 omni 分支，原样复刻
void Compute(double vx,
             double vy,
             double wz,
             const std::array<double, N>& prev_angle,
             const std::array<double, N>& prev_rpm,
             bool feedback_fresh,
             std::array<double, N>* out_angle,
             std::array<double, N>* out_rpm) {
    std::array<double, N> target_angle{};
    std::array<double, N> target_rpm{};
    std::array<double, N> angle_error{};
    for (int i = 0; i < N; ++i) {
        const double sx = (i == 1 || i == 3) ? 1.0 : -1.0;
        const double sy = (i == 0 || i == 1) ? 1.0 : -1.0;
        const double wheel_vx = vx + sx * kW * 0.5 * wz;
        const double wheel_vy = vy + sy * kL * 0.5 * wz;
        const double speed = std::hypot(wheel_vx, wheel_vy);
        const double angle_a = Wrap(std::atan2(wheel_vy, wheel_vx));
        const double angle_b = Wrap(angle_a + M_PI);
        const bool valid_a = std::abs(angle_a) <= kSteerLimit;
        const bool valid_b = std::abs(angle_b) <= kSteerLimit;
        const double cost_a =
            valid_a ? std::abs(Diff(angle_a, prev_angle[i])) : std::numeric_limits<double>::infinity();
        const double cost_b =
            valid_b ? std::abs(Diff(angle_b, prev_angle[i])) : std::numeric_limits<double>::infinity();
        const bool choose_a = cost_a <= cost_b;
        target_angle[i] = choose_a ? angle_a : angle_b;
        target_rpm[i] = (choose_a ? speed : -speed) * 60.0 / (M_PI * kD);
        angle_error[i] = std::abs(Diff(target_angle[i], prev_angle[i]));
    }
    bool steering_done = true;
    for (double e : angle_error) {
        if (e > kAngleEps) {
            steering_done = false;
            break;
        }
    }
    for (int i = 0; i < N; ++i) {
        (*out_angle)[i] = target_angle[i];
        if (std::hypot(vx, vy) <= 1e-4 && std::abs(wz) <= 1e-4) {
            (*out_angle)[i] = prev_angle[i];
            (*out_rpm)[i] = 0.0;
        } else if (!steering_done || !feedback_fresh) {
            (*out_rpm)[i] = 0.0;
        } else {
            (*out_rpm)[i] = prev_rpm[i] + (target_rpm[i] - prev_rpm[i]) * kSpeedRamp;
        }
    }
}
}  // namespace legacy

TEST(ChassisCommand, LegacyEquivalenceOfAngleSelectionAndRpmConversion) {
    std::mt19937 rng(20260916);
    std::uniform_real_distribution<double> v(-1.2, 1.2);
    std::uniform_real_distribution<double> w(-1.0, 1.0);
    std::uniform_real_distribution<double> ang(-2.4, 2.4);
    std::uniform_real_distribution<double> rpm(-200.0, 200.0);
    std::bernoulli_distribution fresh(0.8);
    int compared = 0;
    int skipped = 0;
    int gated = 0;

    for (int trial = 0; trial < 20000; ++trial) {
        const double vx = v(rng);
        const double vy = v(rng);
        const double wz = w(rng);
        std::array<double, N> prev_angle{};
        std::array<double, N> prev_rpm{};
        for (int i = 0; i < N; ++i) {
            prev_angle[i] = ang(rng);
            prev_rpm[i] = rpm(rng);
        }
        const bool valid = fresh(rng);

        std::array<double, N> exp_angle{};
        std::array<double, N> exp_rpm{};
        legacy::Compute(vx, vy, wz, prev_angle, prev_rpm, valid, &exp_angle, &exp_rpm);

        // 关掉本轮新增的三项（对齐限速 / 轮速上限 / 矢量死区），只比"角度选择 + 轮速换算"这一核心。
        // 新增行为各有专门测试；代价度量由环绕改成不环绕是有意修正，故跳过两者不等价的样本。
        auto t = MakeTarget(vx, vy, wz);
        t.config.omni.align_full_rad = 10.0;
        t.config.omni.max_wheel_speed_mps = 0.0;
        t.config.omni.wheel_vector_deadband_mps = 0.0;
        bool cost_metric_agrees = true;
        for (int i = 0; i < N; ++i) {
            const double d = exp_angle[i] - prev_angle[i];
            if (std::abs(std::abs(d) - std::abs(legacy::Wrap(d))) > 1e-12) cost_metric_agrees = false;
        }
        if (!cost_metric_agrees) {
            ++skipped;
            continue;
        }
        ++compared;

        ChassisControl out{};
        GenerateOmniCommand(t, MakeFeedback(prev_angle, prev_rpm, valid), false, &out);

        // 舵角选择：任何情况下都必须一致
        for (int i = 0; i < N; ++i) {
            ASSERT_NEAR(out.chassis_motor_direction[i], exp_angle[i], 1e-12) << "trial " << trial << " wheel " << i;
        }
        // 轮速：legacy 的全或无门控一旦触发就把四轮置零，那是本轮有意改掉的行为，
        // 故只在它不拦的样本上比对轮速换算。
        bool legacy_gate = false;
        for (int i = 0; i < N; ++i) {
            if (std::abs(legacy::Diff(exp_angle[i], prev_angle[i])) > 1.0) legacy_gate = true;
        }
        if (legacy_gate) {
            ++gated;
            continue;
        }
        for (int i = 0; i < N; ++i) {
            ASSERT_NEAR(out.chassis_motor_velocity[i], exp_rpm[i], 1e-9) << "trial " << trial << " wheel " << i;
        }
    }
    EXPECT_GT(compared, 5000) << "有效比对样本太少，说明筛选条件写错了";
    std::cout << "  [等价比对] 角度逐点一致 " << compared << " 组（其中轮速也比对 " << (compared - gated)
              << " 组），跳过：代价度量有意不同 " << skipped << " 组 / legacy 全或无门控触发 " << gated << " 组\n";
}

// ============================================================ 仲裁

TEST(ChassisArbiter, ParkingPreemptsVelocityUntilItsTimeout) {
    ChassisCommandArbiter a;
    const int64_t t0 = 1'000'000'000;
    EXPECT_TRUE(a.OfferVelocity(t0));
    a.OfferParking(t0 + 1'000'000);  // 驻车抢占
    EXPECT_EQ(a.source(), ChassisCommandSource::ParkingAction);
    EXPECT_FALSE(a.OfferVelocity(t0 + 100'000'000)) << "驻车持权 0.1 s 内速度指令应被拒绝";
    // 超过 0.30 s 后驻车释放，速度可重新取得控制权
    EXPECT_TRUE(a.OfferVelocity(t0 + 1'000'000 + 300'000'000));
    EXPECT_EQ(a.source(), ChassisCommandSource::VelocityCommand);
}

TEST(ChassisArbiter, VelocityOwnershipReleasesAfterTimeout) {
    ChassisCommandArbiter a;
    const int64_t t0 = 5'000'000'000;
    EXPECT_TRUE(a.OfferVelocity(t0));
    EXPECT_TRUE(a.OfferVelocity(t0 + 100'000'000));
    EXPECT_EQ(a.source(), ChassisCommandSource::VelocityCommand);
}

TEST(ChassisCommand, TargetExpiryUsesPerSourceTimeout) {
    ChassisTarget t = MakeTarget(0.5, 0, 0);
    t.stamp_ns = 0;
    EXPECT_FALSE(IsTargetExpired(t, 400'000'000));  // 0.4 s < 0.5 s
    EXPECT_TRUE(IsTargetExpired(t, 600'000'000));
    t.source = ChassisCommandSource::ParkingAction;
    EXPECT_FALSE(IsTargetExpired(t, 200'000'000));  // 0.2 s < 0.3 s
    EXPECT_TRUE(IsTargetExpired(t, 400'000'000));
    t.source = ChassisCommandSource::None;
    EXPECT_TRUE(IsTargetExpired(t, 0)) << "无来源即 idle";
}

}  // namespace
}  // namespace chassis_node
