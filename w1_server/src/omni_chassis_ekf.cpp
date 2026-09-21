#include "omni_chassis_ekf.h"

#include <cmath>

namespace chassis_node {

OmniChassisEkf::OmniChassisEkf(const Params& p) : params_(p) { Reset(); }

void OmniChassisEkf::SetState(double x, double y, double theta, double vx, double vy, double omega) {
    x_ << x, y, theta, vx, vy, omega;
    P_ = Mat6::Zero();
    P_(0, 0) = params_.p0_xy;
    P_(1, 1) = params_.p0_xy;
    P_(2, 2) = params_.p0_theta;
    P_(3, 3) = params_.p0_v;
    P_(4, 4) = params_.p0_vy;
    P_(5, 5) = params_.p0_omega;
    is_aerial_ = false;
}

void OmniChassisEkf::SetParams(const Params& p) {
    // 只替换参数，不动 x_ / P_（p0_* 只在 Reset / SetState 生效，避免调参引起协方差跳变）
    params_ = p;
}

void OmniChassisEkf::Reset() { SetState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0); }

OmniChassisEkf::Mat6 OmniChassisEkf::BuildQ(double dt, bool is_stopped, bool use_imu) const {
    // Q 以「每秒」定义，乘 dt 得本步过程噪声；静止时再按 q_scale_stopped 收敛（与差速一致）
    double scale = dt;
    if (is_stopped) {
        scale *= params_.q_scale_stopped;
    }
    // IMU 模式下 F(5,5)=0，P(5,5) 的预测值就是此项——代表陀螺测量噪声而非角速度过程扩散
    const double q_om = (use_imu && !is_stopped) ? params_.q_omega_imu : params_.q_omega;

    Mat6 Q = Mat6::Zero();
    Q(0, 0) = params_.q_xy * scale;
    Q(1, 1) = params_.q_xy * scale;
    Q(2, 2) = params_.q_theta * scale;
    Q(3, 3) = params_.q_v * scale;
    Q(4, 4) = params_.q_vy * scale;
    Q(5, 5) = q_om * scale;
    return Q;
}

bool OmniChassisEkf::Step(double dt, double vx_enc, double vy_enc, double omega_enc, double omega_z_imu,
                          bool imu_available, bool is_stopped) {
    Predict(dt, omega_z_imu, imu_available, is_stopped);
    is_aerial_ = Update(vx_enc, vy_enc, omega_enc, is_stopped, imu_available);
    return is_aerial_;
}

void OmniChassisEkf::Predict(double dt, double omega_z, bool use_imu, bool is_stopped) {
    const double theta = x_(2);
    const double vx = x_(3);
    const double vy = x_(4);
    const double omega = x_(5);

    // 静止时强制 ω=0：否则 IMU 零偏残差逐帧积进 θ 且 θ 无量测永不修正（同差速）
    const double omega_use = is_stopped ? 0.0 : (use_imu ? omega_z : omega);
    const double theta_mid = theta + 0.5 * omega_use * dt;
    const double c = std::cos(theta_mid);
    const double s = std::sin(theta_mid);

    // ---- 状态预测：机体系 (vx, vy) 经中点航向旋到 odom 系 ----
    Vec6 x_pred;
    x_pred(0) = x_(0) + (vx * c - vy * s) * dt;
    x_pred(1) = x_(1) + (vx * s + vy * c) * dt;
    x_pred(2) = theta + omega_use * dt;
    x_pred(3) = vx;         // 匀速——由编码器量测修正
    x_pred(4) = vy;         // 同上；vy 是独立状态，侧移不再被解释成航向误差
    x_pred(5) = omega_use;  // 直接取自 IMU 或保持上一帧
    x_pred(2) = std::atan2(std::sin(x_pred(2)), std::cos(x_pred(2)));

    // ---- 过程模型雅可比 F（中点近似）----
    Mat6 F = Mat6::Identity();
    F(0, 2) = (-vx * s - vy * c) * dt;
    F(0, 3) = c * dt;
    F(0, 4) = -s * dt;
    F(1, 2) = (vx * c - vy * s) * dt;
    F(1, 3) = s * dt;
    F(1, 4) = c * dt;
    if (use_imu && !is_stopped) {
        // ω 由 IMU 直接给定：状态中的 ω 不影响下一时刻的 θ 与 ω
        F(2, 5) = 0.0;
        F(5, 5) = 0.0;
    } else {
        // θ 依赖 ω 状态（纯编码器；静止时同样如此，使 ZUPT 的零速量测能压到 θ 的协方差）
        F(2, 5) = dt;
    }

    P_ = F * P_ * F.transpose() + BuildQ(dt, is_stopped, use_imu);
    x_ = x_pred;
}

bool OmniChassisEkf::Update(double vx_enc, double vy_enc, double omega_enc, bool is_stopped, bool imu_driven) {
    // ---- 量测矩阵 H（3×6）：z = [vx_enc, vy_enc, omega_enc] ----
    Mat36 H = Mat36::Zero();
    H(0, 3) = 1.0;
    H(1, 4) = 1.0;
    H(2, 5) = 1.0;

    // ---- 静止：零速量测（三维都成立）----
    if (is_stopped) {
        const Vec3 y = Vec3::Zero() - H * x_;
        Mat3 R = Mat3::Zero();
        R(0, 0) = params_.r_zupt;
        R(1, 1) = params_.r_zupt;
        R(2, 2) = params_.r_zupt;
        ApplyUpdate<3>(H, y, R);
        return false;  // 静止期不做悬空/打滑判定
    }

    // ---- 检测：始终基于 3 维新息 ----
    // Δω 是转向打滑的信号；Δvy 在全向上额外给出侧滑信号（舵轮侧移时轮胎侧向拖拽更常见）。
    // 即使 IMU 模式下不用 Δω 修正状态，它也必须参与统计检验。
    Vec3 z;
    z << vx_enc, vy_enc, omega_enc;
    const Vec3 y = z - H * x_;

    Mat3 R_normal = Mat3::Zero();
    R_normal(0, 0) = params_.r_v;
    R_normal(1, 1) = params_.r_vy;
    R_normal(2, 2) = params_.r_omega;

    const Mat3 S_test = H * P_ * H.transpose() + R_normal;
    const double det = S_test.determinant();
    const bool aerial = (std::abs(det) > 1e-18) &&
                        ((y.transpose() * S_test.inverse() * y)(0, 0) > params_.aerial_chi2_threshold_3d);

    // ---- 更新：IMU 模式只取 vx、vy 两行 ----
    if (imu_driven) {
        // ω 由 IMU 在 Predict 中直接给定，不再用编码器 ω 去"修正"它（否则 K≈0.5 对半折中，
        // 发布的 ω 不再是发布的 θ 的导数）
        Eigen::Matrix<double, 2, N> H2 = H.topRows<2>();
        Eigen::Matrix<double, 2, 1> y2 = y.head<2>();
        Eigen::Matrix<double, 2, 2> R2 = Eigen::Matrix<double, 2, 2>::Zero();
        R2(0, 0) = aerial ? params_.r_v_aerial : params_.r_v;
        R2(1, 1) = aerial ? params_.r_vy_aerial : params_.r_vy;
        ApplyUpdate<2>(H2, y2, R2);
        return aerial;
    }

    // ---- 纯编码器模式：三维都由编码器提供 ----
    Mat3 R = Mat3::Zero();
    if (aerial) {
        R(0, 0) = params_.r_v_aerial;
        R(1, 1) = params_.r_vy_aerial;
        R(2, 2) = params_.r_omega_aerial;
    } else {
        R = R_normal;
    }
    ApplyUpdate<3>(H, y, R);
    return aerial;
}

}  // namespace chassis_node
