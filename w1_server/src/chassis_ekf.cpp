#include "chassis_ekf.h"

#include <cmath>

namespace chassis_node {

ChassisEkf::ChassisEkf(const Params& p) : params_(p) { Reset(); }

void ChassisEkf::SetParams(const Params& p) {
    params_ = p;
    // 更新参数后重新初始化协方差（不重置位姿）
    P_ = Mat5::Zero();
    P_(0, 0) = params_.p0_xy;
    P_(1, 1) = params_.p0_xy;
    P_(2, 2) = params_.p0_theta;
    P_(3, 3) = params_.p0_v;
    P_(4, 4) = params_.p0_omega;
}

void ChassisEkf::Reset() {
    x_ = Vec5::Zero();
    P_ = Mat5::Zero();
    P_(0, 0) = params_.p0_xy;
    P_(1, 1) = params_.p0_xy;
    P_(2, 2) = params_.p0_theta;
    P_(3, 3) = params_.p0_v;
    P_(4, 4) = params_.p0_omega;
    is_aerial_ = false;
}

bool ChassisEkf::Step(double dt, double v_enc, double omega_enc, double omega_z_imu, bool imu_available) {
    Predict(dt, omega_z_imu, imu_available);
    is_aerial_ = Update(v_enc, omega_enc);
    return is_aerial_;
}

void ChassisEkf::Predict(double dt, double omega_z, bool use_imu) {
    const double theta = x_(2);
    const double v     = x_(3);
    const double omega = x_(4);

    // 中点航向角（减小一阶非线性误差）
    // 与现有代码的 theta_mid 逻辑一致
    const double omega_use  = use_imu ? omega_z : omega;
    const double theta_mid  = theta + 0.5 * omega_use * dt;
    const double cos_mid    = std::cos(theta_mid);
    const double sin_mid    = std::sin(theta_mid);

    // ---- 状态预测 ----
    Vec5 x_pred;
    x_pred(0) = x_(0) + v * cos_mid * dt;
    x_pred(1) = x_(1) + v * sin_mid * dt;
    x_pred(2) = theta + omega_use * dt;
    x_pred(3) = v;           // 匀速—由编码器量测修正
    x_pred(4) = omega_use;   // 直接取自 IMU 或保持上一帧

    // 角度归一化到 [-π, π]
    x_pred(2) = std::atan2(std::sin(x_pred(2)), std::cos(x_pred(2)));

    // ---- 过程模型雅可比 F（对状态 x 求偏导，中点近似） ----
    // F[theta_row] w.r.t. omega_state: IMU 模式为 0（theta 由 IMU 驱动），无 IMU 模式为 dt
    // F[omega_row] w.r.t. omega_state: IMU 模式为 0，无 IMU 模式为 1
    Mat5 F = Mat5::Identity();
    F(0, 2) = -v * sin_mid * dt;
    F(0, 3) =  cos_mid * dt;
    F(1, 2) =  v * cos_mid * dt;
    F(1, 3) =  sin_mid * dt;
    if (!use_imu) {
        // theta 依赖 omega_state
        F(2, 4) = dt;
        // omega 保持不变（F(4,4) = 1 已由 Identity 给出）
    } else {
        // IMU 直接给出 omega，状态中的 omega 不影响下一时刻 theta 和 omega
        F(2, 4) = 0.0;
        F(4, 4) = 0.0;
    }

    // ---- 过程噪声 Q ----
    Mat5 Q = Mat5::Zero();
    Q(0, 0) = params_.q_xy;
    Q(1, 1) = params_.q_xy;
    Q(2, 2) = params_.q_theta;
    Q(3, 3) = params_.q_v;
    Q(4, 4) = params_.q_omega;

    // ---- 协方差预测 ----
    P_ = F * P_ * F.transpose() + Q;

    x_ = x_pred;
}

bool ChassisEkf::Update(double v_enc, double omega_enc) {
    // ---- 量测矩阵 H（2×5）----
    // z = [v_enc, omega_enc] = H * x + noise
    Mat25 H = Mat25::Zero();
    H(0, 3) = 1.0;  // v 对应 x[3]
    H(1, 4) = 1.0;  // omega 对应 x[4]

    // ---- 新息 ----
    Vec2 z;
    z(0) = v_enc;
    z(1) = omega_enc;
    const Vec2 y = z - H * x_;

    // ---- Mahalanobis 检验（用正常噪声 R 计算，判断编码器可信度）----
    Mat2 R_normal = Mat2::Zero();
    R_normal(0, 0) = params_.r_v;
    R_normal(1, 1) = params_.r_omega;

    const Mat2 S_test = H * P_ * H.transpose() + R_normal;
    // 2×2 逆（避免依赖 full inverse，直接手算更快）
    const double det = S_test(0,0) * S_test(1,1) - S_test(0,1) * S_test(1,0);
    const bool aerial = (std::abs(det) > 1e-12) &&
                        ((y.transpose() * S_test.inverse() * y)(0, 0) > params_.aerial_chi2_threshold);

    // ---- 选择量测噪声 ----
    Mat2 R = Mat2::Zero();
    if (aerial) {
        R(0, 0) = params_.r_v_aerial;
        R(1, 1) = params_.r_omega_aerial;
    } else {
        R = R_normal;
    }

    // ---- 新息协方差 & 卡尔曼增益 ----
    const Mat2 S = H * P_ * H.transpose() + R;
    const Eigen::Matrix<double, N, 2> K = P_ * H.transpose() * S.inverse();

    // ---- 状态更新 ----
    x_ += K * y;
    // 角度归一化
    x_(2) = std::atan2(std::sin(x_(2)), std::cos(x_(2)));

    // ---- 协方差更新（Joseph 形式，保持对称正定）----
    const Mat5 IKH = Mat5::Identity() - K * H;
    P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();

    return aerial;
}

}  // namespace chassis_node
