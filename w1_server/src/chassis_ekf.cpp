#include "chassis_ekf.h"

#include <cmath>

namespace chassis_node {

ChassisEkf::ChassisEkf(const Params& p) : params_(p) { Reset(); }

void ChassisEkf::SetParams(const Params& p) {
    // 只替换参数，不动 x_ / P_。
    // 早期实现会在此把 P_ 打回 p0_*，导致运行中调一个噪声参数就让协方差突然跳变、
    // 滤波器不连续。p0_* 的语义是「滤波器复位时的初值」，只应在 Reset() 中生效。
    params_ = p;
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

ChassisEkf::Mat5 ChassisEkf::BuildQ(double dt, bool is_stopped, bool use_imu) const {
    // Q 参数以「每秒」定义，此处乘 dt 得到本步的过程噪声。
    // 若按「每步」使用（早期实现），同一组参数在不同发布频率下代表的物理噪声不同，
    // 滤波行为会随 odom_rate_hz 漂移。
    double scale = dt;
    if (is_stopped) {
        // 静止时「v=0, ω=0」这一模型几乎不会失配，过程噪声应随之收敛，
        // 否则位置协方差会在长时间停车期间无谓地一路膨胀。
        scale *= params_.q_scale_stopped;
    }

    // IMU 模式下 F(4,4)=0，P(4,4) 的预测值直接等于此项，故它代表的是陀螺测量噪声，
    // 而非「无 IMU 时角速度的过程扩散」。用同一个 q_omega 会把 ω 的不确定度高估两个
    // 数量级，既让发布的 twist 协方差失真，也削弱打滑检测的统计显著性。
    const double q_om = (use_imu && !is_stopped) ? params_.q_omega_imu : params_.q_omega;

    Mat5 Q = Mat5::Zero();
    Q(0, 0) = params_.q_xy    * scale;
    Q(1, 1) = params_.q_xy    * scale;
    Q(2, 2) = params_.q_theta * scale;
    Q(3, 3) = params_.q_v     * scale;
    Q(4, 4) = q_om            * scale;
    return Q;
}

bool ChassisEkf::Step(double dt, double v_enc, double omega_enc, double omega_z_imu, bool imu_available,
                      bool is_stopped) {
    Predict(dt, omega_z_imu, imu_available, is_stopped);
    is_aerial_ = Update(v_enc, omega_enc, is_stopped, imu_available);
    return is_aerial_;
}

void ChassisEkf::Predict(double dt, double omega_z, bool use_imu, bool is_stopped) {
    const double theta = x_(2);
    const double v     = x_(3);
    const double omega = x_(4);

    // 中点航向角（减小一阶非线性误差）
    // 与现有代码的 theta_mid 逻辑一致
    // 静止时强制 ω=0：否则 IMU 的零偏残差（初始化后仍有 ~1e-4 rad/s）会被逐帧积进
    // θ，而 θ 在量测矩阵 H 中没有对应行，这份误差永远得不到修正。
    const double omega_use  = is_stopped ? 0.0 : (use_imu ? omega_z : omega);
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
    if (use_imu && !is_stopped) {
        // IMU 直接给出 omega，状态中的 omega 不影响下一时刻 theta 和 omega
        F(2, 4) = 0.0;
        F(4, 4) = 0.0;
    } else {
        // theta 依赖 omega_state（纯编码器模式；静止时同样按此传播，
        // 使 ZUPT 的零速量测能通过 F 作用到 theta 的协方差上）
        F(2, 4) = dt;
        // omega 保持不变（F(4,4) = 1 已由 Identity 给出）
    }

    // ---- 协方差预测 ----
    P_ = F * P_ * F.transpose() + BuildQ(dt, is_stopped, use_imu);

    x_ = x_pred;
}

bool ChassisEkf::Update(double v_enc, double omega_enc, bool is_stopped, bool imu_driven) {
    // ---- 量测矩阵 H（2×5）----
    // z = [v_enc, omega_enc] = H * x + noise
    Mat25 H = Mat25::Zero();
    H(0, 3) = 1.0;  // v 对应 x[3]
    H(1, 4) = 1.0;  // omega 对应 x[4]

    // ---- 静止：零速量测 ----
    // 编码器读数此刻不提供额外信息，而 [v,ω]=[0,0] 是强先验，可同时压住速度状态
    // 与（经 F(2,4)=dt 传播的）航向协方差。ω 通道此时同样成立，故保持 2 维。
    if (is_stopped) {
        const Vec2 y = Vec2::Zero() - H * x_;
        Mat2 R = Mat2::Zero();
        R(0, 0) = params_.r_zupt;
        R(1, 1) = params_.r_zupt;
        ApplyUpdate<2>(H, y, R);
        // 静止期不做悬空/打滑判定：该判定比较的是编码器与 IMU 运动学的分歧，
        // 而此刻两者都指向"没在动"，判定无意义。
        return false;
    }

    // ---- 检测：始终基于 2 维新息 ----
    // Δω（编码器 ω 与预测 ω 之差）是转向打滑唯一有效的信号来源。即使下面不拿
    // 编码器 ω 去修正状态，这一路残差也必须照常参与统计检验。
    Vec2 z;
    z(0) = v_enc;
    z(1) = omega_enc;
    const Vec2 y = z - H * x_;

    Mat2 R_normal = Mat2::Zero();
    R_normal(0, 0) = params_.r_v;
    R_normal(1, 1) = params_.r_omega;

    const Mat2 S_test = H * P_ * H.transpose() + R_normal;
    const double det = S_test(0, 0) * S_test(1, 1) - S_test(0, 1) * S_test(1, 0);
    const bool aerial = (std::abs(det) > 1e-12) &&
                        ((y.transpose() * S_test.inverse() * y)(0, 0) > params_.aerial_chi2_threshold);

    // ---- 更新：IMU 模式只取 v 通道 ----
    if (imu_driven) {
        // ω 由 IMU 在 Predict 中直接给定，此处不再用编码器"修正"它。
        // 原实现把编码器 ω 当作 ω 的量测，P(4,4) 与 r_omega 同量级导致 K≈0.5，
        // 输出成了 IMU 与编码器的对半折中，既浪费了 IMU 的低噪声，也使发布的 ω
        // 不等于发布的 θ 的导数。
        Eigen::Matrix<double, 1, N> H1 = H.row(0);
        Eigen::Matrix<double, 1, 1> y1;
        y1(0, 0) = y(0);
        Eigen::Matrix<double, 1, 1> R1;
        R1(0, 0) = aerial ? params_.r_v_aerial : params_.r_v;
        ApplyUpdate<1>(H1, y1, R1);
        return aerial;
    }

    // ---- 纯编码器模式：v 与 ω 都由编码器提供 ----
    Mat2 R = Mat2::Zero();
    if (aerial) {
        R(0, 0) = params_.r_v_aerial;
        R(1, 1) = params_.r_omega_aerial;
    } else {
        R = R_normal;
    }
    ApplyUpdate<2>(H, y, R);
    return aerial;
}

}  // namespace chassis_node
