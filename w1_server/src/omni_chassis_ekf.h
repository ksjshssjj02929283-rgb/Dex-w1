#pragma once

#include <Eigen/Dense>
#include <cmath>

#include "chassis_ekf.h"  // 共用 Params（含全向专用项）与 DeadReckoningCov

namespace chassis_node {

/**
 * 2D 全向（舵轮）底盘 EKF 里程计融合
 *
 * 状态向量: x = [px, py, theta, vx, vy, omega]^T (6D)
 *   px, py : odom 坐标系位置 (m)
 *   theta  : 航向角 (rad), [-π, π]
 *   vx, vy : 机体系线速度 (m/s)——全向底盘可独立侧移，vy 是真实状态而非约束
 *   omega  : 机体 yaw rate (rad/s)
 *
 * 过程输入 (IMU):  omega_z_imu (rad/s)
 * 量测   (编码器): [vx_enc, vy_enc, omega_enc]（四舵轮速度/转向角反解）
 *
 * 与差速 ChassisEkf 的差别只在 vy 这一维：
 *   - 位置预测 px += (vx cosθ − vy sinθ)·dt，py += (vx sinθ + vy cosθ)·dt
 *   - 悬空/打滑检测用 3 维新息 [Δvx, Δvy, Δω]，阈值取 χ²(3)
 *   - IMU 模式下状态更新只取 vx、vy 两行（ω 由 IMU 直接给定，同差速的"只取 v 行"）
 * 其余设计（Q 按每秒、ZUPT、检测/更新分离、Joseph 形式、p0 语义）与差速完全一致，
 * 参数共用 ChassisEkf::Params，仅多出 q_vy / r_vy / r_vy_aerial / p0_vy / aerial_chi2_threshold_3d。
 * vy ≡ 0 时它退化为差速 EKF（单测 CovarianceMatchesDiffDriveWhenNoLateralMotion 锁住这一点）。
 */
class OmniChassisEkf {
public:
    static constexpr int N = 6;
    using Params = ChassisEkf::Params;
    using Vec6 = Eigen::Matrix<double, N, 1>;
    using Mat6 = Eigen::Matrix<double, N, N>;
    using Vec3 = Eigen::Matrix<double, 3, 1>;
    using Mat3 = Eigen::Matrix<double, 3, 3>;
    using Mat36 = Eigen::Matrix<double, 3, N>;

    explicit OmniChassisEkf(const Params& p);

    /** 重置状态为全零，协方差为初始值 */
    void Reset();

    /** 用外部已知的位姿/速度灌初值（运行中切入 EKF 用），协方差回到 p0_* */
    void SetState(double x, double y, double theta, double vx, double vy, double omega);

    /** 只替换参数，不动状态与协方差（与 ChassisEkf::SetParams 同语义） */
    void SetParams(const Params& p);

    /**
     * 单步融合：预测 + 量测更新
     *
     * @param dt            步长 (s)
     * @param vx_enc        编码器机体 x 向线速度（已乘 odom_factor）(m/s)
     * @param vy_enc        编码器机体 y 向线速度（已乘 odom_factor）(m/s)
     * @param omega_enc     编码器反解角速度 (rad/s)
     * @param omega_z_imu   IMU yaw rate（已乘 imu_factor，已去偏，已旋至水平系）(rad/s)
     * @param imu_available 当前 IMU 数据是否可用
     * @param is_stopped    整车静止 → ZUPT：预测不积零偏残差，更新以 [0,0,0] 为强量测
     * @return true 若检测到悬空/打滑（静止期不做该判定）
     */
    bool Step(double dt,
              double vx_enc,
              double vy_enc,
              double omega_enc,
              double omega_z_imu,
              bool imu_available,
              bool is_stopped = false);

    double GetX() const { return x_(0); }
    double GetY() const { return x_(1); }
    double GetTheta() const { return x_(2); }
    double GetVx() const { return x_(3); }
    double GetVy() const { return x_(4); }
    double GetOmega() const { return x_(5); }
    bool IsAerial() const { return is_aerial_; }

    /** 6×6 状态协方差（供 PublishOdometry 填充消息：0,1,2 位姿；3,4,5 = vx, vy, ω） */
    const Mat6& GetCovariance() const { return P_; }

private:
    void Predict(double dt, double omega_z, bool use_imu, bool is_stopped);
    bool Update(double vx_enc, double vy_enc, double omega_enc, bool is_stopped, bool imu_driven);
    Mat6 BuildQ(double dt, bool is_stopped, bool use_imu) const;

    template <int M>
    void ApplyUpdate(const Eigen::Matrix<double, M, N>& H,
                     const Eigen::Matrix<double, M, 1>& y,
                     const Eigen::Matrix<double, M, M>& R) {
        const Eigen::Matrix<double, M, M> S = H * P_ * H.transpose() + R;
        const Eigen::Matrix<double, N, M> K = P_ * H.transpose() * S.inverse();
        x_ += K * y;
        x_(2) = std::atan2(std::sin(x_(2)), std::cos(x_(2)));
        const Mat6 IKH = Mat6::Identity() - K * H;
        P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
    }

    Vec6 x_;
    Mat6 P_;
    Params params_;
    bool is_aerial_ = false;
};

}  // namespace chassis_node
