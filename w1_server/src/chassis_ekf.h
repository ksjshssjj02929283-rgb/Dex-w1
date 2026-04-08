#pragma once

#include <Eigen/Dense>

namespace chassis_node {

/**
 * 2D 差速底盘 EKF 里程计融合
 *
 * 状态向量: x = [px, py, theta, v, omega]^T (5D)
 *   px, py : odom 坐标系位置 (m)
 *   theta  : 航向角 (rad), [-π, π]
 *   v      : 机体前向线速度 (m/s)
 *   omega  : 机体 yaw rate (rad/s)
 *
 * 过程输入 (IMU):  omega_z_imu (rad/s)
 * 量测   (编码器): [v_enc, omega_enc]
 *
 * 悬空/打滑检测: Mahalanobis 距离检验
 *   正常: D² ~ χ²(2)
 *   异常: D² > threshold → 放大量测噪声，协方差膨胀
 */
class ChassisEkf {
public:
    static constexpr int N = 5;
    using Vec5  = Eigen::Matrix<double, N, 1>;
    using Mat5  = Eigen::Matrix<double, N, N>;
    using Vec2  = Eigen::Matrix<double, 2, 1>;
    using Mat2  = Eigen::Matrix<double, 2, 2>;
    using Mat25 = Eigen::Matrix<double, 2, N>;

    struct Params {
        // 过程噪声 Q（每步）
        double q_xy    = 1e-4;   // 位置扩散 (m²/step)
        double q_theta = 1e-3;   // 航向扩散 (rad²/step)，IMU 模式下影响极小
        double q_v     = 0.04;   // 线速度扩散 ((m/s)²/step)
        double q_omega = 0.01;   // 角速度扩散 ((rad/s)²/step)

        // 编码器量测噪声 R（正常）
        double r_v     = 0.05;   // (m/s)²
        double r_omega = 0.01;   // (rad/s)²

        // 编码器量测噪声 R（悬空/打滑，忽略编码器）
        double r_v_aerial     = 100.0;
        double r_omega_aerial = 100.0;

        // Mahalanobis 距离阈值 χ²(2, 99%) = 9.21
        double aerial_chi2_threshold = 9.21;

        // 初始协方差对角
        double p0_xy    = 0.01;
        double p0_theta = 0.01;
        double p0_v     = 1.0;
        double p0_omega = 1.0;
    };

    explicit ChassisEkf(const Params& p);

    /** 重置状态为全零，协方差为初始值 */
    void Reset();

    void SetParams(const Params& p);

    /**
     * 单步融合：预测 + 量测更新
     *
     * @param dt            步长 (s)
     * @param v_enc         编码器线速度（已乘 odom_factor）(m/s)
     * @param omega_enc     编码器角速度 (rad/s)
     * @param omega_z_imu   IMU yaw rate（已乘 imu_factor，已去偏，已旋至水平系）(rad/s)
     * @param imu_available 当前 IMU 数据是否可用
     * @return true 若检测到悬空/打滑
     */
    bool Step(double dt,
              double v_enc, double omega_enc,
              double omega_z_imu, bool imu_available);

    double GetX()     const { return x_(0); }
    double GetY()     const { return x_(1); }
    double GetTheta() const { return x_(2); }
    double GetV()     const { return x_(3); }
    double GetOmega() const { return x_(4); }
    bool   IsAerial() const { return is_aerial_; }

    /** 5×5 状态协方差（供 PublishOdometry 填充消息） */
    const Mat5& GetCovariance() const { return P_; }

private:
    /**
     * 预测步：运动学传播 + 协方差预测
     * @param use_imu true → 用 omega_z_imu 驱动 theta/omega；false → 匀速假设
     */
    void Predict(double dt, double omega_z, bool use_imu);

    /**
     * 量测更新步
     * @return Mahalanobis 距离是否超阈值（aerial flag）
     */
    bool Update(double v_enc, double omega_enc);

    Vec5   x_;           // 状态均值
    Mat5   P_;           // 状态协方差
    Params params_;
    bool   is_aerial_ = false;
};

}  // namespace chassis_node
