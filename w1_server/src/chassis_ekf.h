#pragma once

#include <Eigen/Dense>

#include <cmath>

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
        // 过程噪声 Q —— 单位为「每秒」，实际使用时乘 dt。
        // 早期版本按「每步」定义，导致滤波行为随发布频率漂移，且 q_theta 量级偏大
        // 5 个数量级（θ 无量测、只增不减，100 s 后 σ 达 180°，协方差失去意义）。
        double q_xy    = 1e-4;   // 位置扩散 (m²/s)，模型失配兜底项
        double q_theta = 1e-6;   // 航向扩散 (rad²/s)，由陀螺零偏残差量级主导
        double q_v     = 1.0;    // 线速度扩散 ((m/s)²/s)，按底盘加速度能力取
        double q_omega = 1.0;    // 角速度扩散 ((rad/s)²/s)，**仅无 IMU 时**起作用
        // IMU 模式下 ω 是外部输入而非被估计量：F(4,4)=0 使 P(4,4) 的预测值直接等于
        // 本项乘 dt，故它应当描述陀螺的测量噪声，而不是「无 IMU 时的角速度扩散」。
        double q_omega_imu = 1e-4;  // (rad/s)²/s ⇒ dt=0.01 时 σ_ω ≈ 0.001 rad/s

        // 编码器量测噪声 R（正常）
        double r_v     = 0.05;   // (m/s)²
        double r_omega = 0.01;   // (rad/s)²

        // 编码器量测噪声 R（悬空/打滑，忽略编码器）
        double r_v_aerial     = 100.0;
        double r_omega_aerial = 100.0;

        // Mahalanobis 距离阈值 χ²(2, 99%) = 9.21
        double aerial_chi2_threshold = 9.21;

        // 零速修正（ZUPT）：静止时以 [v,ω]=[0,0] 作为强量测
        double r_zupt = 1e-6;           // 零速量测噪声 ((m/s)² / (rad/s)²)
        double q_scale_stopped = 0.01;  // 静止时过程噪声缩放（静止模型近乎完美）

        // 初始协方差对角
        // odom frame 按定义即启动位姿：启动瞬间 base 与 odom 重合，位姿恒等于 0 且完全确定，
        // 故初始不确定度应趋近 0。取 1e-9 而非严格 0，只为避开可能的病态求逆路径；
        // 首次 Predict 后 Q·dt 即把 P 抬到 1e-8~1e-6 量级，不会卡在零。
        // 用 0.01（σ=0.1 m / 5.73°）会让初值淹没真实漂移：q_theta=1e-6 跑 91 s 才累积
        // 9.1e-5 rad²，仅占 0.01 的 0.9%，协方差曲线因此几乎是常数、不携带信息。
        double p0_xy    = 1.0e-9;
        double p0_theta = 1.0e-9;
        double p0_v     = 1.0;
        double p0_omega = 1.0;
    };

    explicit ChassisEkf(const Params& p);

    /** 重置状态为全零，协方差为初始值 */
    void Reset();

    /**
     * 用外部已知的位姿/速度灌初值，协方差回到 p0_*。
     * 用于运行中从纯推算切入 EKF：EKF 状态在关闭期间不推进，直接取用会把位姿
     * 跳回旧值（首次为原点）。位姿 p0 取极小值是对的——灌入的就是当前已知位姿。
     */
    void SetState(double x, double y, double theta, double v, double omega);

    void SetParams(const Params& p);

    /**
     * 单步融合：预测 + 量测更新
     *
     * @param dt            步长 (s)
     * @param v_enc         编码器线速度（已乘 odom_factor）(m/s)
     * @param omega_enc     编码器角速度 (rad/s)
     * @param omega_z_imu   IMU yaw rate（已乘 imu_factor，已去偏，已旋至水平系）(rad/s)
     * @param imu_available 当前 IMU 数据是否可用
     * @param is_stopped    整车静止（编码器 ∧ IMU 综合判定）。为 true 时启用 ZUPT：
     *                      预测不积分 IMU 零偏残差，更新以零速为强量测。
     *                      不加此约束时静止期零偏残差会持续积进 θ，而 θ 无量测永不修正。
     * @return true 若检测到悬空/打滑（静止期不做该判定）
     */
    bool Step(double dt,
              double v_enc, double omega_enc,
              double omega_z_imu, bool imu_available,
              bool is_stopped = false);

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
     * @param use_imu    true → 用 omega_z_imu 驱动 theta/omega；false → 匀速假设
     * @param is_stopped true → 角速度按 0 传播（ZUPT），过程噪声按 q_scale_stopped 缩放
     */
    void Predict(double dt, double omega_z, bool use_imu, bool is_stopped);

    /**
     * 量测更新步。
     *
     * 「检测」与「更新」在此分离：
     *   - 悬空/打滑判定**始终**基于 2 维新息 [Δv, Δω]。编码器 ω 与 IMU ω 的分歧正是
     *     转向打滑的信号，也是当前唯一有效的检测通道，不能因为不拿它修正状态就丢掉。
     *   - 状态更新在 IMU 模式下只取 v 一行：ω 由 IMU 直接给定，再用编码器去"修正"它
     *     只会把高带宽低噪声的 IMU 往编码器方向拉（原实现 K≈0.5，等于对半折中），
     *     并导致发布的 ω 不再是发布的 θ 的导数。
     *
     * @param is_stopped true → 以 [0,0] 为零速量测（R=r_zupt），不做悬空判定
     * @param imu_driven true → 本步 ω 来自 IMU，更新仅使用 v 通道
     * @return Mahalanobis 距离是否超阈值（aerial flag）
     */
    bool Update(double v_enc, double omega_enc, bool is_stopped, bool imu_driven);

    /// 组装过程噪声 Q（按 dt 缩放；静止时再乘 q_scale_stopped；IMU 模式下 ω 用 q_omega_imu）
    Mat5 BuildQ(double dt, bool is_stopped, bool use_imu) const;

    /// 通用的卡尔曼更新（Joseph 形式），量测维度 M 由调用方决定
    template <int M>
    void ApplyUpdate(const Eigen::Matrix<double, M, N>& H,
                     const Eigen::Matrix<double, M, 1>& y,
                     const Eigen::Matrix<double, M, M>& R) {
        const Eigen::Matrix<double, M, M> S = H * P_ * H.transpose() + R;
        const Eigen::Matrix<double, N, M> K = P_ * H.transpose() * S.inverse();
        x_ += K * y;
        x_(2) = std::atan2(std::sin(x_(2)), std::cos(x_(2)));
        const Mat5 IKH = Mat5::Identity() - K * H;
        P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
    }

    Vec5   x_;           // 状态均值
    Mat5   P_;           // 状态协方差
    Params params_;
    bool   is_aerial_ = false;
};

/**
 * 纯航位推算（非 EKF 模式）的协方差——EKF 的退化形态。
 *
 * 位姿方差按与 EKF 同一套 Q 累积：无量测修正故只增不减，静止时不增（与 ZUPT 语义一致）。
 * twist 方差直接取量测噪声：v 用 r_v；ω 由 IMU 提供时用 q_omega_imu·dt（EKF 中 IMU 驱动
 * 下的稳态量级），否则用 r_omega。
 * 两种模式因此共用参数与量纲——标定出的 r_v / q_theta 两边通用，NEES 可跨模式比较；
 * 也避免了非 EKF 模式发全 0 协方差被上游当成「无限精确」。
 */
struct DeadReckoningCov {
    double var_xy    = 0.0;
    double var_theta = 0.0;
    double last_dt   = 0.01;

    void Reset(const ChassisEkf::Params& p) {
        var_xy    = p.p0_xy;
        var_theta = p.p0_theta;
    }
    void Propagate(const ChassisEkf::Params& p, double dt, bool is_stopped) {
        if (dt <= 0.0) return;
        last_dt = dt;
        if (is_stopped) return;
        var_xy    += p.q_xy    * dt;
        var_theta += p.q_theta * dt;
    }
    /** 从 EKF 当前协方差接续（EKF → 纯推算切换时保持连续，不产生阶跃） */
    void SeedFrom(const ChassisEkf::Mat5& P) {
        var_xy    = P(0, 0) > P(1, 1) ? P(0, 0) : P(1, 1);
        var_theta = P(2, 2);
    }
    double TwistVarV(const ChassisEkf::Params& p) const { return p.r_v; }
    double TwistVarOmega(const ChassisEkf::Params& p, bool imu_available) const {
        return imu_available ? p.q_omega_imu * last_dt : p.r_omega;
    }
};

}  // namespace chassis_node
