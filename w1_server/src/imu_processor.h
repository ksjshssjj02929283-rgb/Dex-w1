#pragma once

#include <Eigen/Core>
#include <deque>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <tuple>

#include "robot_server_lib/NanoLog.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace chassis_node {

inline Eigen::Matrix<double, 3, 3> skew_x(const Eigen::Matrix<double, 3, 1>& w) {
    Eigen::Matrix<double, 3, 3> w_x;
    w_x << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
    return w_x;
}

class ImuProcessor {
public:
    ImuProcessor();
    ~ImuProcessor() = default;

    // IMU静止初始化
    bool InitImu();

    /**
     * 追加一帧 IMU 数据。
     *
     * @param wheel_stopped 编码器判定的轮子静止状态（静止初始化的第二信源）。
     *        恒定角速度、恒定线加速度、匀速直线三种工况在 IMU 侧的方差与模值都与
     *        静止无异，纯 IMU 判据无法区分，必须靠轮速拆开。为 false 时丢弃已积累
     *        的静止段并停止积累。
     */
    void AddImuData(const sensor_msgs::msg::Imu& imu_data, bool wheel_stopped = true);

    void MaybeNeedInit();

    std::optional<sensor_msgs::msg::Imu> GetRefinedImuData(const sensor_msgs::msg::Imu& imu_data);

    void SetParam(const int max_buff_size = 500,
                  const double acc_excite_threshold = 0.04,
                  const double gyro_excite_threshold = 0.004,
                  const double reinit_duration = 3600.0,
                  const double gyro_bias_warn_threshold = 0.01,
                  const double acc_magnitude_tolerance = 0.2);
    bool SetGravityMagnitude(const double g);

    /// 上次初始化是否在斜坡上完成（倾角 > kSlopeWarnDeg）。true 表示 R_ItoG_ 对齐的是
    /// 斜坡法向而非真实水平面，转到平地行驶后 yaw 会被 ωx 污染。
    bool IsLastInitOnSlope() const { return is_last_init_on_slope_; }

    /// 上次初始化估计出的倾角（deg）。未初始化时为 0。
    double GetLastInitTiltDeg() const { return last_init_tilt_deg_; }

private:
    bool CanComputeImuBias();
    bool ComputeImuBias();
    /// @warning 调用方必须已持有 buf_mutex_；本函数自身不加锁（勿在内部补锁，会死锁）
    void TrimImuData();

private:
    /// 初始化倾角告警门限（deg）。超过则认为在斜坡上初始化。
    static constexpr double kSlopeWarnDeg = 3.0;

    int max_buff_size_ = 500;
    double acc_excite_threshold_ = 0.04;   // 加速度标准差阈值，判断 IMU 是否受激励
    double gyro_excite_threshold_ = 0.004;  // 角速度标准差阈值
    // 陀螺零偏告警门限：静止时 gyr_avg 即零偏本身，超过此值只告警不拒绝初始化。
    // 注意它不是静止判据——把待估的零偏当判据量正是 R01 的根因。
    double gyro_bias_warn_threshold_ = 0.01;
    // |‖acc_avg‖ - g_preset| 容差，用于识别静止期间残留的线加速度
    double acc_magnitude_tolerance_ = 0.2;
    double gravity_magnitude_ = 9.7887;  // m/s^2

    bool is_first_initialized_ = false;
    bool is_need_initialize_ = false;
    bool is_last_init_on_slope_ = false;
    double last_init_tilt_deg_ = 0.0;

    Eigen::Vector3d gravity_vector_ = Eigen::Vector3d(0.0, 0.0, 9.7887);
    // 加速度计的偏置
    Eigen::Vector3d acc_bias_ = Eigen::Vector3d::Zero();
    // 修正陀螺仪的偏置
    Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();

    std::deque<sensor_msgs::msg::Imu> imu_data_buff_;
    std::mutex buf_mutex_;

    // 静止段起始时刻（5s 余振过滤计时用）
    std::optional<builtin_interfaces::msg::Time> start_time_;
    // 上一帧的时间戳，供乱序/回拨检查。独立于 imu_data_buff_：buff 只在需要初始化时
    // 才积累，若拿 buff.back() 当锚点，初始化完成后回拨保护就会随 buff 一起失效。
    std::optional<builtin_interfaces::msg::Time> last_stamp_;

    // Eigen 默认构造不初始化，必须显式置单位阵：GetRefinedImuData 虽有
    // is_first_initialized_ 门禁，但未初始化的矩阵参与运算属未定义行为。
    Eigen::Matrix3d R_GtoI_ = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R_ItoG_ = Eigen::Matrix3d::Identity();

    double reinit_duration_ = 3600.0;
    std::shared_ptr<std::chrono::steady_clock::time_point> last_initailized_time_ = nullptr;
};

}  // namespace chassis_node
