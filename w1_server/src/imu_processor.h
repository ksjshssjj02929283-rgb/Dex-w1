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

namespace chassis_node
{

inline Eigen::Matrix<double, 3, 3> skew_x(const Eigen::Matrix<double, 3, 1>& w)
{
    Eigen::Matrix<double, 3, 3> w_x;
    w_x << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
    return w_x;
}

class ImuProcessor
{
   public:
    ImuProcessor();
    ~ImuProcessor() = default;

    // IMU静止初始化
    bool InitImu();

    void AddImuData(const sensor_msgs::msg::Imu& imu_data);

    void MaybeNeedInit();

    std::optional<sensor_msgs::msg::Imu> GetRefinedImuData(const sensor_msgs::msg::Imu& imu_data);

    void SetParam(const int max_buff_size = 1500, const double acc_excite_threshold = 0.015, const double gyro_excite_threshold = 0.002,
                  const double reinit_duration = 3600.0);
    bool SetGravityMagnitude(const double g);

   private:
    bool CanComputeImuBias();
    bool ComputeImuBias();
    void TrimImuData();

   private:
    int max_buff_size_ = 1500;
    double acc_excite_threshold_ = 0.007;  // IMU激励阈值，用于判断IMU是否静止
    double gyro_excite_threshold_ = 0.002;
    double gravity_magnitude_ = 9.7887;  // m/s^2

    bool is_first_initialized_ = false;
    bool is_need_initialize_ = false;

    Eigen::Vector3d gravity_vector_ = Eigen::Vector3d(0.0, 0.0, 9.7887);
    // 加速度计的偏置
    Eigen::Vector3d acc_bias_ = Eigen::Vector3d::Zero();
    // 修正陀螺仪的偏置
    Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();

    std::deque<sensor_msgs::msg::Imu> imu_data_buff_;
    std::mutex buf_mutex_;

    std::optional<builtin_interfaces::msg::Time> start_time_;

    Eigen::Matrix3d R_GtoI_;
    Eigen::Matrix3d R_ItoG_;

    double reinit_duration_ = 3600.0;
    std::shared_ptr<std::chrono::steady_clock::time_point> last_initailized_time_ = nullptr;
};

}  // namespace chassis_node
