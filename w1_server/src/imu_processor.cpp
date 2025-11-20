#include "imu_processor.h"

#include <Eigen/Dense>
#include <builtin_interfaces/msg/time.hpp>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

namespace chassis_node
{
ImuProcessor::ImuProcessor() {}

void ImuProcessor::SetParam(const int max_buff_size, const double acc_excite_threshold, const double gyro_excite_threshold,
                            const double reinit_duration)
{
    max_buff_size_ = max_buff_size;
    acc_excite_threshold_ = acc_excite_threshold;
    gyro_excite_threshold_ = gyro_excite_threshold;
    reinit_duration_ = reinit_duration;
    RCLCPP_INFO(rclcpp::get_logger("ImuProcessor"),
                "Set IMU processor param: max_buff_size=%d, acc_excite_threshold=%f, gyro_excite_threshold=%f, reinit_duration=%f", max_buff_size_,
                acc_excite_threshold_, gyro_excite_threshold_, reinit_duration_);
    is_first_initialized_ = false;  // 设置参数后需要重新初始化
}

bool ImuProcessor::SetGravityMagnitude(const double g)
{
    if (g <= 0.0)
    {
        RCLCPP_WARN(rclcpp::get_logger("ImuProcessor"), "Invalid gravity magnitude: %f", g);
        return false;
    }
    gravity_vector_ = Eigen::Vector3d(0.0, 0.0, g);
    gravity_magnitude_ = g;
    RCLCPP_INFO(rclcpp::get_logger("ImuProcessor"), "Set gravity magnitude: %f", g);
    return true;
}

bool ImuProcessor::InitImu()
{
    if (imu_data_buff_.empty())
    {
        return false;
    }

    if (!CanComputeImuBias())
    {
        return false;
    }

    if (!ComputeImuBias())
    {
        return false;
    }

    if (!is_first_initialized_)
    {
        is_first_initialized_ = true;
    }

    is_need_initialize_ = false;
    last_initailized_time_ = std::make_shared<std::chrono::steady_clock::time_point>(std::chrono::steady_clock::now());
    RCLCPP_INFO(rclcpp::get_logger("ImuProcessor"), "IMU init success.");
    return true;
}

std::optional<sensor_msgs::msg::Imu> ImuProcessor::GetRefinedImuData(const sensor_msgs::msg::Imu& imu_data)
{
    if (!is_first_initialized_)
    {
        return std::nullopt;
    }

    // 1) 取出原始量测为 Eigen 向量
    Eigen::Vector3d acc_raw(imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z);

    Eigen::Vector3d gyr_raw(imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z);

    // 2) 去零偏（IMU 机体系 I0 下）
    Eigen::Vector3d acc_corr = acc_raw - acc_bias_;
    Eigen::Vector3d gyr_corr = gyr_raw - gyro_bias_;

    // 3) 旋回到世界系（水平参考系）：v_G = R_I0toG * v_I0
    //    R_I0toG = (R_GtoI0)^T

    Eigen::Vector3d acc_level_in_G = R_ItoG_ * acc_corr;  // 世界系下的总加速度（含重力）
    Eigen::Vector3d gyr_level_in_G = R_ItoG_ * gyr_corr;  // 世界系下的角速度

    // 4) 去重力，得到“线加速度”（世界系）
    Eigen::Vector3d lin_acc_in_G = acc_level_in_G - gravity_vector_;  // gravity_vector_ e.g. [0,0,-9.81]

    // 5) 填回消息
    sensor_msgs::msg::Imu res_imu = imu_data;  // 复制原头等元数据

    // 如果你希望输出“线加速度（去重力）”，用 lin_acc_in_G：
    // res_imu.linear_acceleration.x = lin_acc_in_G.x();
    // res_imu.linear_acceleration.y = lin_acc_in_G.y();
    // res_imu.linear_acceleration.z = lin_acc_in_G.z();

    // 如果你希望输出“未去重力但已旋到世界系的加速度”，改为：
    res_imu.linear_acceleration.x = acc_level_in_G.x();
    res_imu.linear_acceleration.y = acc_level_in_G.y();
    res_imu.linear_acceleration.z = acc_level_in_G.z();

    // 角速度改为世界系表达：
    res_imu.angular_velocity.x = gyr_level_in_G.x();
    res_imu.angular_velocity.y = gyr_level_in_G.y();
    res_imu.angular_velocity.z = gyr_level_in_G.z();

    return res_imu;
}

bool ImuProcessor::CanComputeImuBias()
{
    if (imu_data_buff_.size() >= static_cast<size_t>(max_buff_size_))
    {
        return true;
    }

    return false;
}

bool ImuProcessor::ComputeImuBias()
{
    // 快照副本，避免长时间持锁
    std::deque<sensor_msgs::msg::Imu> buf;
    {
        std::lock_guard<std::mutex> lk(buf_mutex_);
        if (imu_data_buff_.size() < static_cast<size_t>(max_buff_size_))
            return false;
        buf.assign(imu_data_buff_.end() - max_buff_size_, imu_data_buff_.end());
    }

    const size_t N = buf.size();
    Eigen::Vector3d acc_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyr_sum = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d> acc_samples;
    acc_samples.reserve(N);
    std::vector<Eigen::Vector3d> gyr_samples;
    gyr_samples.reserve(N);

    for (const auto& meas : buf)
    {
        Eigen::Vector3d acc(meas.linear_acceleration.x, meas.linear_acceleration.y, meas.linear_acceleration.z);
        Eigen::Vector3d omega(meas.angular_velocity.x, meas.angular_velocity.y, meas.angular_velocity.z);
        acc_sum += acc;
        gyr_sum += omega;
        acc_samples.push_back(acc);
        gyr_samples.push_back(omega);
    }
    const Eigen::Vector3d acc_avg = acc_sum / double(N);
    const Eigen::Vector3d gyr_avg = gyr_sum / double(N);

    // 标准差
    auto compute_std = [](const std::vector<Eigen::Vector3d>& xs, const Eigen::Vector3d& mean) -> Eigen::Vector3d
    {
        Eigen::Vector3d v = Eigen::Vector3d::Zero();
        for (const auto& x : xs)
        {
            Eigen::Vector3d d = x - mean;
            v += d.cwiseProduct(d);
        }
        v /= double(std::max<size_t>(1, xs.size() - 1));
        return v.cwiseSqrt();
    };
    const Eigen::Vector3d acc_std = compute_std(acc_samples, acc_avg);
    const Eigen::Vector3d gyr_std = compute_std(gyr_samples, gyr_avg);

    // 使用预设重力大小做静止检查
    const double g_preset = gravity_vector_.norm();  // 预设值，如 9.7887
    const double acc_mag_avg = acc_avg.norm();       // 实测平均加速度模值
    const double acc_mag_bias = std::abs(acc_mag_avg - g_preset);

    // 参数（建议放到 SetParam 或配置）
    const double gyr_mean_thresh = 1e-3;     // rad/s
    const double acc_mag_bias_thresh = 0.2;  // m/s^2

    // 静止判据
    if (acc_std.maxCoeff() > acc_excite_threshold_ || gyr_std.maxCoeff() > gyro_excite_threshold_ || std::abs(gyr_avg.x()) > gyr_mean_thresh ||
        std::abs(gyr_avg.y()) > gyr_mean_thresh || std::abs(gyr_avg.z()) > gyr_mean_thresh || acc_mag_bias > acc_mag_bias_thresh)
    {
        LOG_WARN << "IMU not static enough for bias init. "
                 << "acc_std=[" << acc_std.x() << ", " << acc_std.y() << ", " << acc_std.z() << "], "
                 << "gyr_std=[" << gyr_std.x() << ", " << gyr_std.y() << ", " << gyr_std.z() << "], "
                 << "|g_est|-g_preset=" << acc_mag_bias << ", "
                 << "gyr_avg=[" << gyr_avg.x() << ", " << gyr_avg.y() << ", " << gyr_avg.z() << "]";
        // TODO: 这里默认IMU是100hz，可以改成用时间戳计算
        LOG_WARN << "Reinitialize IMU after " << (max_buff_size_ / 100.0) << " seconds of static.";
        is_need_initialize_ = true;
        imu_data_buff_.clear();  // 清空数据，重新积累
        return false;
    }

    // ==================== 重估重力方向和大小 ====================
    // 1) 从实测数据中提取重力方向
    Eigen::Vector3d z = acc_avg.normalized();  // 重力方向（单位向量）

    // 2) 从实测数据中提取重力大小
    const double g_estimated = acc_avg.norm();  // 重估的重力大小

    RCLCPP_INFO(rclcpp::get_logger("ImuProcessor"), "Gravity estimation: preset=%.4f m/s², estimated=%.4f m/s², diff=%.4f m/s²", g_preset,
                g_estimated, std::abs(g_estimated - g_preset));

    // 3) 构造稳健的正交基（用于定义世界坐标系）

    // 选择与 z 不共线的参考向量
    Eigen::Vector3d e = (std::abs(z.x()) < 0.9) ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();

    // 去掉 z 分量并归一化
    Eigen::Vector3d x = (e - z * (z.dot(e))).normalized();
    Eigen::Vector3d y = z.cross(x).normalized();

    // 重新正交化（消除累计误差）
    x = y.cross(z);

    // 4) 组装并用极分解投影到 SO(3)
    Eigen::Matrix3d R;
    R.col(0) = x;
    R.col(1) = y;
    R.col(2) = z;

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d Ro = svd.matrixU() * svd.matrixV().transpose();
    if (Ro.determinant() < 0)
    {
        Eigen::Matrix3d U = svd.matrixU();
        U.col(2) *= -1;
        Ro = U * svd.matrixV().transpose();
    }

    // 5) 保存旋转与偏置
    R_GtoI_ = Ro;
    R_ItoG_ = R_GtoI_.transpose();

    gyro_bias_ = gyr_avg;

    // 更新重力向量为重估值（用于后续去重力计算）
    gravity_vector_ = Eigen::Vector3d(0.0, 0.0, g_estimated);

    // 使用实测的重力向量（方向+大小）计算加速度零偏
    // gravity_in_world = [0, 0, g_estimated]
    Eigen::Vector3d gravity_estimated_vec(0.0, 0.0, g_estimated);
    acc_bias_ = acc_avg - R_GtoI_ * gravity_estimated_vec;

    LOG_WARN << "Compute imu bias success. "
             << "acc_std=[" << acc_std.x() << ", " << acc_std.y() << ", " << acc_std.z() << "], "
             << "gyr_std=[" << gyr_std.x() << ", " << gyr_std.y() << ", " << gyr_std.z() << "], "
             << "acc_bias=[" << acc_bias_.x() << ", " << acc_bias_.y() << ", " << acc_bias_.z() << "], "
             << "gyro_bias=[" << gyro_bias_.x() << ", " << gyro_bias_.y() << ", " << gyro_bias_.z() << "], "
             << "g_estimated=" << g_estimated << " m/s²";
    return true;
}

void ImuProcessor::TrimImuData()
{
    while (imu_data_buff_.size() > static_cast<size_t>(max_buff_size_))
    {
        imu_data_buff_.pop_front();
    }
}

// 将 builtin_interfaces::msg::Time 转为 double 秒
inline double to_seconds(const builtin_interfaces::msg::Time& t)
{
    // 注意 nanosec 的范围是 [0, 1e9)
    return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

// 返回 a - b 的秒差
inline double time_diff_seconds(const builtin_interfaces::msg::Time& a, const builtin_interfaces::msg::Time& b)
{
    // 直接用 double 秒做差
    return to_seconds(a) - to_seconds(b);
}

void ImuProcessor::AddImuData(const sensor_msgs::msg::Imu& imu_data)
{
    {
        std::unique_lock<std::mutex> lock(buf_mutex_);

        const auto& t_curr = imu_data.header.stamp;

        // 乱序/回拨检查：先判空再比较
        if (!imu_data_buff_.empty())
        {
            const auto& t_last = imu_data_buff_.back().header.stamp;
            const double buff_dt = time_diff_seconds(t_curr, t_last);  // t_curr - t_last
            if (buff_dt < 0.0)
            {
                // 回拨，直接丢弃
                return;
            }
        }

        // 上电 5s 内不使用：以首帧时间为基准
        if (is_need_initialize_)
        {
            if (!start_time_.has_value())
            {
                start_time_ = t_curr;  // 保存 stamp 值本身
            }
            const double dt = time_diff_seconds(t_curr, start_time_.value());
            if (dt < 5.0)
            {
                return;
            }
        }

        imu_data_buff_.push_back(imu_data);
        TrimImuData();
    }

    MaybeNeedInit();

    // 初始化IMU
    if (is_need_initialize_)
    {
        InitImu();
    }
}

void ImuProcessor::MaybeNeedInit()
{
    if (!is_first_initialized_)
    {
        is_need_initialize_ = true;
        return;
    }
    // TODO: linfeng.guo 增加定期初始化的操作
    // if (last_initailized_time_ == nullptr) {
    //   return;
    // }

    // if (reinit_duration_ < 0) {
    //   return;
    // }

    // std::chrono::steady_clock::time_point curr_time = std::chrono::steady_clock::now();
    // double delta_time = std::chrono::duration<double>(curr_time - *last_initailized_time_).count();

    // if (delta_time > reinit_duration_) {
    //   is_need_initialize_ = true;
    //   RCLCPP_WARN(rclcpp::get_logger("ImuProcessor"), "Need reinit imu");
    // }
}

}  // namespace chassis_node
