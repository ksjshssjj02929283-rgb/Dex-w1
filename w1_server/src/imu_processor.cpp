#include "imu_processor.h"

#include <cstdio>

#include <Eigen/Dense>
#include <builtin_interfaces/msg/time.hpp>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

namespace chassis_node {
ImuProcessor::ImuProcessor() {}

void ImuProcessor::SetParam(const int max_buff_size,
                            const double acc_excite_threshold,
                            const double gyro_excite_threshold,
                            const double reinit_duration,
                            const double gyro_bias_warn_threshold,
                            const double acc_magnitude_tolerance) {
    // 只有影响静止判据的参数变化才需要重新初始化；告警门限与重初始化间隔不影响
    // 已估出的零偏，运行中改它们不应中断里程计输出。
    const bool criteria_changed =
        (max_buff_size_ != max_buff_size) || (acc_excite_threshold_ != acc_excite_threshold) ||
        (gyro_excite_threshold_ != gyro_excite_threshold) || (acc_magnitude_tolerance_ != acc_magnitude_tolerance);

    max_buff_size_ = max_buff_size;
    acc_excite_threshold_ = acc_excite_threshold;
    gyro_excite_threshold_ = gyro_excite_threshold;
    reinit_duration_ = reinit_duration;
    gyro_bias_warn_threshold_ = gyro_bias_warn_threshold;
    acc_magnitude_tolerance_ = acc_magnitude_tolerance;

    RCLCPP_INFO(rclcpp::get_logger("ImuProcessor"),
                "Set IMU processor param: max_buff_size=%d, acc_excite_threshold=%f, gyro_excite_threshold=%f, "
                "reinit_duration=%f, gyro_bias_warn_threshold=%f, acc_magnitude_tolerance=%f",
                max_buff_size_, acc_excite_threshold_, gyro_excite_threshold_, reinit_duration_,
                gyro_bias_warn_threshold_, acc_magnitude_tolerance_);

    if (criteria_changed && is_first_initialized_) {
        RCLCPP_WARN(rclcpp::get_logger("ImuProcessor"),
                    "Static criteria changed, IMU re-initialization required. Odometry will pause until the "
                    "robot stays still again.");
        is_first_initialized_ = false;
    }
}

bool ImuProcessor::SetGravityMagnitude(const double g) {
    if (g <= 0.0) {
        RCLCPP_WARN(rclcpp::get_logger("ImuProcessor"), "Invalid gravity magnitude: %f", g);
        return false;
    }
    gravity_vector_ = Eigen::Vector3d(0.0, 0.0, g);
    gravity_magnitude_ = g;
    RCLCPP_INFO(rclcpp::get_logger("ImuProcessor"), "Set gravity magnitude: %f", g);
    return true;
}

bool ImuProcessor::InitImu() {
    // 数量是否够由 CanComputeImuBias() 统一判定（它持锁）；此处不再无锁读 buff.empty()
    if (!CanComputeImuBias()) {
        return false;
    }

    if (!ComputeImuBias()) {
        return false;
    }

    if (!is_first_initialized_) {
        is_first_initialized_ = true;
    }

    is_need_initialize_ = false;
    last_initailized_time_ = std::make_shared<std::chrono::steady_clock::time_point>(std::chrono::steady_clock::now());

    // 零偏与 R_GtoI_ 已落成员，buff 使命完成，立即释放。
    // 下次重初始化（R26）必须从零重新积累一段当下的静止数据——留着旧帧会让重初始化
    // 一触发就拿陈旧姿态算出错误的 R_GtoI_，而方差判据对「干净但过期」的数据无能为力。
    {
        std::lock_guard<std::mutex> lk(buf_mutex_);
        imu_data_buff_.clear();
        imu_data_buff_.shrink_to_fit();
        start_time_.reset();
        last_reject_.clear();
    }

    RCLCPP_INFO(rclcpp::get_logger("ImuProcessor"), "IMU init success.");
    return true;
}

std::optional<sensor_msgs::msg::Imu> ImuProcessor::GetRefinedImuData(const sensor_msgs::msg::Imu& imu_data) {
    if (!is_first_initialized_) {
        return std::nullopt;
    }

    // 1) 取出原始量测为 Eigen 向量
    Eigen::Vector3d acc_raw(imu_data.linear_acceleration.x, imu_data.linear_acceleration.y,
                            imu_data.linear_acceleration.z);

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

bool ImuProcessor::CanComputeImuBias() {
    std::lock_guard<std::mutex> lk(buf_mutex_);
    return imu_data_buff_.size() >= static_cast<size_t>(max_buff_size_);
}

bool ImuProcessor::ComputeImuBias() {
    // 快照副本，避免长时间持锁
    std::deque<sensor_msgs::msg::Imu> buf;
    {
        std::lock_guard<std::mutex> lk(buf_mutex_);
        if (imu_data_buff_.size() < static_cast<size_t>(max_buff_size_)) return false;
        buf.assign(imu_data_buff_.end() - max_buff_size_, imu_data_buff_.end());
    }

    const size_t N = buf.size();
    Eigen::Vector3d acc_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyr_sum = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d> acc_samples;
    acc_samples.reserve(N);
    std::vector<Eigen::Vector3d> gyr_samples;
    gyr_samples.reserve(N);

    for (const auto& meas : buf) {
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
    auto compute_std = [](const std::vector<Eigen::Vector3d>& xs, const Eigen::Vector3d& mean) -> Eigen::Vector3d {
        Eigen::Vector3d v = Eigen::Vector3d::Zero();
        for (const auto& x : xs) {
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

    // ==================== 静止判据 ====================
    // 只用「方差 + 加速度模值」判断是否受激励。
    //
    // 这里刻意不看 gyr_avg：静止时 gyr_avg 恒等于陀螺零偏，也就是本函数下面要估计的
    // gyro_bias_。拿待估量当判据量，会让判据同时对「真在转」和「零偏大」两种情况给出
    // 相同结论，产生两类错误：
    //   · 零偏大 + 真静止 → 误拒，零偏超过门限的器件永远无法完成初始化
    //   · 零偏小 + 真运动 → 误纳，拿运动数据估出错误零偏与错误 R_ItoG_，此后 yaw 持续漂
    // 恒定角速度、恒定线加速度、匀速直线三种工况在 IMU 侧的方差与模值都与静止无异，
    // 必须靠编码器轮速拆开——该门禁在 AddImuData() 入口按 wheel_stopped 处理。
    if (acc_std.maxCoeff() > acc_excite_threshold_ || gyr_std.maxCoeff() > gyro_excite_threshold_ ||
        acc_mag_bias > acc_magnitude_tolerance_) {
        LOG_WARN << "IMU not static enough for bias init. "
                 << "acc_std=[" << acc_std.x() << ", " << acc_std.y() << ", " << acc_std.z() << "] (max allowed "
                 << acc_excite_threshold_ << "), "
                 << "gyr_std=[" << gyr_std.x() << ", " << gyr_std.y() << ", " << gyr_std.z() << "] (max allowed "
                 << gyro_excite_threshold_ << "), "
                 << "|‖acc_avg‖-g_preset|=" << acc_mag_bias << " (max allowed " << acc_magnitude_tolerance_ << ")";
        // TODO: 这里默认IMU是100hz，可以改成用时间戳计算
        LOG_WARN << "Reinitialize IMU after " << (max_buff_size_ / 100.0) << " seconds of static.";
        is_need_initialize_ = true;
        {
            char why[256];
            std::snprintf(why, sizeof(why), "acc_std %.4f (max %.3f) | gyr_std %.4f (max %.4f) | |acc|-g %.3f (max %.2f)",
                          acc_std.maxCoeff(), acc_excite_threshold_, gyr_std.maxCoeff(), gyro_excite_threshold_,
                          acc_mag_bias, acc_magnitude_tolerance_);
            std::lock_guard<std::mutex> lk(buf_mutex_);
            imu_data_buff_.clear();  // 清空数据，重新积累
            start_time_.reset();     // 静止段重新计时，重新等满 5s
            last_reject_ = why;      // 供 GetInitStatus 外发，告诉上层卡在哪条判据
        }
        return false;
    }

    // 零偏健康检查：此时已确认静止，gyr_avg 就是纯零偏。异常大说明器件本身有问题
    // 或初始化期间机体有极缓慢的运动，只告警不阻断——初始化仍按此零偏进行。
    if (gyr_avg.cwiseAbs().maxCoeff() > gyro_bias_warn_threshold_) {
        LOG_WARN << "Gyro bias unusually large: [" << gyr_avg.x() << ", " << gyr_avg.y() << ", " << gyr_avg.z()
                 << "] rad/s exceeds warn threshold " << gyro_bias_warn_threshold_
                 << ". Check the device or make sure the robot is completely still during init.";
    }

    // ==================== 估计 IMU 安装姿态（roll/pitch）及偏置 ====================
    // 设计原则：固定重力幅值（使用配置参数 gravity_magnitude_，不从数据中重估）。
    //   理由：在斜坡上静止时 acc_avg.norm() ≈ g（重力大小不变，仅方向偏离水平面），
    //         重新估计重力大小并覆盖预设值会破坏 GetRefinedImuData 的去重力一致性。
    //   Yaw 不可观：加速度计仅约束重力方向（2 个自由度），yaw 固定为 0（仅估计 roll+pitch）。

    // 1) 从实测加速度均值提取重力在 IMU 坐标系的单位方向
    //    静止时：acc_avg ≈ R_GtoI_ * [0, 0, g]，故 acc_avg.normalized() = R_GtoI_.col(2)
    Eigen::Vector3d z = acc_avg.normalized();

    // 2) 计算等效 roll/pitch（用于日志和斜坡告警）
    const double roll_est = std::atan2(acc_avg.y(), acc_avg.z());
    const double pitch_est = std::atan2(-acc_avg.x(), std::sqrt(acc_avg.y() * acc_avg.y() + acc_avg.z() * acc_avg.z()));
    // 倾角（IMU Z 轴偏离世界 Z 轴的角度）
    const double tilt_deg = std::acos(std::min(1.0, std::abs(z.z()))) * 180.0 / M_PI;
    // 记录到成员供下游查询：仅打日志不足以让上层知道本次初始化的可信度（R15）
    last_init_tilt_deg_ = tilt_deg;
    is_last_init_on_slope_ = (tilt_deg > kSlopeWarnDeg);
    if (is_last_init_on_slope_) {
        LOG_WARN << "IMU initialized on a slope! tilt=" << tilt_deg << " deg. "
                 << "R_ItoG_ aligns to slope normal, not true horizontal. "
                 << "Yaw error on flat ground ≈ sin(tilt) * ωx "
                 << "(tilt=8 deg with ωx=0.1 rad/s gives about 48 deg/min of false yaw rate). "
                 << "Please re-initialize on flat ground.";
    }

    // 3) 构造稳健的正交基（yaw=0：X 轴沿 IMU X 轴在重力坐标系水平面的投影）
    Eigen::Vector3d e = (std::abs(z.x()) < 0.9) ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
    Eigen::Vector3d x = (e - z * (z.dot(e))).normalized();
    Eigen::Vector3d y = z.cross(x).normalized();
    x = y.cross(z);  // 重新正交化，消除累计误差

    // 4) 组装矩阵并通过 SVD 极分解投影到 SO(3)
    Eigen::Matrix3d R;
    R.col(0) = x;
    R.col(1) = y;
    R.col(2) = z;

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d Ro = svd.matrixU() * svd.matrixV().transpose();
    if (Ro.determinant() < 0) {
        Eigen::Matrix3d U = svd.matrixU();
        U.col(2) *= -1;
        Ro = U * svd.matrixV().transpose();
    }

    // 5) 保存旋转矩阵（R_GtoI_：世界系→IMU系；R_ItoG_：IMU系→世界系）
    R_GtoI_ = Ro;
    R_ItoG_ = R_GtoI_.transpose();

    // 6) 陀螺零偏：静止均值
    gyro_bias_ = gyr_avg;

    // 7) 加速度零偏：使用固定重力幅值（gravity_vector_ 不在此处修改，始终为 SetGravityMagnitude() 预设值）
    //    acc_bias = acc_avg - R_GtoI_ * [0, 0, g_fixed]^T
    const Eigen::Vector3d gravity_in_imu = R_GtoI_ * Eigen::Vector3d(0.0, 0.0, gravity_magnitude_);
    acc_bias_ = acc_avg - gravity_in_imu;

    LOG_WARN << "IMU bias computed (g_fixed=" << gravity_magnitude_ << " m/s²). "
             << "roll=" << roll_est * 180.0 / M_PI << " deg, "
             << "pitch=" << pitch_est * 180.0 / M_PI << " deg, "
             << "tilt=" << tilt_deg << " deg, "
             << "acc_std=[" << acc_std.x() << ", " << acc_std.y() << ", " << acc_std.z() << "], "
             << "gyr_std=[" << gyr_std.x() << ", " << gyr_std.y() << ", " << gyr_std.z() << "], "
             << "acc_bias=[" << acc_bias_.x() << ", " << acc_bias_.y() << ", " << acc_bias_.z() << "], "
             << "gyro_bias=[" << gyro_bias_.x() << ", " << gyro_bias_.y() << ", " << gyro_bias_.z() << "]";
    return true;
}

void ImuProcessor::TrimImuData() {
    while (imu_data_buff_.size() > static_cast<size_t>(max_buff_size_)) {
        imu_data_buff_.pop_front();
    }
}

// 将 builtin_interfaces::msg::Time 转为 double 秒
inline double to_seconds(const builtin_interfaces::msg::Time& t) {
    // 注意 nanosec 的范围是 [0, 1e9)
    return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

// 返回 a - b 的秒差
inline double time_diff_seconds(const builtin_interfaces::msg::Time& a, const builtin_interfaces::msg::Time& b) {
    // 直接用 double 秒做差
    return to_seconds(a) - to_seconds(b);
}

void ImuProcessor::AddImuData(const sensor_msgs::msg::Imu& imu_data, bool wheel_stopped) {
    const auto& t_curr = imu_data.header.stamp;

    // ---- 1) 乱序/回拨检查（始终执行，与 buff 无关）----
    // 锚点用 last_stamp_ 而非 buff.back()：buff 只在需要初始化时才积累，
    // 拿它当锚点会让回拨保护在初始化完成后一并失效。
    {
        std::lock_guard<std::mutex> lock(buf_mutex_);
        if (last_stamp_.has_value() && time_diff_seconds(t_curr, last_stamp_.value()) < 0.0) {
            return;  // 时间戳回拨，丢弃
        }
        last_stamp_ = t_curr;
    }

    // ---- 2) 不需要初始化就到此为止 ----
    // imu_data_buff_ 唯一的消费者是 ComputeImuBias()，而它只在 is_need_initialize_
    // 为真时经 InitImu() 触发；GetRefinedImuData() 用的是初始化时算好的
    // acc_bias_ / gyro_bias_ / R_ItoG_，不读 buff。因此已初始化且无需重初始化时
    // 完全不必维护这个 500 帧窗口——否则就是每帧一次拷贝养一个永远没人读的缓冲区。
    MaybeNeedInit();
    if (!is_need_initialize_) {
        return;
    }

    // ---- 3) 采集静止段数据供初始化 ----
    {
        std::lock_guard<std::mutex> lock(buf_mutex_);

        // 轮子一转，静止段即告中断：当前帧不入 buff，已积累的整段一并作废。
        //
        // 作废而非保留，是因为 R_GtoI_ 由 acc_avg 的方向确定，**与当时的机体姿态绑定**。
        // 若保留：机器人行驶 10 s 后停下，buff 里仍是 10 s 前那 500 帧——它们内部完全
        // 干净，方差判据挑不出毛病，但描述的是车子上一个位置的姿态。中途若上了坡，
        // 估出的重力方向仍是平地的，且没有任何判据会报警。方差只能验证一批数据内部
        // 自洽，验证不了它是否属于当下。
        if (!wheel_stopped) {
            if (!imu_data_buff_.empty()) {  // 仅「从有到无」这一次需要处理
                imu_data_buff_.clear();
                start_time_.reset();
            }
            blocked_by_motion_ = true;
            return;
        }
        blocked_by_motion_ = false;

        // 静止段起始后 5s 内的数据不使用：等待上电瞬态与机体余振衰减。
        // 与上面的 clear 同处本分支内，两者门控对称——静止段一旦中断，计时随之归零。
        if (!start_time_.has_value()) {
            start_time_ = t_curr;
        }
        if (time_diff_seconds(t_curr, start_time_.value()) < 5.0) {
            return;
        }

        imu_data_buff_.push_back(imu_data);
        TrimImuData();
    }

    InitImu();
}

ImuProcessor::InitStatus ImuProcessor::GetInitStatus() {
    InitStatus st;
    st.done = is_first_initialized_;
    st.required = max_buff_size_;
    st.on_slope = is_last_init_on_slope_;
    st.tilt_deg = last_init_tilt_deg_;
    std::lock_guard<std::mutex> lk(buf_mutex_);
    st.blocked_by_motion = blocked_by_motion_;
    st.settling = start_time_.has_value() && imu_data_buff_.empty();
    st.collected = static_cast<int>(imu_data_buff_.size());
    st.last_reject = last_reject_;
    return st;
}

void ImuProcessor::MaybeNeedInit() {
    if (!is_first_initialized_) {
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
