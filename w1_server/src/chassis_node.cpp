#include "chassis_node.h"

#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace chassis_node {

ChassisNode::ChassisNode(const rclcpp::NodeOptions &options) : rclcpp::Node("chassis_node", options) {
    // 声明参数
    wheel_track_ = this->declare_parameter<double>("wheel_track", 0.4390);  // 轮距 b（示例值）
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_footprint");
    odom_rate_hz_ = this->declare_parameter<double>("odom_rate_hz", 100.0);
    use_imu_yaw_ = this->declare_parameter<bool>("use_imu_yaw", true);

    odom_factor_ = this->declare_parameter<double>("odom_factor", 1.0);
    imu_factor_ = this->declare_parameter<double>("imu_factor", 1.0);

    // IMU初始化参数 (根据IMU等级调整)
    // 工业级IMU推荐值：
    imu_max_buff_size_ = this->declare_parameter<int>("imu_max_buff_size", 500);  // 样本数量 (100Hz时需静止5秒)
    acc_excite_threshold_ = this->declare_parameter<double>("acc_excite_threshold", 0.04);  // 加速度标准差阈值 (m/s²)
    gyro_excite_threshold_ =
        this->declare_parameter<double>("gyro_excite_threshold", 0.004);  // 角速度标准差阈值 (rad/s ≈ 0.22°/s)
    imu_reinit_duration_ = this->declare_parameter<double>("imu_reinit_duration", 3600.0);  // 重新初始化间隔 (s)
    gravity_magnitude_ = this->declare_parameter<double>("gravity_magnitude", 9.7887);  // 当地重力加速度 (m/s²)

    // 死区参数
    speed_deadzone_ = this->declare_parameter<double>("speed_deadzone", 1e-3);   // 线速度死区 (m/s)
    omega_deadzone_ = this->declare_parameter<double>("omega_deadzone", 0.008);  // 角速度死区 (rad/s)
    imu_timeout_ = this->declare_parameter<double>("imu_timeout", 0.1);          // IMU超时时间 (s)

    imu_processor_ = std::make_unique<ImuProcessor>();

    imu_processor_->SetGravityMagnitude(gravity_magnitude_);
    imu_processor_->SetParam(imu_max_buff_size_, acc_excite_threshold_, gyro_excite_threshold_, imu_reinit_duration_);

    // 订阅 IMU
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/chassis", rclcpp::SystemDefaultsQoS(),
        std::bind(&ChassisNode::ImuMsgCallback, this, std::placeholders::_1));

    // 发布器与 TF
    refined_imu_pub_ =
        this->create_publisher<sensor_msgs::msg::Imu>("/imu/chassis/refined", rclcpp::SystemDefaultsQoS());
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/feedback/odom", rclcpp::SystemDefaultsQoS());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // 定时器（按频率发布；若你用纯事件驱动，也可不需要定时器）
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, odom_rate_hz_));
    timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                     std::bind(&ChassisNode::onTimer, this));

    callback_handle_ =
        this->add_on_set_parameters_callback(std::bind(&ChassisNode::parametersCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "ChassisNode started. track=%.3f m, rate=%.1f Hz, frames: %s -> %s", wheel_track_,
                odom_rate_hz_, odom_frame_.c_str(), base_frame_.c_str());
}

void ChassisNode::ResetOdometry() {
    std::lock_guard<std::mutex> lk(imu_yaw_mutex_);
    odom_x_ = 0.0;
    odom_y_ = 0.0;
    odom_theta_ = 0.0;
    speed_v_ = 0.0;
    omega_r_ = 0.0;
    is_stopped_ = true;
    prev_chassis_data_.reset();
    curr_chassis_data_.reset();
    prev_imu_.reset();
    curr_imu_.reset();
    is_imu_inited_ = false;
    RCLCPP_INFO(this->get_logger(), "Odometry reset.");
}

void ChassisNode::ImuMsgCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    imu_processor_->AddImuData(*msg);

    const auto refined_imu = imu_processor_->GetRefinedImuData(*msg);
    if (!refined_imu.has_value()) {
        is_imu_inited_ = false;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "IMU is not initialized ...");
        return;
    }

    is_imu_inited_ = true;
    // RCLCPP_INFO(this->get_logger(), "Got refined imu data");
    // 打印未优化的IMU
    RCLCPP_DEBUG(this->get_logger(), "Raw IMU acc: %.3f, %.3f, %.3f; gyro: %.3f, %.3f, %.3f",
                 msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
                 msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    RCLCPP_DEBUG(this->get_logger(), "Refined IMU acc: %.3f, %.3f, %.3f; gyro: %.3f, %.3f, %.3f",
                 refined_imu->linear_acceleration.x, refined_imu->linear_acceleration.y,
                 refined_imu->linear_acceleration.z, refined_imu->angular_velocity.x, refined_imu->angular_velocity.y,
                 refined_imu->angular_velocity.z);
    refined_imu_pub_->publish(*refined_imu);

    // IMU process 初始化后，使用修正后的IMU
    IntegrateImuOnce(refined_imu.value());
}

// IMU 回调：在这里进行角度积分（时间戳最准确）
void ChassisNode::IntegrateImuOnce(const sensor_msgs::msg::Imu &imu) {
    const double wz = imu.angular_velocity.z;
    if (!std::isfinite(wz)) {
        curr_imu_.reset();  // 当IMU数据无效时，重置当前IMU
        return;
    }

    std::lock_guard<std::mutex> lk(imu_yaw_mutex_);

    // 更新当前 IMU（用于速度发布）
    curr_imu_ = std::make_shared<sensor_msgs::msg::Imu>(imu);

    // 更新 IMU 时间戳
    last_imu_time_ = imu.header.stamp;

    // 第一次：仅缓存，不积分
    if (!prev_imu_) {
        prev_imu_ = curr_imu_;
        return;
    }

    const rclcpp::Time t_curr = imu.header.stamp;
    const rclcpp::Time t_prev = prev_imu_->header.stamp;
    const double dt = (t_curr - t_prev).seconds();

    if (!(dt > 0.0) || dt > 0.5) {
        // 时间戳异常或超时：仅更新 prev，不进行积分
        prev_imu_ = curr_imu_;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "IMU dt=%.3f s, skipping integration", dt);
        return;
    }

    // 检查是否应该进行角度积分（只有运动时才积分）
    // 注意：这里使用一个标志位，由底盘数据更新时设置
    if (!is_stopped_) {
        // 两帧中点法（梯形积分）计算 Δθ
        const double wz_prev = prev_imu_->angular_velocity.z;
        const double wz_mid = 0.5 * (wz + wz_prev);
        const double delta_theta = wz_mid * dt * imu_factor_;

        // 积分角度并 wrap 到 [-pi, pi]
        odom_theta_ += delta_theta;
        odom_theta_ = std::atan2(std::sin(odom_theta_), std::cos(odom_theta_));

        RCLCPP_DEBUG(this->get_logger(), "IMU yaw: %.3f deg, wz: %.3f rad/s, dt: %.3f s", odom_theta_ * 180.0 / M_PI,
                     wz_mid, dt);
    } else {
        RCLCPP_DEBUG(this->get_logger(), "Robot stopped, skipping IMU yaw integration");
    }

    prev_imu_ = curr_imu_;
}

// 检查 IMU 是否可用
bool ChassisNode::IsImuAvailable(const rclcpp::Time &curr_time) {
    if (!use_imu_yaw_ || curr_imu_ == nullptr) {
        return false;
    }

    // 使用 nanoseconds 进行时间差计算，避免不同时间源的问题
    const int64_t time_diff_ns = curr_time.nanoseconds() - last_imu_time_.nanoseconds();
    const double time_since_last_imu = time_diff_ns / 1e9;
    const bool imu_timeout = (time_since_last_imu > imu_timeout_);

    if (imu_timeout) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "IMU timeout (%.3f s > %.3f s), falling back to encoder", time_since_last_imu,
                             imu_timeout_);
    }

    return !imu_timeout;
}

// 主循环：消费反馈（如果有），按时间积分后发布
void ChassisNode::onTimer() {
    auto &feedback_shm_instance = FeedbackSharedMemory::GetInstance();
    RobotFeedback feedback_data{};
    bool ret = feedback_shm_instance.PeekLatest(&feedback_data);
    const auto &chassis_data = feedback_data.chassis_feedback;
    if (use_imu_yaw_ && !is_imu_inited_) {
        // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "Waiting for chassis initialized ...");
        return;
    }

    IntegrateOdomOnce(chassis_data);
    PublishOdometry();
}

void ChassisNode::IntegrateOdomOnce(const ChassisFeedback &chassis_data) {
    bool valid_data = (chassis_data.timestamp > 0);
    if (!valid_data) {
        // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "Waiting for valid chassis data ...");
        return;
    }

    curr_chassis_data_ = std::make_shared<ChassisFeedback>(chassis_data);

    // 第一帧：只缓存，不积分
    if (!prev_chassis_data_) {
        prev_chassis_data_ = curr_chassis_data_;
        return;
    }

    rclcpp::Time curr_time(curr_chassis_data_->timestamp, RCL_SYSTEM_TIME);
    rclcpp::Time prev_time(prev_chassis_data_->timestamp, RCL_SYSTEM_TIME);
    const double dt = (curr_time - prev_time).seconds();

    if (dt <= 0.0 || dt > 0.5) {
        // 时间戳异常或超时：不积分，只更新时间基
        prev_chassis_data_ = curr_chassis_data_;
        curr_chassis_data_.reset();
        return;
    }

    // 获取轮速数据
    const double v_l = curr_chassis_data_->left_velocity;
    const double v_r = curr_chassis_data_->right_velocity;
    const double v_l_prev = prev_chassis_data_->left_velocity;
    const double v_r_prev = prev_chassis_data_->right_velocity;

    // 预先计算平均线速度和角速度（梯形积分）
    const double speed_avg = 0.5 * (0.5 * (v_l + v_r) + 0.5 * (v_l_prev + v_r_prev));
    const double omega_enc = (v_r - v_l) / wheel_track_;
    const double omega_enc_prev = (v_r_prev - v_l_prev) / wheel_track_;
    const double omega_enc_avg = 0.5 * (omega_enc + omega_enc_prev);

    // 检查 IMU 可用性
    const bool imu_available = IsImuAvailable(curr_time);

    // 判断是否静止（线速度 AND 角速度都接近 0）
    const bool linear_stopped = (std::abs(v_l) < speed_deadzone_ && std::abs(v_r) < speed_deadzone_ &&
                                 std::abs(v_l_prev) < speed_deadzone_ && std::abs(v_r_prev) < speed_deadzone_);

    // 根据 IMU 可用性判断角速度是否静止
    bool angular_stopped = false;
    if (imu_available) {
        // 使用 IMU 角速度判断
        std::lock_guard<std::mutex> lk(imu_yaw_mutex_);
        const double omega_imu = curr_imu_->angular_velocity.z;
        angular_stopped = (std::abs(omega_imu) < omega_deadzone_);
    } else {
        // 使用编码器角速度判断（需要检查当前和前一帧）
        angular_stopped = (std::abs(omega_enc) < omega_deadzone_ && std::abs(omega_enc_prev) < omega_deadzone_);
    }

    const bool is_stopped = linear_stopped && angular_stopped;

    // 更新停止状态标志（用于 IMU 回调判断）
    {
        std::lock_guard<std::mutex> lk(imu_yaw_mutex_);
        is_stopped_ = is_stopped;
    }

    // 记录积分前的角度
    double theta_start = 0.0;
    {
        std::lock_guard<std::mutex> lk(imu_yaw_mutex_);
        theta_start = odom_theta_;
    }

    // 更新航向角（运动中且 IMU 不可用时使用编码器）
    if (!is_stopped && !imu_available) {
        // 运动中且不使用 IMU，或 IMU 不可用/超时时，使用编码器角速度
        const double delta_theta = omega_enc_avg * dt;

        std::lock_guard<std::mutex> lk(imu_yaw_mutex_);
        odom_theta_ += delta_theta;
        odom_theta_ = std::atan2(std::sin(odom_theta_), std::cos(odom_theta_));

        RCLCPP_DEBUG(this->get_logger(), "Using encoder yaw: %.3f deg", odom_theta_ * 180.0 / M_PI);
    }
    // else: 如果使用 IMU，角度已经在 IntegrateImuOnce 中更新了；静止时不积分

    // 获取积分后的角度
    double theta_end = 0.0;
    {
        std::lock_guard<std::mutex> lk(imu_yaw_mutex_);
        theta_end = odom_theta_;
    }

    // 计算位移（使用积分期间的中间角度）
    const double theta_mid =
        std::atan2(std::sin(0.5 * (theta_start + theta_end)), std::cos(0.5 * (theta_start + theta_end)));
    const double delta_s = speed_avg * dt;

    // 更新位置
    odom_x_ += delta_s * std::cos(theta_mid) * odom_factor_;
    odom_y_ += delta_s * std::sin(theta_mid) * odom_factor_;

    // 更新速度（用于发布）
    speed_v_ = speed_avg * odom_factor_;

    if (is_stopped) {
        // 静止状态：角速度置零
        omega_r_ = 0.0;
    } else {
        // 运动状态：根据 IMU 可用性选择角速度源
        if (imu_available) {
            // 使用 IMU 角速度
            std::lock_guard<std::mutex> lk(imu_yaw_mutex_);
            omega_r_ = curr_imu_->angular_velocity.z * imu_factor_;
        } else {
            // 使用编码器角速度
            omega_r_ = omega_enc;
        }

        // 应用死区
        if (std::abs(omega_r_) < omega_deadzone_) {
            omega_r_ = 0.0;
        }
    }

    RCLCPP_DEBUG(this->get_logger(), "odom: x=%.3f m, y=%.3f m, theta=%.3f deg, v=%.3f m/s, w=%.3f rad/s, stopped=%d",
                 odom_x_, odom_y_, theta_end * 180.0 / M_PI, speed_v_, omega_r_, is_stopped);

    // 更新时间基
    prev_chassis_data_ = curr_chassis_data_;
    curr_chassis_data_.reset();
}

void ChassisNode::PublishOdometry() {
    nav_msgs::msg::Odometry odom_msg;
    rclcpp::Time curr_time = rclcpp::Clock().now();
    odom_msg.header.stamp = curr_time;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = odom_x_;
    odom_msg.pose.pose.position.y = odom_y_;
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, odom_theta_);
    odom_msg.pose.pose.orientation = tf2::toMsg(q);

    odom_msg.twist.twist.linear.x = speed_v_;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = omega_r_;

    // 可选：协方差（示例，按需调整）
    // auto & Pc = odom_msg.pose.covariance;
    // auto & Tc = odom_msg.twist.covariance;
    // Pc[0] = Pc[7] = 0.02 * 0.02;   // x,y
    // Pc[35]= 0.02 * 0.02;           // yaw
    // Tc[0] = Tc[7] = 0.05 * 0.05;   // vx, vy
    // Tc[35]= 0.05 * 0.05;           // wz

    odom_pub_->publish(odom_msg);

    // TF: odom -> base_footprint
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = curr_time;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform.translation.x = odom_x_;
    tf_msg.transform.translation.y = odom_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_msg.pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf_msg);
}

rcl_interfaces::msg::SetParametersResult ChassisNode::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "Parameters updated successfully";

    for (const auto &changed_parameter : parameters) {
        if (changed_parameter.get_name() == "odom_factor") {
            odom_factor_ = changed_parameter.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated odom_factor: %.3f", odom_factor_);
        } else if (changed_parameter.get_name() == "imu_factor") {
            imu_factor_ = changed_parameter.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated imu_factor: %.3f", imu_factor_);
        } else if (changed_parameter.get_name() == "imu_max_buff_size") {
            imu_max_buff_size_ = changed_parameter.as_int();
            imu_processor_->SetParam(imu_max_buff_size_, acc_excite_threshold_, gyro_excite_threshold_,
                                     imu_reinit_duration_);
            RCLCPP_INFO(this->get_logger(), "Updated imu_max_buff_size: %d", imu_max_buff_size_);
        } else if (changed_parameter.get_name() == "acc_excite_threshold") {
            acc_excite_threshold_ = changed_parameter.as_double();
            imu_processor_->SetParam(imu_max_buff_size_, acc_excite_threshold_, gyro_excite_threshold_,
                                     imu_reinit_duration_);
            RCLCPP_INFO(this->get_logger(), "Updated acc_excite_threshold: %.3f", acc_excite_threshold_);
        } else if (changed_parameter.get_name() == "gyro_excite_threshold") {
            gyro_excite_threshold_ = changed_parameter.as_double();
            imu_processor_->SetParam(imu_max_buff_size_, acc_excite_threshold_, gyro_excite_threshold_,
                                     imu_reinit_duration_);
            RCLCPP_INFO(this->get_logger(), "Updated gyro_excite_threshold: %.3f", gyro_excite_threshold_);
        } else if (changed_parameter.get_name() == "imu_reinit_duration") {
            imu_reinit_duration_ = changed_parameter.as_double();
            imu_processor_->SetParam(imu_max_buff_size_, acc_excite_threshold_, gyro_excite_threshold_,
                                     imu_reinit_duration_);
            RCLCPP_INFO(this->get_logger(), "Updated imu_reinit_duration: %.3f", imu_reinit_duration_);
        } else if (changed_parameter.get_name() == "gravity_magnitude") {
            gravity_magnitude_ = changed_parameter.as_double();
            imu_processor_->SetGravityMagnitude(gravity_magnitude_);
            RCLCPP_INFO(this->get_logger(), "Updated gravity_magnitude: %.4f", gravity_magnitude_);
        } else if (changed_parameter.get_name() == "wheel_track") {
            wheel_track_ = changed_parameter.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated wheel_track: %.3f", wheel_track_);
        } else if (changed_parameter.get_name() == "odom_rate_hz") {
            odom_rate_hz_ = changed_parameter.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated odom_rate_hz: %.1f", odom_rate_hz_);
            // 可选：根据新的频率重启定时器
            const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, odom_rate_hz_));
            timer_->reset();
            timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                             std::bind(&ChassisNode::onTimer, this));
        } else if (changed_parameter.get_name() == "use_imu_yaw") {
            use_imu_yaw_ = changed_parameter.as_bool();
            RCLCPP_INFO(this->get_logger(), "Updated use_imu_yaw: %s", use_imu_yaw_ ? "true" : "false");
        } else if (changed_parameter.get_name() == "speed_deadzone") {
            speed_deadzone_ = changed_parameter.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated speed_deadzone: %.5f m/s", speed_deadzone_);
        } else if (changed_parameter.get_name() == "omega_deadzone") {
            omega_deadzone_ = changed_parameter.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated omega_deadzone: %.5f rad/s", omega_deadzone_);
        } else if (changed_parameter.get_name() == "imu_timeout") {
            imu_timeout_ = changed_parameter.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated imu_timeout: %.3f s", imu_timeout_);
        }
    }
    ResetOdometry();  // 参数变更后重置里程计
    return result;
}

}  // namespace chassis_node
