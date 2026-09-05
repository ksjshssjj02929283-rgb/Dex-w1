#include "chassis_node.h"

#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace chassis_node {

// 内部辅助：找到 chassis_params.yaml 并构建 NodeOptions
static rclcpp::NodeOptions MakeChassisOptions() {
    rclcpp::NodeOptions opts;
    const auto pkg = ament_index_cpp::get_package_share_directory("w1_server");
    const auto yaml_path = std::filesystem::path(pkg) / "config" / "chassis_params.yaml";
    if (std::filesystem::exists(yaml_path)) {
        opts.arguments({"--ros-args", "--params-file", yaml_path.string()});
        std::cout << "[ChassisNode] params loaded from: " << yaml_path << std::endl;
    } else {
        std::cout << "[ChassisNode] " << yaml_path << " not found, using compiled defaults." << std::endl;
    }
    return opts;
}

ChassisNode::ChassisNode() : ChassisNode(MakeChassisOptions()) {}

ChassisNode::ChassisNode(const rclcpp::NodeOptions& options) : rclcpp::Node("chassis_node", options) {
    // 声明参数
    // 尺度参数的分工（避免 odom_factor 与 wheel_radius 解释同一自由度）：
    //   wheel_radius / wheel_track_eff —— 物理常量，不参与标定，钉在名义值；
    //   odom_factor / imu_factor       —— 标定量，由 calib_odom_factor 用激光真值写入。
    //   odom_factor 乘在**轮速层**：轮径误差同时污染 v 与 ω，只修 v 会让 wheel_track_eff 被迫吸收轮径误差。
    wheel_track_     = this->declare_parameter<double>("wheel_track",     0.4390);  // 几何轴距 (m)，仅作参考基准，不参与计算
    wheel_radius_    = this->declare_parameter<double>("wheel_radius",    0.0865);  // 轮半径 (m)，物理常量
    wheel_track_eff_ = this->declare_parameter<double>("wheel_track_eff", wheel_track_);  // 有效轴距 (m)，物理常量
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_footprint");
    odom_rate_hz_ = this->declare_parameter<double>("odom_rate_hz", 100.0);
    // 里程计模式：单参数四选一（encoder_only | imu_yaw | ekf | ekf_no_imu），
    // 替代原 use_imu_yaw / use_ekf_fusion 两个布尔——它们的 2×2 组合里第四种（EKF 无 IMU）
    // 其实是 IMU 掉线时的自动退化态，用两个布尔表达时没人知道它存在。
    {
        const std::string mode_str = this->declare_parameter<std::string>("odom_mode", "ekf");
        const auto parsed = ParseOdomMode(mode_str);
        if (!parsed) {
            RCLCPP_ERROR(this->get_logger(), "Invalid odom_mode '%s', falling back to 'ekf' (valid: %s)",
                         mode_str.c_str(), kOdomModeChoices);
        }
        odom_mode_      = parsed.value_or(OdomMode::Ekf);
        use_imu_yaw_    = ModeUsesImu(odom_mode_);
        use_ekf_fusion_ = ModeUsesEkf(odom_mode_);
    }

    odom_factor_ = this->declare_parameter<double>("odom_factor", 1.0);  // 编码器线尺度标定量（乘在轮速层）
    imu_factor_ = this->declare_parameter<double>("imu_factor", 1.0);

    // IMU初始化参数 (根据IMU等级调整)
    // 工业级IMU推荐值：
    imu_max_buff_size_ = this->declare_parameter<int>("imu_max_buff_size", 500);  // 样本数量 (100Hz时需静止5秒)
    acc_excite_threshold_ = this->declare_parameter<double>("acc_excite_threshold", 0.04);  // 加速度标准差阈值 (m/s²)
    gyro_excite_threshold_ =
        this->declare_parameter<double>("gyro_excite_threshold", 0.004);  // 角速度标准差阈值 (rad/s ≈ 0.22°/s)
    imu_reinit_duration_ = this->declare_parameter<double>("imu_reinit_duration", 3600.0);  // 重新初始化间隔 (s)
    gravity_magnitude_ = this->declare_parameter<double>("gravity_magnitude", 9.7887);  // 当地重力加速度 (m/s²)
    // 零偏告警门限：静止时 gyr_avg 即零偏，超过只告警不拒绝初始化（不是静止判据）
    gyro_bias_warn_threshold_ =
        this->declare_parameter<double>("gyro_bias_warn_threshold", 0.01);  // rad/s ≈ 0.57°/s
    acc_magnitude_tolerance_ =
        this->declare_parameter<double>("acc_magnitude_tolerance", 0.2);  // |‖acc‖-g| 容差 (m/s²)

    // 死区参数
    speed_deadzone_ = this->declare_parameter<double>("speed_deadzone", 1e-3);   // 线速度死区 (m/s)
    omega_deadzone_ = this->declare_parameter<double>("omega_deadzone", 0.008);  // 角速度死区 (rad/s)
    imu_timeout_ = this->declare_parameter<double>("imu_timeout", 0.1);          // IMU超时时间 (s)
    odom_stale_timeout_ = this->declare_parameter<double>("odom_stale_timeout", 0.5);  // 底盘反馈失效判定 (s)

    // EKF 参数（Q 为「每秒」量纲，内部乘 dt；勿按每步理解）
    ekf_params_.q_xy    = this->declare_parameter<double>("ekf_q_xy",    1e-4);
    ekf_params_.q_theta = this->declare_parameter<double>("ekf_q_theta", 1e-6);
    ekf_params_.q_v     = this->declare_parameter<double>("ekf_q_v",     1.0);
    ekf_params_.q_omega = this->declare_parameter<double>("ekf_q_omega", 1.0);
    ekf_params_.q_omega_imu = this->declare_parameter<double>("ekf_q_omega_imu", 1e-4);
    ekf_params_.r_zupt  = this->declare_parameter<double>("ekf_r_zupt",  1e-6);
    ekf_params_.q_scale_stopped = this->declare_parameter<double>("ekf_q_scale_stopped", 0.01);
    ekf_params_.r_v           = this->declare_parameter<double>("ekf_r_v",           0.05);
    ekf_params_.r_omega       = this->declare_parameter<double>("ekf_r_omega",       0.01);
    ekf_params_.r_v_aerial    = this->declare_parameter<double>("ekf_r_v_aerial",    100.0);
    ekf_params_.r_omega_aerial= this->declare_parameter<double>("ekf_r_omega_aerial",100.0);
    ekf_params_.aerial_chi2_threshold = this->declare_parameter<double>("ekf_aerial_chi2_threshold", 9.21);
    ekf_params_.p0_xy    = this->declare_parameter<double>("ekf_p0_xy",    1.0e-9);
    ekf_params_.p0_theta = this->declare_parameter<double>("ekf_p0_theta", 1.0e-9);
    ekf_params_.p0_v     = this->declare_parameter<double>("ekf_p0_v",     1.0);
    ekf_params_.p0_omega = this->declare_parameter<double>("ekf_p0_omega", 1.0);

    imu_source_detect_grace_ = this->declare_parameter<double>("imu_source_detect_grace", 5.0);

    // 模型配置文件路径 ==================
    const auto package_share_path = ament_index_cpp::get_package_share_directory("w1_server");

    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot_config.yaml";

    // ================== 2. 加载配置 ==================
    LoadChassisConfig(robot_cfg_path);

    imu_processor_ = std::make_unique<ImuProcessor>();
    imu_processor_->SetGravityMagnitude(gravity_magnitude_);
    imu_processor_->SetParam(imu_max_buff_size_, acc_excite_threshold_, gyro_excite_threshold_, imu_reinit_duration_,
                             gyro_bias_warn_threshold_, acc_magnitude_tolerance_);

    ekf_ = std::make_unique<ChassisEkf>(ekf_params_);


    dr_cov_.Reset(ekf_params_);
    // 订阅 IMU
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/chassis", rclcpp::SystemDefaultsQoS(),
        std::bind(&ChassisNode::ImuMsgCallback, this, std::placeholders::_1));

    // 发布器与 TF
    refined_imu_pub_ =
        this->create_publisher<sensor_msgs::msg::Imu>("/imu/chassis/refined", rclcpp::SystemDefaultsQoS());
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/feedback/odom", rclcpp::SystemDefaultsQoS());
    // 话题名不带 omni：消息已兼容差速与全向，由 chassis_type 区分。
    // 对照控制侧 /control/omni_chassis_motors —— 那个带 omni 是名副其实的
    //（差速的控制走 /control/cmd_vel，根本不经这条链路）。
    motor_feedback_pub_ = this->create_publisher<chassis_interfaces::msg::ChassisMotorFeedback>(
        "/feedback/chassis_motor", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // 定时器（按频率发布；若你用纯事件驱动，也可不需要定时器）
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, odom_rate_hz_));
    timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                     std::bind(&ChassisNode::onTimer, this));

    callback_handle_ =
        this->add_on_set_parameters_callback(std::bind(&ChassisNode::parametersCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "ChassisNode started. track=%.3f m, rate=%.1f Hz, frames: %s -> %s, odom_mode=%s",
                wheel_track_, odom_rate_hz_, odom_frame_.c_str(), base_frame_.c_str(),
                OdomModeName(odom_mode_));
}

void ChassisNode::ApplyOdomMode(OdomMode new_mode) {
    const bool new_imu = ModeUsesImu(new_mode);
    const bool new_ekf = ModeUsesEkf(new_mode);
    std::lock_guard<std::mutex> lk(imu_yaw_mutex_);

    if (new_ekf != use_ekf_fusion_ && ekf_) {
        if (new_ekf) {
            // 切入 EKF：EKF 状态在关闭期间不推进，直接取用会把位姿跳回旧值（首次为原点）。
            // 用当前位姿/速度灌初值，位姿 p0 极小——灌入的就是已知位姿。
            ekf_->SetState(odom_x_, odom_y_, odom_theta_, speed_v_, omega_r_);
            is_aerial_ = false;
        } else {
            // 切出 EKF：纯推算协方差从 EKF 当前 P 接续，避免协方差阶跃
            dr_cov_.SeedFrom(ekf_->GetCovariance());
        }
    }
    if (new_imu != use_imu_yaw_) {
        // 切换航向源：清 IMU 缓存，不重置里程计（θ 在同一变量上继续积分，无跳变）
        curr_imu_.reset();
        prev_imu_.reset();
    }

    const OdomMode old_mode = odom_mode_;
    odom_mode_      = new_mode;
    use_imu_yaw_    = new_imu;
    use_ekf_fusion_ = new_ekf;
    RCLCPP_INFO(this->get_logger(), "Updated odom_mode: %s -> %s (imu=%s, ekf=%s)%s",
                OdomModeName(old_mode), OdomModeName(new_mode), new_imu ? "on" : "off", new_ekf ? "on" : "off",
                (new_ekf != ModeUsesEkf(old_mode))
                    ? (new_ekf ? " | EKF state seeded from current pose" : " | dead-reckoning covariance seeded from EKF")
                    : "");
}

void ChassisNode::LoadChassisConfig(const std::filesystem::path& cfg_path) {
    if (!std::filesystem::exists(cfg_path)) {
        RCLCPP_FATAL(this->get_logger(), "robot_config.yaml not found: %s", cfg_path.c_str());
        throw std::runtime_error("robot_config.yaml not found");
    }

    YAML::Node config;
    try {
        config = YAML::LoadFile(cfg_path.string());
    } catch (const YAML::Exception& e) {
        RCLCPP_FATAL(this->get_logger(), "YAML parse error in %s: %s", cfg_path.c_str(), e.what());
        throw;
    }

    // ---------- chassis_config ----------
    if (!config["chassis_config"]) {
        RCLCPP_FATAL(this->get_logger(), "robot_config.yaml missing field: chassis_config");
        throw std::runtime_error("chassis_config missing");
    }

    if (!config["chassis_config"]["chassis_type"]) {
        RCLCPP_FATAL(this->get_logger(), "robot_config.yaml missing field: chassis_config.chassis_type");
        throw std::runtime_error("chassis_config.chassis_type missing");
    }

    const std::string chassis_type = config["chassis_config"]["chassis_type"].as<std::string>();

    if (chassis_type == "omni") {
        chassis_model_ = 1;
    } else if (chassis_type == "diff_drive") {
        chassis_model_ = 0;
    } else {
        RCLCPP_FATAL(this->get_logger(), "Unknown chassis_type: %s (expect omni / diff_drive)", chassis_type.c_str());
        throw std::runtime_error("unknown chassis_type");
    }
}

void ChassisNode::ApplyImuProcessorParams() {
    if (!imu_processor_) {
        return;
    }
    imu_processor_->SetParam(imu_max_buff_size_, acc_excite_threshold_, gyro_excite_threshold_, imu_reinit_duration_,
                             gyro_bias_warn_threshold_, acc_magnitude_tolerance_);
}

void ChassisNode::ResetOdometry() {
    std::lock_guard<std::mutex> lk(imu_yaw_mutex_);
    odom_x_ = 0.0;
    odom_y_ = 0.0;
    odom_theta_ = 0.0;
    speed_v_ = 0.0;
    speed_v_y_ = 0.0;
    omega_r_ = 0.0;
    is_stopped_ = true;
    is_aerial_ = false;
    prev_chassis_data_.reset();
    curr_chassis_data_.reset();
    prev_imu_.reset();
    curr_imu_.reset();
    is_imu_inited_ = false;
    if (ekf_) {
        ekf_->Reset();
    }
    dr_cov_.Reset(ekf_params_);
    RCLCPP_INFO(this->get_logger(), "Odometry reset.");
}

void ChassisNode::DetectImuSource(const ImuData& imu_data, bool feedback_valid) {
    if (imu_source_ != ImuSource::Unknown) {
        return;
    }
    const auto now = this->get_clock()->now();
    if (imu_detect_start_.nanoseconds() == 0) {
        imu_detect_start_ = now;
        last_shm_imu_stamp_ = imu_data.timestamp;
        return;
    }

    // 判据是「时间戳在变」而不是「时间戳非零」：EtherCAT 尚未就绪时 shm 全零，
    // 只看非零会把 0.3.1 误判成 0.3.0。
    if (feedback_valid && imu_data.timestamp != last_shm_imu_stamp_) {
        imu_source_ = ImuSource::Shm;
        RCLCPP_INFO(this->get_logger(),
                    "IMU source = SHM (hardware 0.3.1: IMU via EtherCAT shared memory, "
                    "/imu/chassis topic ignored)");
        return;
    }

    if ((now - imu_detect_start_).seconds() > imu_source_detect_grace_) {
        imu_source_ = ImuSource::Topic;
        RCLCPP_INFO(this->get_logger(),
                    "IMU source = TOPIC (hardware 0.3.0: no IMU update in shared memory within %.1f s, "
                    "using /imu/chassis)",
                    imu_source_detect_grace_);
    }
}

sensor_msgs::msg::Imu ChassisNode::ConvertShmImu(const ImuData& d) {
    sensor_msgs::msg::Imu m;
    // 用节点时钟，不用 d.timestamp（HI91 协议计数器，uint32 溢出且与系统时钟无关）
    m.header.stamp = this->get_clock()->now();
    m.header.frame_id = "imu_chassis_link";  // 与 w1_server 的 PublishImu 保持一致
    m.orientation.w = d.orientation.w;
    m.orientation.x = d.orientation.x;
    m.orientation.y = d.orientation.y;
    m.orientation.z = d.orientation.z;
    m.angular_velocity.x = d.angular_velocity.x;
    m.angular_velocity.y = d.angular_velocity.y;
    m.angular_velocity.z = d.angular_velocity.z;
    m.linear_acceleration.x = d.linear_acceleration.x;
    m.linear_acceleration.y = d.linear_acceleration.y;
    m.linear_acceleration.z = d.linear_acceleration.z;
    return m;
}

void ChassisNode::ImuMsgCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // 已判定走共享内存时忽略话题消息：0.3.1 硬件上 w1_server 也会转发 /imu/chassis，
    // 若两路都处理会重复喂 IMU（频率翻倍、静止初始化的方差统计被污染）。
    if (imu_source_ == ImuSource::Shm) {
        return;
    }
    ProcessImu(*msg);
}

void ChassisNode::ProcessImu(const sensor_msgs::msg::Imu& imu_in) {
    const sensor_msgs::msg::Imu* msg = &imu_in;
    // 超时判据的基准：记录**消息时间戳**（衡量「数据有多旧」，可检出队列积压）。
    // 两条来源路径都经过这里，故判据统一。shm 路径的 stamp 由 ConvertShmImu 用
    // 节点时钟现打，话题路径的由发布方打——两者同为 RCL_ROS_TIME，可直接相减。
    last_imu_stamp_ = imu_in.header.stamp;

    // 编码器静止状态作为静止初始化的第二信源：IMU 侧的方差与模值无法区分
    // 恒定角速度 / 恒定线加速度 / 匀速直线与真静止。
    bool wheel_stopped = true;
    {
        std::lock_guard<std::mutex> lk(imu_yaw_mutex_);
        wheel_stopped = is_wheel_stopped_;
    }
    imu_processor_->AddImuData(*msg, wheel_stopped);

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
void ChassisNode::IntegrateImuOnce(const sensor_msgs::msg::Imu& imu) {
    const double wz = imu.angular_velocity.z;
    if (!std::isfinite(wz)) {
        curr_imu_.reset();  // 当IMU数据无效时，重置当前IMU
        return;
    }

    std::lock_guard<std::mutex> lk(imu_yaw_mutex_);

    // 更新当前 IMU（用于速度发布及 EKF 输入）
    curr_imu_ = std::make_shared<sensor_msgs::msg::Imu>(imu);
    // 第一次：仅缓存，不积分
    if (!prev_imu_) {
        prev_imu_ = curr_imu_;
        return;
    }

    // EKF 模式：不在这里积分 theta，由 IntegrateOdomOnce 中 EKF::Step 统一处理
    if (use_ekf_fusion_) {
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

// 仅按编码器轮速更新静止标志，供 IMU 静止初始化交叉确认。
// 不掺入 IMU 角速度：IMU 初始化本身依赖该标志，掺入会形成循环依赖。
void ChassisNode::UpdateWheelStoppedFlag(const ChassisFeedback &chassis_data) {
    if (chassis_data.timestamp <= 0) {
        // 反馈无效分两种：
        // ① 开机阶段还没来——保持上次状态（初值 true）：此时机器人确实静止，视为静止可让初始化正常起步；
        // ② 运行中失效（odom_stale_）——静止与否不可知，置 false 关掉 ZUPT，宁可少修正也不错修正。
        if (odom_stale_) {
            std::lock_guard<std::mutex> lk(imu_yaw_mutex_);
            is_wheel_stopped_ = false;
        }
        return;
    }

    constexpr double kRpmToRads = 2.0 * M_PI / 60.0;
    bool stopped = true;

    if (chassis_model_ == 0) {
        const double v_l = chassis_data.left_velocity * kRpmToRads * wheel_radius_ * odom_factor_;
        const double v_r = chassis_data.right_velocity * kRpmToRads * wheel_radius_ * odom_factor_;
        stopped = (std::abs(v_l) < speed_deadzone_ && std::abs(v_r) < speed_deadzone_);
    } else {
        for (size_t i = 0; i < OMNI_MOTOR_NUM; ++i) {
            const double v_i = chassis_data.chassis_motor_velocity[i] * 0.001 * OMNI_CHASSIS_WHEEL_D * M_PI / 60.0 * odom_factor_;
            if (std::abs(v_i) >= speed_deadzone_) {
                stopped = false;
                break;
            }
        }
    }

    std::lock_guard<std::mutex> lk(imu_yaw_mutex_);
    is_wheel_stopped_ = stopped;
}

/**
 * 检查 IMU 是否可用（超时判据）。
 *
 * ⚠️ 原实现拿 `chassis_data.timestamp`（共享内存，CLOCK_MONOTONIC）减 `imu.header.stamp`
 *    （ROS 时钟，CLOCK_REALTIME），两者相差「开机时刻的 Unix 时间」约 1.788e9 秒，
 *    差值恒为大负数 ⇒ `> imu_timeout_` **恒假** ⇒ IMU 永不判超时、永不降级。
 *    原注释「用 nanoseconds 避免不同时间源的问题」说反了：`nanoseconds()` 只是绕过了
 *    rclcpp 对 clock type 的断言，把崩溃变成了静默错误。（实测见 2026-09-01 记录）
 *
 * 现行做法：用节点时钟减**消息 stamp**（两条 IMU 来源均以节点时钟打戳：w1_server 同进程；
 * dexe_sensors/imu 已改用节点时钟）。选 stamp 而非接收时刻，是为了一并检出消息在订阅
 * 队列里积压的情形——拿 200 ms 前的角速度去积分比降级更糟。跨源前提由下方 sanity 检查兜底。
 */
bool ChassisNode::IsImuAvailable() {
    if (!use_imu_yaw_ || curr_imu_ == nullptr) {
        return false;
    }

    // 两侧同为 RCL_ROS_TIME，可直接相减
    const double imu_age = (this->get_clock()->now() - last_imu_stamp_).seconds();

    // ── sanity 检查：不把「跨时钟源」误当成「数据新鲜」──
    // R08 的教训：跨源相减得到 −1.788e9 s，判据 `> timeout` 恒假 ⇒ 永不降级，
    // 且毫无症状。这里显式识别荒谬值并**保守判为不可用**，同时报错指明原因。
    // 阈值取 1 小时：真实停摆再久也该降级，而跨源偏差是 10^9 秒量级，两者不会混淆。
    // 阈值分两级，避免把「长时间无数据」误报成「跨时钟源」：
    //   跨源偏差是 10^9 s 量级（REALTIME 与 MONOTONIC 相差开机时刻的 Unix 时间）；
    //   而真实的「IMU 停了很久」通常在 10^3~10^5 s。用 1e6 s 分界，两者不会混淆。
    // 2026-09-02 实机教训：原先只用 3600 s 一档，IMU 停了 16 小时被报成「疑似不同源」，
    // 指向了错误的排查方向。
    constexpr double kCrossClockSuspectSec = 1.0e6;  // 超过它才可能是跨时钟源
    if (imu_age < -1.0 || imu_age > kCrossClockSuspectSec) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "IMU stamp and node clock appear to be different sources (age=%.3g s), "
                              "treating IMU as unavailable. Check whether the publisher uses "
                              "rclcpp::Clock() (defaults to RCL_SYSTEM_TIME, ignores use_sim_time)",
                              imu_age);
        return false;
    }

    const bool imu_timeout = (imu_age > imu_timeout_);
    if (imu_timeout) {
        // 长时间无数据与瞬时超时分开表述：前者多半是 IMU 链路断了（查 driver 日志），
        // 后者是抖动或短暂丢帧。
        if (imu_age > 60.0) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "No IMU data for %.0f s, falling back to encoder-only. "
                                  "Check whether the IMU driver is disconnected", imu_age);
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "IMU timeout (%.3f s > %.3f s), falling back to encoder", imu_age, imu_timeout_);
        }
    }

    return !imu_timeout;
}

// 底盘反馈失效判定（fail-fast）。
// PeekLatest 自带新鲜度门：元素写入时打 CLOCK_MONOTONIC 戳，读取时同钟比较，
// 默认 10 ms 超时——所以 feedback_valid 已经是「最近 10 ms 内有新数据」。
// 本函数只在其上加两件事：① 持续时长门槛（odom_stale_timeout_，滤掉抖动）
// ② 失效/恢复的状态迁移与统计。锚点用节点时钟自比较，不与 shm 时间戳跨源相减。
void ChassisNode::UpdateOdomHealth(bool feedback_valid) {
    const rclcpp::Time now = this->get_clock()->now();

    if (feedback_valid) {
        last_feedback_valid_time_ = now;
        has_feedback_ever_ = true;
        if (odom_stale_) {
            odom_stale_ = false;
            const double outage = (now - odom_stale_since_).seconds();
            odom_stale_total_sec_ += outage;
            // 恢复后首帧只重建时间基不积分：prev/curr 清空后 Integrate* 走「第一帧只缓存」分支，
            // 避免拿失效前的旧帧与新帧做一次跨越整段失效期的大 dt 积分。
            prev_chassis_data_.reset();
            curr_chassis_data_.reset();
            RCLCPP_INFO(this->get_logger(),
                        "Chassis feedback recovered after %.3f s, odometry output resumed | "
                        "outages: %u, total stale %.1f s",
                        outage, odom_stale_count_, odom_stale_total_sec_);
        }
        return;
    }

    // 还没收到过任何有效反馈：属于「未初始化」，不是失效，交给 onTimer 的 WARN
    if (!has_feedback_ever_) {
        return;
    }

    const double stale_sec = (now - last_feedback_valid_time_).seconds();
    if (stale_sec <= odom_stale_timeout_) {
        return;  // 抖动窗口内，保持现状
    }
    if (!odom_stale_) {
        odom_stale_ = true;
        odom_stale_since_ = last_feedback_valid_time_;
        ++odom_stale_count_;
    }
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "Chassis feedback stale for %.3f s (> %.3f s), odometry output suspended | "
                          "outage #%u",
                          stale_sec, odom_stale_timeout_, odom_stale_count_);
}

// 主循环：消费反馈（如果有），按时间积分后发布
void ChassisNode::onTimer() {
    auto& feedback_shm_instance = FeedbackSharedMemory::GetInstance();
    RobotFeedback feedback_data{};
    const bool feedback_valid = feedback_shm_instance.PeekLatest(&feedback_data);
    const auto& chassis_data = feedback_data.chassis_feedback;

    // 失效判定要在所有消费反馈的步骤之前：它决定本周期是否停发。
    UpdateOdomHealth(feedback_valid);

    // 只在反馈有效时发：无效时 chassis_data 是零初始化值，发出去会污染标定数据集
    if (feedback_valid) {
        PublishMotorFeedback(chassis_data);
    } else if (!has_feedback_ever_) {
        // 开机阶段底盘反馈常晚于 IMU，此时只是「还没来」，不是失效
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "No chassis feedback available from shared memory yet.");
    }

    // 必须在下面的 IMU 门禁之前更新：IMU 静止初始化依赖该标志，而非 EKF 模式下
    // IMU 未初始化又会在此 return，若放到 IntegrateOdomOnce 里更新则二者互相等待。
    UpdateWheelStoppedFlag(chassis_data);

    // IMU 来源判定与 shm 路径供数。放在 UpdateWheelStoppedFlag 之后：ProcessImu 里的
    // 静止初始化要读 is_wheel_stopped_，先更新才能拿到本周期的值。
    DetectImuSource(feedback_data.imu_data, feedback_valid);
    if (imu_source_ == ImuSource::Shm && feedback_valid) {
        // 只在 shm 时间戳变化时喂：同一帧重复喂会让静止判据的方差统计失真
        if (feedback_data.imu_data.timestamp != last_shm_imu_stamp_) {
            last_shm_imu_stamp_ = feedback_data.imu_data.timestamp;
            ProcessImu(ConvertShmImu(feedback_data.imu_data));
        }
    }

    // 非 EKF 模式且 IMU 未初始化时，等待 IMU
    if (use_imu_yaw_ && !use_ekf_fusion_ && !is_imu_inited_) {
        return;
    }
    // 底盘反馈失效：停发里程计与 TF（fail-fast）。不再按上一次位姿续发——
    // 那会向下游断言「机器人静止且置信度不变」，把通信故障掩盖成正常状态；
    // 上层对 odom→base TF 本就有超时保护，停车是它的既定行为。
    if (odom_stale_) {
        return;
    }
    if (chassis_model_ == 0)
        IntegrateOdomOnce(chassis_data);
    else
        IntegrateOmniOdomOnce(chassis_data);
    PublishOdometry();
}

void ChassisNode::IntegrateOdomOnce(const ChassisFeedback& chassis_data) {
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

    // ⚠️ clock type 必须是 STEADY：chassis_data.timestamp 实测为 CLOCK_MONOTONIC
    // （2026-09-01 三时钟对比，与 CLOCK_MONOTONIC 差 0.000 s）。标成 SYSTEM_TIME 时
    // 数值上不影响 dt（同源相减），但会让「拿它与 ROS 时间比较」的误用**静默通过**；
    // 标成 STEADY 后 rclcpp 会直接抛异常，误用立即暴露。
    rclcpp::Time curr_time(curr_chassis_data_->timestamp, RCL_STEADY_TIME);
    rclcpp::Time prev_time(prev_chassis_data_->timestamp, RCL_STEADY_TIME);
    const double dt = (curr_time - prev_time).seconds();

    if (dt <= 0.0 || dt > 0.5) {
        // 时间戳异常或超时：不积分，只更新时间基
        prev_chassis_data_ = curr_chassis_data_;
        curr_chassis_data_.reset();
        return;
    }

    // 获取轮速数据（RPM → m/s，左右轮半径相同）
    constexpr double kRpmToRads = 2.0 * M_PI / 60.0;
    // odom_factor 在轮速层生效：v 与 ω_enc 同时得到线尺度修正（轮径误差对两者是同一个比例）
    const double v_l      = curr_chassis_data_->left_velocity  * kRpmToRads * wheel_radius_ * odom_factor_;
    const double v_r      = curr_chassis_data_->right_velocity * kRpmToRads * wheel_radius_ * odom_factor_;
    const double v_l_prev = prev_chassis_data_->left_velocity  * kRpmToRads * wheel_radius_ * odom_factor_;
    const double v_r_prev = prev_chassis_data_->right_velocity * kRpmToRads * wheel_radius_ * odom_factor_;
    speed_v_y_ = 0.0;  // 差速底盘无侧向速度；该字段只在全向路径更新

    // 预先计算平均线速度和角速度（梯形积分，omega 使用有效轴距 b_eff）
    const double speed_avg = 0.5 * (0.5 * (v_l + v_r) + 0.5 * (v_l_prev + v_r_prev));
    const double omega_enc = (v_r - v_l) / wheel_track_eff_;
    const double omega_enc_prev = (v_r_prev - v_l_prev) / wheel_track_eff_;
    const double omega_enc_avg = 0.5 * (omega_enc + omega_enc_prev);

    // 检查 IMU 可用性
    const bool imu_available = IsImuAvailable();

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

    // ===== EKF 融合路径 =====
    if (use_ekf_fusion_ && ekf_) {
        double omega_z_imu = 0.0;
        bool imu_valid = false;
        {
            std::lock_guard<std::mutex> lk(imu_yaw_mutex_);
            if (curr_imu_ != nullptr && imu_available) {
                omega_z_imu = curr_imu_->angular_velocity.z * imu_factor_;
                imu_valid = true;
            }
        }

        // v 与 ω 都用梯形均值（区间中点）：量测模型假设二者同时刻，
        // 混用末端瞬时值会在角加速阶段给新息掺入 ω̇·dt/2 的假残差，抬高 Mahalanobis 距离
        is_aerial_ = ekf_->Step(dt,
                                speed_avg,
                                omega_enc_avg,
                                omega_z_imu,
                                imu_valid,
                                is_stopped);

        {
            std::lock_guard<std::mutex> lk(imu_yaw_mutex_);
            odom_x_     = ekf_->GetX();
            odom_y_     = ekf_->GetY();
            odom_theta_ = ekf_->GetTheta();
        }
        speed_v_ = ekf_->GetV();
        omega_r_ = ekf_->GetOmega();

        if (is_aerial_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Aerial/slip detected! Encoder unreliable, expanding covariance.");
        }

        prev_chassis_data_ = curr_chassis_data_;
        curr_chassis_data_.reset();
        return;
    }
    // ===== 非EKF路径（原有逻辑） =====

    // 记录积分前的角度
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
    // else: 如果使用 IMU，角度已经在 IntegrateImuOnce 中更新了；静止时不积分

    // 获取积分后的角度
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
    // 更新位置
    odom_x_ += delta_s * std::cos(theta_mid);
    odom_y_ += delta_s * std::sin(theta_mid);

    // 更新速度（用于发布）
    speed_v_ = speed_avg;

    // 非 EKF 模式协方差：与 EKF 共用 Q 累积位姿不确定度
    dr_cov_.Propagate(ekf_params_, dt, is_stopped);
    last_imu_available_ = imu_available;

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

void ChassisNode::PublishMotorFeedback(const ChassisFeedback& chassis_data) {
    chassis_interfaces::msg::ChassisMotorFeedback msg;
    // 用节点时钟做消息时间戳；共享内存自带的时间戳单独放 shm_timestamp。
    // 两者不同源（写入方在 EtherCAT 侧），混用会引入未知偏差——R08 未解决前尤其不能混。
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = base_frame_;
    msg.shm_timestamp = chassis_data.timestamp;

    msg.robot_overcurrent   = chassis_data.robot_overcurrent;
    msg.status_word         = chassis_data.status_word;
    msg.driver_temperature  = chassis_data.driver_temperature;

    if (chassis_model_ == 1) {
        // ---- 全向：4 舵轮，转向角 + 轮速 ----
        msg.chassis_type = chassis_interfaces::msg::ChassisMotorFeedback::CHASSIS_OMNI;
        msg.position.resize(OMNI_MOTOR_NUM);
        msg.speed_rpm.resize(OMNI_MOTOR_NUM);
        msg.speed_mps.resize(OMNI_MOTOR_NUM);
        for (size_t i = 0; i < OMNI_MOTOR_NUM; ++i) {
            msg.position[i]  = chassis_data.chassis_motor_direction[i];   // rad
            msg.speed_rpm[i] = chassis_data.chassis_motor_velocity[i];    // RPM
            // 0.001 是 mm→m（OMNI_CHASSIS_WHEEL_D 以毫米计），/60 是每分钟→每秒
            msg.speed_mps[i] = chassis_data.chassis_motor_velocity[i] * 0.001 * OMNI_CHASSIS_WHEEL_D * M_PI / 60.0 * odom_factor_;
        }
        msg.wheel_motor_current.resize(OMNI_MOTOR_NUM);
        msg.steering_motor_current.resize(OMNI_MOTOR_NUM);
        for (size_t i = 0; i < OMNI_MOTOR_NUM; ++i) {
            msg.wheel_motor_current[i]    = chassis_data.wheel_motor_current[i];
            msg.steering_motor_current[i] = chassis_data.steering_motor_current[i];
        }
        // 错误码与电机温度：共享内存只有 error_code_left/right 与 left/right_temperature
        // 两路（差速语义），全向 4 轮的对应字段硬件未提供，故**留空数组**而非填 0。
        //   · 不填 left/right 到前两位：轮索引对不上（左/右 vs 左前/右前/左后/右后），
        //     消费方无从察觉这是错位的
        //   · 不填 0：错误码 0 表示"无错误"、温度 0 也是合法值，填 0 会让消费方
        //     无法区分"没有这个数据"与"数据就是 0"
        // 空数组则语义明确：本机型不提供此项。与差速下 position/steering 留空同一原则。
        msg.error_code.clear();
        msg.motor_temperature.clear();
    } else {
        // ---- 差速：2 驱动轮，无转向电机 ----
        msg.chassis_type = chassis_interfaces::msg::ChassisMotorFeedback::CHASSIS_DIFF_DRIVE;
        // position 留空：差速没有转向电机。不要拿它塞轮子累计转角，那是另一个语义。
        msg.position.clear();
        msg.steering_motor_current.clear();

        constexpr double kRpmToRads = 2.0 * M_PI / 60.0;
        msg.speed_rpm = {chassis_data.left_velocity, chassis_data.right_velocity};          // RPM
        // 与积分所用轮速同一量：含 odom_factor，否则 NIS 新息里会多出一个常数比例项
        msg.speed_mps = {chassis_data.left_velocity  * kRpmToRads * wheel_radius_ * odom_factor_,
                         chassis_data.right_velocity * kRpmToRads * wheel_radius_ * odom_factor_};
        msg.wheel_motor_current = {chassis_data.left_current, chassis_data.right_current};
        msg.error_code          = {chassis_data.error_code_left, chassis_data.error_code_right};
        msg.motor_temperature   = {chassis_data.left_temperature, chassis_data.right_temperature};
    }

    motor_feedback_pub_->publish(msg);
}

void ChassisNode::IntegrateOmniOdomOnce(const ChassisFeedback& chassis_data) {
    bool valid_data = (chassis_data.timestamp > 0);
    if (!valid_data) {
        return;
    }

    // 车体几何参数（米）
    constexpr double L = CHASSIS_SIZE_L * 0.001;  // m
    constexpr double W = CHASSIS_SIZE_W * 0.001;  // m

    curr_chassis_data_ = std::make_shared<ChassisFeedback>(chassis_data);

    // 第一帧：只缓存，不积分
    if (!prev_chassis_data_) {
        prev_chassis_data_ = curr_chassis_data_;
        return;
    }

    // ⚠️ clock type 必须是 STEADY：chassis_data.timestamp 实测为 CLOCK_MONOTONIC
    // （2026-09-01 三时钟对比，与 CLOCK_MONOTONIC 差 0.000 s）。标成 SYSTEM_TIME 时
    // 数值上不影响 dt（同源相减），但会让「拿它与 ROS 时间比较」的误用**静默通过**；
    // 标成 STEADY 后 rclcpp 会直接抛异常，误用立即暴露。
    rclcpp::Time curr_time(curr_chassis_data_->timestamp, RCL_STEADY_TIME);
    rclcpp::Time prev_time(prev_chassis_data_->timestamp, RCL_STEADY_TIME);
    const double dt = (curr_time - prev_time).seconds();

    if (dt <= 0.0 || dt > 0.5) {
        prev_chassis_data_ = curr_chassis_data_;
        curr_chassis_data_.reset();
        return;
    }

    // ========= 1) 由四个舵轮速度/角度解算底盘速度（base_link坐标系） =========
    // 说明：这里沿用你原来的速度标定（rpm -> m/s 的换算），后续建议改成参数。
    double vx_i[4]{};
    double vy_i[4]{};
    double vx_i_prev[4]{};
    double vy_i_prev[4]{};

    for (int i = 0; i < 4; ++i) {
        // 当前帧
        double speed = curr_chassis_data_->chassis_motor_velocity[i];
        double angle = curr_chassis_data_->chassis_motor_direction[i];
        speed = speed * 0.001 * OMNI_CHASSIS_WHEEL_D * M_PI / 60.0 * odom_factor_;  // TODO: 轮径改为参数
        vx_i[i] = speed * std::cos(angle);
        vy_i[i] = speed * std::sin(angle);

        // 上一帧
        double speed_prev = prev_chassis_data_->chassis_motor_velocity[i];
        double angle_prev = prev_chassis_data_->chassis_motor_direction[i];
        speed_prev = speed_prev * 0.001 * OMNI_CHASSIS_WHEEL_D * M_PI / 60.0 * odom_factor_;  // TODO: 轮径改为参数
        vx_i_prev[i] = speed_prev * std::cos(angle_prev);
        vy_i_prev[i] = speed_prev * std::sin(angle_prev);
    }

    // 平移速度：四轮平均
    const double vx = 0.25 * (vx_i[0] + vx_i[1] + vx_i[2] + vx_i[3]);
    const double vy = 0.25 * (vy_i[0] + vy_i[1] + vy_i[2] + vy_i[3]);
    const double vx_prev = 0.25 * (vx_i_prev[0] + vx_i_prev[1] + vx_i_prev[2] + vx_i_prev[3]);
    const double vy_prev = 0.25 * (vy_i_prev[0] + vy_i_prev[1] + vy_i_prev[2] + vy_i_prev[3]);

    // 角速度：前后 & 左右对称解算（与你原逻辑一致，只是加了上一帧做梯形积分）
    const double vw_x = (vx_i[1] + vx_i[3] - vx_i[0] - vx_i[2]) / (2.0 * W);
    const double vw_y = (vy_i[0] + vy_i[1] - vy_i[2] - vy_i[3]) / (2.0 * L);
    const double vw = 0.5 * (vw_x + vw_y);

    const double vw_x_prev = (vx_i_prev[1] + vx_i_prev[3] - vx_i_prev[0] - vx_i_prev[2]) / (2.0 * W);
    const double vw_y_prev = (vy_i_prev[0] + vy_i_prev[1] - vy_i_prev[2] - vy_i_prev[3]) / (2.0 * L);
    const double vw_prev = 0.5 * (vw_x_prev + vw_y_prev);

    /* ===== 2. 静止判断（与差速一致：当前 + 上一帧） ===== */
    const bool linear_stopped = std::abs(vx) < speed_deadzone_ && std::abs(vy) < speed_deadzone_ &&
                                std::abs(vx_prev) < speed_deadzone_ && std::abs(vy_prev) < speed_deadzone_;

    bool angular_stopped = false;
    const bool imu_available = IsImuAvailable();

    if (imu_available) {
        std::lock_guard<std::mutex> lk(imu_yaw_mutex_);
        angular_stopped = std::abs(curr_imu_->angular_velocity.z) < omega_deadzone_;
    } else {
        angular_stopped = std::abs(vw) < omega_deadzone_ && std::abs(vw_prev) < omega_deadzone_;
    }

    const bool is_stopped = linear_stopped && angular_stopped;

    {
        std::lock_guard<std::mutex> lk(imu_yaw_mutex_);
        is_stopped_ = is_stopped;
    }

    // ========= 3) 角度积分（优先IMU；否则用编码器角速度vw） =========
    double theta_start = 0.0;
    {
        std::lock_guard<std::mutex> lk(imu_yaw_mutex_);
        theta_start = odom_theta_;
    }

    if (!is_stopped && !imu_available) {
        // 运动中且不使用 IMU，或 IMU 不可用时，使用底盘解算角速度
        const double omega_enc = 0.5 * (vw + vw_prev);  // 梯形法
        const double delta_theta = omega_enc * dt;

        std::lock_guard<std::mutex> lk(imu_yaw_mutex_);
        odom_theta_ += delta_theta;
        odom_theta_ = std::atan2(std::sin(odom_theta_), std::cos(odom_theta_));
    }

    double theta_end = 0.0;
    {
        std::lock_guard<std::mutex> lk(imu_yaw_mutex_);
        theta_end = odom_theta_;
    }

    const double theta_mid =
        std::atan2(std::sin(0.5 * (theta_start + theta_end)), std::cos(0.5 * (theta_start + theta_end)));

    // ========= 4) 位置积分 =========
    // 先在 base 坐标系对 vx,vy 做梯形积分得到 body 位移，再旋转到 odom 坐标系
    const double dx_body = 0.5 * (vx + vx_prev) * dt;
    const double dy_body = 0.5 * (vy + vy_prev) * dt;

    const double c = std::cos(theta_mid);
    const double s = std::sin(theta_mid);

    const double dx_odom = dx_body * c - dy_body * s;
    const double dy_odom = dx_body * s + dy_body * c;

    odom_x_ += dx_odom;
    odom_y_ += dy_odom;

    // ========= 5) 更新速度（用于发布） =========
    speed_v_ = vx;
    dr_cov_.Propagate(ekf_params_, dt, is_stopped);
    last_imu_available_ = imu_available;
    speed_v_y_ = vy;

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
            omega_r_ = vw;
            if (std::abs(omega_r_) < omega_deadzone_) {
                omega_r_ = 0.0;
            }
        }
    }

    // 更新时间基
    prev_chassis_data_ = curr_chassis_data_;
    curr_chassis_data_.reset();
}

void ChassisNode::PublishOdometry() {
    nav_msgs::msg::Odometry odom_msg;
    // 节点时钟（RCL_ROS_TIME）：仿真下遵守 use_sim_time，与订阅方同基；
    // rclcpp::Clock() 默认 RCL_SYSTEM_TIME，会让 odom/TF 与 IMU、上层估计器时间基错开。
    const rclcpp::Time curr_time = this->get_clock()->now();
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
    odom_msg.twist.twist.linear.y = speed_v_y_;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = omega_r_;

    // 填充协方差
    if (use_ekf_fusion_ && ekf_) {
        // EKF 状态协方差：从 EKF 对角线获取
        const auto &P       = ekf_->GetCovariance();
        const double scale  = is_aerial_ ? 100.0 : 1.0;
        auto &Pc            = odom_msg.pose.covariance;
        auto &Tc            = odom_msg.twist.covariance;
        Pc[0]  = P(0, 0) * scale;  // x-x
        Pc[7]  = P(1, 1) * scale;  // y-y
        Pc[35] = P(2, 2) * scale;  // yaw-yaw
        Tc[0]  = P(3, 3) * scale;  // vx-vx
        Tc[35] = P(4, 4) * scale;  // wz-wz
    } else {
        // 非 EKF 模式：不能发全 0——上游会把 <1e-9 的方差夹成 1e-9，等价「无限精确」，
        // 其它传感器权重归零；NEES 也直接除零。按 EKF 退化形态填，参数与量纲两模式一致。
        auto &Pc = odom_msg.pose.covariance;
        auto &Tc = odom_msg.twist.covariance;
        Pc[0]  = dr_cov_.var_xy;
        Pc[7]  = dr_cov_.var_xy;
        Pc[35] = dr_cov_.var_theta;
        Tc[0]  = dr_cov_.TwistVarV(ekf_params_);
        Tc[35] = dr_cov_.TwistVarOmega(ekf_params_, last_imu_available_);
    }

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
    const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "Parameters updated successfully";

    for (const auto& changed_parameter : parameters) {
        if (changed_parameter.get_name() == "odom_factor") {
            odom_factor_ = changed_parameter.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated odom_factor: %.3f", odom_factor_);
        } else if (changed_parameter.get_name() == "imu_factor") {
            imu_factor_ = changed_parameter.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated imu_factor: %.3f", imu_factor_);
        } else if (changed_parameter.get_name() == "imu_max_buff_size") {
            imu_max_buff_size_ = changed_parameter.as_int();
            ApplyImuProcessorParams();
            RCLCPP_INFO(this->get_logger(), "Updated imu_max_buff_size: %d", imu_max_buff_size_);
        } else if (changed_parameter.get_name() == "acc_excite_threshold") {
            acc_excite_threshold_ = changed_parameter.as_double();
            ApplyImuProcessorParams();
            RCLCPP_INFO(this->get_logger(), "Updated acc_excite_threshold: %.3f", acc_excite_threshold_);
        } else if (changed_parameter.get_name() == "gyro_excite_threshold") {
            gyro_excite_threshold_ = changed_parameter.as_double();
            ApplyImuProcessorParams();
            RCLCPP_INFO(this->get_logger(), "Updated gyro_excite_threshold: %.3f", gyro_excite_threshold_);
        } else if (changed_parameter.get_name() == "imu_reinit_duration") {
            imu_reinit_duration_ = changed_parameter.as_double();
            ApplyImuProcessorParams();
            RCLCPP_INFO(this->get_logger(), "Updated imu_reinit_duration: %.3f", imu_reinit_duration_);
        } else if (changed_parameter.get_name() == "gyro_bias_warn_threshold") {
            gyro_bias_warn_threshold_ = changed_parameter.as_double();
            ApplyImuProcessorParams();
            RCLCPP_INFO(this->get_logger(), "Updated gyro_bias_warn_threshold: %.4f rad/s", gyro_bias_warn_threshold_);
        } else if (changed_parameter.get_name() == "acc_magnitude_tolerance") {
            acc_magnitude_tolerance_ = changed_parameter.as_double();
            ApplyImuProcessorParams();
            RCLCPP_INFO(this->get_logger(), "Updated acc_magnitude_tolerance: %.3f m/s^2", acc_magnitude_tolerance_);
        } else if (changed_parameter.get_name() == "gravity_magnitude") {
            gravity_magnitude_ = changed_parameter.as_double();
            imu_processor_->SetGravityMagnitude(gravity_magnitude_);
            RCLCPP_INFO(this->get_logger(), "Updated gravity_magnitude: %.4f", gravity_magnitude_);
        } else if (changed_parameter.get_name() == "wheel_track") {
            wheel_track_ = changed_parameter.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated wheel_track: %.4f m", wheel_track_);
        } else if (changed_parameter.get_name() == "wheel_radius") {
            wheel_radius_ = changed_parameter.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated wheel_radius: %.5f m", wheel_radius_);
        } else if (changed_parameter.get_name() == "wheel_track_eff") {
            wheel_track_eff_ = changed_parameter.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated wheel_track_eff: %.4f m", wheel_track_eff_);
        } else if (changed_parameter.get_name() == "odom_rate_hz") {
            odom_rate_hz_ = changed_parameter.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated odom_rate_hz: %.1f", odom_rate_hz_);
            // 可选：根据新的频率重启定时器
            const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, odom_rate_hz_));
            timer_->reset();
            timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                             std::bind(&ChassisNode::onTimer, this));
        } else if (changed_parameter.get_name() == "odom_mode") {
            const auto parsed = ParseOdomMode(changed_parameter.as_string());
            if (!parsed) {
                result.successful = false;
                result.reason = "Invalid odom_mode '" + changed_parameter.as_string() + "' (valid: " + kOdomModeChoices + ")";
                RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
                return result;
            }
            if (*parsed != odom_mode_) {
                ApplyOdomMode(*parsed);
            }
            // EKF 参数热更新。名称必须与上面的 declare_parameter 完全一致：
            // 早期这里写作 "ekf.q_xy" 而 declare 用的是 "ekf_q_xy"，ROS2 不允许设置
            // 未声明的参数，热更新因此从未生效过。
        } else if (changed_parameter.get_name() == "ekf_q_xy") {
            ekf_params_.q_xy = changed_parameter.as_double();
            if (ekf_) ekf_->SetParams(ekf_params_);
            RCLCPP_INFO(this->get_logger(), "Updated ekf_q_xy: %.3e m^2/s", ekf_params_.q_xy);
        } else if (changed_parameter.get_name() == "ekf_q_theta") {
            ekf_params_.q_theta = changed_parameter.as_double();
            if (ekf_) ekf_->SetParams(ekf_params_);
            RCLCPP_INFO(this->get_logger(), "Updated ekf_q_theta: %.3e rad^2/s", ekf_params_.q_theta);
        } else if (changed_parameter.get_name() == "ekf_q_v") {
            ekf_params_.q_v = changed_parameter.as_double();
            if (ekf_) ekf_->SetParams(ekf_params_);
            RCLCPP_INFO(this->get_logger(), "Updated ekf_q_v: %.3e (m/s)^2/s", ekf_params_.q_v);
        } else if (changed_parameter.get_name() == "ekf_q_omega") {
            ekf_params_.q_omega = changed_parameter.as_double();
            if (ekf_) ekf_->SetParams(ekf_params_);
            RCLCPP_INFO(this->get_logger(), "Updated ekf_q_omega: %.3e (rad/s)^2/s", ekf_params_.q_omega);
        } else if (changed_parameter.get_name() == "ekf_r_v") {
            ekf_params_.r_v = changed_parameter.as_double();
            if (ekf_) ekf_->SetParams(ekf_params_);
            RCLCPP_INFO(this->get_logger(), "Updated ekf_r_v: %.3e (m/s)^2", ekf_params_.r_v);
        } else if (changed_parameter.get_name() == "ekf_r_omega") {
            ekf_params_.r_omega = changed_parameter.as_double();
            if (ekf_) ekf_->SetParams(ekf_params_);
            RCLCPP_INFO(this->get_logger(), "Updated ekf_r_omega: %.3e (rad/s)^2", ekf_params_.r_omega);
        } else if (changed_parameter.get_name() == "ekf_q_omega_imu") {
            ekf_params_.q_omega_imu = changed_parameter.as_double();
            if (ekf_) ekf_->SetParams(ekf_params_);
            RCLCPP_INFO(this->get_logger(), "Updated ekf_q_omega_imu: %.3e (rad/s)^2/s", ekf_params_.q_omega_imu);
        } else if (changed_parameter.get_name() == "ekf_r_zupt") {
            ekf_params_.r_zupt = changed_parameter.as_double();
            if (ekf_) ekf_->SetParams(ekf_params_);
            RCLCPP_INFO(this->get_logger(), "Updated ekf_r_zupt: %.3e", ekf_params_.r_zupt);
        } else if (changed_parameter.get_name() == "ekf_q_scale_stopped") {
            ekf_params_.q_scale_stopped = changed_parameter.as_double();
            if (ekf_) ekf_->SetParams(ekf_params_);
            RCLCPP_INFO(this->get_logger(), "Updated ekf_q_scale_stopped: %.4f", ekf_params_.q_scale_stopped);
        } else if (changed_parameter.get_name() == "speed_deadzone") {
            speed_deadzone_ = changed_parameter.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated speed_deadzone: %.5f m/s", speed_deadzone_);
        } else if (changed_parameter.get_name() == "omega_deadzone") {
            omega_deadzone_ = changed_parameter.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated omega_deadzone: %.5f rad/s", omega_deadzone_);
        } else if (changed_parameter.get_name() == "imu_timeout") {
            imu_timeout_ = changed_parameter.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated imu_timeout: %.3f s", imu_timeout_);
        } else if (changed_parameter.get_name() == "odom_stale_timeout") {
            odom_stale_timeout_ = changed_parameter.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated odom_stale_timeout: %.3f s", odom_stale_timeout_);
        }
    }
    // 不再无条件 ResetOdometry()，避免每次参数更新都清零里程计
    return result;
}

}  // namespace chassis_node
