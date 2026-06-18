#include "w1_server.h"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <set>
#include <sstream>

namespace robot_server_lib {

RobotServer::RobotServer(std::filesystem::path robot_cfg_path, bool is_sim)
    : RobotServerBase(robot_cfg_path, is_sim), line_interpolator_(manager_dt_micro / torque_ctrl_dt_micro) {
    LOG_INFO << "w1_server version: " << W1_SERVER_VERSION;

    // 先初始化ndof_config_, 后续再自动识别模式
    ndof_config_ = ndof_robot;
    sensor_force_.setZero();
    sensor_wrench_.setZero();
    sensor_wrench_gravity_.setZero();
    raw_sensor_wrench_.setZero();
    latest_torque_ext_.setZero();
    latest_arm_forward_torque_.setZero();
    latest_tool_compensation_torque_.setZero();
    latest_joint_admittance_q_offset_.setZero();
    latest_joint_admittance_dq_offset_.setZero();
    latest_joint_position_ref_admittance_.setZero();
    latest_joint_impedance_error_after_admittance_.setZero();
    sensor_zero_offset_.setZero();
    std::vector<std::string> tool_compensation_warnings;
    const auto tool_compensation_load_result =
        dynamics::tool_compensation::LoadToolGravityCompensationConfig(robot_cfg_path, &tool_compensation_warnings);
    tool_compensation_config_ = tool_compensation_load_result.config;
    inertia_primary_joint_id_resolved_ = false;
    for (int i = 0; i < 7; ++i) inertia_primary_joint_id_[i] = -1;
    for (const auto& warning : tool_compensation_warnings) {
        RCLCPP_WARN(this->get_logger(), "%s", warning.c_str());
        LOG_WARN << warning;
    }
    if (tool_compensation_config_.gravity_enabled) {
        RCLCPP_INFO(this->get_logger(), "Tool gravity compensation enabled, mass_kg=%.4f, gravity=%.4f, model_sign=%d",
                    tool_compensation_config_.mass_kg, tool_compensation_config_.gravity_magnitude,
                    tool_compensation_config_.model_sign);
    }
    if (tool_compensation_config_.inertia_enabled) {
        RCLCPP_INFO(this->get_logger(),
                    "Tool inertia compensation enabled (primary). I_C is COM-referenced and expressed in EE frame "
                    "(kg*m^2); diag=[%.6e, %.6e, %.6e]",
                    tool_compensation_config_.inertia_com(0, 0), tool_compensation_config_.inertia_com(1, 1),
                    tool_compensation_config_.inertia_com(2, 2));
        RCLCPP_INFO(this->get_logger(),
                    "Tool inertia compensation filter cutoff: %.2f Hz (1 kHz sampling, 2nd-order critically damped)",
                    tool_compensation_config_.inertia_filter_cutoff_hz);
    }
    dynamics::tool_compensation::InitializeToolCompensationFilters(tool_compensation_state_,
                                                                   tool_compensation_config_.inertia_filter_cutoff_hz);
    dynamics::tool_compensation::ResetToolInertiaCompensationState(tool_compensation_state_);

    // 创建 /feedback/joint 发布器
    feedback_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/feedback/joint", 10);

    // 创建 /imu/chassis 发布器 (替代原有 imu node)
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/chassis", 20);

    // 创建 /feedback/joint_velocity_torque_limit 发布器
    velocity_torque_limit_feedback_publisher_ =
        this->create_publisher<JointVelocityTorqueLimitFeedbackMsg>("/feedback/joint_velocity_torque_limit", 10);

    // 创建 /control/joint 订阅器
    control_subscriber_ = this->create_subscription<JointControlMsg>(
        "/control/joint", 10, std::bind(&RobotServer::ControlCallback, this, std::placeholders::_1));

    // 创建 /control/joint_position 订阅器
    position_control_subscriber_ = this->create_subscription<JointPositionControlMsg>(
        "/control/joint_position", 10, std::bind(&RobotServer::PositionControlCallback, this, std::placeholders::_1));

    // 创建 /control/omni_chassis 订阅器
    omni_chassis_control_subscriber_ = this->create_subscription<ChassisWholeControlMsg>(
        "/control/omni_chassis", 10, std::bind(&RobotServer::OmniChassisControlCallback, this, std::placeholders::_1));

    // 创建 /control/omni_chassis_motors 订阅器
    omni_chassis_motor_control_subscriber_ = this->create_subscription<ChassisMotorControlMsg>(
        "/control/omni_chassis_motors", 10,
        std::bind(&RobotServer::OmniChassisMotorControlCallback, this, std::placeholders::_1));

    // 创建 /control/cmd_vel 订阅器
    diff_chassis_control_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/control/cmd_vel", 10, std::bind(&RobotServer::DiffChassisControlCallback, this, std::placeholders::_1));
    chassis_shaper_ = std::make_shared<EIInputShaper>(chassis_shaper_type_, DiffChassisNatureFreq, torque_ctrl_dt,
                                                      DiffChassisMaxRampRpm);

    force_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "force_array", rclcpp::QoS(10), std::bind(&RobotServer::ForceArrayCallback, this, std::placeholders::_1));

    // 创建 /control/debug 订阅器
    debug_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "/control/debug", 10, std::bind(&RobotServer::DebugCallback, this, std::placeholders::_1));

    // 创建定时器，定时发布feedback 定时器回调函数
    control_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() { PublishFeedback(); });

    // 创建定时器，定时发布 IMU 数据 (2ms = 500Hz，与原 imu node 一致)
    imu_timer_ = this->create_wall_timer(std::chrono::milliseconds(2), [this]() { PublishImu(); });

    // 初始化巴特沃斯滤波器和滤波开关
    for (int i = 0; i < ndof_robot; ++i) {
        butterworth_filter_[i] = std::make_shared<ButterworthFilter>(
            this->robot_model_.robot_property_default.butterworth_filter_frequency[i]);
        input_shapers_[i] = std::make_shared<EIInputShaper>(
            input_shaper_type_, this->robot_model_.robot_property_default.input_shaper_frequency[i], torque_ctrl_dt,
            input_shaper_max_ramp_rad_per_sec);
    }

    // 初始化力传感器
    primary_sensor_ = std::make_shared<SriForceSensor>();
    primary_sensor_->SetToolGravityCompensationForZeroCalibration(
        tool_compensation_config_.gravity_enabled, tool_compensation_config_.mass_kg, tool_compensation_config_.com_m,
        tool_compensation_config_.gravity_magnitude, tool_compensation_config_.model_sign,
        [this]() -> std::optional<Eigen::Matrix3d> { return GetEERotationWorldFromEE(0); });
    if (!primary_sensor_->Init(460800)) {
        RCLCPP_WARN(this->get_logger(), "Failed to initialize primary force sensor");
        LOG_WARN << "Failed to initialize primary force sensor";
        primary_sensor_ = nullptr;
    } else {
        Vector6d primary_zero_offset = Vector6d::Zero();
        primary_sensor_->GetZeroOffset(&primary_zero_offset);
        sensor_zero_offset_.head<6>() = primary_zero_offset;
        sensor_zero_offset_valid_[0] = primary_sensor_->HasValidZeroOffset();
        sensor_zero_offset_samples_[0] = primary_sensor_->GetZeroOffsetSampleCount();
        RCLCPP_INFO(this->get_logger(), "Primary force sensor initialized successfully");
        RCLCPP_INFO(this->get_logger(),
                    "Primary force sensor zero offset: valid=%d, samples=%d, [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                    sensor_zero_offset_valid_[0], sensor_zero_offset_samples_[0], primary_zero_offset[0],
                    primary_zero_offset[1], primary_zero_offset[2], primary_zero_offset[3], primary_zero_offset[4],
                    primary_zero_offset[5]);
        LOG_INFO << "Primary force sensor initialized successfully";
        LOG_INFO << "Primary force sensor zero offset: valid=" << sensor_zero_offset_valid_[0]
                 << ", samples=" << sensor_zero_offset_samples_[0] << ", offset=" << primary_zero_offset.transpose();
    }
    // secondary_sensor_ = std::make_shared<ForceSensorReader>("/dev/ttyUSB1");  // 暂未使用
    if (!primary_sensor_ && !secondary_sensor_) {
        tool_compensation_config_.gravity_enabled = false;
    }
    this->Run();
}

void RobotServer::ForceArrayCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    // 示例：若期望 3 维力数据
    if (msg->data.size() < 3) {
        RCLCPP_WARN(this->get_logger(), "Force array size %zu < 3", msg->data.size());
        return;
    }
    // 使用数据，例如拷贝前 3 个
    for (size_t i = 0; i < 3; ++i) sensor_force_[i] = msg->data[i];
    RCLCPP_INFO(this->get_logger(), "Received force array: [%f, %f, %f]", sensor_force_[0], sensor_force_[1],
                sensor_force_[2]);
}

std::optional<Eigen::Matrix3d> RobotServer::GetEERotationWorldFromEE(int arm_index) const {
    // PlannerManager 在 manager_state_ ∈ {Init, Error} 时仍然把 14 维 ee_pose_world_measurement
    // 写入 buffer, 但里面是默认构造的 (0,0,0, 1,0,0,0) —— 看起来 valid 实际是占位 identity.
    // 不加这条门, 力传感器零偏会在 R=I 假设下结算, 残留 ~m·g 的工具重力分量.
    if (this->manager_state_ != Status::Idle && this->manager_state_ != Status::Running) {
        LOG_WARN_THROTTLE(5000) << "tool compensation waiting for ee_pose_world: manager_state="
                                << static_cast<int>(this->manager_state_);
        return std::nullopt;
    }
    const auto& ee_pose_world = this->manager_info_.ee_pose_world_measurement;
    const auto rotation_result = utils::GetEERotationWorldFromPoseVector(ee_pose_world, arm_index);
    if (!rotation_result.rotation) {
        switch (rotation_result.reason) {
            case utils::EERotationParseError::InvalidArmIndex: {
                LOG_WARN_THROTTLE(5000) << "tool compensation failed: invalid arm_index=" << arm_index;
                break;
            }
            case utils::EERotationParseError::InsufficientSize: {
                LOG_WARN_THROTTLE(5000) << "tool compensation failed: ee_pose_world_measurement size="
                                        << ee_pose_world.size() << ", required=" << rotation_result.required_size
                                        << ", arm_index=" << arm_index;
                break;
            }
            case utils::EERotationParseError::NonFiniteQuaternion: {
                LOG_WARN_THROTTLE(5000)
                    << "tool compensation failed: ee_pose_world quaternion is non-finite, arm_index=" << arm_index;
                break;
            }
            case utils::EERotationParseError::NearZeroQuaternion: {
                LOG_WARN_THROTTLE(5000)
                    << "tool compensation failed: ee_pose_world quaternion norm is near zero, arm_index=" << arm_index;
                break;
            }
            case utils::EERotationParseError::None:
                break;
        }
        return std::nullopt;
    }
    return rotation_result.rotation;
}

void RobotServer::TorqueControlThreadFunc() {
    if (!is_sim_) {
        Setup_Realtime();
    }
    TorqueControllerManager torque_controller_manager(this->robot_model_, this->manager_buffers_, this->robot_buffers_);
    // 初始化共享内存
    auto& feedback_shm_instance = FeedbackSharedMemory::GetInstance();
    auto& control_shm_instance = ControlSharedMemory::GetInstance();

    Int64 loop_counter = 0;
    this->is_robot_ready_ = false;
    auto next_cycle_time_point = std::chrono::steady_clock::now();
    double usage_sum = 0.0;
    double peak_usage = 0.0;
    int usage_count = 0;

    while (rclcpp::ok()) {
        loop_counter++;
        next_cycle_time_point += std::chrono::microseconds(torque_ctrl_dt_micro);
        this->manager_buffers_.planner_manager_info->PeekLatest(&this->manager_info_);

        auto t_start = std::chrono::steady_clock::now();
        // 力传感器零偏延迟结算: 等到 EE 位姿可用后才完成校准并去除工具重力, 然后镜像 ready 标志.
        if (primary_sensor_) {
            if (primary_sensor_->TryFinalizeBias()) {
                RCLCPP_INFO(this->get_logger(), "Primary force sensor zero offset finalized; samples=%d",
                            primary_sensor_->GetZeroOffsetSampleCount());
                LOG_INFO << "Primary force sensor zero offset finalized; samples="
                         << primary_sensor_->GetZeroOffsetSampleCount();
            }
            primary_sensor_ready_ = primary_sensor_->HasValidZeroOffset();
        } else {
            primary_sensor_ready_ = false;
        }
        if (secondary_sensor_) {
            if (secondary_sensor_->TryFinalizeBias()) {
                RCLCPP_INFO(this->get_logger(), "Secondary force sensor zero offset finalized; samples=%d",
                            secondary_sensor_->GetZeroOffsetSampleCount());
                LOG_INFO << "Secondary force sensor zero offset finalized; samples="
                         << secondary_sensor_->GetZeroOffsetSampleCount();
            }
            secondary_sensor_ready_ = secondary_sensor_->HasValidZeroOffset();
        } else {
            secondary_sensor_ready_ = false;
        }

        // 读取力传感器数据
        Vector6d sensor_data_primary = Vector6d::Zero();
        Vector6d sensor_data_secondary = Vector6d::Zero();
        Vector6d zeroed_sensor_data_primary = Vector6d::Zero();
        Vector6d zeroed_sensor_data_secondary = Vector6d::Zero();
        Vector6d raw_sensor_data_primary = Vector6d::Zero();
        Vector6d raw_sensor_data_secondary = Vector6d::Zero();
        const bool primary_wrench_valid =
            primary_sensor_ && primary_sensor_ready_ && primary_sensor_->PeekLatest(&zeroed_sensor_data_primary);
        const bool secondary_wrench_valid = secondary_sensor_ && secondary_sensor_ready_ &&
                                            secondary_sensor_->PeekLatest(&zeroed_sensor_data_secondary);
        if (primary_sensor_ && primary_sensor_ready_) {
            primary_sensor_->PeekLatestRaw(&raw_sensor_data_primary);
            Vector6d primary_zero_offset = Vector6d::Zero();
            primary_sensor_->GetZeroOffset(&primary_zero_offset);
            sensor_zero_offset_.head<6>() = primary_zero_offset;
            sensor_zero_offset_valid_[0] = primary_sensor_->HasValidZeroOffset();
            sensor_zero_offset_samples_[0] = primary_sensor_->GetZeroOffsetSampleCount();
        }
        if (secondary_sensor_ && secondary_sensor_ready_) {
            secondary_sensor_->PeekLatestRaw(&raw_sensor_data_secondary);
            Vector6d secondary_zero_offset = Vector6d::Zero();
            secondary_sensor_->GetZeroOffset(&secondary_zero_offset);
            sensor_zero_offset_.tail<6>() = secondary_zero_offset;
            sensor_zero_offset_valid_[1] = secondary_sensor_->HasValidZeroOffset();
            sensor_zero_offset_samples_[1] = secondary_sensor_->GetZeroOffsetSampleCount();
        }
        // 工具补偿状态: 用于"启用了但本周期未扣到"的 force-control 安全策略 (见下方清零逻辑).
        bool gravity_applied_primary = false;
        bool gravity_applied_secondary = false;
        bool inertia_applied_primary = false;

        if (primary_wrench_valid) {
            std::string gravity_failure_reason;
            sensor_data_primary = dynamics::tool_compensation::ApplyToolGravityCompensation(
                zeroed_sensor_data_primary, tool_compensation_config_, GetEERotationWorldFromEE(0),
                &gravity_applied_primary, &gravity_failure_reason);
            if (!gravity_applied_primary && !gravity_failure_reason.empty()) {
                LOG_WARN_THROTTLE(1000) << "tool gravity compensation lapsed: " << gravity_failure_reason
                                        << ", arm_index=0";
            }
        }
        if (secondary_wrench_valid) {
            std::string gravity_failure_reason;
            sensor_data_secondary = dynamics::tool_compensation::ApplyToolGravityCompensation(
                zeroed_sensor_data_secondary, tool_compensation_config_, GetEERotationWorldFromEE(1),
                &gravity_applied_secondary, &gravity_failure_reason);
            if (!gravity_applied_secondary && !gravity_failure_reason.empty()) {
                LOG_WARN_THROTTLE(1000) << "tool gravity compensation lapsed: " << gravity_failure_reason
                                        << ", arm_index=1";
            }
        }
        Vector6d sensor_data_primary_before_inertia = sensor_data_primary;

        // 使用临时变量接收共享内存数据
        RobotFeedback feedback_temp{};
        bool ret = feedback_shm_instance.PeekLatest(&feedback_temp) && feedback_temp.cnt > 0;

        // 根据模式排序反馈数据到ndof_robot个关节标准顺序
        if (ret) {
            if (!robot_mode_auto_detected_ && RobotMode::Unknown != DetectRobotModeFromFeedback(feedback_temp)) {
                robot_mode_auto_detected_ = true;
            }

            if (SortFeedbackByJointName(feedback_temp)) {
                robot_feedback_ = feedback_temp;
            } else {
                ret = false;
            }
        } else {
            robot_feedback_.robot_status_word = 0;
            for (int i = 0; i < MAX_JOINT_NUM; ++i) {
                robot_feedback_.al_status[i] = 0;
                robot_feedback_.joint_feedback[i].joint_error_code = 0;
                robot_feedback_.joint_feedback[i].joint_temperature = 0;
            }
        }

        // 工具惯量补偿 (仅 primary): 在重力补偿之后, 在 sensor_data_primary 写入测量之前.
        // 用单一布尔门控所有前置条件, 防止后续修改任一条件时漏掉 Reset 路径.
        bool should_run_inertia = false;
        if (tool_compensation_config_.inertia_enabled) {
            // 首次解析 primary 7 个关节在 whole-body 模型下的 joint id, 之后从缓存读.
            // 失败则永久关闭惯量补偿 (URDF/yaml 不匹配是配置错, 不是运行时偶发).
            if (!inertia_primary_joint_id_resolved_) {
                const auto& wb_model = *robot_model_.model_ptr;
                const auto& primary_names = robot_model_.params_ptr->joints_primary_names;
                bool ok = (primary_names.size() == 7);
                for (int i = 0; ok && i < 7; ++i) {
                    const int joint_id = wb_model.getJointId(primary_names[i]) - 1;
                    if (joint_id < 0 || joint_id >= ndof_robot) {
                        ok = false;
                        break;
                    }
                    inertia_primary_joint_id_[i] = joint_id;
                }
                inertia_primary_joint_id_resolved_ = ok;
                if (!ok) {
                    RCLCPP_WARN(this->get_logger(),
                                "tool inertia compensation: failed to resolve primary joint ids, disabling");
                    LOG_WARN << "tool inertia compensation: failed to resolve primary joint ids, disabling";
                    tool_compensation_config_.inertia_enabled = false;
                }
            }

            should_run_inertia = ret && primary_wrench_valid && robot_model_.primary_arm_model_ptr &&
                                 tool_compensation_config_.inertia_enabled && inertia_primary_joint_id_resolved_;

            if (should_run_inertia) {
                sensor_data_primary = dynamics::tool_compensation::ApplyToolGravityMatchedFilterPrimary(
                    sensor_data_primary, tool_compensation_state_);
                sensor_data_primary_before_inertia = sensor_data_primary;

                // 叠加手动输入的力数据；没有六维力传感器时保持控制律输入 wrench 为零。
                sensor_data_primary.head<3>() += sensor_force_;

                for (int i = 0; i < 7; ++i) {
                    const int jid = inertia_primary_joint_id_[i];
                    inertia_q_primary_buf_[i] = robot_feedback_.joint_feedback[jid].joint_position;
                    inertia_dq_primary_buf_[i] = robot_feedback_.joint_feedback[jid].joint_velocity;
                }
                std::string inertia_failure_reason;
                sensor_data_primary = dynamics::tool_compensation::ApplyToolInertiaCompensationPrimary(
                    sensor_data_primary, *robot_model_.primary_arm_model_ptr, inertia_q_primary_buf_,
                    inertia_dq_primary_buf_, tool_compensation_config_, tool_compensation_state_,
                    &inertia_applied_primary, &inertia_failure_reason);
                if (!inertia_applied_primary && !inertia_failure_reason.empty()) {
                    LOG_WARN_THROTTLE(1000) << "tool inertia compensation lapsed: " << inertia_failure_reason;
                }
            } else {
                dynamics::tool_compensation::ResetToolInertiaCompensationState(tool_compensation_state_);
            }
        }
        if (!should_run_inertia && primary_wrench_valid) {
            sensor_data_primary.head<3>() += sensor_force_;
        }

        sensor_wrench_gravity_ = utils::GetSensorWrench(sensor_data_primary_before_inertia, sensor_data_secondary);

        VectorDof tool_torque_feedforward = VectorDof::Zero();
        if (ret && robot_model_.primary_arm_model_ptr) {
            Vector6d tool_wrench_primary = Vector6d::Zero();
            bool tool_gravity_wrench_applied = false;
            std::string tool_gravity_wrench_failure_reason;

            // 计算工具重力和惯量补偿的关节前馈补偿力矩

            // 工具重力补偿 wrench
            tool_wrench_primary += dynamics::tool_compensation::ComputeToolGravityWrenchEE(
                tool_compensation_config_, GetEERotationWorldFromEE(0), &tool_gravity_wrench_applied,
                &tool_gravity_wrench_failure_reason);
            if (!tool_gravity_wrench_applied && !tool_gravity_wrench_failure_reason.empty()) {
                LOG_WARN_THROTTLE(1000) << "tool feedforward gravity wrench lapsed: "
                                        << tool_gravity_wrench_failure_reason;
            }
            // 工具惯量补偿 wrench
            // if (inertia_applied_primary) {
            //     for (int i = 0; i < 6; ++i) {
            //         tool_wrench_primary[i] +=
            //             tool_compensation_state_.tool_inertia_wrench_ee[i].load(std::memory_order_relaxed);
            //     }
            // }
            // 将计算的 wrench 转为关节空间的前馈补偿力矩
            if (!tool_wrench_primary.isZero(FloatZeroTol)) {
                VectorDof feedback_q_whole = VectorDof::Zero();
                for (int i = 0; i < ndof_robot; ++i) {
                    feedback_q_whole[i] = robot_feedback_.joint_feedback[i].joint_position;
                }
                Eigen::VectorXd feedback_q_primary;
                utils::GetJointFromWholeBody(robot_model_, feedback_q_whole, feedback_q_primary, MotionGroup::Primary);
                bool tool_torque_applied = false;
                std::string tool_torque_failure_reason;
                const Eigen::VectorXd tool_torque_primary =
                    dynamics::tool_compensation::ComputeToolJointTorqueFromWrench(
                        *robot_model_.primary_arm_model_ptr, feedback_q_primary, tool_wrench_primary,
                        &tool_torque_applied, &tool_torque_failure_reason);
                if (tool_torque_applied) {
                    utils::MapJointToWholeBody(robot_model_, tool_torque_primary, tool_torque_feedforward,
                                               MotionGroup::Primary);
                } else if (!tool_torque_failure_reason.empty()) {
                    LOG_WARN_THROTTLE(1000) << "tool feedforward torque lapsed: " << tool_torque_failure_reason;
                }
            }
        }

        // 将计算的工具补偿力矩写入共享内存, 供外部使用
        latest_tool_compensation_torque_ = tool_torque_feedforward;

        // 补偿失效时直接清零对应外力输入, 避免把工具自重或惯性当成外力.
        const bool primary_compensation_failed =
            primary_wrench_valid && ((tool_compensation_config_.gravity_enabled && !gravity_applied_primary) ||
                                     (tool_compensation_config_.inertia_enabled && !inertia_applied_primary));
        const bool secondary_compensation_failed =
            secondary_wrench_valid && tool_compensation_config_.gravity_enabled && !gravity_applied_secondary;
        if (primary_compensation_failed) {
            sensor_data_primary.setZero();
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "primary tool compensation lapsed (gravity_applied=%d, inertia_applied=%d); "
                                 "zeroing sensor_data_primary so torque_ext=0 in joint impedance",
                                 gravity_applied_primary, inertia_applied_primary);
            LOG_WARN_THROTTLE(1000) << "primary tool compensation lapsed (gravity_applied=" << gravity_applied_primary
                                    << ", inertia_applied=" << inertia_applied_primary
                                    << "); zeroing sensor_data_primary so torque_ext=0 in joint impedance";
        }
        if (secondary_compensation_failed) {
            sensor_data_secondary.setZero();
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "secondary tool gravity compensation lapsed; zeroing sensor_data_secondary");
            LOG_WARN_THROTTLE(1000) << "secondary tool gravity compensation lapsed; zeroing sensor_data_secondary";
        }

        // 根据状态写入机器人控制字
        this->is_robot_ready_ = false;
        robot_control_.robot_control_word = CTRLWORD_MOTOR_SWITCH_ON;
        if (ret) {
            this->SwitchControlWord();
        }

        auto t_feedback = std::chrono::steady_clock::now();

        // 如果机器人就绪，则写入测量数据
        if (this->is_robot_ready_) {
            // 关节反馈数据写入测量数据
            JointRawMeasurement raw_measurement{};
            raw_measurement.sensor_wrench = sensor_wrench_ =
                utils::GetSensorWrench(sensor_data_primary, sensor_data_secondary);
            raw_measurement.tool_torque_feedforward = tool_torque_feedforward;
            raw_sensor_wrench_ = utils::GetSensorWrench(raw_sensor_data_primary, raw_sensor_data_secondary);

            for (int i = 0; i < ndof_robot; ++i) {
                raw_measurement.position[i] = robot_feedback_.joint_feedback[i].joint_position;
                raw_measurement.velocity[i] = robot_feedback_.joint_feedback[i].joint_velocity;
                raw_measurement.effort[i] = robot_feedback_.joint_feedback[i].joint_torque;
                // 检测力矩限制是否达到阈值
                if (robot_feedback_.joint_feedback[i].joint_torque >=
                        robot_control_.joint_control[i].joint_positive_torque_limit * 0.9 ||
                    robot_feedback_.joint_feedback[i].joint_torque <=
                        -robot_control_.joint_control[i].joint_negative_torque_limit * 0.9) {
                    is_torque_limit_reached_[i] = true;
                } else {
                    is_torque_limit_reached_[i] = false;
                }
                raw_measurement.time_point = TimePointSystem::Now();
            }
            this->robot_buffers_.joint_raw_measurement->AppendNewElement(raw_measurement);
        } else {
            this->robot_buffers_.joint_raw_measurement->Reset();
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Robot is not ready, shm status: %d, statusword: %s", ret,
                                 utils::RobotStatuswordToJsonString(robot_feedback_.robot_status_word).str().c_str());
            LOG_CRIT_THROTTLE(5000) << "Robot is not ready, shm status: " << ret
                                    << ", statusword: " << robot_feedback_.robot_status_word;
        }
        auto t_log = std::chrono::steady_clock::now();

        // 力控计算处理
        torque_controller_manager.LoopOnce();

        auto t_torque = std::chrono::steady_clock::now();

        // 获取并写入控制数据: 关节位置, 速度, 关节力矩限制, 外部力矩
        TorqueCtrlCommand torque_cmd{};
        this->planner_cmd_ = PlannerManagerCommand{};  // 默认值，避免未写入时使用了旧数据
        this->is_cmd_valid_ = false;
        this->manager_buffers_.planner_manager_info->PeekLatest(&this->manager_info_);
        if (this->manager_buffers_.planner_manager_command->PeekLatest(&this->planner_cmd_) &&
            this->robot_buffers_.torque_controller_command->PeekLatest(&torque_cmd)) {
            this->is_cmd_valid_ = true;
            latest_torque_ext_ = torque_cmd.torque_ext;
            latest_arm_forward_torque_ = torque_cmd.arm_forward_torque;
            latest_joint_admittance_q_offset_ = torque_cmd.joint_admittance_q_offset;
            latest_joint_admittance_dq_offset_ = torque_cmd.joint_admittance_dq_offset;
            latest_joint_position_ref_admittance_ = torque_cmd.joint_position_ref_admittance;
            latest_joint_impedance_error_after_admittance_ = torque_cmd.joint_impedance_error_after_admittance;
            // 位置指令线性插值
            VectorDof interpolated_position = line_interpolator_.Interpolate(this->planner_cmd_.cmd.position);
            JointMeasurement manager_measurement{};
            for (int i = 0; i < ndof_robot; ++i) {
                this->robot_control_.joint_control[i].joint_position = interpolated_position[i];
                this->robot_control_.joint_control[i].joint_velocity = this->planner_cmd_.cmd.velocity[i];
                this->robot_control_.joint_control[i].joint_positive_torque_limit =
                    robot_model_.params_ptr->joint_torque_limit[i];
                this->robot_control_.joint_control[i].joint_negative_torque_limit =
                    robot_model_.params_ptr->joint_torque_limit[i];
                switch (this->manager_info_.robot_property.run_mode) {
                    case RunMode::Position:
                        if ((previous_mode_ != this->manager_info_.robot_property.run_mode) &&
                            (switch_counter_ < max_switch_count_) && this->robot_model_.IsDynamicParamsAvailable()) {
                            // 切换之前执行位置保持
                            if (i == 0) {
                                switch_counter_++;
                            }
                            // 位置等于反馈位置
                            if (manager_buffers_.joint_measurement->PeekLatest(&manager_measurement)) {
                                if (IsArmJoint(i)) {
                                    manager_measurement.q_desired_pre[i] =
                                        robot_feedback_.joint_feedback[i].joint_position;
                                    this->robot_control_.joint_control[i].joint_position =
                                        robot_feedback_.joint_feedback[i].joint_position;
                                    line_interpolator_.Reset();
                                }
                                manager_buffers_.joint_measurement->AppendNewElement(manager_measurement);
                            }
                            this->robot_control_.joint_control[i].joint_torque_feedforward =
                                torque_cmd.torque_feedforward[i];
                            this->robot_control_.robot_control_word |= CTRLWORD_MOTOR_CST_MODE;
                        } else {
                            switch_counter_ = 0;
                            previous_mode_ = this->manager_info_.robot_property.run_mode;
                            // 更新关节力矩限制
                            this->robot_control_.joint_control[i].joint_positive_torque_limit = std::clamp(
                                torque_cmd.torque_feedforward[i] + robot_model_.params_ptr->joint_torque_limit[i] *
                                                                       this->manager_info_.robot_property.safe_level,
                                0.1 * robot_model_.params_ptr->joint_torque_limit[i],
                                robot_model_.params_ptr->joint_torque_limit[i]);
                            this->robot_control_.joint_control[i].joint_negative_torque_limit = std::clamp(
                                -torque_cmd.torque_feedforward[i] + robot_model_.params_ptr->joint_torque_limit[i] *
                                                                        this->manager_info_.robot_property.safe_level,
                                0.1 * robot_model_.params_ptr->joint_torque_limit[i],
                                robot_model_.params_ptr->joint_torque_limit[i]);
                            this->robot_control_.joint_control[i].joint_torque_feedforward =
                                torque_cmd.torque_feedforward[i];
                        }
                        break;
                    case RunMode::Torque:
                        previous_mode_ = this->manager_info_.robot_property.run_mode;
                        this->robot_control_.joint_control[i].joint_torque_feedforward = torque_cmd.torque[i];
                        this->robot_control_.robot_control_word |= CTRLWORD_MOTOR_CST_MODE;
                        break;
                    case RunMode::Drag:
                        previous_mode_ = this->manager_info_.robot_property.run_mode;
                        this->robot_control_.joint_control[i].joint_torque_feedforward =
                            torque_cmd.torque_feedforward[i];
                        if (this->robot_model_.IsDynamicParamsAvailable()) {
                            // 位置等于反馈位置
                            if (manager_buffers_.joint_measurement->PeekLatest(&manager_measurement)) {
                                if (IsArmJoint(i)) {
                                    manager_measurement.q_desired_pre[i] =
                                        robot_feedback_.joint_feedback[i].joint_position;
                                }
                                manager_buffers_.joint_measurement->AppendNewElement(manager_measurement);
                            }
                            this->robot_control_.robot_control_word |= CTRLWORD_MOTOR_CST_MODE;
                        } else {
                            // 位置等于反馈位置
                            if (manager_buffers_.joint_measurement->PeekLatest(&manager_measurement)) {
                                if (IsArmJoint(i) && is_torque_limit_reached_[i]) {
                                    manager_measurement.q_desired_pre[i] =
                                        robot_feedback_.joint_feedback[i].joint_position;
                                }
                                manager_buffers_.joint_measurement->AppendNewElement(manager_measurement);
                            }
                            // 更新关节力矩限制
                            if (torque_cmd.torque_feedforward[i] >= 0) {
                                robot_control_.joint_control[i].joint_positive_torque_limit =
                                    std::clamp(torque_cmd.torque_feedforward[i], 0.0,
                                               robot_model_.params_ptr->joint_torque_limit[i]);
                                robot_control_.joint_control[i].joint_negative_torque_limit = 0.0;
                            } else {
                                robot_control_.joint_control[i].joint_positive_torque_limit = 0.0;
                                robot_control_.joint_control[i].joint_negative_torque_limit =
                                    std::clamp(-torque_cmd.torque_feedforward[i], 0.0,
                                               robot_model_.params_ptr->joint_torque_limit[i]);
                            }
                        }
                        break;
                    case RunMode::CartesianImpedance:
                        previous_mode_ = this->manager_info_.robot_property.run_mode;
                        this->robot_control_.joint_control[i].joint_torque_feedforward = torque_cmd.torque_impedance[i];
                        this->robot_control_.robot_control_word |= CTRLWORD_MOTOR_CST_MODE;
                        break;
                    case RunMode::DragImpedance:
                        previous_mode_ = this->manager_info_.robot_property.run_mode;
                        this->robot_control_.joint_control[i].joint_torque_feedforward = torque_cmd.torque[i];
                        this->robot_control_.robot_control_word |= CTRLWORD_MOTOR_CST_MODE;
                        // 位置等于反馈位置
                        if (manager_buffers_.joint_measurement->PeekLatest(&manager_measurement)) {
                            if (IsArmJoint(i)) {
                                manager_measurement.q_desired_pre[i] = robot_feedback_.joint_feedback[i].joint_position;
                            }
                            manager_buffers_.joint_measurement->AppendNewElement(manager_measurement);
                        }
                        break;
                    case RunMode::Sleep:
                    case RunMode::QuickStop:
                        previous_mode_ = this->manager_info_.robot_property.run_mode;
                        break;
                    default:
                        assert(false && "Unknown RunMode");
                        break;
                }
                joint_position_before_filter_[i] = this->robot_control_.joint_control[i].joint_position;
                // 对所有关节应用巴特沃斯滤波 (在 EI 输入整形之前)
                if (this->manager_info_.robot_property.enable_butterworth_filter[i]) {
                    robot_control_.joint_control[i].joint_position =
                        butterworth_filter_[i]->Apply(robot_control_.joint_control[i].joint_position);
                } else {
                    butterworth_filter_[i]
                        ->Reset();  // 如果当前关节不启用滤波器，则重置滤波器状态，避免下次启用时的突变
                }
                joint_position_after_filter_[i] = this->robot_control_.joint_control[i].joint_position;

                // 对所有关节应用 EI 输入整形
                if (this->manager_info_.robot_property.enable_input_shaper[i]) {
                    robot_control_.joint_control[i].joint_position =
                        input_shapers_[i]->Apply(robot_control_.joint_control[i].joint_position);
                } else {
                    input_shapers_[i]->Reset();  // 如果当前关节不启用输入整形，则重置状态
                }
            }
        } else {
            latest_torque_ext_.setZero();
            latest_arm_forward_torque_.setZero();
            latest_tool_compensation_torque_.setZero();
            latest_joint_admittance_q_offset_.setZero();
            latest_joint_admittance_dq_offset_.setZero();
            latest_joint_position_ref_admittance_.setZero();
            latest_joint_impedance_error_after_admittance_.setZero();
            line_interpolator_.Reset();
            for (int i = 0; i < ndof_robot; ++i) {
                butterworth_filter_[i]->Reset();  // 重置滤波器状态
                input_shapers_[i]->Reset();       // 重置输入整形器状态
            }
        }
        // 如果开启了记录, 则在独立的记录日志中记录数据
        // 此处位于 sensor_wrench_ 更新和 LoopOnce() 之后, 使 force_sensor 与 torque_ext 来自同一 tick 的计算
        if (this->manager_info_.robot_property.enable_record &&
            (manager_state_ == Status::Idle || manager_state_ == Status::Running) && loop_counter % 1 == 0) {
            int index_offset = GetFirstJointIndex();
            double timestamp_s = this->now().nanoseconds() / 1e9;

            std::array<double, MAX_JOINT_NUM> position_log_arr_{};
            std::array<double, MAX_JOINT_NUM> velocity_log_arr_{};
            std::array<double, MAX_JOINT_NUM> torque_log_arr_{};
            std::array<double, MAX_JOINT_NUM> position_cmd_log_arr_{};
            std::array<double, MAX_JOINT_NUM> torque_ff_log_arr_{};
            std::array<double, MAX_JOINT_NUM> torque_ext_log_arr_{};
            Vector12d force_sensor_log = sensor_wrench_;
            for (int i = 0; i < ndof_robot; ++i) {
                position_log_arr_[i] = robot_feedback_.joint_feedback[i].joint_position;
                velocity_log_arr_[i] = robot_feedback_.joint_feedback[i].joint_velocity;
                torque_log_arr_[i] = robot_feedback_.joint_feedback[i].joint_torque;
                position_cmd_log_arr_[i] = robot_control_.joint_control[i].joint_position;
                torque_ff_log_arr_[i] = robot_control_.joint_control[i].joint_torque_feedforward;
                torque_ext_log_arr_[i] = latest_torque_ext_[i];
            }

            nanolog::NanoLogLine logline(nanolog::LogLevel::CRIT, __FILE__, __func__, __LINE__, false);
            logline
                << "{\"timestamp\": \"" << std::to_string(timestamp_s) << "\", "
                << "\"joint_position\": "
                << utils::VectorToJsonArrayString(position_log_arr_, index_offset, ndof_config_).str()
                << ", \"joint_velocity\": "
                << utils::VectorToJsonArrayString(velocity_log_arr_, index_offset, ndof_config_).str()
                << ", \"joint_torque\":"
                << utils::VectorToJsonArrayString(torque_log_arr_, index_offset, ndof_config_).str()
                << ", \"joint_position_input\": "
                << utils::VectorToJsonArrayString(joint_position_input_, index_offset, ndof_config_).str()
                << ", \"joint_position_after_ruckig\": "
                << utils::VectorToJsonArrayString(joint_position_before_filter_, index_offset, ndof_config_).str()
                << ", \"joint_position_after_filter\": "
                << utils::VectorToJsonArrayString(joint_position_after_filter_, index_offset, ndof_config_).str()
                << ", \"joint_position_cmd\": "
                << utils::VectorToJsonArrayString(position_cmd_log_arr_, index_offset, ndof_config_).str()
                << ", \"joint_torque_feedforward\": "
                << utils::VectorToJsonArrayString(torque_ff_log_arr_, index_offset, ndof_config_).str()
                << ", \"torque_ext\": "
                << utils::VectorToJsonArrayString(torque_ext_log_arr_, index_offset, ndof_config_).str()
                << ", \"arm_forward_torque\": "
                << utils::VectorToJsonArrayString(latest_arm_forward_torque_, index_offset, ndof_config_).str()
                << ", \"tool_compensation_torque\": "
                << utils::VectorToJsonArrayString(latest_tool_compensation_torque_, index_offset, ndof_config_).str()
                << ", \"joint_admittance_q_offset\": "
                << utils::VectorToJsonArrayString(latest_joint_admittance_q_offset_, index_offset, ndof_config_).str()
                << ", \"joint_admittance_dq_offset\": "
                << utils::VectorToJsonArrayString(latest_joint_admittance_dq_offset_, index_offset, ndof_config_).str()
                << ", \"joint_position_ref_admittance\": "
                << utils::VectorToJsonArrayString(latest_joint_position_ref_admittance_, index_offset, ndof_config_)
                       .str()
                << ", \"joint_impedance_error_after_admittance\": "
                << utils::VectorToJsonArrayString(latest_joint_impedance_error_after_admittance_, index_offset,
                                                  ndof_config_)
                       .str()
                << ", \"force_sensor\": " << utils::VectorToJsonArrayString(force_sensor_log).str()
                << ", \"force_sensor_gravity\": " << utils::VectorToJsonArrayString(sensor_wrench_gravity_).str()
                << ", \"raw_force_sensor\": " << utils::VectorToJsonArrayString(raw_sensor_wrench_).str()
                << ", \"force_sensor_bias\": " << utils::VectorToJsonArrayString(sensor_zero_offset_).str()
                << ", \"force_sensor_inertia\": ["
                << this->tool_compensation_state_.tool_inertia_wrench_ee[0].load(std::memory_order_relaxed) << ","
                << this->tool_compensation_state_.tool_inertia_wrench_ee[1].load(std::memory_order_relaxed) << ","
                << this->tool_compensation_state_.tool_inertia_wrench_ee[2].load(std::memory_order_relaxed) << ","
                << this->tool_compensation_state_.tool_inertia_wrench_ee[3].load(std::memory_order_relaxed) << ","
                << this->tool_compensation_state_.tool_inertia_wrench_ee[4].load(std::memory_order_relaxed) << ","
                << this->tool_compensation_state_.tool_inertia_wrench_ee[5].load(std::memory_order_relaxed) << "]"
                << ", \"ee_omega_ee_frame\": ["
                << dynamics::tool_compensation::JsonFiniteOrZero(
                       this->tool_compensation_state_.ee_omega_F_primary[0].load(std::memory_order_relaxed))
                << ","
                << dynamics::tool_compensation::JsonFiniteOrZero(
                       this->tool_compensation_state_.ee_omega_F_primary[1].load(std::memory_order_relaxed))
                << ","
                << dynamics::tool_compensation::JsonFiniteOrZero(
                       this->tool_compensation_state_.ee_omega_F_primary[2].load(std::memory_order_relaxed))
                << ",0,0,0]"
                << ", \"ee_alpha_ee_frame\": ["
                << dynamics::tool_compensation::JsonFiniteOrZero(
                       this->tool_compensation_state_.ee_alpha_F_primary[0].load(std::memory_order_relaxed))
                << ","
                << dynamics::tool_compensation::JsonFiniteOrZero(
                       this->tool_compensation_state_.ee_alpha_F_primary[1].load(std::memory_order_relaxed))
                << ","
                << dynamics::tool_compensation::JsonFiniteOrZero(
                       this->tool_compensation_state_.ee_alpha_F_primary[2].load(std::memory_order_relaxed))
                << ",0,0,0]}";
            nanolog::LogManager::instance().write_to_logger("record", std::move(logline));
        }
        // 如果状态字发生变化了则打印
        static int16_t last_status_word_ = -1;
        static int16_t last_control_word_ = -1;
        if (robot_feedback_.robot_status_word != last_status_word_ ||
            robot_control_.robot_control_word != last_control_word_) {
            RCLCPP_INFO(this->get_logger(), "Status word changed to: %s, control word changed to %s",
                        utils::RobotStatuswordToJsonString(robot_feedback_.robot_status_word).str().c_str(),
                        utils::RobotControlwordToJsonString(robot_control_.robot_control_word).str().c_str());
            LOG_INFO << "";
            LOG_INFO << "Status word changed to "
                     << utils::RobotStatuswordToJsonString(robot_feedback_.robot_status_word).str()
                     << ", control word changed to "
                     << utils::RobotControlwordToJsonString(robot_control_.robot_control_word).str()
                     << ", robot_server_state:" << GetRobotServerStateMsg().data.c_str();
            last_status_word_ = robot_feedback_.robot_status_word;
            last_control_word_ = robot_control_.robot_control_word;
        }

        // 底盘控制：应用输入整形并写入共享内存
        if (robot_mode_ == RobotMode::W1) {
            robot_control_.chassis_control.chassis_mode = chassis_control_target_.chassis_mode;
            robot_control_.chassis_control.cnt++;

            if (chassis_control_target_.chassis_mode == 1) {
                // 差速模式：对左右轮速度应用输入整形
                if (this->manager_info_.robot_property.enable_chassis_shaper) {
                    auto [shaper_left, shaper_right] = chassis_shaper_->Apply(
                        chassis_control_target_.chassis_left_velocity, chassis_control_target_.chassis_right_velocity);
                    robot_control_.chassis_control.chassis_left_velocity = shaper_left;
                    robot_control_.chassis_control.chassis_right_velocity = shaper_right;
                } else {
                    chassis_shaper_->Reset();
                    robot_control_.chassis_control.chassis_left_velocity =
                        chassis_control_target_.chassis_left_velocity;
                    robot_control_.chassis_control.chassis_right_velocity =
                        chassis_control_target_.chassis_right_velocity;
                }
            } else if (chassis_control_target_.chassis_mode == 2) {
                // 全向电机控制模式：直接复制
                for (size_t i = 0; i < OMNI_MOTOR_NUM; i++) {
                    robot_control_.chassis_control.chassis_motor_velocity[i] =
                        chassis_control_target_.chassis_motor_velocity[i];
                    robot_control_.chassis_control.chassis_motor_direction[i] =
                        chassis_control_target_.chassis_motor_direction[i];
                }
                robot_control_.chassis_control.chassis_estop = chassis_control_target_.chassis_estop;
                robot_control_.chassis_control.chassis_neutral = chassis_control_target_.chassis_neutral;
            } else if (chassis_control_target_.chassis_mode == 0) {
                RCLCPP_WARN_ONCE(this->get_logger(), "Whole chassis mode is not supported");
            }
        }
        RobotControl robot_control_temp = robot_control_;
        SortControlByJointName(robot_control_temp);
        robot_control_temp.timestamp = this->now().nanoseconds();
        control_shm_instance.AppendNewElement(robot_control_temp);

        auto now_time = std::chrono::steady_clock::now();
        double current_usage =
            static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(now_time - t_start).count()) /
            torque_ctrl_dt_micro * 100.0;
        usage_sum += current_usage;
        usage_count++;
        if (current_usage > peak_usage) {
            peak_usage = current_usage;
        }

        static nanolog::ThrottleState stats_throttle;
        if (stats_throttle.should_log(5000)) {
            RCLCPP_INFO(this->get_logger(), "Avg Usage(last 5s): %.2f%%, Peak Usage: %.2f%%",
                        (usage_sum / std::max(1, usage_count)), peak_usage);
            LOG_INFO << "Avg Usage(last 5s): " << (usage_sum / std::max(1, usage_count)) << "%, Peak: " << peak_usage
                     << "%";
            usage_sum = 0.0;
            usage_count = 0;
            peak_usage = 0.0;
        }

        if (now_time > next_cycle_time_point) {
            auto expected_start = next_cycle_time_point - std::chrono::microseconds(torque_ctrl_dt_micro);
            auto elapsed_micro =
                std::chrono::duration_cast<std::chrono::microseconds>(now_time - expected_start).count();
            double usage = static_cast<double>(elapsed_micro) / torque_ctrl_dt_micro * 100.0;
            auto dt_feedback = std::chrono::duration_cast<std::chrono::microseconds>(t_feedback - t_start).count();
            auto dt_log = std::chrono::duration_cast<std::chrono::microseconds>(t_log - t_feedback).count();
            auto dt_torque = std::chrono::duration_cast<std::chrono::microseconds>(t_torque - t_log).count();
            auto dt_cmd = std::chrono::duration_cast<std::chrono::microseconds>(now_time - t_torque).count();
            LOG_WARN_THROTTLE(1000) << "Torque control thread cycle time exceeded 100%! Usage: " << usage
                                    << "%. Breakdown(us): Feedback=" << dt_feedback << ", Log=" << dt_log
                                    << ", Torque=" << dt_torque << ", CmdAndWrite=" << dt_cmd;
            next_cycle_time_point = now_time;
        }

        std::this_thread::sleep_until(next_cycle_time_point);
    }
}

void RobotServer::SwitchControlWord() {
    static bool inited = false;
    // 0. Sleep/QuickStop 模式下进入QUICK_STOP，阻止写入measurement
    if (this->manager_info_.robot_property.run_mode == RunMode::Sleep ||
        this->manager_info_.robot_property.run_mode == RunMode::QuickStop) {
        this->is_robot_ready_ = false;
        line_interpolator_.Reset();
        robot_control_.robot_control_word = CTRLWORD_MOTOR_QUICK_STOP;
        for (int i = 0; i < ndof_robot; i++) {
            this->robot_control_.joint_control[i].joint_position =
                this->robot_feedback_.joint_feedback[i].joint_position;
            this->robot_control_.joint_control[i].joint_positive_torque_limit =
                robot_model_.params_ptr->joint_torque_limit[i];
            this->robot_control_.joint_control[i].joint_negative_torque_limit =
                robot_model_.params_ptr->joint_torque_limit[i];
        }
        return;
    }
    // 1. 根据状态字切换控制字
    switch (robot_feedback_.robot_status_word) {
        case STATUSWORD_MOTOR_READY:
            // ready状态is_robot_ready_置为true, 可以写入measurement, 生成cmd后再使能电机
            this->is_robot_ready_ = true;
            inited = true;
            if (this->is_cmd_valid_) {
                this->robot_control_.robot_control_word |= CTRLWORD_MOTOR_ENABLE;
            } else {
                line_interpolator_.Reset();
                for (int i = 0; i < ndof_robot; i++) {
                    this->robot_control_.joint_control[i].joint_position =
                        this->robot_feedback_.joint_feedback[i].joint_position;
                    this->robot_control_.joint_control[i].joint_positive_torque_limit =
                        robot_model_.params_ptr->joint_torque_limit[i];
                    this->robot_control_.joint_control[i].joint_negative_torque_limit =
                        robot_model_.params_ptr->joint_torque_limit[i];
                }
            }
            break;
        case STATUSWORD_MOTOR_ENABLED:
            this->is_robot_ready_ = true;
            inited = true;
            this->robot_control_.robot_control_word |= CTRLWORD_MOTOR_ENABLE;
            if (!this->is_cmd_valid_) {
                line_interpolator_.Reset();
                for (int i = 0; i < ndof_robot; i++) {
                    this->robot_control_.joint_control[i].joint_position =
                        this->robot_feedback_.joint_feedback[i].joint_position;
                    this->robot_control_.joint_control[i].joint_positive_torque_limit =
                        robot_model_.params_ptr->joint_torque_limit[i];
                    this->robot_control_.joint_control[i].joint_negative_torque_limit =
                        robot_model_.params_ptr->joint_torque_limit[i];
                }
            }
            break;
        case STATUSWORD_MOTOR_NOT_READY:
        case STATUSWORD_MOTOR_FAULT:
        case STATUSWORD_MOTOR_QUICK_STOP:
        default:
            line_interpolator_.Reset();
            for (int i = 0; i < ndof_robot; i++) {
                this->robot_control_.joint_control[i].joint_position =
                    this->robot_feedback_.joint_feedback[i].joint_position;
                this->robot_control_.joint_control[i].joint_positive_torque_limit =
                    robot_model_.params_ptr->joint_torque_limit[i];
                this->robot_control_.joint_control[i].joint_negative_torque_limit =
                    robot_model_.params_ptr->joint_torque_limit[i];
            }
            break;
    }
    // 2. 如果初次启动或者状态字是故障，则发送清除故障命令
    if (!inited || manager_info_.robot_property.clear_error_flag) {
        this->robot_control_.robot_control_word |= CTRLWORD_MOTOR_CLEAR_FAULT;
    }
}

void RobotServer::PublishThreadFunc() {
    if (!is_sim_) {
        Setup_Realtime(60);
    }
    auto next_cycle_time_point = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
        next_cycle_time_point += std::chrono::microseconds(publish_dt_micro);
        this->robot_server_state_publisher_->publish(GetRobotServerStateMsg());
        auto now_time = std::chrono::steady_clock::now();
        if (now_time > next_cycle_time_point) {
            next_cycle_time_point = now_time;
        }
        std::this_thread::sleep_until(next_cycle_time_point);
    }
}

// clang-format off
std_msgs::msg::String RobotServer::GetRobotServerStateMsg()
{
    auto robot_server_state_msg = std_msgs::msg::String();
    uint64_t timestamp_ns = this->now().nanoseconds();
    // A1/Arm模式下数据在虚拟20关节表示中的6-19位置，需要提取
    int index_offset = GetFirstJointIndex();
    // 从 JointFeedback 数组中提取字段
    std::array<double, MAX_JOINT_NUM> position_arr_{}, velocity_arr_{}, torque_arr_{}, torque_sensor_arr_{}, temperature_arr_{};
    std::array<uint16_t, MAX_JOINT_NUM> motor_status_arr_{}, error_code_arr_{};
    for (int i = 0; i < ndof_robot; ++i) {
        position_arr_[i] = this->robot_feedback_.joint_feedback[i].joint_position;
        velocity_arr_[i] = this->robot_feedback_.joint_feedback[i].joint_velocity;
        torque_arr_[i] = this->robot_feedback_.joint_feedback[i].joint_torque;
        torque_sensor_arr_[i] = this->robot_feedback_.joint_feedback[i].joint_sensor_torque;
        motor_status_arr_[i] = this->robot_feedback_.al_status[i];
        temperature_arr_[i] = this->robot_feedback_.joint_feedback[i].joint_temperature;
        error_code_arr_[i] = this->robot_feedback_.joint_feedback[i].joint_error_code;
    }
    // 从 JointControl 数组中提取字段
    std::array<double, MAX_JOINT_NUM> position_cmd_arr_{}, velocity_cmd_arr_{}, torque_ff_arr_{}, positive_torque_limit_arr_{}, negative_torque_limit_arr_{};
    for (int i = 0; i < ndof_robot; ++i) {
        position_cmd_arr_[i] = this->robot_control_.joint_control[i].joint_position;
        velocity_cmd_arr_[i] = this->robot_control_.joint_control[i].joint_velocity;
        torque_ff_arr_[i] = this->robot_control_.joint_control[i].joint_torque_feedforward;
        positive_torque_limit_arr_[i] = this->robot_control_.joint_control[i].joint_positive_torque_limit;
        negative_torque_limit_arr_[i] = this->robot_control_.joint_control[i].joint_negative_torque_limit;
    }
    std::ostringstream force_sensor_inertia_ss;
    force_sensor_inertia_ss << "[";
    for (int i = 0; i < 6; ++i) {
        if (i > 0) {
            force_sensor_inertia_ss << ",";
        }
        force_sensor_inertia_ss << dynamics::tool_compensation::JsonFiniteOrZero(
            this->tool_compensation_state_.tool_inertia_wrench_ee[i].load(std::memory_order_relaxed));
    }
    force_sensor_inertia_ss << "]";

    robot_server_state_msg.data =
        std::string("{\n  \"timestamp\":") + std::to_string(timestamp_ns) +
        ", \"server_version\":\"" W1_SERVER_VERSION "\"" +
        ", \"robot_type\":\"" + this->robot_model_.params_ptr->robot_type + "\"" +
        ", \"robot_version\":\"" + this->robot_model_.params_ptr->robot_version + "\"" +
        ", \"robot_mode\":\"" + ToString(this->robot_mode_) + "\"" +
        ",\n  \"status\":\"" + ToString(this->manager_state_) + "\"" +
        ", \"robot_statusword\":" + utils::RobotStatuswordToJsonString(this->robot_feedback_.robot_status_word).str() +
        ", \"robot_controlword\":" + utils::RobotControlwordToJsonString(robot_control_.robot_control_word).str() +
        ",\n  \"motor_status\":" + utils::MotorStatusToJsonArrayString(motor_status_arr_, index_offset, ndof_config_).str() +
        ",\n  \"motor_error_code\":" + utils::VectorToJsonArrayString(error_code_arr_, index_offset, ndof_config_).str() +
        ",\n  \"motor_temperature\":" + utils::VectorToJsonArrayString(temperature_arr_, index_offset, ndof_config_).str() +
        ",\n  \"run_mode\":\"" + ToString(this->manager_info_.robot_property.run_mode) + "\"" +
        ", \"allow_force_control\":\"" + std::to_string(this->manager_info_.robot_property.allow_force_control) + "\"" +
        ", \"enable_cartesian_impedance\":\"" + std::to_string(this->manager_info_.robot_property.enable_cartesian_impedance) + "\"" +
        ", \"server_error_code\":" + utils::VectorToJsonArrayString(this->manager_info_.robot_property.error_codes).str() +
        ", \"safe_level\":\"" + std::to_string(this->manager_info_.robot_property.safe_level) + "\"" +
        ", \"global_speed_ratio\":\"" + std::to_string(this->manager_info_.robot_property.global_speed_ratio) + "\"" +
        ",\n  \"joint_name\":" + utils::VectorToJsonArrayString(this->robot_model_.params_ptr->joint_names, index_offset, ndof_config_).str() +
        ",\n  \"joint_position\":" + utils::VectorToJsonArrayString(position_arr_, index_offset, ndof_config_).str() +
        ",\n  \"joint_position_cmd\":" + utils::VectorToJsonArrayString(position_cmd_arr_, index_offset, ndof_config_).str() +
        ",\n  \"joint_velocity\":" + utils::VectorToJsonArrayString(velocity_arr_, index_offset, ndof_config_).str() +
        ",\n  \"joint_velocity_cmd\":" + utils::VectorToJsonArrayString(velocity_cmd_arr_, index_offset, ndof_config_).str() +
        ",\n  \"joint_torque\":" + utils::VectorToJsonArrayString(torque_arr_, index_offset, ndof_config_).str() +
        ",\n  \"joint_torque_cmd\":" + utils::VectorToJsonArrayString(torque_ff_arr_, index_offset, ndof_config_).str() +
        ",\n  \"joint_torque_sensor\":" + utils::VectorToJsonArrayString(torque_sensor_arr_, index_offset, ndof_config_).str() +
        ",\n  \"torque_ext\":" + utils::VectorToJsonArrayString(latest_torque_ext_, index_offset, ndof_config_).str() +
        ",\n  \"arm_forward_torque\":" + utils::VectorToJsonArrayString(latest_arm_forward_torque_, index_offset, ndof_config_).str() +
        ",\n  \"tool_compensation_torque\":" + utils::VectorToJsonArrayString(latest_tool_compensation_torque_, index_offset, ndof_config_).str() +
        ",\n  \"joint_admittance_q_offset\":" + utils::VectorToJsonArrayString(latest_joint_admittance_q_offset_, index_offset, ndof_config_).str() +
        ",\n  \"joint_admittance_dq_offset\":" + utils::VectorToJsonArrayString(latest_joint_admittance_dq_offset_, index_offset, ndof_config_).str() +
        ",\n  \"joint_position_ref_admittance\":" + utils::VectorToJsonArrayString(latest_joint_position_ref_admittance_, index_offset, ndof_config_).str() +
        ",\n  \"joint_impedance_error_after_admittance\":" + utils::VectorToJsonArrayString(latest_joint_impedance_error_after_admittance_, index_offset, ndof_config_).str() +
        ",\n  \"force_sensor\":" + utils::VectorToJsonArrayString(sensor_wrench_).str() +
        ",\n  \"force_sensor_gravity\":" + utils::VectorToJsonArrayString(sensor_wrench_gravity_).str() +
        ",\n  \"raw_force_sensor\":" + utils::VectorToJsonArrayString(raw_sensor_wrench_).str() +
        ",\n  \"force_sensor_bias\":" + utils::VectorToJsonArrayString(sensor_zero_offset_).str() +
        ",\n  \"force_sensor_inertia\":" + force_sensor_inertia_ss.str() +
        ",\n  \"torque_limit_negative\":" + utils::VectorToJsonArrayString(negative_torque_limit_arr_, index_offset, ndof_config_).str() +
        ",\n  \"torque_limit_positive\":" + utils::VectorToJsonArrayString(positive_torque_limit_arr_, index_offset, ndof_config_).str() +
        ",\n  \"ee_pose\":" + utils::VectorToJsonArrayString(this->manager_info_.ee_pose_measurement, 0, this->robot_mode_ == RobotMode::SingleArm? 7 : 14).str() +
        ",\n  \"torso_pose_X_Z_RY\":" + utils::VectorToJsonArrayString(this->manager_info_.torso_pose_measurement).str() +
        ",\n  \"latest_queued_id\":" + std::to_string(this->latest_report_id_.queued) +
        ", \"latest_finished_id\":" + std::to_string(this->latest_report_id_.finished) +
        ", \"latest_errordrop_id\":" + std::to_string(this->latest_report_id_.error_drop) +
        ",\n  \"enable_streaming_ruckig\":" + std::to_string(this->manager_info_.robot_property.enable_streaming_ruckig) +
        ", \"joint_streaming_bounds\":" + this->manager_info_.robot_property.joint_streaming_bounds.ToString() +
        ",\n  \"enable_butterworth_filter\":" + utils::VectorToJsonArrayString(this->manager_info_.robot_property.enable_butterworth_filter, index_offset, ndof_config_).str() +
        ",\n  \"butterworth_filter_frequency\":" + utils::VectorToJsonArrayString(this->manager_info_.robot_property.butterworth_filter_frequency, index_offset, ndof_config_).str() +
        ",\n  \"enable_input_shaper\":" + utils::VectorToJsonArrayString(this->manager_info_.robot_property.enable_input_shaper, index_offset, ndof_config_).str() +
        ",\n  \"input_shaper_frequency\":" + utils::VectorToJsonArrayString(this->manager_info_.robot_property.input_shaper_frequency, index_offset, ndof_config_).str() +
        ",\n  \"enable_record\":" + std::to_string(this->manager_info_.robot_property.enable_record) +
        ", \"enable_chassis_shaper\":" + std::to_string(this->manager_info_.robot_property.enable_chassis_shaper) +
        ",\n  \"joint_stiffness\":" + utils::VectorToJsonArrayString(this->manager_info_.robot_property.joint_stiffness, index_offset, ndof_config_).str() +
        ",\n  \"joint_damping\":" + utils::VectorToJsonArrayString(this->manager_info_.robot_property.joint_damping, index_offset, ndof_config_).str() +
        ",\n  \"joint_integral_ratio\":" + utils::VectorToJsonArrayString(this->manager_info_.robot_property.joint_integral_ratio, index_offset, ndof_config_).str() +
        ",\n  \"cart_stiffness\":" + utils::VectorToJsonArrayString(this->manager_info_.robot_property.cart_stiffness, 0, this->robot_mode_ == RobotMode::SingleArm? 6 : 12).str() +
        ",\n  \"cart_damping_ratio\":" + utils::VectorToJsonArrayString(this->manager_info_.robot_property.cart_damping_ratio, 0, this->robot_mode_ == RobotMode::SingleArm? 6 : 12).str() +
        ",\n  \"chassis_status_word\":" + std::to_string(this->robot_feedback_.chassis_feedback.status_word) +
        ", \"chassis_error_code_left\":" + std::to_string(this->robot_feedback_.chassis_feedback.error_code_left) +
        ", \"chassis_error_code_right\":" + std::to_string(this->robot_feedback_.chassis_feedback.error_code_right) +
        ",\n \"chassis_left_velocity\":" + std::to_string(this->robot_feedback_.chassis_feedback.left_velocity) +
        ", \"chassis_right_velocity\":" + std::to_string(this->robot_feedback_.chassis_feedback.right_velocity) +
        ", \"chassis_left_current\":" + std::to_string(this->robot_feedback_.chassis_feedback.left_current) +
        ", \"chassis_right_current\":" + std::to_string(this->robot_feedback_.chassis_feedback.right_current) +
        ",\n \"chassis_driver_temperature\":" + std::to_string(this->robot_feedback_.chassis_feedback.driver_temperature) +
        ", \"chassis_left_temperature\":" + std::to_string(this->robot_feedback_.chassis_feedback.left_temperature) +
        ", \"chassis_right_temperature\":" + std::to_string(this->robot_feedback_.chassis_feedback.right_temperature) +
        "}";
    return robot_server_state_msg;
}
// clang-format on

void RobotServer::PublishImu() {
    const auto& imu = robot_feedback_.imu_data;
    if (imu_timestamp_ != imu.timestamp) {
        imu_timestamp_ = imu.timestamp;
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.frame_id = "imu_chassis_link";

        imu_msg.orientation.w = imu.orientation.w;
        imu_msg.orientation.x = imu.orientation.x;
        imu_msg.orientation.y = imu.orientation.y;
        imu_msg.orientation.z = imu.orientation.z;

        imu_msg.angular_velocity.x = imu.angular_velocity.x;
        imu_msg.angular_velocity.y = imu.angular_velocity.y;
        imu_msg.angular_velocity.z = imu.angular_velocity.z;

        imu_msg.linear_acceleration.x = imu.linear_acceleration.x;
        imu_msg.linear_acceleration.y = imu.linear_acceleration.y;
        imu_msg.linear_acceleration.z = imu.linear_acceleration.z;

        imu_publisher_->publish(imu_msg);
    }
}

void RobotServer::PublishFeedback() {
    // 发布关节状态反馈，A1模式只发布14个关节，W1模式发布20个关节
    sensor_msgs::msg::JointState feedback_msg;
    feedback_msg.header.stamp = this->get_clock()->now();

    feedback_msg.name.resize(ndof_config_);
    feedback_msg.position.resize(ndof_config_);
    feedback_msg.velocity.resize(ndof_config_);
    feedback_msg.effort.resize(ndof_config_);

    int index_offset = GetFirstJointIndex();
    for (int i = 0; i < ndof_config_; ++i) {
        feedback_msg.name[i] = robot_model_.params_ptr->joint_names[index_offset + i];
        feedback_msg.position[i] = robot_feedback_.joint_feedback[index_offset + i].joint_position;
        feedback_msg.velocity[i] = robot_feedback_.joint_feedback[index_offset + i].joint_velocity;
        feedback_msg.effort[i] = robot_feedback_.joint_feedback[index_offset + i].joint_torque;
    }

    feedback_publisher_->publish(feedback_msg);

    // 发布关节速度和力矩限制反馈
    joint_interfaces::msg::JointVelocityTorqueLimitFeedback limit_msg;
    limit_msg.header.stamp = this->get_clock()->now();

    limit_msg.name.resize(ndof_config_);
    limit_msg.positive_velocity_limit.resize(ndof_config_);
    limit_msg.negative_velocity_limit.resize(ndof_config_);
    limit_msg.positive_torque_limit.resize(ndof_config_);
    limit_msg.negative_torque_limit.resize(ndof_config_);

    // A1模式下控制数据在虚拟20关节表示中的6-19位置，需要提取
    for (int i = 0; i < ndof_config_; ++i) {
        limit_msg.name[i] = robot_model_.params_ptr->joint_names[index_offset + i];
        // 新结构体中没有 velocity_limit 字段，默认设置为0
        limit_msg.positive_velocity_limit[i] = 0.0;
        limit_msg.negative_velocity_limit[i] = 0.0;
        limit_msg.positive_torque_limit[i] =
            this->robot_control_.joint_control[index_offset + i].joint_positive_torque_limit;
        limit_msg.negative_torque_limit[i] =
            this->robot_control_.joint_control[index_offset + i].joint_negative_torque_limit;
    }

    velocity_torque_limit_feedback_publisher_->publish(limit_msg);
}

Int64 RobotServer::StartStreaming(PlannerRequest planner_request) {
    auto request_id = MakeNewRequestID(planner_request);

    if (request_id < 0) {
        return request_id;
    }
    planner_request.request_id = request_id;
    this->manager_buffers_.request_queue->EnQueue(planner_request);

    RCLCPP_INFO(this->get_logger(), "Request_id = %ld, StartJointStreaming Received", planner_request.request_id);
    LOG_INFO << "Request_id = " << planner_request.request_id << ", Received, type = StartJointStreaming";
    return request_id;
}

void RobotServer::ControlCallback(const JointControlMsg::SharedPtr msg) {
    auto robot_control_temp = robot_control_;
    bool position_valid = true;
    for (size_t i = 0; i < msg->name.size(); ++i) {
        for (int j = 0; j < ndof_robot; ++j) {
            if (robot_model_.params_ptr->joint_names[j] == msg->name[i]) {
                // 新结构体中没有 velocity_limit 字段
                robot_control_.joint_control[j].joint_positive_torque_limit = msg->positive_torque_limit[i];
                robot_control_.joint_control[j].joint_negative_torque_limit = msg->negative_torque_limit[i];
                if (std::abs(robot_control_temp.joint_control[j].joint_position - msg->position[i]) >
                        position_diff_threshold_ &&
                    position_valid) {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Control command for joint %s rejected: position difference %.4f exceeds threshold %.4f",
                        robot_model_.params_ptr->joint_names[j].c_str(),
                        std::abs(robot_control_temp.joint_control[j].joint_position - msg->position[i]),
                        position_diff_threshold_);
                    LOG_CRIT << "Control command for joint " << robot_model_.params_ptr->joint_names[j]
                             << " rejected: position difference "
                             << std::abs(robot_control_temp.joint_control[j].joint_position - msg->position[i])
                             << " exceeds threshold " << position_diff_threshold_;
                    position_valid = false;
                }
                robot_control_temp.joint_control[j].joint_position = msg->position[i];

                break;
            }
        }
    }
    // 仅当所有位置命令均有效时才启动或发送流式命令
    if (position_valid) {
        if (manager_state_ == Status::Idle) {
            PlannerRequest planner_request;
            planner_request.type = RequestType::StartJointStreaming;
            MoveRequest move_request;
            move_request.motion_group = MotionGroup::WholeBody;
            planner_request.data = move_request;
            streaming_request_id_ = StartStreaming(planner_request);
        }
        if (streaming_request_id_ > -1) {
            StreamingCommandMsg::SharedPtr streaming_msg = std::make_shared<StreamingCommandMsg>();
            streaming_msg->request_id = streaming_request_id_;
            streaming_msg->streaming_finished = false;
            std::vector<float> position_vec(ndof_robot);
            for (int j = 0; j < ndof_robot; ++j) {
                position_vec[j] = robot_control_temp.joint_control[j].joint_position;
            }
            streaming_msg->position = position_vec;
            this->StreamingCommandCallback(streaming_msg);
        }
    }
}

void RobotServer::PositionControlCallback(const JointPositionControlMsg::SharedPtr msg) {
    auto robot_control_temp = robot_control_;
    bool position_valid = true;
    for (size_t i = 0; i < msg->name.size(); ++i) {
        for (int j = 0; j < ndof_robot; ++j) {
            if (robot_model_.params_ptr->joint_names[j] == msg->name[i] && position_valid) {
                if (std::abs(robot_control_temp.joint_control[j].joint_position - msg->position[i]) >
                    position_diff_threshold_) {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Control command for joint %s rejected: position difference %.4f exceeds threshold %.4f",
                        robot_model_.params_ptr->joint_names[j].c_str(),
                        std::abs(robot_control_temp.joint_control[j].joint_position - msg->position[i]),
                        position_diff_threshold_);
                    LOG_CRIT << "Control command for joint " << robot_model_.params_ptr->joint_names[j]
                             << " rejected: position difference "
                             << std::abs(robot_control_temp.joint_control[j].joint_position - msg->position[i])
                             << " exceeds threshold " << position_diff_threshold_;
                    position_valid = false;
                }
                robot_control_temp.joint_control[j].joint_position = msg->position[i];
                break;  // 已经找到并处理了对应的关节，跳出内层循环继续处理下一个输入关节
            } else if (robot_model_.params_ptr->ee_primary_name == msg->name[i]) {
                // 单独处理末端执行器位置命令
                break;
            } else if (robot_model_.params_ptr->ee_secondary_name == msg->name[i]) {
                // 单独处理末端执行器位置命令
                break;
            }
        }
    }
    // 所有关节位置均有效才发送
    if (position_valid) {
        if (manager_state_ == Status::Idle) {
            PlannerRequest planner_request;
            planner_request.type = RequestType::StartJointStreaming;
            MoveRequest move_request;
            move_request.motion_group = MotionGroup::WholeBody;
            move_request.streaming_timeout = streaming_timeout_;
            planner_request.data = move_request;
            streaming_request_id_ = StartStreaming(planner_request);
        }
        if (streaming_request_id_ != -1) {
            StreamingCommandMsg::SharedPtr streaming_msg = std::make_shared<StreamingCommandMsg>();
            streaming_msg->request_id = streaming_request_id_;
            streaming_msg->streaming_finished = false;
            std::vector<float> position_vec(ndof_robot);
            for (int j = 0; j < ndof_robot; ++j) {
                position_vec[j] = robot_control_temp.joint_control[j].joint_position;
            }
            streaming_msg->position = position_vec;
            this->StreamingCommandCallback(streaming_msg);
        }
    }
}

void RobotServer::DebugCallback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "/control/debug message received: %s", msg->data.c_str());
    LOG_INFO << "/control/debug message received: " << msg->data;

    try {
        const auto json_data = nlohmann::json::parse(msg->data);

        auto get_bool_value = [](const nlohmann::json& obj, const char* key) -> std::optional<bool> {
            if (!obj.contains(key)) return std::nullopt;
            const auto& v = obj[key];
            if (v.is_boolean()) return v.get<bool>();
            if (v.is_number_integer()) return v.get<int>() != 0;
            return std::nullopt;
        };

        auto update_bool_param = [this, &json_data, &get_bool_value](const char* key, bool& param) {
            auto value = get_bool_value(json_data, key);
            if (value.has_value()) {
                param = value.value();
                RCLCPP_INFO(this->get_logger(), "%s set to %s", key, param ? "true" : "false");
                LOG_INFO << key << " set to " << (param ? "true" : "false");
            }
        };

        auto update_atomic_bool_param = [this, &json_data, &get_bool_value](const char* key, std::atomic<bool>& param) {
            auto value = get_bool_value(json_data, key);
            if (value.has_value()) {
                bool bool_value = value.value();
                param.store(bool_value);
                RCLCPP_INFO(this->get_logger(), "%s set to %s", key, bool_value ? "true" : "false");
                LOG_INFO << key << " set to " << (bool_value ? "true" : "false");
            }
        };

        update_bool_param("enable_record", this->manager_info_.robot_property.enable_record);
        update_bool_param("enable_chassis_shaper", this->manager_info_.robot_property.enable_chassis_shaper);

    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to parse /control/debug JSON: %s", e.what());
    }
}

void RobotServer::OmniChassisControlCallback(const ChassisWholeControlMsg::SharedPtr msg) {
    chassis_control_target_.chassis_mode = 0;  // 设置为全向整车模式
    chassis_control_target_.chassis_whole_mode = msg->mode;
    chassis_control_target_.chassis_velocity = msg->speed;
    chassis_control_target_.chassis_direction = msg->direction;
    RCLCPP_DEBUG(this->get_logger(), "Omni chassis control: mode=%ld, speed=%.2f, direction=%.2f", msg->mode,
                 msg->speed, msg->direction);
}

// 全向底盘电机控制回调
void RobotServer::OmniChassisMotorControlCallback(const ChassisMotorControlMsg::SharedPtr msg) {
    chassis_control_target_.chassis_mode = 2;  // 设置为电机控制模式
    for (size_t i = 0; i < msg->speed.size() && i < OMNI_MOTOR_NUM; i++) {
        chassis_control_target_.chassis_motor_velocity[i] = msg->speed[i];
        chassis_control_target_.chassis_motor_direction[i] = msg->position[i];
    }
    chassis_control_target_.chassis_estop = msg->estop;
    chassis_control_target_.chassis_neutral = msg->neutral;
    RCLCPP_DEBUG(this->get_logger(), "Omni chassis motor control: estop=%ld, neutral=%ld", msg->estop, msg->neutral);
}

void RobotServer::DiffChassisControlCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    chassis_control_target_.chassis_mode = 1;  // 设置为差速模式
    double chassis_right_velocity = msg->linear.x + msg->angular.z * DiffChassisWidth * 0.5;
    double chassis_left_velocity = msg->linear.x - msg->angular.z * DiffChassisWidth * 0.5;

    chassis_control_target_.chassis_right_velocity = chassis_right_velocity * 60 / (M_PI * DiffChassisWheelD);
    chassis_control_target_.chassis_left_velocity = chassis_left_velocity * 60 / (M_PI * DiffChassisWheelD);

    RCLCPP_DEBUG(this->get_logger(), "Diff chassis control: linear.x=%.2f, angular.z=%.2f => left=%.2f, right=%.2f",
                 msg->linear.x, msg->angular.z, chassis_control_target_.chassis_left_velocity,
                 chassis_control_target_.chassis_right_velocity);
}

bool RobotServer::SortFeedbackByJointName(RobotFeedback& feedback) {
    // 检测反馈数据中的有效关节数量
    int valid_count = 0;
    for (int i = 0; i < MAX_JOINT_NUM; ++i) {
        if (strlen(feedback.joint_feedback[i].joint_name) > 0) {
            valid_count++;
        }
    }

    if (valid_count != ndof_config_) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "SortFeedbackByJointName: joint name count mismatch. Expected %d, got %d", ndof_config_,
                              valid_count);
        LOG_CRIT_THROTTLE(5000) << "SortFeedbackByJointName: joint name count mismatch. Expected " << ndof_config_
                                << ", got " << valid_count;
        return false;
    }

    // 创建临时数组，初始化为默认值
    std::array<JointFeedback, MAX_JOINT_NUM> sorted_joints{};
    int matched_count = 0;

    for (int target_idx = 0; target_idx < ndof_robot; ++target_idx) {
        const std::string& target_name = robot_model_.params_ptr->joint_names[target_idx];

        // 在反馈数据中查找匹配的关节名称
        for (int src_idx = 0; src_idx < MAX_JOINT_NUM; ++src_idx) {
            std::string src_name(feedback.joint_feedback[src_idx].joint_name);
            if (src_name == target_name) {
                // 找到匹配，复制数据到目标位置
                sorted_joints[target_idx] = feedback.joint_feedback[src_idx];
                matched_count++;
                break;
            }
        }
    }

    if (matched_count != ndof_config_) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "SortFeedbackByJointName: matched count mismatch. Expected %d, matched %d", ndof_config_,
                              matched_count);
        LOG_CRIT_THROTTLE(5000) << "SortFeedbackByJointName: matched count mismatch. Expected " << ndof_config_
                                << ", matched " << matched_count;
        return false;
    }

    // 只有在成功匹配时才重置日志标志
    has_logged_sort_feedback_error_ = false;

    // 将排序后的数据复制回反馈结构
    for (int i = 0; i < ndof_robot; ++i) {
        feedback.joint_feedback[i] = sorted_joints[i];
    }

    return true;
}

void RobotServer::SortControlByJointName(RobotControl& control) {
    const auto& joint_names_config = robot_model_.params_ptr->joint_names;
    const std::vector<std::string>& mode_joint_names = [this]() -> const std::vector<std::string>& {
        switch (robot_mode_) {
            case RobotMode::A1:
                return robot_model_.params_ptr->joints_a1;
            case RobotMode::SingleArm:
                return robot_model_.params_ptr->joints_single_arm;
            case RobotMode::DualArm:
                return robot_model_.params_ptr->joints_dual_arm;
            default:
                return robot_model_.params_ptr->joint_names;
        }
    }();

    for (int i = 0; i < ndof_config_; ++i) {
        const std::string& target_name = mode_joint_names[i];
        for (int j = 0; j < ndof_robot; ++j) {
            if (joint_names_config[j] == target_name) {
                control.joint_control[i] = robot_control_.joint_control[j];
                strncpy(control.joint_control[i].joint_name, target_name.c_str(),
                        sizeof(control.joint_control[i].joint_name) - 1);
                control.joint_control[i].joint_name[sizeof(control.joint_control[i].joint_name) - 1] = '\0';
                break;
            }
        }
    }
}

RobotMode RobotServer::DetectRobotModeFromFeedback(const RobotFeedback& feedback) {
    std::set<std::string> feedback_joint_names;
    for (int i = 0; i < ndof_robot; ++i) {
        std::string jname(feedback.joint_feedback[i].joint_name);
        if (!jname.empty()) {
            feedback_joint_names.insert(jname);
        }
    }

    auto is_exact_match = [&feedback_joint_names](const std::vector<std::string>& config_joints) -> bool {
        if (feedback_joint_names.size() != config_joints.size()) {
            return false;
        }
        for (const auto& j : config_joints) {
            if (!feedback_joint_names.count(j)) {
                return false;
            }
        }
        return true;
    };

    if (is_exact_match(robot_model_.params_ptr->joint_names)) {
        robot_mode_ = RobotMode::W1;
        ndof_config_ = ndof_robot;
        LOG_INFO << "自动检测到 W1 模式，实际关节数: " << ndof_config_;
        RCLCPP_INFO(this->get_logger(), "自动检测到 W1 模式，实际关节数: %d", ndof_config_);
        has_logged_unknown_mode_ = false;
        return RobotMode::W1;
    }
    if (is_exact_match(robot_model_.params_ptr->joints_dual_arm)) {
        robot_mode_ = RobotMode::DualArm;
        ndof_config_ = robot_model_.params_ptr->joints_dual_arm.size();
        LOG_INFO << "自动检测到 DualArm 模式，实际关节数: " << ndof_config_;
        RCLCPP_INFO(this->get_logger(), "自动检测到 DualArm 模式，实际关节数: %d", ndof_config_);
        has_logged_unknown_mode_ = false;
        return RobotMode::DualArm;
    }
    if (is_exact_match(robot_model_.params_ptr->joints_a1)) {
        robot_mode_ = RobotMode::A1;
        ndof_config_ = robot_model_.params_ptr->joints_a1.size();
        LOG_INFO << "自动检测到 A1 模式，实际关节数: " << ndof_config_;
        RCLCPP_INFO(this->get_logger(), "自动检测到 A1 模式，实际关节数: %d", ndof_config_);
        has_logged_unknown_mode_ = false;
        return RobotMode::A1;
    }
    if (is_exact_match(robot_model_.params_ptr->joints_single_arm)) {
        robot_mode_ = RobotMode::SingleArm;
        ndof_config_ = robot_model_.params_ptr->joints_single_arm.size();
        LOG_INFO << "自动检测到 SingleArm 模式，实际关节数: " << ndof_config_;
        RCLCPP_INFO(this->get_logger(), "自动检测到 SingleArm 模式，实际关节数: %d", ndof_config_);
        has_logged_unknown_mode_ = false;
        return RobotMode::SingleArm;
    }

    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "未匹配到任何已知模式");
    LOG_WARN_THROTTLE(5000) << "未匹配到任何已知模式";
    return RobotMode::Unknown;
}

bool RobotServer::IsArmJoint(int joint_index) const {
    if (joint_index < 0 || joint_index >= ndof_robot) {
        return false;
    }
    const std::string& joint_name = robot_model_.params_ptr->joint_names[joint_index];

    for (const auto& name : robot_model_.params_ptr->joints_dual_arm) {
        if (joint_name == name) return true;
    }
    return false;
}

int RobotServer::GetFirstJointIndex() const {
    auto mode_joint_names = robot_model_.params_ptr->joint_names;
    switch (robot_mode_) {
        case RobotMode::SingleArm:
            mode_joint_names = robot_model_.params_ptr->joints_single_arm;
            break;
        case RobotMode::DualArm:
            mode_joint_names = robot_model_.params_ptr->joints_dual_arm;
            break;
        case RobotMode::A1:
            mode_joint_names = robot_model_.params_ptr->joints_a1;
            break;
        case RobotMode::W1:
        default:
            mode_joint_names = robot_model_.params_ptr->joint_names;
            break;
    }

    const std::string& first_joint_name = mode_joint_names[0];
    for (int i = 0; i < ndof_robot; ++i) {
        if (robot_model_.params_ptr->joint_names[i] == first_joint_name) {
            return i;
        }
    }
    return 0;
}

}  // namespace robot_server_lib
