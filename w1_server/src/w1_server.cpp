#include "w1_server.h"

#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <set>

namespace robot_server_lib {

RobotServer::RobotServer(std::filesystem::path robot_cfg_path, bool is_sim)
    : RobotServerBase(robot_cfg_path, is_sim), line_interpolator_(manager_dt_micro / torque_ctrl_dt_micro) {
    chassis_type_ = utils::GetChassisTypeFromConfig(robot_cfg_path);
    // 在创建订阅器和启动控制线程前设置有效的底层输出模式。
    RCLCPP_INFO(this->get_logger(), "Chassis type loaded from %s: %s", robot_cfg_path.c_str(),
                chassis_type_ == utils::ChassisType::Omni ? "omni" : "diff_drive");
    LOG_INFO << "w1_server version: " << W1_SERVER_VERSION;

    // 先初始化ndof_config_, 后续再自动识别模式
    ndof_config_ = ndof_robot;

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

    // 底盘的订阅、解算与参数全部归 chassis_node；本节点只在 1 kHz 线程消费目标并写共享内存。
    // 目标经进程内无锁缓冲递送（见 chassis_command.h），配置随目标一起过来，实时侧零同步。
    if (!chassis_target_buffer_) {
        chassis_target_buffer_ = std::make_shared<ChassisTargetBuffer>();
    }
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
    // primary_sensor_ = std::make_shared<SriForceSensor>();
    // if (!primary_sensor_->Init(460800)) {
    //     RCLCPP_WARN(this->get_logger(), "Failed to initialize primary force sensor");
    //     LOG_WARN << "Failed to initialize primary force sensor";
    //     primary_sensor_ = nullptr;
    // } else {
    //     primary_sensor_ready_ = true;
    //     RCLCPP_INFO(this->get_logger(), "Primary force sensor initialized successfully");
    //     LOG_INFO << "Primary force sensor initialized successfully";
    // }
    // secondary_sensor_ = std::make_shared<ForceSensorReader>("/dev/ttyUSB1");  // 暂未使用
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

void RobotServer::TorqueControlThreadFunc() {
    if (!is_sim_) {
        Setup_Realtime();
    }
    TorqueControllerManager torque_controller_manager(this->robot_model_, this->manager_buffers_, this->robot_buffers_);
    // 初始化共享内存
    auto& feedback_shm_instance = FeedbackSharedMemory::GetInstance();
    auto& control_shm_instance = ControlSharedMemory::GetInstance();

    Int64 loop_counter = 0;
    chassis_node::ChassisControlGuard chassis_control_guard;
    this->is_robot_ready_ = false;
    auto next_cycle_time_point = std::chrono::steady_clock::now();
    double usage_sum = 0.0;
    double peak_usage = 0.0;
    int usage_count = 0;
    auto thread_cpu_time_us = []() {
        timespec ts{};
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
        return static_cast<int64_t>(ts.tv_sec) * 1000000 + ts.tv_nsec / 1000;
    };

    while (rclcpp::ok()) {
        loop_counter++;
        next_cycle_time_point += std::chrono::microseconds(torque_ctrl_dt_micro);

        auto t_start = std::chrono::steady_clock::now();
        const int64_t thread_cpu_start_us = thread_cpu_time_us();

        // 读取力传感器数据
        Vector6d sensor_data_primary = Vector6d::Zero();
        Vector6d sensor_data_secondary = Vector6d::Zero();
        if (primary_sensor_ && primary_sensor_ready_) {
            primary_sensor_->PeekLatest(&sensor_data_primary);
        }
        if (secondary_sensor_ && secondary_sensor_ready_) {
            secondary_sensor_->PeekLatest(&sensor_data_secondary);
        }

        // 叠加手动输入的力数据
        sensor_data_primary.head<3>() += sensor_force_;

        // 使用临时变量接收共享内存数据
        RobotFeedback feedback_temp{};
        // 🔧 FEEDBACK_SPLIT: 共享内存帧有效、上半身关节反馈有效、底盘反馈有效是三件事。
        // 关节名排序失败不能阻断底盘轮速反馈，否则全向解算会只转舵、不输出驱动轮速。
        const bool feedback_available = feedback_shm_instance.PeekLatest(&feedback_temp) && feedback_temp.cnt > 0;

        int feedback_joint_count = 0;
        if (feedback_available) {
            for (int i = 0; i < ndof_robot; ++i) {
                if (feedback_temp.joint_feedback[i].joint_name[0] != '\0') {
                    ++feedback_joint_count;
                }
            }
        }

        // 🔧 FEEDBACK_SPLIT: 首帧可能暂时没有关节名并被识别为 Chassis；后续出现真实关节反馈时重新探测，
        // 避免 ndof_config_=0 永久锁死并持续产生 “Expected 0, got 20”。
        const bool should_reprobe_mode =
            feedback_available &&
            (!robot_mode_auto_detected_ || (robot_mode_ == RobotMode::Chassis && feedback_joint_count > 0));
        if (should_reprobe_mode) {
            if (RobotMode::Unknown != DetectRobotModeFromFeedback(feedback_temp)) {
                robot_mode_auto_detected_ = true;
            }
        }

        bool joint_feedback_valid = false;
        if (feedback_available && robot_mode_ != RobotMode::Chassis) {
            joint_feedback_valid = SortFeedbackByJointName(feedback_temp);
        }

        if (joint_feedback_valid) {
            robot_feedback_ = feedback_temp;
        } else {
            // 共享内存无效或关节排序失败时清理状态，避免发布上一个周期遗留的 OP/错误数据。
            robot_feedback_.robot_status_word = 0;
            for (int i = 0; i < MAX_JOINT_NUM; ++i) {
                robot_feedback_.al_status[i] = 0;
                robot_feedback_.joint_feedback[i].joint_temperature = 0;
            }
        }
        // 🔧 FEEDBACK_SPLIT: 底盘反馈独立于上半身关节数据，且要求底盘反馈时间戳有效。
        robot_feedback_.chassis_feedback = feedback_temp.chassis_feedback;
        const bool chassis_feedback_valid = feedback_available && feedback_temp.chassis_feedback.timestamp > 0;

        // 根据状态写入机器人控制字
        this->is_robot_ready_ = false;
        robot_control_.robot_control_word = CTRLWORD_MOTOR_SWITCH_ON;
        // Chassis 模式没有关节电机：跳过关节状态机切换，控制字保持在 SWITCH_ON，
        // 永远不会带上 CTRLWORD_MOTOR_ENABLE 位，避免对不存在的从站做使能。
        // 随之 is_robot_ready_ 恒为 false，关节测量不会写入 planner——这正是期望行为；
        // 底盘控制的下发不经过该标志，照常工作。
        if (joint_feedback_valid && robot_mode_ != RobotMode::Chassis) {
            this->SwitchControlWord();
        }

        auto t_feedback = std::chrono::steady_clock::now();

        // 如果开启了记录, 则在独立的记录日志中记录数据
        if (this->manager_info_.robot_property.enable_record && manager_state_ == Status::Running &&
            loop_counter % 1 == 0) {
            int index_offset = GetFirstJointIndex();
            double timestamp_s = this->now().nanoseconds() / 1e9;

            // 从 JointFeedback 中提取字段用于日志记录
            std::array<double, MAX_JOINT_NUM> position_log_arr_{};
            std::array<double, MAX_JOINT_NUM> velocity_log_arr_{};
            std::array<double, MAX_JOINT_NUM> torque_log_arr_{};
            std::array<double, MAX_JOINT_NUM> temperature_log_arr_{};
            std::array<double, MAX_JOINT_NUM> position_cmd_log_arr_{};
            std::array<double, MAX_JOINT_NUM> torque_ff_log_arr_{};
            for (int i = 0; i < ndof_robot; ++i) {
                position_log_arr_[i] = robot_feedback_.joint_feedback[i].joint_position;
                velocity_log_arr_[i] = robot_feedback_.joint_feedback[i].joint_velocity;
                torque_log_arr_[i] = robot_feedback_.joint_feedback[i].joint_torque;
                temperature_log_arr_[i] = robot_feedback_.joint_feedback[i].joint_temperature;
                position_cmd_log_arr_[i] = robot_control_.joint_control[i].joint_position;
                torque_ff_log_arr_[i] = robot_control_.joint_control[i].joint_torque_feedforward;
            }

            nanolog::NanoLogLine logline(nanolog::LogLevel::CRIT, __FILE__, __func__, __LINE__, false);
            logline << "{\"timestamp\": \"" << std::to_string(timestamp_s) << "\", "
                    << "\"joint_position\": "
                    << utils::VectorToJsonArrayString(position_log_arr_, index_offset, ndof_config_).str()
                    << ", \"joint_velocity\": "
                    << utils::VectorToJsonArrayString(velocity_log_arr_, index_offset, ndof_config_).str()
                    << ", \"joint_torque\":"
                    << utils::VectorToJsonArrayString(torque_log_arr_, index_offset, ndof_config_).str()
                    << ", \"joint_temperature\": "
                    << utils::VectorToJsonArrayString(temperature_log_arr_, index_offset, ndof_config_).str()
                    << ", \"joint_position_input\": "
                    << utils::VectorToJsonArrayString(joint_position_input_, index_offset, ndof_config_).str()
                    << ", \"joint_position_after_ruckig\": "
                    << utils::VectorToJsonArrayString(joint_position_before_filter_, index_offset, ndof_config_).str()
                    << ", \"joint_position_after_filter\": "
                    << utils::VectorToJsonArrayString(joint_position_after_filter_, index_offset, ndof_config_).str()
                    << ", \"joint_position_cmd\": "
                    << utils::VectorToJsonArrayString(position_cmd_log_arr_, index_offset, ndof_config_).str()
                    << ", \"joint_torque_feedforward\": "
                    << utils::VectorToJsonArrayString(torque_ff_log_arr_, index_offset, ndof_config_).str() << "}";
            nanolog::LogManager::instance().write_to_logger("record", std::move(logline));
        }

        // 如果机器人就绪，则写入测量数据
        if (this->is_robot_ready_) {
            // 关节反馈数据写入测量数据
            JointRawMeasurement raw_measurement{};
            raw_measurement.sensor_wrench = sensor_wrench_ =
                utils::GetSensorWrench(sensor_data_primary, sensor_data_secondary);

            for (int i = 0; i < ndof_robot; ++i) {
                raw_measurement.position[i] = robot_feedback_.joint_feedback[i].joint_position;
                raw_measurement.velocity[i] = robot_feedback_.joint_feedback[i].joint_velocity;
                raw_measurement.torque[i] = robot_feedback_.joint_feedback[i].joint_torque;
                raw_measurement.sensor_torque[i] = robot_feedback_.joint_feedback[i].joint_sensor_torque;
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
            // 🔧 FEEDBACK_SPLIT: 纯底盘模式没有上身关节就绪状态，不再输出误导性的 NOT_READY 告警。
            if (robot_mode_ != RobotMode::Chassis) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000, "Robot is not ready, shm status: %d, statusword: %s",
                    joint_feedback_valid,
                    utils::RobotStatuswordToJsonString(robot_feedback_.robot_status_word).str().c_str());
                LOG_CRIT_THROTTLE(5000) << "Robot is not ready, shm status: " << joint_feedback_valid
                                        << ", statusword: " << robot_feedback_.robot_status_word;
            }
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
                                torque_cmd.torque_feedforward[i];  // 切换模式时保持仅前馈力矩
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
            line_interpolator_.Reset();
            for (int i = 0; i < ndof_robot; ++i) {
                butterworth_filter_[i]->Reset();  // 重置滤波器状态
                input_shapers_[i]->Reset();       // 重置输入整形器状态
            }
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

        // 底盘：从进程内缓冲取目标（含配置快照），用本周期的共享内存反馈直接生成指令。
        // 生成函数是纯函数、无锁无分配，可在本实时线程内调用；反馈取自 shm 而非话题，
        // 比切分前的 /feedback/chassis_motor 路径新约 10 ms。
        chassis_node::ChassisTarget chassis_target{};
        const bool has_target = chassis_target_buffer_->PeekLatest(&chassis_target);
        const int64_t now_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
                .count();
        if (!has_target) {
            // 还没收到过目标（开机瞬间）：按本机底盘类型兜底，行为等同 idle
            chassis_target.config.chassis_type = chassis_type_;
        }

        chassis_node::WheelFeedback wheel_fb{};
        for (size_t i = 0; i < OMNI_MOTOR_NUM; ++i) {
            wheel_fb.angle_rad[i] = robot_feedback_.chassis_feedback.chassis_motor_direction[i];
            wheel_fb.speed_rpm[i] = robot_feedback_.chassis_feedback.chassis_motor_velocity[i];
        }
        wheel_fb.left_rpm = robot_feedback_.chassis_feedback.left_velocity;
        wheel_fb.right_rpm = robot_feedback_.chassis_feedback.right_velocity;
        // 纯底盘豁免上半身状态；整机要求关节反馈有效且 ready。
        const bool chassis_robot_ready =
            robot_mode_ == RobotMode::Chassis || (joint_feedback_valid && is_robot_ready_);
        const auto chassis_decision = chassis_control_guard.Update(
            chassis_target, has_target, wheel_fb, chassis_feedback_valid, chassis_robot_ready, now_ns);
        wheel_fb.valid = chassis_decision.feedback_valid;
        const bool chassis_idle = chassis_decision.idle;
        ChassisControl chassis_cmd{};
        chassis_node::GenerateChassisCommand(chassis_target, wheel_fb, chassis_idle, &chassis_cmd);

        robot_control_.chassis_control.chassis_mode = chassis_cmd.chassis_mode;
        robot_control_.chassis_control.cnt++;
        if (chassis_target.config.chassis_type == utils::ChassisType::DiffDrive) {
            // 差速：左右轮速度过 EI 输入整形（按 1 ms 设计的离散滤波器，只能在本线程跑）
            if (!chassis_idle && this->manager_info_.robot_property.enable_chassis_shaper) {
                auto [shaper_left, shaper_right] =
                    chassis_shaper_->Apply(chassis_cmd.chassis_left_velocity, chassis_cmd.chassis_right_velocity);
                robot_control_.chassis_control.chassis_left_velocity = shaper_left;
                robot_control_.chassis_control.chassis_right_velocity = shaper_right;
            } else {
                chassis_shaper_->Reset();
                robot_control_.chassis_control.chassis_left_velocity = chassis_cmd.chassis_left_velocity;
                robot_control_.chassis_control.chassis_right_velocity = chassis_cmd.chassis_right_velocity;
            }
        } else {
            for (size_t i = 0; i < OMNI_MOTOR_NUM; ++i) {
                robot_control_.chassis_control.chassis_motor_velocity[i] = chassis_cmd.chassis_motor_velocity[i];
                robot_control_.chassis_control.chassis_motor_direction[i] = chassis_cmd.chassis_motor_direction[i];
            }
            robot_control_.chassis_control.chassis_estop = chassis_cmd.chassis_estop;
        }
        // 最终保护：启动前下使能；运行异常零速并锁存反馈舵角，保持闭环使能。
        chassis_node::ApplyChassisControlDecision(chassis_decision, &robot_control_.chassis_control);
        RobotControl robot_control_temp = robot_control_;
        SortControlByJointName(robot_control_temp);
        robot_control_temp.timestamp = this->now().nanoseconds();
        control_shm_instance.AppendNewElement(robot_control_temp);

        const auto work_end_time = std::chrono::steady_clock::now();
        const double work_usage =
            static_cast<double>(
                std::chrono::duration_cast<std::chrono::microseconds>(work_end_time - t_start).count()) /
            torque_ctrl_dt_micro * 100.0;
        usage_sum += work_usage;
        usage_count++;
        peak_usage = std::max(peak_usage, work_usage);

        static nanolog::ThrottleState stats_throttle;
        if (stats_throttle.should_log(5000)) {
            LOG_INFO << "Avg Usage(last 5s): " << usage_sum / std::max(1, usage_count) << "%, Peak: " << peak_usage
                     << "%";
            usage_sum = 0.0;
            usage_count = 0;
            peak_usage = 0.0;
        }

        const auto now_time = std::chrono::steady_clock::now();
        const int64_t thread_cpu_elapsed_us = thread_cpu_time_us() - thread_cpu_start_us;
        const double current_usage =
            static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(now_time - t_start).count()) /
            torque_ctrl_dt_micro * 100.0;

        if (now_time > next_cycle_time_point) {
            auto expected_start = next_cycle_time_point - std::chrono::microseconds(torque_ctrl_dt_micro);
            auto wake_lateness_us = std::max<int64_t>(
                0, std::chrono::duration_cast<std::chrono::microseconds>(t_start - expected_start).count());
            auto deadline_lateness_us =
                std::chrono::duration_cast<std::chrono::microseconds>(now_time - next_cycle_time_point).count();
            auto dt_feedback = std::chrono::duration_cast<std::chrono::microseconds>(t_feedback - t_start).count();
            auto dt_log = std::chrono::duration_cast<std::chrono::microseconds>(t_log - t_feedback).count();
            auto dt_torque = std::chrono::duration_cast<std::chrono::microseconds>(t_torque - t_log).count();
            auto dt_cmd = std::chrono::duration_cast<std::chrono::microseconds>(now_time - t_torque).count();
            LOG_WARN_THROTTLE(1000) << "Torque control thread missed deadline. Compute usage: " << current_usage
                                    << "%, WakeLate=" << wake_lateness_us << "us, DeadlineLate=" << deadline_lateness_us
                                    << "us, ThreadCPU=" << thread_cpu_elapsed_us
                                    << "us. Breakdown(us): Feedback=" << dt_feedback << ", Log=" << dt_log
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
        ", \"allow_torque_control\":\"" + std::to_string(this->manager_info_.robot_property.allow_torque_control) + "\"" +
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
        ",\n  \"force_sensor\":" + utils::VectorToJsonArrayString(sensor_wrench_).str() +
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
        ",\n  \"enable_pos_limit_check\":" + std::to_string(this->manager_info_.robot_property.safety_check_config.check_position_limit) +
        ", \"enable_chassis_shaper\":" + std::to_string(this->manager_info_.robot_property.enable_chassis_shaper) +
        ",\n  \"joint_stiffness\":" + utils::VectorToJsonArrayString(this->manager_info_.robot_property.joint_stiffness, index_offset, ndof_config_).str() +
        ",\n  \"joint_damping\":" + utils::VectorToJsonArrayString(this->manager_info_.robot_property.joint_damping, index_offset, ndof_config_).str() +
        ",\n  \"joint_integral_ratio\":" + utils::VectorToJsonArrayString(this->manager_info_.robot_property.joint_integral_ratio, index_offset, ndof_config_).str() +
        ",\n  \"cart_stiffness\":" + utils::VectorToJsonArrayString(this->manager_info_.robot_property.cart_stiffness, 0, this->robot_mode_ == RobotMode::SingleArm? 6 : 12).str() +
        ",\n  \"cart_damping_ratio\":" + utils::VectorToJsonArrayString(this->manager_info_.robot_property.cart_damping_ratio, 0, this->robot_mode_ == RobotMode::SingleArm? 6 : 12).str() +
        ",\n  \"drag_button\":" + std::to_string(this->robot_feedback_.drag_mode) + 
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

    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to parse /control/debug JSON: %s", e.what());
    }
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

    // 关节名集合为空 ⇒ 总线上没有任何关节电机，只有底盘。
    // 本函数仅在 feedback_available（PeekLatest 成功且 cnt>0）时被调用，因此这里的「空」
    // 确实代表「没有关节从站」，而不会与「共享内存尚无数据」混淆。
    if (feedback_joint_names.empty()) {
        robot_mode_ = RobotMode::Chassis;
        ndof_config_ = 0;
        LOG_INFO << "自动检测到 Chassis 模式（仅底盘，无关节电机）";
        RCLCPP_INFO(this->get_logger(), "自动检测到 Chassis 模式（仅底盘，无关节电机）");
        return RobotMode::Chassis;
    }

    if (is_exact_match(robot_model_.params_ptr->joint_names)) {
        robot_mode_ = RobotMode::W1;
        ndof_config_ = ndof_robot;
        LOG_INFO << "自动检测到 W1 模式，实际关节数: " << ndof_config_;
        RCLCPP_INFO(this->get_logger(), "自动检测到 W1 模式，实际关节数: %d", ndof_config_);
        return RobotMode::W1;
    }
    if (is_exact_match(robot_model_.params_ptr->joints_dual_arm)) {
        robot_mode_ = RobotMode::DualArm;
        ndof_config_ = robot_model_.params_ptr->joints_dual_arm.size();
        LOG_INFO << "自动检测到 DualArm 模式，实际关节数: " << ndof_config_;
        RCLCPP_INFO(this->get_logger(), "自动检测到 DualArm 模式，实际关节数: %d", ndof_config_);
        return RobotMode::DualArm;
    }
    if (is_exact_match(robot_model_.params_ptr->joints_a1)) {
        robot_mode_ = RobotMode::A1;
        ndof_config_ = robot_model_.params_ptr->joints_a1.size();
        LOG_INFO << "自动检测到 A1 模式，实际关节数: " << ndof_config_;
        RCLCPP_INFO(this->get_logger(), "自动检测到 A1 模式，实际关节数: %d", ndof_config_);
        return RobotMode::A1;
    }
    if (is_exact_match(robot_model_.params_ptr->joints_single_arm)) {
        robot_mode_ = RobotMode::SingleArm;
        ndof_config_ = robot_model_.params_ptr->joints_single_arm.size();
        LOG_INFO << "自动检测到 SingleArm 模式，实际关节数: " << ndof_config_;
        RCLCPP_INFO(this->get_logger(), "自动检测到 SingleArm 模式，实际关节数: %d", ndof_config_);
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
