#include "w1_server.h"

#include <cstdlib>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <system_error>

namespace robot_server_lib {

RobotServer::RobotServer(std::filesystem::path robot_cfg_path, bool is_sim, RobotMode robot_mode)
    : RobotServerBase(robot_cfg_path, is_sim), robot_mode_(robot_mode) {
    LOG_INFO << "w1_server version: " << W1_SERVER_VERSION;

    // 根据传入参数设置模式
    switch (robot_mode_) {
        case RobotMode::Arm:
            ndof_config_ = 7;
            LOG_INFO << "Arm 模式已启用，实际关节数: " << ndof_config_;
            RCLCPP_INFO(this->get_logger(), "Arm 模式已启用，实际关节数: %d", ndof_config_);
            break;
        case RobotMode::A1:
            ndof_config_ = 14;
            LOG_INFO << "A1 模式已启用，实际关节数: " << ndof_config_;
            RCLCPP_INFO(this->get_logger(), "A1 模式已启用，实际关节数: %d", ndof_config_);
            break;
        case RobotMode::W1:
        default:
            ndof_config_ = ndof_robot;
            LOG_INFO << "W1 模式已启用，实际关节数: " << ndof_config_;
            RCLCPP_INFO(this->get_logger(), "W1 模式已启用，实际关节数: %d", ndof_config_);
            break;
    }

    // 创建 /feedback/joint 发布器
    feedback_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/feedback/joint", 10);

    // 创建 /feedback/joint_velocity_torque_limit 发布器
    velocity_torque_limit_feedback_publisher_ =
        this->create_publisher<JointVelocityTorqueLimitFeedbackMsg>("/feedback/joint_velocity_torque_limit", 10);

    // 创建 /control/joint 订阅器
    control_subscriber_ = this->create_subscription<JointControlMsg>(
        "/control/joint", 10, std::bind(&RobotServer::ControlCallback, this, std::placeholders::_1));

    // 创建 /control/joint_position 订阅器
    position_control_subscriber_ = this->create_subscription<JointPositionControlMsg>(
        "/control/joint_position", 10, std::bind(&RobotServer::PositionControlCallback, this, std::placeholders::_1));

    // A1/Arm 模式下不创建底盘订阅器
    if (robot_mode_ == RobotMode::W1) {
        // 创建 /control/omni_chassis 订阅器
        omni_chassis_control_subscriber_ = this->create_subscription<ChassisWholeControlMsg>(
            "/control/omni_chassis", 10,
            std::bind(&RobotServer::OmniChassisControlCallback, this, std::placeholders::_1));

        // 创建 /control/omni_chassis_motors 订阅器
        omni_chassis_motor_control_subscriber_ = this->create_subscription<ChassisMotorControlMsg>(
            "/control/omni_chassis_motors", 10,
            std::bind(&RobotServer::OmniChassisMotorControlCallback, this, std::placeholders::_1));

        // 创建 /control/cmd_vel 订阅器
        diff_chassis_control_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/control/cmd_vel", 10, std::bind(&RobotServer::DiffChassisControlCallback, this, std::placeholders::_1));
        chassis_shaper_ = std::make_unique<EIInputShaper>(chassis_shaper_type_, DiffChassisNatureFreq, torque_ctrl_dt,
                                                          DiffChassisMaxRampRpm);
    }

    // torso J1-J4 输入整形器
    torso_shapers_[0] = std::make_unique<EIInputShaper>(torso_shaper_type_, TorsoJ1NatureFreq, torque_ctrl_dt,
                                                        TorsoShaperMaxRampRadPerSec);
    torso_shapers_[1] = std::make_unique<EIInputShaper>(torso_shaper_type_, TorsoJ2NatureFreq, torque_ctrl_dt,
                                                        TorsoShaperMaxRampRadPerSec);
    torso_shapers_[2] = std::make_unique<EIInputShaper>(torso_shaper_type_, TorsoJ3NatureFreq, torque_ctrl_dt,
                                                        TorsoShaperMaxRampRadPerSec);
    torso_shapers_[3] = std::make_unique<EIInputShaper>(torso_shaper_type_, TorsoJ4NatureFreq, torque_ctrl_dt,
                                                        TorsoShaperMaxRampRadPerSec);

    // 初始化整形器状态追踪变量
    prev_chassis_shaper_enabled_ = enable_chassis_shaper_.load();
    prev_torso_shaper_enabled_ = enable_torso_shaper_.load();

    force_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "force_array", rclcpp::QoS(10), std::bind(&RobotServer::ForceArrayCallback, this, std::placeholders::_1));

    // 创建 /control/debug 订阅器
    debug_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "/control/debug", 10, std::bind(&RobotServer::DebugCallback, this, std::placeholders::_1));

    // 参数初始化
    InitParamConfig();

    // 创建定时器，定时发布feedback 定时器回调函数
    control_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() { PublishFeedback(); });

    for (int i = 0; i < ndof_robot; ++i) {
        butterworth_filter_[i] = std::make_shared<ButterworthFilter>();
    }

    // 初始化力传感器
    // primary_sensor_ = std::make_shared<ForceSensorReader>("/dev/ttyUSB2");
    // secondary_sensor_ = std::make_shared<ForceSensorReader>("/dev/ttyUSB1");

    this->Run();
}

void RobotServer::ForceArrayCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    // msg->data 是 std::vector<double>
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

void RobotServer::InitParamConfig() {
    auto config_path = server_data_dir_ / "w1_config.json";

    const std::set<std::string> kAllowedRootKeys{"enable_ruckig", "enable_chassis_shaper", "enable_torso_shaper",
                                                 "enable_record", "enable_butterworth_filter"};

    nlohmann::json root;
    if (!std::filesystem::exists(config_path)) {
        root["enable_ruckig"] = this->enable_ruckig_;
        root["enable_chassis_shaper"] = this->enable_chassis_shaper_.load();
        root["enable_torso_shaper"] = this->enable_torso_shaper_.load();
        root["enable_record"] = this->enable_record_;
        root["enable_butterworth_filter"] = this->enable_butterworth_filter_.load();

        std::ofstream ofs(config_path);
        ofs << root.dump(2);
        RCLCPP_INFO(this->get_logger(), "Created default w1_config.json at: %s", config_path.string().c_str());
        return;
    } else {
        std::ifstream ifs(config_path);
        try {
            root = nlohmann::json::parse(ifs);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse json: %s", e.what());
            return;
        }
    }

    bool need_rewrite = false;

    for (auto it = root.begin(); it != root.end(); ++it) {
        if (!kAllowedRootKeys.count(it.key())) {
            need_rewrite = true;
            break;
        }
    }

    if (!root.contains("enable_ruckig")) {
        root["enable_ruckig"] = this->enable_ruckig_;
        need_rewrite = true;
    }
    if (!root.contains("enable_chassis_shaper")) {
        root["enable_chassis_shaper"] = this->enable_chassis_shaper_.load();
        need_rewrite = true;
    }
    if (!root.contains("enable_torso_shaper")) {
        root["enable_torso_shaper"] = this->enable_torso_shaper_.load();
        need_rewrite = true;
    }
    if (!root.contains("enable_record")) {
        root["enable_record"] = this->enable_record_;
        need_rewrite = true;
    }
    if (!root.contains("enable_butterworth_filter")) {
        root["enable_butterworth_filter"] = this->enable_butterworth_filter_.load();
        need_rewrite = true;
    }

    this->enable_ruckig_ = root.value("enable_ruckig", this->enable_ruckig_);
    this->enable_chassis_shaper_ = root.value("enable_chassis_shaper", this->enable_chassis_shaper_.load());
    this->enable_torso_shaper_ = root.value("enable_torso_shaper", this->enable_torso_shaper_.load());
    this->enable_record_ = root.value("enable_record", this->enable_record_);
    this->enable_butterworth_filter_ = root.value("enable_butterworth_filter", this->enable_butterworth_filter_.load());

    // A1 和 Arm 模式下禁用 torso shaper
    if (robot_mode_ != RobotMode::W1) {
        this->enable_torso_shaper_ = false;
    }

    nlohmann::json root_clean;
    root_clean["enable_ruckig"] = this->enable_ruckig_;
    root_clean["enable_chassis_shaper"] = this->enable_chassis_shaper_.load();
    root_clean["enable_torso_shaper"] = this->enable_torso_shaper_.load();
    root_clean["enable_record"] = this->enable_record_;
    root_clean["enable_butterworth_filter"] = this->enable_butterworth_filter_.load();

    if (need_rewrite) {
        std::ofstream ofs(config_path);
        ofs << root_clean.dump(2);
    }

    RCLCPP_INFO(this->get_logger(),
                "Loaded w1_config.json from: %s \n"
                "enable_ruckig=%s, \n"
                "enable_chassis_shaper=%s, \n"
                "enable_torso_shaper=%s, \n"
                "enable_record=%s, \n"
                "enable_butterworth_filter=%s)",
                config_path.string().c_str(), this->enable_ruckig_ ? "true" : "false",
                this->enable_chassis_shaper_.load() ? "true" : "false",
                this->enable_torso_shaper_.load() ? "true" : "false", this->enable_record_ ? "true" : "false",
                this->enable_butterworth_filter_.load() ? "true" : "false");
    LOG_INFO << "Loaded w1_config.json from: " << config_path << ", enable_ruckig=" << this->enable_ruckig_
             << ", enable_chassis_shaper=" << this->enable_chassis_shaper_.load()
             << ", enable_torso_shaper=" << this->enable_torso_shaper_.load()
             << ", enable_record=" << this->enable_record_
             << ", enable_butterworth_filter=" << this->enable_butterworth_filter_.load();
}

void RobotServer::TorqueControlThreadFunc() {
    if (!is_sim_) {
        Setup_Realtime();
    }
    TorqueControllerManager torque_controller_manager(this->robot_model_, this->manager_buffers_, this->robot_buffers_);
    // 初始化共享内存
    auto &feedback_shm_instance = FeedbackSharedMemory::GetInstance();
    auto &control_shm_instance = ControlSharedMemory::GetInstance();

    Int64 loop_counter = 0;
    this->is_robot_ready_ = false;
    auto next_cycle_time_point = std::chrono::steady_clock::now();

    while (rclcpp::ok()) {
        loop_counter++;
        next_cycle_time_point += std::chrono::microseconds(torque_ctrl_dt_micro);

        // 读取力传感器数据
        Vector6d sensor_data_primary = Vector6d::Zero();
        Vector6d sensor_data_secondary = Vector6d::Zero();
        if (primary_sensor_) {
            primary_sensor_->Read(sensor_data_primary);
        }
        if (secondary_sensor_) {
            secondary_sensor_->Read(sensor_data_secondary);
        }

        // 叠加手动输入的力数据
        sensor_data_primary.head<3>() += sensor_force_;

        // 使用临时变量接收共享内存数据
        RobotFeedback feedback_temp{};
        bool ret = feedback_shm_instance.PeekLatest(&feedback_temp) && feedback_temp.cnt > 0;

        // 如果A1/Arm模式：将反馈展开为20关节虚拟反馈
        if (ret) {
            if (robot_mode_ == RobotMode::Arm) {
                Expand7ToVirtual20(feedback_temp, robot_feedback_);
            } else if (robot_mode_ == RobotMode::A1) {
                Expand14ToVirtual20(feedback_temp, robot_feedback_);
            } else {
                robot_feedback_ = feedback_temp;
            }
        }

        // 根据状态写入机器人控制字
        this->is_robot_ready_ = false;
        robot_control_.robot_control_word = CTRLWORD_MOTOR_SWITCH_ON;
        if (ret) {
            this->SwitchControlWord();
        }
        // 如果开启了记录, 则在独立的记录日志中记录数据
        if (this->enable_record_ && manager_state_ == Status::Running && loop_counter % 4 == 0) {
            int idx_offset = (robot_mode_ != RobotMode::W1) ? first_arm_joint_index_ : 0;
            nanolog::NanoLogLine logline(nanolog::LogLevel::CRIT, __FILE__, __func__, __LINE__);
            logline << "{ \"joint_position\": "
                    << utils::VectorToJsonArrayString(robot_feedback_.position, idx_offset, ndof_config_).str()
                    << ", \"joint_velocity\": "
                    << utils::VectorToJsonArrayString(robot_feedback_.velocity, idx_offset, ndof_config_).str()
                    << ", \"joint_torque\":"
                    << utils::VectorToJsonArrayString(robot_feedback_.torque, idx_offset, ndof_config_).str()
                    << ", \"joint_position_cmd\": "
                    << utils::VectorToJsonArrayString(robot_control_.position, idx_offset, ndof_config_).str()
                    << ", \"joint_torque_feedforward\": "
                    << utils::VectorToJsonArrayString(robot_control_.torque_feedforward, idx_offset, ndof_config_).str()
                    << ", \"chassis_target_left_velocity\": " << chassis_control_target_.chassis_left_velocity
                    << ", \"chassis_target_right_velocity\": " << chassis_control_target_.chassis_right_velocity
                    << ", \"chassis_cmd_left_velocity\": " << robot_control_.chassis_left_velocity
                    << ", \"chassis_cmd_right_velocity\": " << robot_control_.chassis_right_velocity
                    << ", \"chassis_feedback_left_velocity\": " << robot_feedback_.chassis_feedback.left_velocity
                    << ", \"chassis_feedback_right_velocity\": " << robot_feedback_.chassis_feedback.right_velocity
                    << "}";
            nanolog::LogManager::instance().write_to_logger("record", std::move(logline));
        }
        // 如果机器人就绪，则写入测量数据
        if (this->is_robot_ready_) {
            // 关节反馈数据写入测量数据
            JointRawMeasurement raw_measurement{};
            raw_measurement.sensor_wrench = sensor_wrench_ =
                utils::GetSensorWrench(sensor_data_primary, sensor_data_secondary);

            for (int i = 0; i < ndof_robot; ++i) {
                raw_measurement.position[i] = robot_feedback_.position[i];
                raw_measurement.velocity[i] = robot_feedback_.velocity[i];
                raw_measurement.effort[i] = robot_feedback_.torque[i];
                // 检测力矩限制是否达到阈值
                if (robot_feedback_.torque[i] >= robot_control_.positive_torque_limit[i] * 0.9 ||
                    robot_feedback_.torque[i] <= -robot_control_.negative_torque_limit[i] * 0.9) {
                    is_torque_limit_reached_[i] = true;
                } else {
                    is_torque_limit_reached_[i] = false;
                }
                raw_measurement.time_point = TimePointSystem::Now();
            }
            this->robot_buffers_.joint_raw_measurement->AppendNewElement(raw_measurement);
        } else {
            this->robot_buffers_.joint_raw_measurement->Reset();
            if (loop_counter % 5000 == 0) {
                RCLCPP_WARN(this->get_logger(), "Robot is not ready, shm status: %d, statusword: %s", ret,
                            utils::RobotStatuswordToJsonString(robot_feedback_.robot_status_word).str().c_str());
                LOG_CRIT << "Robot is not ready, shm status: " << ret
                         << ", statusword: " << robot_feedback_.robot_status_word;
            }
        }

        // 力控计算处理
        torque_controller_manager.LoopOnce();

        // 获取并写入控制数据: 关节位置, 速度, 关节力矩限制, 外部力矩
        PlannerManagerCommand planner_cmd{};
        TorqueCtrlCommand torque_cmd{};
        this->is_cmd_valid_ = false;
        this->manager_buffers_.planner_manager_info->PeekLatest(&this->manager_info_);
        if (this->manager_buffers_.planner_manager_command->PeekLatest(&planner_cmd) &&
            this->robot_buffers_.torque_controller_command->PeekLatest(&torque_cmd)) {
            this->is_cmd_valid_ = true;
            JointMeasurement manager_measurement{};
            for (int i = 0; i < ndof_robot; ++i) {
                this->robot_control_.position[i] = planner_cmd.cmd.position[i];
                this->robot_control_.velocity[i] = planner_cmd.cmd.velocity[i];
                this->robot_control_.positive_torque_limit[i] = torque_limit_.positive_limit_Nm[i];
                this->robot_control_.negative_torque_limit[i] = torque_limit_.negative_limit_Nm[i];
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
                                if (i >= first_arm_joint_index_) {
                                    manager_measurement.q_desired_pre[i] = robot_feedback_.position[i];
                                    this->robot_control_.position[i] = robot_feedback_.position[i];
                                }
                                manager_buffers_.joint_measurement->AppendNewElement(manager_measurement);
                            }
                            this->robot_control_.torque_feedforward[i] = torque_cmd.torque_feedforward[i];
                            this->robot_control_.robot_control_word |= CTRLWORD_MOTOR_CST_MODE;
                        } else {
                            switch_counter_ = 0;
                            previous_mode_ = this->manager_info_.robot_property.run_mode;
                            // 更新关节力矩限制
                            this->robot_control_.positive_torque_limit[i] = std::clamp(
                                torque_cmd.torque_feedforward[i] +
                                    torque_limit_.positive_limit_Nm[i] * this->manager_info_.robot_property.safe_level,
                                0.1 * torque_limit_.positive_limit_Nm[i], torque_limit_.positive_limit_Nm[i]);
                            this->robot_control_.negative_torque_limit[i] = std::clamp(
                                -torque_cmd.torque_feedforward[i] +
                                    torque_limit_.negative_limit_Nm[i] * this->manager_info_.robot_property.safe_level,
                                0.1 * torque_limit_.negative_limit_Nm[i], torque_limit_.negative_limit_Nm[i]);
                            this->robot_control_.torque_feedforward[i] = torque_cmd.torque_feedforward[i];
                        }
                        break;
                    case RunMode::Torque:
                        previous_mode_ = this->manager_info_.robot_property.run_mode;
                        this->robot_control_.torque_feedforward[i] = torque_cmd.torque[i];
                        this->robot_control_.robot_control_word |= CTRLWORD_MOTOR_CST_MODE;
                        break;
                    case RunMode::Drag:
                        previous_mode_ = this->manager_info_.robot_property.run_mode;
                        this->robot_control_.torque_feedforward[i] = torque_cmd.torque_feedforward[i];
                        if (this->robot_model_.IsDynamicParamsAvailable()) {
                            // 位置等于反馈位置
                            if (manager_buffers_.joint_measurement->PeekLatest(&manager_measurement)) {
                                if (i >= first_arm_joint_index_) {
                                    manager_measurement.q_desired_pre[i] = robot_feedback_.position[i];
                                }
                                manager_buffers_.joint_measurement->AppendNewElement(manager_measurement);
                            }
                            this->robot_control_.robot_control_word |= CTRLWORD_MOTOR_CST_MODE;
                        } else {
                            // 位置等于反馈位置
                            if (manager_buffers_.joint_measurement->PeekLatest(&manager_measurement)) {
                                if (i >= first_arm_joint_index_ && is_torque_limit_reached_[i]) {
                                    manager_measurement.q_desired_pre[i] = robot_feedback_.position[i];
                                }
                                manager_buffers_.joint_measurement->AppendNewElement(manager_measurement);
                            }
                            // 更新关节力矩限制
                            if (torque_cmd.torque_feedforward[i] >= 0) {
                                robot_control_.positive_torque_limit[i] = std::clamp(
                                    torque_cmd.torque_feedforward[i], 0.0, torque_limit_.positive_limit_Nm[i]);
                                robot_control_.negative_torque_limit[i] = 0.0;
                            } else {
                                robot_control_.positive_torque_limit[i] = 0.0;
                                robot_control_.negative_torque_limit[i] = std::clamp(
                                    -torque_cmd.torque_feedforward[i], 0.0, torque_limit_.negative_limit_Nm[i]);
                            }
                        }
                        break;
                    case RunMode::CartesianImpedance:
                        previous_mode_ = this->manager_info_.robot_property.run_mode;
                        this->robot_control_.torque_feedforward[i] = torque_cmd.torque_impedance[i];
                        this->robot_control_.robot_control_word |= CTRLWORD_MOTOR_CST_MODE;
                        break;
                    case RunMode::DragImpedance:
                        previous_mode_ = this->manager_info_.robot_property.run_mode;
                        this->robot_control_.torque_feedforward[i] = torque_cmd.torque[i];
                        this->robot_control_.robot_control_word |= CTRLWORD_MOTOR_CST_MODE;
                        // 位置等于反馈位置
                        if (manager_buffers_.joint_measurement->PeekLatest(&manager_measurement)) {
                            if (i >= first_arm_joint_index_) {
                                manager_measurement.q_desired_pre[i] = robot_feedback_.position[i];
                            }
                            manager_buffers_.joint_measurement->AppendNewElement(manager_measurement);
                        }
                        break;
                    default:
                        assert(false && "Unknown RunMode");
                        break;
                }

                // 对 torso J1-J4 (索引 0-3) 应用 EI 输入整形
                if (i >= 0 && i < 4) {
                    // 检测 torso 整形器开关状态变化
                    bool current_torso_enabled = enable_torso_shaper_.load();
                    if (current_torso_enabled != prev_torso_shaper_enabled_) {
                        torso_shapers_[i]->Reset();
                        prev_torso_shaper_enabled_ = current_torso_enabled;
                        RCLCPP_INFO(this->get_logger(), "Torso J%d shaper %s, buffer reset", i + 1,
                                    current_torso_enabled ? "enabled" : "disabled");
                    }

                    // 应用整形
                    if (current_torso_enabled) {
                        robot_control_.position[i] = torso_shapers_[i]->Apply(robot_control_.position[i]);
                    }
                }
            }
        } else {
            for (int i = 0; i < ndof_robot; ++i) {
                this->butterworth_filter_[i]->Reset();  // 重置滤波器状态
            }
            for (int i = 0; i < 4; ++i) {
                torso_shapers_[i]->Reset();
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

        // 底盘控制：应用输入整形并写入共享内存
        if (robot_mode_ == RobotMode::Arm) {
            RobotControl arm_control{};
            ExtractVirtual20To7(robot_control_, arm_control);
            control_shm_instance.AppendNewElement(arm_control);
        } else if (robot_mode_ == RobotMode::A1) {
            RobotControl a1_control{};
            ExtractVirtual20To14(robot_control_, a1_control);
            control_shm_instance.AppendNewElement(a1_control);
        } else {
            robot_control_.chassis_mode = chassis_control_target_.chassis_mode;
            robot_control_.chassis_cnt++;

            if (chassis_control_target_.chassis_mode == 1) {
                // 差速模式：对左右轮速度应用输入整形
                bool current_chassis_enabled = enable_chassis_shaper_.load();
                if (current_chassis_enabled != prev_chassis_shaper_enabled_) {
                    chassis_shaper_->Reset();
                    prev_chassis_shaper_enabled_ = current_chassis_enabled;
                    RCLCPP_INFO(this->get_logger(), "Chassis shaper %s, buffer reset",
                                current_chassis_enabled ? "enabled" : "disabled");
                }
                // 应用整形
                if (current_chassis_enabled) {
                    auto [shaper_left, shaper_right] = chassis_shaper_->Apply(
                        chassis_control_target_.chassis_left_velocity, chassis_control_target_.chassis_right_velocity);
                    robot_control_.chassis_left_velocity = shaper_left;
                    robot_control_.chassis_right_velocity = shaper_right;
                } else {
                    robot_control_.chassis_left_velocity = chassis_control_target_.chassis_left_velocity;
                    robot_control_.chassis_right_velocity = chassis_control_target_.chassis_right_velocity;
                }
            } else if (chassis_control_target_.chassis_mode == 2) {
                // 全向电机控制模式：直接复制
                for (size_t i = 0; i < OMNI_MOTOR_NUM; i++) {
                    robot_control_.chassis_motor_velocity[i] = chassis_control_target_.chassis_motor_velocity[i];
                    robot_control_.chassis_motor_direction[i] = chassis_control_target_.chassis_motor_direction[i];
                }
                robot_control_.chassis_estop = chassis_control_target_.chassis_estop;
                robot_control_.chassis_neutral = chassis_control_target_.chassis_neutral;
            } else if (chassis_control_target_.chassis_mode == 0) {
                // 全向整车控制模式：复制参数
                robot_control_.chassis_whole_mode = chassis_control_target_.chassis_whole_mode;
                robot_control_.chassis_velocity = chassis_control_target_.chassis_velocity;
                robot_control_.chassis_direction = chassis_control_target_.chassis_direction;
            }
            control_shm_instance.AppendNewElement(robot_control_);
        }
        std::this_thread::sleep_until(next_cycle_time_point);
    }
}

void RobotServer::SwitchControlWord() {
    static bool inited = false;
    // 0. Sleep 模式下进入QUICK_STOP，阻止写入measurement
    if (this->manager_info_.robot_property.run_mode == RunMode::Sleep) {
        this->is_robot_ready_ = false;
        robot_control_.robot_control_word = CTRLWORD_MOTOR_QUICK_STOP;
        for (int i = 0; i < ndof_robot; i++) {
            this->robot_control_.position[i] = this->robot_feedback_.position[i];
            this->robot_control_.positive_torque_limit[i] = this->torque_limit_.positive_limit_Nm[i];
            this->robot_control_.negative_torque_limit[i] = this->torque_limit_.negative_limit_Nm[i];
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
                for (int i = 0; i < ndof_robot; i++) {
                    this->robot_control_.position[i] = this->robot_feedback_.position[i];
                    this->robot_control_.positive_torque_limit[i] = this->torque_limit_.positive_limit_Nm[i];
                    this->robot_control_.negative_torque_limit[i] = this->torque_limit_.negative_limit_Nm[i];
                }
            }
            break;
        case STATUSWORD_MOTOR_ENABLED:
            this->is_robot_ready_ = true;
            inited = true;
            this->robot_control_.robot_control_word |= CTRLWORD_MOTOR_ENABLE;
            if (!this->is_cmd_valid_) {
                for (int i = 0; i < ndof_robot; i++) {
                    this->robot_control_.position[i] = this->robot_feedback_.position[i];
                    this->robot_control_.positive_torque_limit[i] = this->torque_limit_.positive_limit_Nm[i];
                    this->robot_control_.negative_torque_limit[i] = this->torque_limit_.negative_limit_Nm[i];
                }
            }
            break;
        case STATUSWORD_MOTOR_NOT_READY:
        case STATUSWORD_MOTOR_FAULT:
        case STATUSWORD_MOTOR_QUICK_STOP:
        default:
            for (int i = 0; i < ndof_robot; i++) {
                this->robot_control_.position[i] = this->robot_feedback_.position[i];
                this->robot_control_.positive_torque_limit[i] = this->torque_limit_.positive_limit_Nm[i];
                this->robot_control_.negative_torque_limit[i] = this->torque_limit_.negative_limit_Nm[i];
            }
            break;
    }
    // 2. 如果初次启动或者状态字是故障，则发送清除故障命令
    if (!inited || manager_info_.robot_property.clear_error_flag) {
        this->robot_control_.robot_control_word |= CTRLWORD_MOTOR_CLEAR_FAULT;
    }
}

void RobotServer::PublishThreadFunc() {
    JointRawMeasurement raw_measurement;
    Eigen::VectorXd ee_pose;
    Eigen::VectorXd torso_X_Z_RY;
    PlannerManagerCommand planner_cmd{};
    auto next_cycle_time_point = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
        next_cycle_time_point += std::chrono::microseconds(publish_dt_micro);
        // Publish /feedback/robot_server_state

        this->robot_server_state_publisher_->publish(GetRobotServerStateMsg());
        std::this_thread::sleep_until(next_cycle_time_point);
    }
}

// clang-format off
std_msgs::msg::String RobotServer::GetRobotServerStateMsg()
{
    auto robot_server_state_msg = std_msgs::msg::String();
    uint64_t timestamp_ns = this->now().nanoseconds();
    // A1/Arm模式下数据在虚拟20关节表示中的6-19位置，需要提取
    int idx_offset = (robot_mode_ != RobotMode::W1) ? first_arm_joint_index_ : 0;
    int cart_param_count = (robot_mode_ == RobotMode::Arm) ? 6 : static_cast<int>(this->manager_info_.robot_property.cart_stiffness.size());
    robot_server_state_msg.data =
        std::string("{\n  \"timestamp\":") + std::to_string(timestamp_ns) +
        ", \"robot_type\":\"" + this->robot_model_.params_ptr->robot_name + "\"" +
        ", \"robot_version\":\"" + this->robot_model_.params_ptr->robot_version + "\"" +
        ", \"latest_queued_id\":" + std::to_string(this->latest_report_id_.queued) +
        ", \"latest_finished_id\":" + std::to_string(this->latest_report_id_.finished) +
        ", \"latest_errordrop_id\":" + std::to_string(this->latest_report_id_.error_drop) +
        ",\n  \"status\":\"" + ToString(this->manager_state_) + "\"" +
        ", \"run_mode\":\"" + ToString(this->manager_info_.robot_property.run_mode) + "\"" +
        ", \"server_error_code\":\"" + ToString(this->manager_info_.robot_property.error_code) + "\"" +
        ", \"safe_level\":\"" + std::to_string(this->manager_info_.robot_property.safe_level) + "\"" +
        ", \"global_speed_ratio\":\"" + std::to_string(this->manager_info_.robot_property.global_speed_ratio) + "\"" +
        ",\n  \"joint_stiffness\":" + utils::VectorToJsonArrayString(this->manager_info_.robot_property.joint_stiffness, idx_offset, ndof_config_).str() +
        ",\n  \"joint_damping\":" + utils::VectorToJsonArrayString(this->manager_info_.robot_property.joint_damping, idx_offset, ndof_config_).str() +
        ",\n  \"cart_stiffness\":" + utils::VectorToJsonArrayString(this->manager_info_.robot_property.cart_stiffness, 0, cart_param_count).str() +
        ",\n  \"cart_damping_ratio\":" + utils::VectorToJsonArrayString(this->manager_info_.robot_property.cart_damping_ratio, 0, cart_param_count).str() +
        ",\n  \"cart_impedance_force\":" + utils::VectorToJsonArrayString(this->manager_info_.robot_property.cart_impedance_force, 0, cart_param_count).str() +
        ",\n  \"robot_statusword\":" + utils::RobotStatuswordToJsonString(this->robot_feedback_.robot_status_word).str() +
        ", \"robot_controlword\":" + utils::RobotControlwordToJsonString(robot_control_.robot_control_word).str() +
        ",\n  \"motor_status\":" + utils::MotorStatusToJsonArrayString(this->robot_feedback_.al_status, idx_offset, ndof_config_).str() +
        ",\n  \"motor_error_code\":" + utils::MotorErrorcodeToJsonArrayString(this->robot_feedback_.motor_error_code, idx_offset, ndof_config_).str() +
        ",\n  \"motor_temperature\":" + utils::VectorToJsonArrayString(this->robot_feedback_.motor_temperature, idx_offset, ndof_config_).str() +
        ",\n  \"joint_position\":" + utils::VectorToJsonArrayString(this->robot_feedback_.position, idx_offset, ndof_config_).str() +
        ",\n  \"joint_position_cmd\":" + utils::VectorToJsonArrayString(robot_control_.position, idx_offset, ndof_config_).str() +
        ",\n  \"joint_velocity\":" + utils::VectorToJsonArrayString(this->robot_feedback_.velocity, idx_offset, ndof_config_).str() +
        ",\n  \"joint_velocity_cmd\":" + utils::VectorToJsonArrayString(this->robot_control_.velocity, idx_offset, ndof_config_).str() +
        ",\n  \"joint_torque\":" + utils::VectorToJsonArrayString(this->robot_feedback_.torque, idx_offset, ndof_config_).str() +
        ",\n  \"joint_torque_cmd\":" + utils::VectorToJsonArrayString(this->robot_control_.torque_feedforward, idx_offset, ndof_config_).str() +
        ",\n  \"force_sensor\":" + utils::VectorToJsonArrayString(sensor_wrench_).str() +
        ",\n  \"torque_limit_negative\":" + utils::VectorToJsonArrayString(this->robot_control_.negative_torque_limit, idx_offset, ndof_config_).str() +
        ",\n  \"torque_limit_positive\":" + utils::VectorToJsonArrayString(this->robot_control_.positive_torque_limit, idx_offset, ndof_config_).str() +
        ",\n  \"ee_pose\":" + utils::VectorToJsonArrayString(this->manager_info_.ee_pose_measurement, 0, cart_param_count).str() +
        ",\n  \"torso_pose_X_Z_RY\":" + utils::VectorToJsonArrayString(this->manager_info_.torso_pose_measurement, 0, cart_param_count).str() + "}";
    return robot_server_state_msg;
}
// clang-format on

void RobotServer::PublishFeedback() {
    // 发布关节状态反馈，A1模式只发布14个关节，W1模式发布20个关节
    sensor_msgs::msg::JointState feedback_msg;
    feedback_msg.header.stamp = this->get_clock()->now();

    feedback_msg.name.resize(ndof_config_);
    feedback_msg.position.resize(ndof_config_);
    feedback_msg.velocity.resize(ndof_config_);
    feedback_msg.effort.resize(ndof_config_);

    // A1/Arm模式下数据在虚拟20关节表示中的6-19位置，需要提取
    int idx_offset = (robot_mode_ != RobotMode::W1) ? first_arm_joint_index_ : 0;
    for (int i = 0; i < ndof_config_; ++i) {
        feedback_msg.name[i] = ecat_joint_name_[idx_offset + i];
        feedback_msg.position[i] = robot_feedback_.position[idx_offset + i];
        feedback_msg.velocity[i] = robot_feedback_.velocity[idx_offset + i];
        feedback_msg.effort[i] = robot_feedback_.torque[idx_offset + i];
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
        limit_msg.name[i] = ecat_joint_name_[idx_offset + i];
        limit_msg.positive_velocity_limit[i] = this->robot_control_.positive_velocity_limit[idx_offset + i];
        limit_msg.negative_velocity_limit[i] = this->robot_control_.negative_velocity_limit[idx_offset + i];
        limit_msg.positive_torque_limit[i] = this->robot_control_.positive_torque_limit[idx_offset + i];
        limit_msg.negative_torque_limit[i] = this->robot_control_.negative_torque_limit[idx_offset + i];
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
            if (ecat_joint_name_[j] == msg->name[i]) {
                robot_control_.positive_velocity_limit[j] = msg->positive_velocity_limit[i];
                robot_control_.negative_velocity_limit[j] = msg->negative_velocity_limit[i];
                robot_control_.positive_torque_limit[j] = msg->positive_torque_limit[i];
                robot_control_.negative_torque_limit[j] = msg->negative_torque_limit[i];
                if (std::abs(robot_control_temp.position[j] - msg->position[i]) > position_diff_threshold_ &&
                    position_valid) {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Control command for joint %s rejected: position difference %.4f exceeds threshold %.4f",
                        ecat_joint_name_[j].c_str(), std::abs(robot_control_temp.position[j] - msg->position[i]),
                        position_diff_threshold_);
                    LOG_CRIT << "Control command for joint " << ecat_joint_name_[j] << " rejected: position difference "
                             << std::abs(robot_control_temp.position[j] - msg->position[i]) << " exceeds threshold "
                             << position_diff_threshold_;
                    position_valid = false;
                }
                robot_control_temp.position[j] = msg->position[i];

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
            move_request.optional_mode = enable_ruckig_ ? false : true;  // true: no ruckig 模式, false: ruckig 模式
            planner_request.data = move_request;
            streaming_request_id_ = StartStreaming(planner_request);
        }
        if (streaming_request_id_ > -1) {
            // 对位置指令应用巴特沃斯滤波
            if (this->enable_butterworth_filter_.load()) {
                for (int j = 0; j < ndof_robot; ++j) {
                    robot_control_temp.position[j] = butterworth_filter_[j]->Apply(robot_control_temp.position[j]);
                }
            }
            StreamingCommandMsg::SharedPtr streaming_msg = std::make_shared<StreamingCommandMsg>();
            streaming_msg->request_id = streaming_request_id_;
            streaming_msg->streaming_finished = false;
            streaming_msg->position =
                std::vector<float>(robot_control_temp.position, robot_control_temp.position + ndof_robot);
            this->StreamingCommandCallback(streaming_msg);
        }
    }
}

void RobotServer::PositionControlCallback(const JointPositionControlMsg::SharedPtr msg) {
    auto robot_control_temp = robot_control_;
    bool position_valid = true;
    for (size_t i = 0; i < msg->name.size(); ++i) {
        for (int j = 0; j < ndof_robot; ++j) {
            if (ecat_joint_name_[j] == msg->name[i] && position_valid) {
                if (std::abs(robot_control_temp.position[j] - msg->position[i]) > position_diff_threshold_) {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Control command for joint %s rejected: position difference %.4f exceeds threshold %.4f",
                        ecat_joint_name_[j].c_str(), std::abs(robot_control_temp.position[j] - msg->position[i]),
                        position_diff_threshold_);
                    LOG_CRIT << "Control command for joint " << ecat_joint_name_[j] << " rejected: position difference "
                             << std::abs(robot_control_temp.position[j] - msg->position[i]) << " exceeds threshold "
                             << position_diff_threshold_;
                    position_valid = false;
                } else {
                    robot_control_temp.position[j] = msg->position[i];
                }
                break;  // 已经找到并处理了对应的关节，跳出内层循环继续处理下一个输入关节
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
            move_request.optional_mode = enable_ruckig_ ? false : true;  // true: no ruckig 模式, false: ruckig 模式
            move_request.streaming_timeout = streaming_timeout_;
            planner_request.data = move_request;
            streaming_request_id_ = StartStreaming(planner_request);
        }
        if (streaming_request_id_ != -1) {
            // 对位置指令应用巴特沃斯滤波
            if (this->enable_butterworth_filter_.load()) {
                for (int j = 0; j < ndof_robot; ++j) {
                    robot_control_temp.position[j] = butterworth_filter_[j]->Apply(robot_control_temp.position[j]);
                }
            }
            StreamingCommandMsg::SharedPtr streaming_msg = std::make_shared<StreamingCommandMsg>();
            streaming_msg->request_id = streaming_request_id_;
            streaming_msg->streaming_finished = false;
            streaming_msg->position =
                std::vector<float>(robot_control_temp.position, robot_control_temp.position + ndof_robot);
            this->StreamingCommandCallback(streaming_msg);
        }
    }
}
void RobotServer::DebugCallback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "/control/debug message received: %s", msg->data.c_str());
    LOG_INFO << "/control/debug message received: " << msg->data;

    try {
        const auto json_data = nlohmann::json::parse(msg->data);

        auto get_bool_value = [](const nlohmann::json &obj, const char *key) -> std::optional<bool> {
            if (!obj.contains(key)) return std::nullopt;
            const auto &v = obj[key];
            if (v.is_boolean()) return v.get<bool>();
            if (v.is_number_integer()) return v.get<int>() != 0;
            return std::nullopt;
        };

        auto update_bool_param = [this, &json_data, &get_bool_value](const char *key, bool &param) {
            auto value = get_bool_value(json_data, key);
            if (value.has_value()) {
                param = value.value();
                RCLCPP_INFO(this->get_logger(), "%s set to %s", key, param ? "true" : "false");
                LOG_INFO << key << " set to " << (param ? "true" : "false");
            }
        };

        auto update_atomic_bool_param = [this, &json_data, &get_bool_value](const char *key, std::atomic<bool> &param) {
            auto value = get_bool_value(json_data, key);
            if (value.has_value()) {
                bool bool_value = value.value();
                param.store(bool_value);
                RCLCPP_INFO(this->get_logger(), "%s set to %s", key, bool_value ? "true" : "false");
                LOG_INFO << key << " set to " << (bool_value ? "true" : "false");
            }
        };

        update_bool_param("enable_record", this->enable_record_);
        update_atomic_bool_param("enable_chassis_shaper", this->enable_chassis_shaper_);
        update_atomic_bool_param("enable_torso_shaper", this->enable_torso_shaper_);
        update_atomic_bool_param("enable_butterworth_filter", this->enable_butterworth_filter_);

    } catch (const std::exception &e) {
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

// A1 模式虚拟关节转换函数：将14关节反馈扩展为20关节虚拟反馈
void RobotServer::Expand14ToVirtual20(const RobotFeedback &real_feedback, RobotFeedback &virtual_feedback) {
    // 复制基本字段
    virtual_feedback.cnt = real_feedback.cnt;
    virtual_feedback.robot_status_word = real_feedback.robot_status_word;
    virtual_feedback.robot_error_code = real_feedback.robot_error_code;

    // 躯干 (0-5) 的电机状态保持为0
    // 双臂 (6-19) <- real[0-13]，左臂real[0-6]->virtual[6-12]，右臂real[7-13]->virtual[13-19]
    for (int i = 0; i < ndof_config_; ++i) {
        virtual_feedback.al_status[first_arm_joint_index_ + i] = real_feedback.al_status[i];
        virtual_feedback.motor_error_code[first_arm_joint_index_ + i] = real_feedback.motor_error_code[i];
        virtual_feedback.motor_temperature[first_arm_joint_index_ + i] = real_feedback.motor_temperature[i];
        virtual_feedback.position[first_arm_joint_index_ + i] = real_feedback.position[i];
        virtual_feedback.velocity[first_arm_joint_index_ + i] = real_feedback.velocity[i];
        virtual_feedback.torque[first_arm_joint_index_ + i] = real_feedback.torque[i];
    }
}

// A1 模式虚拟关节转换函数：将20关节虚拟控制提取为14关节实际控制
void RobotServer::ExtractVirtual20To14(const RobotControl &virtual_control, RobotControl &real_control) {
    // 复制基本字段
    real_control.cnt = virtual_control.cnt;
    real_control.robot_control_word = virtual_control.robot_control_word;
    // A1无底盘控制，跳过底盘控制字段

    // 双臂 (0-13) <- virtual[6-19]，左臂virtual[6-12]->real[0-6]，右臂virtual[13-19]->real[7-13]
    for (int i = 0; i < ndof_config_; ++i) {
        real_control.position[i] = virtual_control.position[first_arm_joint_index_ + i];
        real_control.velocity[i] = virtual_control.velocity[first_arm_joint_index_ + i];
        real_control.acceleration[i] = virtual_control.acceleration[first_arm_joint_index_ + i];
        real_control.torque_feedforward[i] = virtual_control.torque_feedforward[first_arm_joint_index_ + i];
        real_control.torque_external[i] = virtual_control.torque_external[first_arm_joint_index_ + i];
        real_control.positive_velocity_limit[i] = virtual_control.positive_velocity_limit[first_arm_joint_index_ + i];
        real_control.negative_velocity_limit[i] = virtual_control.negative_velocity_limit[first_arm_joint_index_ + i];
        real_control.positive_torque_limit[i] = virtual_control.positive_torque_limit[first_arm_joint_index_ + i];
        real_control.negative_torque_limit[i] = virtual_control.negative_torque_limit[first_arm_joint_index_ + i];
    }
}

// Arm 模式虚拟关节转换函数：将7关节反馈扩展为20关节虚拟反馈
void RobotServer::Expand7ToVirtual20(const RobotFeedback &real_feedback, RobotFeedback &virtual_feedback) {
    // 复制基本字段
    virtual_feedback.cnt = real_feedback.cnt;
    virtual_feedback.robot_status_word = real_feedback.robot_status_word;
    virtual_feedback.robot_error_code = real_feedback.robot_error_code;

    // 躯干 (0-5) 和右臂 (13-19) 的电机状态保持为0
    // 左臂 (6-12) <- real[0-6]
    for (int i = 0; i < ndof_config_; ++i) {
        virtual_feedback.al_status[first_arm_joint_index_ + i] = real_feedback.al_status[i];
        virtual_feedback.motor_error_code[first_arm_joint_index_ + i] = real_feedback.motor_error_code[i];
        virtual_feedback.motor_temperature[first_arm_joint_index_ + i] = real_feedback.motor_temperature[i];
        virtual_feedback.position[first_arm_joint_index_ + i] = real_feedback.position[i];
        virtual_feedback.velocity[first_arm_joint_index_ + i] = real_feedback.velocity[i];
        virtual_feedback.torque[first_arm_joint_index_ + i] = real_feedback.torque[i];
    }
}

// Arm 模式虚拟关节转换函数：将20关节虚拟控制提取为7关节实际控制
void RobotServer::ExtractVirtual20To7(const RobotControl &virtual_control, RobotControl &real_control) {
    // 复制基本字段
    real_control.cnt = virtual_control.cnt;
    real_control.robot_control_word = virtual_control.robot_control_word;
    // Arm无底盘控制，跳过底盘控制字段

    // 左臂 (0-6) <- virtual[6-12]
    for (int i = 0; i < ndof_config_; ++i) {
        real_control.position[i] = virtual_control.position[first_arm_joint_index_ + i];
        real_control.velocity[i] = virtual_control.velocity[first_arm_joint_index_ + i];
        real_control.acceleration[i] = virtual_control.acceleration[first_arm_joint_index_ + i];
        real_control.torque_feedforward[i] = virtual_control.torque_feedforward[first_arm_joint_index_ + i];
        real_control.torque_external[i] = virtual_control.torque_external[first_arm_joint_index_ + i];
        real_control.positive_velocity_limit[i] = virtual_control.positive_velocity_limit[first_arm_joint_index_ + i];
        real_control.negative_velocity_limit[i] = virtual_control.negative_velocity_limit[first_arm_joint_index_ + i];
        real_control.positive_torque_limit[i] = virtual_control.positive_torque_limit[first_arm_joint_index_ + i];
        real_control.negative_torque_limit[i] = virtual_control.negative_torque_limit[first_arm_joint_index_ + i];
    }
}

}  // namespace robot_server_lib
