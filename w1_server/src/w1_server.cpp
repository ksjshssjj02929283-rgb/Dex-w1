#include "w1_server.h"

#include <cstdlib>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <system_error>

namespace robot_server_lib
{

RobotServer::RobotServer(std::filesystem::path robot_cfg_path, bool is_sim) : RobotServerBase(robot_cfg_path, is_sim)
{
	LOG_INFO << "w1_server version: " << W1_SERVER_VERSION;
    
    // 创建 /feedback/joint 发布器
    feedback_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/feedback/joint", 10);

    // 创建 /feedback/joint_velocity_torque_limit 发布器
    velocity_torque_limit_feedback_publisher_ =
        this->create_publisher<JointVelocityTorqueLimitFeedbackMsg>("/feedback/joint_velocity_torque_limit", 10);

    // 创建 /control/joint 订阅器
    control_subscriber_ =
        this->create_subscription<JointControlMsg>("/control/joint", 10, std::bind(&RobotServer::ControlCallback, this, std::placeholders::_1));

    // 创建 /control/joint_position 订阅器
    position_control_subscriber_ = this->create_subscription<JointPositionControlMsg>(
        "/control/joint_position", 10, std::bind(&RobotServer::PositionControlCallback, this, std::placeholders::_1));

    // 创建 /control/omni_chassis 订阅器
    omni_chassis_control_subscriber_ = this->create_subscription<ChassisWholeControlMsg>(
        "/control/omni_chassis", 10, std::bind(&RobotServer::OmniChassisControlCallback, this, std::placeholders::_1));

    // 创建 /control/omni_chassis_motors 订阅器
    omni_chassis_motor_control_subscriber_ = this->create_subscription<ChassisMotorControlMsg>(
        "/control/omni_chassis_motors", 10, std::bind(&RobotServer::OmniChassisMotorControlCallback, this, std::placeholders::_1));

    // 创建 /control/cmd_vel 订阅器
    diff_chassis_control_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/control/cmd_vel", 10, std::bind(&RobotServer::DiffChassisControlCallback, this, std::placeholders::_1));

    force_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "force_array", rclcpp::QoS(10), std::bind(&RobotServer::ForceArrayCallback, this, std::placeholders::_1));

    // 参数初始化
    InitParamConfig();
    if (enable_debug_log_)
    {
        nanolog::set_log_level(nanolog::LogLevel::DEBUG);
    }
    // 创建定时器，定时发布feedback 定时器回调函数
    control_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() { PublishFeedback(); });

    for (int i = 0; i < ndof_robot; ++i)
    {
        butterworth_filter_[i] = std::make_shared<ButterworthFilter>();
    }
    // 初始化力传感器
    // primary_sensor_ = std::make_shared<ForceSensorReader>("/dev/ttyUSB2");
    // secondary_sensor_ = std::make_shared<ForceSensorReader>("/dev/ttyUSB1");

    this->Run();
}

void RobotServer::ForceArrayCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // msg->data 是 std::vector<double>
    // 示例：若期望 3 维力数据
    if (msg->data.size() < 3)
    {
        RCLCPP_WARN(this->get_logger(), "Force array size %zu < 3", msg->data.size());
        return;
    }
    // 使用数据，例如拷贝前 3 个
    for (size_t i = 0; i < 3; ++i) sensor_force_[i] = msg->data[i];
    RCLCPP_INFO(this->get_logger(), "Received force array: [%f, %f, %f]", sensor_force_[0], sensor_force_[1], sensor_force_[2]);
}

void RobotServer::InitParamConfig()
{
    auto config_path = server_data_dir_ / "w1_config.json";

    const std::set<std::string> kAllowedRootKeys{"enable_debug_log", "enable_ruckig", "enable_cst_mode"};

    nlohmann::json root;
    // 若文件不存在则创建默认配置
    if (!std::filesystem::exists(config_path))
    {
        root["enable_debug_log"] = this->enable_debug_log_;
        root["enable_ruckig"] = this->enable_ruckig_;
        root["enable_cst_mode"] = this->enable_cst_mode_;

        std::ofstream ofs(config_path);
        ofs << root.dump(2);
        RCLCPP_INFO(this->get_logger(), "Created default w1_config.json at: %s", config_path.string().c_str());
        return;
    }
    else
    {
        std::ifstream ifs(config_path);
        try
        {
            root = nlohmann::json::parse(ifs);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse json: %s", e.what());
            return;
        }
    }

    bool need_rewrite = false;

    // 清理未定义根字段
    for (auto it = root.begin(); it != root.end(); ++it)
    {
        if (!kAllowedRootKeys.count(it.key()))
        {
            need_rewrite = true;
            break;
        }
    }

    // 若缺失则补默认
    if (!root.contains("enable_debug_log"))
    {
        root["enable_debug_log"] = this->enable_debug_log_;
        need_rewrite = true;
    }
    if (!root.contains("enable_ruckig"))
    {
        root["enable_ruckig"] = this->enable_ruckig_;
        need_rewrite = true;
    }
    if (!root.contains("enable_cst_mode"))
    {
        root["enable_cst_mode"] = this->enable_cst_mode_;
        need_rewrite = true;
    }

    // 同步布尔值
    this->enable_debug_log_ = root.value("enable_debug_log", this->enable_debug_log_);
    this->enable_ruckig_ = root.value("enable_ruckig", this->enable_ruckig_);
    this->enable_cst_mode_ = root.value("enable_cst_mode", this->enable_cst_mode_);

    // 构建干净配置
    nlohmann::json root_clean;
    root_clean["enable_debug_log"] = this->enable_debug_log_;
    root_clean["enable_ruckig"] = this->enable_ruckig_;
    root_clean["enable_cst_mode"] = this->enable_cst_mode_;

    // 如果发现未定义字段或缺失字段，则回写干净配置
    if (need_rewrite)
    {
        std::ofstream ofs(config_path);
        ofs << root_clean.dump(2);
    }

    RCLCPP_INFO(this->get_logger(), "Loaded w1_config.json from: %s (enable_ruckig=%s, enable_debug_log=%s, enable_cst_mode=%s)",
                config_path.string().c_str(), this->enable_debug_log_ ? "true" : "false", this->enable_ruckig_ ? "true" : "false",
                this->enable_cst_mode_ ? "true" : "false");
    LOG_INFO << "Loaded w1_config.json from: " << config_path << ", enable_debug_log=" << this->enable_debug_log_
             << ", enable_ruckig=" << this->enable_ruckig_ << ", enable_cst_mode=" << this->enable_cst_mode_;
}

void RobotServer::TorqueControlThreadFunc()
{
    TorqueControllerManager torque_controller_manager(this->robot_model_, this->manager_buffers_, this->robot_buffers_);
    // 初始化共享内存
    auto& feedback_shm_instance = FeedbackSharedMemory::GetInstance();
    auto& control_shm_instance = ControlSharedMemory::GetInstance();

    Int64 loop_counter = 0;
    this->is_robot_ready_ = false;
    auto next_cycle_time_point = std::chrono::steady_clock::now();
    while (rclcpp::ok())
    {
        loop_counter++;
        next_cycle_time_point += std::chrono::microseconds(torque_ctrl_dt_micro);

        // 读取力传感器数据
        Vector6d sensor_data_primary = Vector6d::Zero();
        Vector6d sensor_data_secondary = Vector6d::Zero();
        if (primary_sensor_)
        {
            primary_sensor_->Read(sensor_data_primary);
        }
        if (secondary_sensor_)
        {
            secondary_sensor_->Read(sensor_data_secondary);
        }

        // 叠加手动输入的力数据
        sensor_data_primary.head<3>() += sensor_force_;

        bool ret = feedback_shm_instance.PeekLatest(&robot_feedback_) && robot_feedback_.cnt > 0;
        // 根据状态写入机器人控制字
        robot_control_.robot_control_word = CTRLWORD_MOTOR_SWITCH_ON;
        this->is_robot_ready_ = false;
        if (ret)
        {
            if ((manager_state_ == Status::Running || manager_state_ == Status::Idle) && loop_counter % 4 == 0)
            {
                LOG_DEBUG << "{ \"joint_position\": " << utils::VectorToJsonArrayString(robot_feedback_.position, 6, ndof_robot).str()
                          << ", \"joint_velocity\": " << utils::VectorToJsonArrayString(robot_feedback_.velocity, 6, ndof_robot).str()
                          << ", \"joint_torque\":" << utils::VectorToJsonArrayString(robot_feedback_.torque, 6, ndof_robot).str()
                          << ", \"joint_position_cmd\": " << utils::VectorToJsonArrayString(robot_control_.position, 6, ndof_robot).str()
                          << ", \"joint_torque_feedforward\": "
                          << utils::VectorToJsonArrayString(robot_control_.torque_feedforward, 6, ndof_robot).str() << "}";
            }
            // 根据反馈状态字处理控制字
            this->SwitchControlWord();
        }

        // 如果机器人就绪，则写入测量数据
        if (this->is_robot_ready_)
        {
            // 关节反馈数据写入测量数据
            JointRawMeasurement raw_measurement{};
            raw_measurement.sensor_wrench = sensor_wrench_ = utils::GetSensorWrench(sensor_data_primary, sensor_data_secondary);

            for (int i = 0; i < ndof_robot; ++i)
            {
                raw_measurement.position[i] = robot_feedback_.position[i];
                raw_measurement.velocity[i] = robot_feedback_.velocity[i];
                raw_measurement.effort[i] = robot_feedback_.torque[i];
                // 检测力矩限制是否达到阈值
                if (robot_feedback_.torque[i] >= robot_control_.positive_torque_limit[i] * 0.9 ||
                    robot_feedback_.torque[i] <= -robot_control_.negative_torque_limit[i] * 0.9)
                {
                    is_torque_limit_reached_[i] = true;
                }
                else
                {
                    is_torque_limit_reached_[i] = false;
                }
                raw_measurement.time_point = TimePointSystem::Now();
            }
            this->robot_buffers_.joint_raw_measurement->AppendNewElement(raw_measurement);
        }
        else
        {
            this->robot_buffers_.joint_raw_measurement->Reset();
            if (loop_counter % 5000 == 0)
            {
                RCLCPP_WARN(this->get_logger(), "Robot is not ready, shm status: %d, statusword: %s", ret,
                            utils::RobotStatuswordToJsonString(robot_feedback_.robot_status_word).str().c_str());
                LOG_CRIT << "Robot is not ready, shm status: " << ret << ", statusword: " << robot_feedback_.robot_status_word;
            }
        }

        // 力控计算处理
        torque_controller_manager.LoopOnce();

        // 获取并写入控制数据: 关节位置, 速度, 关节力矩限制, 外部力矩
        PlannerManagerCommand planner_cmd{};
        TorqueCtrlCommand torque_cmd{};
        is_cmd_valid_ = false;
        this->manager_buffers_.planner_manager_info->PeekLatest(&planner_info_);
        if (this->manager_buffers_.planner_manager_command->PeekLatest(&planner_cmd) &&
            this->robot_buffers_.torque_controller_command->PeekLatest(&torque_cmd))
        {
            is_cmd_valid_ = true;
            JointMeasurement manager_measurement{};
            for (int i = 0; i < ndof_robot; ++i)
            {
                switch (planner_info_.robot_property.run_mode)
                {
                case RunMode::Position:
                case RunMode::Impedance:
                    // 更新关节力矩限制
                    robot_control_.position[i] = planner_cmd.cmd.compliance_position[i];
                    robot_control_.velocity[i] = planner_cmd.cmd.velocity[i];
                    robot_control_.torque_feedforward[i] = torque_cmd.torque_feedforward[i];
                    robot_control_.positive_torque_limit[i] =
                        std::clamp(torque_cmd.torque_feedforward[i] + torque_limit_.positive_limit_Nm[i] * planner_info_.robot_property.safe_level,
                                   0.1 * torque_limit_.positive_limit_Nm[i], torque_limit_.positive_limit_Nm[i]);
                    robot_control_.negative_torque_limit[i] =
                        std::clamp(-torque_cmd.torque_feedforward[i] + torque_limit_.negative_limit_Nm[i] * planner_info_.robot_property.safe_level,
                                   0.1 * torque_limit_.negative_limit_Nm[i], torque_limit_.negative_limit_Nm[i]);
                    break;
                case RunMode::Drag:
                    robot_control_.position[i] = planner_cmd.cmd.position[i];
                    robot_control_.velocity[i] = planner_cmd.cmd.velocity[i];
                    robot_control_.torque_feedforward[i] = torque_cmd.torque_feedforward[i];
                    // 位置等于反馈位置
                    if (manager_buffers_.joint_measurement->PeekLatest(&manager_measurement))
                    {
                        if (i >= first_arm_joint_index_ && is_torque_limit_reached_[i])
                        {
                            manager_measurement.q_desired_pre[i] = robot_feedback_.position[i];
                        }
                        manager_buffers_.joint_measurement->AppendNewElement(manager_measurement);
                    }
                    // 更新关节力矩限制
                    if (torque_cmd.torque_feedforward[i] >= 0)
                    {
                        robot_control_.positive_torque_limit[i] = std::clamp(torque_cmd.torque_feedforward[i], 0.0, torque_limit_.positive_limit_Nm[i]);
                        robot_control_.negative_torque_limit[i] = 0.0;
                    }
                    else
                    {
                        robot_control_.positive_torque_limit[i] = 0.0;
                        robot_control_.negative_torque_limit[i] = std::clamp(-torque_cmd.torque_feedforward[i], 0.0, torque_limit_.negative_limit_Nm[i]);
                    }
                    break;
                default:
                    assert(false && "Unknown RunMode");
                    break;
                }
                // 滤波关节位置指令
                robot_control_.position[i] = butterworth_filter_[i]->Apply(robot_control_.position[i]);
                // 更新外部力矩
                robot_control_.torque_external[i] = torque_cmd.torque_external[i];
            }
        }
        else
        {
            for (int i = 0; i < ndof_robot; ++i)
            {
                this->butterworth_filter_[i]->Reset(); // 重置滤波器状态
            }
        }
        // 如果状态字发生变化了则打印
        static int16_t last_status_word_ = -1;
        static int16_t last_control_word_ = -1;
        if (robot_feedback_.robot_status_word != last_status_word_ || robot_control_.robot_control_word != last_control_word_)
        {
            RCLCPP_INFO(this->get_logger(), "Status word changed to: %s, control word changed to %s",
                        utils::RobotStatuswordToJsonString(robot_feedback_.robot_status_word).str().c_str(),
                        utils::RobotControlwordToJsonString(robot_control_.robot_control_word).str().c_str());
            LOG_INFO << "";
            LOG_INFO << "Status word changed to " << utils::RobotStatuswordToJsonString(robot_feedback_.robot_status_word).str()
                     << ", control word changed to " << utils::RobotControlwordToJsonString(robot_control_.robot_control_word).str()
                     << ", robot_server_state:" << GetRobotServerStateMsg().data.c_str();
            last_status_word_ = robot_feedback_.robot_status_word;
            last_control_word_ = robot_control_.robot_control_word;
        }
        control_shm_instance.AppendNewElement(robot_control_);
        std::this_thread::sleep_until(next_cycle_time_point);
    }
}

void RobotServer::SwitchControlWord()
{
    static bool inited = false;
    switch (robot_feedback_.robot_status_word)
    {
    case STATUSWORD_MOTOR_READY:
        // ready状态is_robot_ready_置为true, 可以写入measurement, 生成cmd后再使能电机
        this->is_robot_ready_ = true;
        inited = true;
        if (this->is_cmd_valid_)
        {
            this->robot_control_.robot_control_word |= CTRLWORD_MOTOR_ENABLE;
            if (this->enable_cst_mode_)
            {
                this->robot_control_.robot_control_word |= CTRLWORD_MOTOR_CST_MODE;
            }
        }
        else
        {
            for (int i = 0; i < ndof_robot; i++)
            {
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
        if (this->is_cmd_valid_)
        {
            if (this->enable_cst_mode_)
            {
                this->robot_control_.robot_control_word |= CTRLWORD_MOTOR_CST_MODE;
            }
        }
        else
        {
            for (int i = 0; i < ndof_robot; i++)
            {
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
        for (int i = 0; i < ndof_robot; i++)
        {
            this->robot_control_.position[i] = this->robot_feedback_.position[i];
            this->robot_control_.positive_torque_limit[i] = this->torque_limit_.positive_limit_Nm[i];
            this->robot_control_.negative_torque_limit[i] = this->torque_limit_.negative_limit_Nm[i];
        }
        break;
    }

    if (!inited || planner_info_.robot_property.clear_error_flag)
    {
        this->robot_control_.robot_control_word |= CTRLWORD_MOTOR_CLEAR_FAULT;
    }
}

void RobotServer::PublishThreadFunc()
{
    JointRawMeasurement raw_measurement;
    Eigen::VectorXd ee_pose;
    Eigen::VectorXd torso_X_Z_RY;
    PlannerManagerCommand planner_cmd{};
    auto next_cycle_time_point = std::chrono::steady_clock::now();
    while (rclcpp::ok())
    {
        next_cycle_time_point += std::chrono::microseconds(publish_dt_micro);
        // Publish /feedback/robot_server_state

        this->robot_server_state_publisher_->publish(GetRobotServerStateMsg());
        std::this_thread::sleep_until(next_cycle_time_point);
    }
}

std_msgs::msg::String RobotServer::GetRobotServerStateMsg()
{
    auto robot_server_state_msg = std_msgs::msg::String();
    uint64_t timestamp_ns = this->now().nanoseconds();
    robot_server_state_msg.data =
        std::string("{\n  \"timestamp\":") + std::to_string(timestamp_ns) +
        ", \"latest_queued_id\":" + std::to_string(this->latest_report_id_.queued) +
        ", \"latest_finished_id\":" + std::to_string(this->latest_report_id_.finished) +
        ", \"latest_errordrop_id\":" + std::to_string(this->latest_report_id_.error_drop) + ",\n  \"status\":\"" + ToString(this->manager_state_) +
        "\"" + ", \"server_error_code\":\"" + ToString(this->planner_info_.robot_property.error_code) + "\"" + ", \"run_mode\":\"" +
        ToString(this->planner_info_.robot_property.run_mode) + "\"" +
        ",\n  \"robot_statusword\":" + utils::RobotStatuswordToJsonString(robot_feedback_.robot_status_word).str() +
        ", \"robot_controlword\":" + utils::RobotControlwordToJsonString(robot_control_.robot_control_word).str() +
        ",\n  \"motor_status\":" + utils::MotorStatusToJsonArrayString(robot_feedback_.al_status, 0, ndof_robot).str() +
        ",\n  \"motor_error_code\":" + utils::MotorErrorcodeToJsonArrayString(robot_feedback_.motor_error_code, 0, ndof_robot).str() +
        ",\n  \"motor_temperature\":" + utils::VectorToJsonArrayString(robot_feedback_.motor_temperature, 0, ndof_robot).str() +
        ",\n  \"joint_position_cmd\":" + utils::VectorToJsonArrayString(robot_control_.position, 0, ndof_robot).str() +
        ",\n  \"joint_position\":" + utils::VectorToJsonArrayString(robot_feedback_.position, 0, ndof_robot).str() +
        ",\n  \"joint_velocity\":" + utils::VectorToJsonArrayString(robot_feedback_.velocity, 0, ndof_robot).str() +
        ",\n  \"joint_torque\":" + utils::VectorToJsonArrayString(robot_feedback_.torque, 0, ndof_robot).str() +
        ",\n  \"force_sensor\":" + utils::VectorToJsonArrayString(sensor_wrench_).str() +
        ",\n  \"joint_torque_feedforward\":" + utils::VectorToJsonArrayString(robot_control_.torque_feedforward, 6, ndof_robot).str() +
        ",\n  \"torque_limit_negative\":" + utils::VectorToJsonArrayString(robot_control_.negative_torque_limit, 6, ndof_robot).str() +
        ",\n  \"torque_limit_positive\":" + utils::VectorToJsonArrayString(robot_control_.positive_torque_limit, 6, ndof_robot).str() +
        ",\n  \"ee_pose\":" + utils::VectorToJsonArrayString(planner_info_.ee_pose_measurement).str() +
        ",\n  \"torso_pose_X_Z_RY\":" + utils::VectorToJsonArrayString(planner_info_.torso_pose_measurement).str() + "}";
    return robot_server_state_msg;
}

void RobotServer::PublishFeedback()
{
    // 发布关节状态反馈
    sensor_msgs::msg::JointState feedback_msg;
    feedback_msg.header.stamp = this->get_clock()->now();

    feedback_msg.name.resize(ndof_robot);
    feedback_msg.position.resize(ndof_robot);
    feedback_msg.velocity.resize(ndof_robot);
    feedback_msg.effort.resize(ndof_robot);

    for (int i = 0; i < ndof_robot; ++i)
    {
        feedback_msg.name[i] = ecat_joint_name_[i];
        feedback_msg.position[i] = robot_feedback_.position[i];
        feedback_msg.velocity[i] = robot_feedback_.velocity[i];
        feedback_msg.effort[i] = robot_feedback_.torque[i];
    }

    feedback_publisher_->publish(feedback_msg);

    // 发布关节速度和力矩限制反馈
    joint_interfaces::msg::JointVelocityTorqueLimitFeedback limit_msg;
    limit_msg.header.stamp = this->get_clock()->now();

    limit_msg.name.resize(ndof_robot);
    limit_msg.positive_velocity_limit.resize(ndof_robot);
    limit_msg.negative_velocity_limit.resize(ndof_robot);
    limit_msg.positive_torque_limit.resize(ndof_robot);
    limit_msg.negative_torque_limit.resize(ndof_robot);

    for (int i = 0; i < ndof_robot; ++i)
    {
        limit_msg.name[i] = ecat_joint_name_[i];
        limit_msg.positive_velocity_limit[i] = robot_control_.positive_velocity_limit[i];
        limit_msg.negative_velocity_limit[i] = robot_control_.negative_velocity_limit[i];
        limit_msg.positive_torque_limit[i] = robot_control_.positive_torque_limit[i];
        limit_msg.negative_torque_limit[i] = robot_control_.negative_torque_limit[i];
    }

    velocity_torque_limit_feedback_publisher_->publish(limit_msg);
}

void RobotServer::SetPropertyCallback(const std::shared_ptr<SetProperty::Request> request, std::shared_ptr<SetProperty::Response> response)
{
    PlannerRequest planner_request = utils::ParseSetPropertyRequest(request);
    planner_request.request_id = MakeNewRequestID(planner_request);
    this->manager_buffers_.request_queue->EnQueue(planner_request);
    response->request_id = planner_request.request_id;

    RCLCPP_INFO(this->get_logger(), "Request_id = %ld, %s Received, value: %s", planner_request.request_id, request->property_type.c_str(),
                request->value.c_str());
    LOG_INFO << "Request_id = " << planner_request.request_id << ", Received, type = " << request->property_type << ", value = " << request->value;
}

Int64 RobotServer::StartStreaming(PlannerRequest planner_request)
{
    auto request_id = MakeNewRequestID(planner_request);

    if (request_id < 0)
    {
        return request_id;
    }
    planner_request.request_id = request_id;
    this->manager_buffers_.request_queue->EnQueue(planner_request);

    RCLCPP_INFO(this->get_logger(), "Request_id = %ld, StartJointStreaming Received", planner_request.request_id);
    LOG_INFO << "Request_id = " << planner_request.request_id << ", Received, type = StartJointStreaming";
    return request_id;
}

void RobotServer::ControlCallback(const JointControlMsg::SharedPtr msg)
{
    auto robot_control_temp = robot_control_;
    bool position_valid = true;
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        for (int j = 0; j < ndof_robot; ++j)
        {
            if (ecat_joint_name_[j] == msg->name[i])
            {
                robot_control_.positive_velocity_limit[j] = msg->positive_velocity_limit[i];
                robot_control_.negative_velocity_limit[j] = msg->negative_velocity_limit[i];
                robot_control_.positive_torque_limit[j] = msg->positive_torque_limit[i];
                robot_control_.negative_torque_limit[j] = msg->negative_torque_limit[i];
                if (std::abs(robot_control_temp.position[j] - msg->position[i]) > position_diff_threshold_ && position_valid)
                {
                    RCLCPP_WARN(this->get_logger(), "Control command for joint %s rejected: position difference %.4f exceeds threshold %.4f",
                                ecat_joint_name_[j].c_str(), std::abs(robot_control_temp.position[j] - msg->position[i]), position_diff_threshold_);
                    LOG_CRIT << "Control command for joint " << ecat_joint_name_[j] << " rejected: position difference "
                             << std::abs(robot_control_temp.position[j] - msg->position[i]) << " exceeds threshold " << position_diff_threshold_;
                    position_valid = false;
                }
                robot_control_temp.position[j] = msg->position[i];

                break;
            }
        }
    }
    // 仅当所有位置命令均有效时才启动或发送流式命令
    if (position_valid)
    {
        if (manager_state_ == Status::Idle)
        {
            PlannerRequest planner_request;
            planner_request.type = RequestType::StartJointStreaming;
            MoveRequest move_request;
            move_request.motion_group = MotionGroup::WholeBody;
            move_request.optional_mode = enable_ruckig_ ? false : true;  // true: no ruckig 模式, false: ruckig 模式
            planner_request.data = move_request;
            streaming_request_id_ = StartStreaming(planner_request);
        }
        if (streaming_request_id_ > -1)
        {
            StreamingCommandMsg::SharedPtr streaming_msg = std::make_shared<StreamingCommandMsg>();
            streaming_msg->request_id = streaming_request_id_;
            streaming_msg->streaming_finished = false;
            streaming_msg->position = std::vector<float>(robot_control_temp.position, robot_control_temp.position + ndof_robot);
            this->StreamingCommandCallback(streaming_msg);
        }
    }
}

void RobotServer::PositionControlCallback(const JointPositionControlMsg::SharedPtr msg)
{
    auto robot_control_temp = robot_control_;
    bool position_valid = true;
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        for (int j = 0; j < ndof_robot; ++j)
        {
            if (ecat_joint_name_[j] == msg->name[i])
            {
                if (std::abs(robot_control_temp.position[j] - msg->position[i]) > position_diff_threshold_ && position_valid)
                {
                    RCLCPP_WARN(this->get_logger(), "Control command for joint %s rejected: position difference %.4f exceeds threshold %.4f",
                                ecat_joint_name_[j].c_str(), std::abs(robot_control_temp.position[j] - msg->position[i]), position_diff_threshold_);
                    LOG_CRIT << "Control command for joint " << ecat_joint_name_[j] << " rejected: position difference "
                             << std::abs(robot_control_temp.position[j] - msg->position[i]) << " exceeds threshold " << position_diff_threshold_;
                    position_valid = false;
                }
                robot_control_temp.position[j] = msg->position[i];
                break;
            }
        }
    }
    // 所有关节位置均有效才发送
    if (position_valid)
    {
        if (manager_state_ == Status::Idle)
        {
            PlannerRequest planner_request;
            planner_request.type = RequestType::StartJointStreaming;
            MoveRequest move_request;
            move_request.motion_group = MotionGroup::WholeBody;
            move_request.optional_mode = enable_ruckig_ ? false : true;  // true: no ruckig 模式, false: ruckig 模式
            planner_request.data = move_request;
            streaming_request_id_ = StartStreaming(planner_request);
        }
        if (streaming_request_id_ != -1)
        {
            StreamingCommandMsg::SharedPtr streaming_msg = std::make_shared<StreamingCommandMsg>();
            streaming_msg->request_id = streaming_request_id_;
            streaming_msg->streaming_finished = false;
            streaming_msg->position = std::vector<float>(robot_control_temp.position, robot_control_temp.position + ndof_robot);
            this->StreamingCommandCallback(streaming_msg);
        }
    }
}

void RobotServer::OmniChassisControlCallback(const ChassisWholeControlMsg::SharedPtr msg)
{
    robot_control_.chassis_mode = 0;  // 设置为全向整车模式
    robot_control_.chassis_whole_mode = msg->mode;
    robot_control_.chassis_velocity = msg->speed;
    robot_control_.chassis_direction = msg->direction;
    robot_control_.chassis_cnt++;
    RCLCPP_DEBUG(this->get_logger(), "Omni chassis control: mode=%ld, speed=%.2f, direction=%.2f", msg->mode, msg->speed, msg->direction);
}

// 全向底盘电机控制回调
void RobotServer::OmniChassisMotorControlCallback(const ChassisMotorControlMsg::SharedPtr msg)
{
    robot_control_.chassis_mode = 2;  // 设置为电机控制模式
    for (size_t i = 0; i < msg->speed.size() && i < OMNI_MOTOR_NUM; i++)
    {
        robot_control_.chassis_motor_velocity[i] = msg->speed[i];
        robot_control_.chassis_motor_direction[i] = msg->position[i];
    }
    robot_control_.chassis_cnt++;
    robot_control_.chassis_estop = msg->estop;
    robot_control_.chassis_neutral = msg->neutral;
    RCLCPP_DEBUG(this->get_logger(), "Omni chassis motor control: estop=%ld, neutral=%ld", msg->estop, msg->neutral);
}

void RobotServer::DiffChassisControlCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    robot_control_.chassis_mode = 1;  // 设置为差速模式
    robot_control_.chassis_right_velocity = msg->linear.x + msg->angular.z * DiffChassisWidth * 0.5;
    robot_control_.chassis_left_velocity = msg->linear.x - msg->angular.z * DiffChassisWidth * 0.5;
    robot_control_.chassis_cnt++;
    RCLCPP_DEBUG(this->get_logger(), "Diff chassis control: linear.x=%.2f, angular.z=%.2f => left=%.2f, right=%.2f", msg->linear.x, msg->angular.z,
                 robot_control_.chassis_left_velocity, robot_control_.chassis_right_velocity);
}

}  // namespace robot_server_lib
