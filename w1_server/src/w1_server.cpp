#include "w1_server.h"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cstdlib>
#include <filesystem>
#include <system_error>

namespace robot_server_lib
{

RobotServer::RobotServer(std::filesystem::path robot_cfg_path, bool is_sim) : RobotServerBase(robot_cfg_path, is_sim)
{
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

    // 创建定时器，定时发布feedback 定时器回调函数
    control_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() { PublishFeedback(); });

    // 初始化力传感器
    // primary_sensor_ = std::make_shared<ForceSensorReader>("/dev/ttyUSB0");
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

    using boost::property_tree::ptree;
    constexpr std::size_t kNumJoints = ndof_robot;

    auto array_to_pt = [](const double* a, std::size_t n)
    {
        ptree arr;
        for (std::size_t i = 0; i < n; ++i)
        {
            ptree v;
            v.put("", a[i]);
            arr.push_back(std::make_pair("", v));
        }
        return arr;
    };

    // 配置文件不存在则创建文件并写入默认配置
    if (!std::filesystem::exists(config_path))
    {
        ptree root;
        ptree torque_node;
        torque_node.add_child("positive_limit_Nm", array_to_pt(torque_limit_.positive_limit_Nm, kNumJoints));
        torque_node.add_child("negative_limit_Nm", array_to_pt(torque_limit_.negative_limit_Nm, kNumJoints));
        root.add_child("torque_limit", torque_node);

        boost::property_tree::write_json(config_path.string(), root);
        RCLCPP_INFO(this->get_logger(), "Created default config.json at: %s", config_path.string().c_str());
        LOG_INFO << "Created default config.json at: " << config_path;
        return;
    }

    // 配置文件存在则读取；缺少 torque_limit 时写入默认
    ptree root;
    boost::property_tree::read_json(config_path.string(), root);

    auto torque_opt = root.get_child_optional("torque_limit");
    if (!torque_opt)
    {
        RCLCPP_WARN(this->get_logger(), "config.json has no 'torque_limit' section, writing defaults.");
        LOG_CRIT << "config.json has no 'torque_limit' section, writing defaults.";

        ptree torque_node;
        torque_node.add_child("positive_limit_Nm", array_to_pt(torque_limit_.positive_limit_Nm, kNumJoints));
        torque_node.add_child("negative_limit_Nm", array_to_pt(torque_limit_.negative_limit_Nm, kNumJoints));
        root.put_child("torque_limit", torque_node);
        boost::property_tree::write_json(config_path.string(), root);

        torque_opt = root.get_child_optional("torque_limit");
        if (!torque_opt)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get 'torque_limit' after writing defaults.");
            LOG_CRIT << "Failed to get 'torque_limit' after writing defaults.";
            return;
        }
    }

    const ptree& torque_node = *torque_opt;
    auto read_array = [&](const char* key, double out[kNumJoints]) -> bool
    {
        auto child_opt = torque_node.get_child_optional(key);
        if (!child_opt)
            return false;
        const ptree& arr = *child_opt;
        if (static_cast<std::size_t>(std::distance(arr.begin(), arr.end())) != kNumJoints)
            return false;
        std::size_t idx = 0;
        for (const auto& kv : arr)
        {
            out[idx++] = kv.second.get_value<double>();
        }
        return true;
    };

    read_array("positive_limit_Nm", torque_limit_.positive_limit_Nm);
    read_array("negative_limit_Nm", torque_limit_.negative_limit_Nm);

    RCLCPP_INFO(this->get_logger(), "Loaded config.json from: %s", config_path.string().c_str());
    LOG_INFO << "Loaded config.json from: " << config_path;
}

void RobotServer::TorqueControlThreadFunc()
{
    TorqueControllerManager torque_controller_manager(this->robot_model_, this->manager_buffers_, this->robot_buffers_);
    // 初始化共享内存
    auto& feedback_shm_instance = FeedbackSharedMemory::GetInstance();
    auto& control_shm_instance = ControlSharedMemory::GetInstance();

    Int64 loop_counter = 0;
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

        // 读取机器人反馈数据
        bool ret = feedback_shm_instance.PeekLatest(&robot_feedback_);
        bool motor_ready = robot_feedback_.robot_status_word & STATUSWORD_MOTOR_READY;
        bool motor_enabled = robot_feedback_.robot_status_word & STATUSWORD_MOTOR_ENABLED;
        this->is_robot_ready_ = ret && motor_ready;
        robot_control_.robot_control_word = CTRLWORD_MOTOR_SWITCH_ON | CTRLWORD_MOTOR_CLEAR_FAULT;
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
            if (loop_counter % 3000 == 0)
            {
                RCLCPP_WARN(this->get_logger(), "Robot is not ready, shm status: %d, statusword: %d, please check Connection, OP or Error state.", ret, robot_feedback_.robot_status_word);
                LOG_CRIT << "Robot is not ready, shm status: " << ret << ", statusword: " << robot_feedback_.robot_status_word
                         << ", please check Connection, OP or Error state.";
            }
        }

        // 力控计算处理
        torque_controller_manager.LoopOnce();

        // 获取并写入控制数据: 关节位置, 速度, 关节力矩限制, 外部力矩
        PlannerManagerCommand planner_cmd{};
        TorqueCtrlCommand torque_cmd{};
        if (this->manager_buffers_.planner_manager_command->PeekLatest(&planner_cmd) &&
            this->robot_buffers_.torque_controller_command->PeekLatest(&torque_cmd))
        {
            JointMeasurement manager_measurement{};
            robot_property_ = planner_cmd.robot_property;
            for (int i = 0; i < ndof_robot; ++i)
            {
                switch (robot_property_.run_mode)
                {
                case RunMode::Position:
                case RunMode::Impedance:
                    // 更新关节力矩限制
                    robot_control_.position[i] = planner_cmd.cmd.compliance_position[i];
                    robot_control_.velocity[i] = planner_cmd.cmd.velocity[i];
                    robot_control_.torque_feedforward[i] = torque_cmd.torque[i];
                    robot_control_.positive_torque_limit[i] =
                        std::clamp(torque_cmd.torque[i] + torque_limit_.positive_limit_Nm[i] * robot_property_.safe_level,
                                   0.1 * torque_limit_.positive_limit_Nm[i], torque_limit_.positive_limit_Nm[i]);
                    robot_control_.negative_torque_limit[i] =
                        std::clamp(-torque_cmd.torque[i] + torque_limit_.negative_limit_Nm[i] * robot_property_.safe_level,
                                   0.1 * torque_limit_.negative_limit_Nm[i], torque_limit_.negative_limit_Nm[i]);
                    break;
                case RunMode::Drag:
                    robot_control_.position[i] = planner_cmd.cmd.position[i];
                    robot_control_.velocity[i] = planner_cmd.cmd.velocity[i];
                    robot_control_.torque_feedforward[i] = torque_cmd.torque[i];
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
                    if (torque_cmd.torque[i] >= 0)
                    {
                        robot_control_.positive_torque_limit[i] = std::clamp(torque_cmd.torque[i], 0.0, torque_limit_.positive_limit_Nm[i]);
                        robot_control_.negative_torque_limit[i] = 0.0;
                    }
                    else
                    {
                        robot_control_.positive_torque_limit[i] = 0.0;
                        robot_control_.negative_torque_limit[i] = std::clamp(-torque_cmd.torque[i], 0.0, torque_limit_.negative_limit_Nm[i]);
                    }
                    break;
                default:
                    assert(false && "Unknown RunMode");
                    break;
                }

                // 更新外部力矩
                robot_control_.torque_external[i] = torque_cmd.torque_external[i];
            }
            // if (loop_counter % 3000 == 0) // Debug output
            // {
            //     std::cout << "torque_cmd.torque[6..12]: " << torque_cmd.torque.segment(6, 7).transpose() << std::endl;
            // }
            // robot_control_.robot_control_word = CTRLWORD_MOTOR_SWITCH_ON | CTRLWORD_MOTOR_ENABLE | CTRLWORD_MOTOR_CLEAR_FAULT |
            // CTRLWORD_MOTOR_CST_MODE;
            robot_control_.robot_control_word = CTRLWORD_MOTOR_SWITCH_ON | CTRLWORD_MOTOR_ENABLE | CTRLWORD_MOTOR_CLEAR_FAULT;
        }

        control_shm_instance.AppendNewElement(robot_control_);
        std::this_thread::sleep_until(next_cycle_time_point);
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
        uint64_t timestamp_ns = this->now().nanoseconds();

        // Publish /feedback/robot_server_state
        std_msgs::msg::String robot_server_state_msg;
        std::ostringstream robot_statusword_ss = utils::RobotStatuswordToJsonString(robot_feedback_.robot_status_word);
        std::ostringstream joint_position_ss = utils::VectorToJsonArrayString(robot_feedback_.position, 0, ndof_robot);
        std::ostringstream joint_velocity_ss = utils::VectorToJsonArrayString(robot_feedback_.velocity, 0, ndof_robot);
        std::ostringstream joint_torque_ss = utils::VectorToJsonArrayString(robot_feedback_.torque, 0, ndof_robot);
        std::ostringstream joint_sensor_wrench_ss = utils::VectorToJsonArrayString(sensor_wrench_);
        std::ostringstream motor_status_ss = utils::MotorStatusToJsonArrayString(robot_feedback_.al_status, 0, ndof_robot);
        std::ostringstream motor_temperature_ss = utils::VectorToJsonArrayString(robot_feedback_.motor_temperature, 0, ndof_robot);
        std::ostringstream motor_error_code_ss = utils::MotorErrorcodeToJsonArrayString(robot_feedback_.motor_error_code, 0, ndof_robot);
        std::ostringstream torque_limit_negative_ss = utils::VectorToJsonArrayString(robot_control_.negative_torque_limit, 6, ndof_robot);
        std::ostringstream torque_limit_positive_ss = utils::VectorToJsonArrayString(robot_control_.positive_torque_limit, 6, ndof_robot);

        robot_server_state_msg.data = std::string("{\"timestamp\":") + std::to_string(timestamp_ns) + ", \"status\":\"" +
                                      ToString(this->manager_state_) + "\", \"robot_statusword\":" + robot_statusword_ss.str() + ", \"run_mode\":\"" + ToString(robot_property_.run_mode) + "\"" +
                                      ",\"latest_queued_id\":" + std::to_string(this->latest_report_id_.queued) +
                                      ",\"latest_finished_id\":" + std::to_string(this->latest_report_id_.finished) +
                                      ",\"latest_errordrop_id\":" + std::to_string(this->latest_report_id_.error_drop) +
                                      ",\"motor_status\":" + motor_status_ss.str() + ",\"motor_temperature\":" + motor_temperature_ss.str() +
                                      ",\"motor_error_code\":" + motor_error_code_ss.str() + ",\"joint_position\":" + joint_position_ss.str() +
                                      ",\"joint_velocity\":" + joint_velocity_ss.str() + ",\"joint_torque\":" + joint_torque_ss.str() +
                                      ",\"force_sensor\":" + joint_sensor_wrench_ss.str() + ",\"torque_limit_negative\":" + torque_limit_negative_ss.str() +
                                      ",\"torque_limit_positive\":" + torque_limit_positive_ss.str();

        if (this->manager_buffers_.ee_pose_measurement->PeekLatest(&ee_pose) &&
            this->manager_buffers_.torso_pose_measurement->PeekLatest(&torso_X_Z_RY))
        {
            std::ostringstream ee_pose_ss = utils::VectorToJsonArrayString(ee_pose);
            std::ostringstream torso_X_Z_RY_ss = utils::VectorToJsonArrayString(torso_X_Z_RY);
            robot_server_state_msg.data += ",\"ee_pose\":" + ee_pose_ss.str() + ",\"torso_pose_X_Z_RY\":" + torso_X_Z_RY_ss.str() + "}";
        }
        else
        {
            robot_server_state_msg.data += ",\"ee_pose\":\"no valid data\", \"torso_pose_X_Z_RY\":\"no valid data\"}";
        }
        this->robot_server_state_publisher_->publish(robot_server_state_msg);
        std::this_thread::sleep_until(next_cycle_time_point);
    }
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

    JointRawMeasurement raw_measurement{};
    if (this->robot_buffers_.joint_raw_measurement->PeekLatest(&raw_measurement))
    {
        for (int i = 0; i < ndof_robot; ++i)
        {
            feedback_msg.name[i] = ecat_joint_name_[i];
            feedback_msg.position[i] = raw_measurement.position[i];
            feedback_msg.velocity[i] = raw_measurement.velocity[i];
            feedback_msg.effort[i] = raw_measurement.effort[i];
        }

        feedback_publisher_->publish(feedback_msg);
    }

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

Int64 RobotServer::StartStreaming(PlannerRequest planner_request)
{
    auto request_id = MakeNewRequestID(planner_request.type);

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
            move_request.optional_mode = true;  // 使用 no ruckig 模式
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
            move_request.optional_mode = true;  // 使用 no ruckig 模式
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
    robot_control_.chassis_estop = msg->estop;
    robot_control_.chassis_neutral = msg->neutral;
    RCLCPP_DEBUG(this->get_logger(), "Omni chassis motor control: estop=%ld, neutral=%ld", msg->estop, msg->neutral);
}

void RobotServer::DiffChassisControlCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    robot_control_.chassis_mode = 1;  // 设置为差速模式
    robot_control_.chassis_right_velocity = msg->linear.x + msg->angular.z * DiffChassisWidth * 0.5;
    robot_control_.chassis_left_velocity = msg->linear.x - msg->angular.z * DiffChassisWidth * 0.5;
    RCLCPP_DEBUG(this->get_logger(), "Diff chassis control: linear.x=%.2f, angular.z=%.2f => left=%.2f, right=%.2f", msg->linear.x, msg->angular.z,
                 robot_control_.chassis_left_velocity, robot_control_.chassis_right_velocity);
}

}  // namespace robot_server_lib
