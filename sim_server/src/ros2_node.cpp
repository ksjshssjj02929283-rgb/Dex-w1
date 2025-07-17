#include "ros2_node.hpp"

using namespace robot_server_lib;

Ros2Node::Ros2Node() : Node("sim_server")
{
    InitRobotServer();
    InitSimulation();
    RCLCPP_INFO(this->get_logger(), "Initialized successfully.");
    LOG_INFO << "Initialized successfully.";
}

void Ros2Node::InitRobotServer()
{
    nanolog::initialize(nanolog::GuaranteedLogger(), "log/", "robot_server", 10);
    nanolog::set_log_level(
        nanolog::LogLevel::DEBUG);  // options: DEBUG, INFO, WARN, CRIT 高频过程数据打印设置为DEBUG

    // 创建 /feedback/robot_server_state 发布器
    robot_server_state_publisher_ =
        this->create_publisher<std_msgs::msg::String>("/feedback/robot_server_state", 10);

    // 创建 /control/streaming_command 订阅器
    streaming_command_subscriber_ =
        this->create_subscription<robot_server_interfaces::msg::StreamingCommand>(
            "/control/streaming_command", 10,
            std::bind(&Ros2Node::OnStreamingCommand, this, std::placeholders::_1));

    // 创建 /control/set_property 服务
    set_property_srv_ = this->create_service<robot_server_interfaces::srv::SetProperty>(
        "/control/set_property", std::bind(&Ros2Node::SetPropertyOnRequest, this,
                                           std::placeholders::_1, std::placeholders::_2));

    // 创建 /control/move_robot action server
    move_robot_action_server_ =
        rclcpp_action::create_server<robot_server_interfaces::action::MoveRobot>(
            this, "/control/move_robot",
            std::bind(&Ros2Node::HandleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&Ros2Node::HandleCancel, this, std::placeholders::_1),
            std::bind(&Ros2Node::HandleAccepted, this, std::placeholders::_1));

    const auto package_share_path = ament_index_cpp::get_package_share_directory("sim_server");
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot.cfg";
    json config_json;
    if (!std::filesystem::exists(robot_cfg_path))
    {
        LOG_CRIT << "Robot configuration file not found: " << robot_cfg_path;
        assert(false && "Robot configuration file not found");
    }
    std::ifstream ifs(robot_cfg_path);
    if (!ifs.is_open() || !(ifs >> config_json))
    {
        LOG_CRIT << "Can't open or parse robot.cfg: " << robot_cfg_path;
        assert(false && "Can't open or parse robot.cfg");
    }
    // 初始化机器人模型
    RobotParams params;
    if (!utilities::GetRobotParamsFromConfig(config_json, params))
    {
        LOG_CRIT << "Failed to load robot.cfg";
        assert(false);
    }
    pinocchio::Model model;
    pinocchio::urdf::buildModel(params.urdf_path, model);
    std::vector<std::string> joints_to_lock = {
        "L_indexMCP",  "L_indexPIP", "L_littleMCP", "L_littlePIP", "L_middleMCP",
        "L_middlePIP", "L_ringMCP",  "L_ringPIP",   "L_thumbMCP1", "L_thumbMCP2",
        "R_indexMCP",  "R_indexPIP", "R_littleMCP", "R_littlePIP", "R_middleMCP",
        "R_middlePIP", "R_ringMCP",  "R_ringPIP",   "R_thumbMCP1", "R_thumbMCP2"};

    std::vector<pinocchio::JointIndex> locked_joints;
    for (const auto& name : joints_to_lock)
    {
        locked_joints.push_back(model.getJointId(name));
    }
    Eigen::VectorXd reference_configuration = Eigen::VectorXd::Zero(model.nq);

    pinocchio::Model reduced_model =
        pinocchio::buildReducedModel(model, locked_joints, reference_configuration);

    // 建立左臂和右臂模型 Create left and right arm models.
    pinocchio::Model left_arm_model;
    pinocchio::Model right_arm_model;
    pinocchio::urdf::buildModel(params.left_arm_urdf_path, left_arm_model);
    pinocchio::urdf::buildModel(params.right_arm_urdf_path, right_arm_model);

    robot_model_.model_ptr = std::make_shared<pinocchio::Model>(reduced_model);
    robot_model_.left_arm_model_ptr = std::make_shared<pinocchio::Model>(left_arm_model);
    robot_model_.right_arm_model_ptr = std::make_shared<pinocchio::Model>(right_arm_model);
    robot_model_.params_ptr = std::make_shared<RobotParams>(params);

    // 初始化全局共享buffer
    manager_buffers_.Initialize();
    robot_buffers_.Initialize();

    manager_thread_ = std::thread(
        [this]() -> void
        {
            auto report_request_status = [this](Int64 request_id, RequestType request_type,
                                                RequestStatus request_status, Status manager_state)
            {
                manager_state_ = manager_state;

                // Log the manager and request status
                if (request_id < 0)
                {
                    RCLCPP_INFO(this->get_logger(), "manager state changed to %s",
                                ToString(manager_state_));
                    LOG_INFO << "Manager state changed to: " << ToString(manager_state_);
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Request id = %ld, %s %s", request_id,
                            ToString(request_type), ToString(request_status));
                LOG_INFO << "Request id = " << request_id << " " << ToString(request_status)
                         << ", type = " << ToString(request_type);
                // Update the latest report id based on the request status
                switch (request_status)
                {
                case RequestStatus::Queued:
                    utilities::SetMax(latest_report_id_.queued, request_id);
                    break;
                case RequestStatus::Executed:
                    utilities::SetMax(latest_report_id_.finished, request_id);
                    if (request_type >= RequestType::MoveJoint)
                    {
                        std::shared_ptr<rclcpp_action::ServerGoalHandle<
                            robot_server_interfaces::action::MoveRobot>>
                            handle_temp;
                        if (request_handle_queue_.DeQueue(&handle_temp))
                        {
                            auto result_msg = std::make_shared<
                                robot_server_interfaces::action::MoveRobot::Result>();
                            result_msg->request_id = request_id;
                            result_msg->success = true;
                            handle_temp->succeed(result_msg);
                        }
                    }
                    break;
                case RequestStatus::ErrorDrop:
                    utilities::SetMax(latest_report_id_.error_drop, request_id);
                    if (request_type >= RequestType::MoveJoint)
                    {
                        std::shared_ptr<rclcpp_action::ServerGoalHandle<
                            robot_server_interfaces::action::MoveRobot>>
                            handle_temp;
                        if (request_handle_queue_.DeQueue(&handle_temp))
                        {
                            auto result_msg = std::make_shared<
                                robot_server_interfaces::action::MoveRobot::Result>();
                            result_msg->request_id = request_id;
                            result_msg->success = false;
                            handle_temp->succeed(result_msg);
                        }
                    }
                    break;
                default:
                    break;
                }
            };

            PlannerManager planner_manager(this->robot_model_, this->manager_buffers_,
                                           report_request_status);

            while (rclcpp::ok())
            {
                auto start_time = std::chrono::high_resolution_clock::now();

                planner_manager.LoopOnce();

                auto end_time = std::chrono::high_resolution_clock::now();
                auto elapsed_time =
                    std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time)
                        .count();

                if (elapsed_time < manager_dt_micro)
                {
                    std::this_thread::sleep_for(
                        std::chrono::microseconds(manager_dt_micro - elapsed_time));
                }
            }
        });

    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),  // 10Hz
        [this]()
        {
            std_msgs::msg::String robot_server_state_msg;
            JointMeasurement measurement;
            std::ostringstream joint_position_ss;
            Eigen::VectorXd ee_pose;
            std::ostringstream ee_pose_ss;
            if (manager_buffers_.joint_measurement->PeekLatest(&measurement))
            {
                joint_position_ss << "[";
                for (int i = 0; i < measurement.position.size(); ++i)
                {
                    joint_position_ss << measurement.position[i];
                    if (i != measurement.position.size() - 1)
                        joint_position_ss << ",";
                }
                joint_position_ss << "]";
            }
            if (manager_buffers_.ee_pose_measurement->PeekLatest(&ee_pose))
            {
                ee_pose_ss << "[";
                for (int i = 0; i < ee_pose.size(); ++i)
                {
                    ee_pose_ss << ee_pose[i];
                    if (i != ee_pose.size() - 1)
                        ee_pose_ss << ",";
                }
                ee_pose_ss << "]";
            }
            robot_server_state_msg.data =
                std::string("{\"status\":\"") + ToString(manager_state_) + "\"" +
                ",\"latest_queued_id\":" + std::to_string(latest_report_id_.queued) +
                ",\"latest_finished_id\":" + std::to_string(latest_report_id_.finished) +
                ",\"latest_errordrop_id\":" + std::to_string(latest_report_id_.error_drop) +
                ",\"joint_position\":" + joint_position_ss.str() +
                ",\"ee_pose\":" + ee_pose_ss.str() + "}";
            robot_server_state_publisher_->publish(robot_server_state_msg);
        });
}

void Ros2Node::InitSimulation()
{
// 创建仿真线程, 模拟执行torque_cmd生成joint raw measurement
#if TORQUECONTROL
    std::thread simulation_thread(
        [this]()
        {
            pinocchio::Data robot_data(*robot_model_.model_ptr);
            JointRawMeasurement cache{};
            // 初始更新关节测量数据
            Eigen::VectorXd q = Eigen::VectorXd::Zero(robot_model_.model_ptr->nv);
            Eigen::VectorXd dq = Eigen::VectorXd::Zero(robot_model_.model_ptr->nv);
            Eigen::VectorXd ddq = Eigen::VectorXd::Zero(robot_model_.model_ptr->nv);
            Eigen::VectorXd torque_cmd = Eigen::VectorXd::Zero(robot_model_.model_ptr->nv);
            cache.position = q;
            cache.velocity = dq;
            cache.effort = torque_cmd;
            cache.time_point = TimePointSystem::Now();
            auto ret = robot_buffers_.joint_raw_measurement->AppendNewElement(cache);
            VectorDof temp_torque{};

            for (int i = 0; i < 25000 && rclcpp::ok(); ++i)
            {
                if (robot_buffers_.torque_command_vector &&
                    robot_buffers_.torque_command_vector->PeekLatest(&temp_torque))
                {
                    torque_cmd = temp_torque;
                }
                else
                {
                    torque_cmd.setZero();
                }

                ddq = pinocchio::aba(*robot_model_.model_ptr, robot_data, q, dq, torque_cmd);

                q += simulation_dt * dq;
                dq += simulation_dt * ddq;
                // 更新关节测量数据
                cache.position = q;
                cache.velocity = dq;
                cache.effort = torque_cmd;
                cache.time_point = TimePointSystem::Now();
                auto ret = robot_buffers_.joint_raw_measurement->AppendNewElement(cache);
                assert(ret);
                std::this_thread::sleep_for(std::chrono::microseconds(simulation_dt_micro));
            }
        });

    // Init torque control thread
    torque_control_thread_ = std::thread(
        [this]() -> void
        {
            TorqueControllerManager torque_controller_manager(
                this->robot_model_, this->manager_buffers_, this->robot_buffers_);
            while (rclcpp::ok())
            {
                torque_controller_manager.LoopOnce();
                std::this_thread::sleep_for(std::chrono::microseconds(torque_ctrl_dt_micro));
            }
        });
#else
    simulation_thread_ = std::thread(
        [this]() -> void
        {
            PlannerManagerCommand q_cmd{};
            JointMeasurement cache{};
            while (rclcpp::ok())
            {
                if (!manager_buffers_.planner_manager_command->PeekLatest(&q_cmd))
                {
                    q_cmd.cmd.position << 0.5, -1.0, 0.5, 0.0, 0.5, -1.2, 0.0, -1.0, 0.0, -0.5,
                        -0.5, -0.5, 1.2, 0.0, 1.0, 0.0, 0.5, 0.5, 0.0, 0.0;
                    q_cmd.cmd.velocity.setZero();
                    cache.q_desired_pre = q_cmd.cmd.position;
                }
                // 更新关节测量数据
                cache.position = q_cmd.cmd.position;
                cache.velocity = q_cmd.cmd.velocity;
                auto ret = manager_buffers_.joint_measurement->AppendNewElement(cache);
                cache.q_desired_pre = cache.position;
                assert(ret);
                std::this_thread::sleep_for(std::chrono::microseconds(simulation_dt_micro));
            }
        });
#endif
}

rclcpp_action::GoalResponse Ros2Node::HandleGoal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const robot_server_interfaces::action::MoveRobot::Goal> goal)
{
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void Ros2Node::HandleAccepted(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<robot_server_interfaces::action::MoveRobot>>
        request_handle)
{
    PlannerRequest planner_request = ParseMoveRequest(request_handle);
    auto request_id = MakeNewRequestID(planner_request.type);

    // Publish feedback with request id
    auto feedback = std::make_shared<robot_server_interfaces::action::MoveRobot::Feedback>();
    feedback->request_id = request_id;
    request_handle->publish_feedback(feedback);

    if (request_id < 0)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Still existed previous move request, failed to queue start streaming request");
        LOG_CRIT << "Still existed previous move request, failed to queue start streaming request";
        return;
    }
    planner_request.request_id = request_id;
    manager_buffers_.request_queue->EnQueue(planner_request);
    request_handle_queue_.EnQueue(request_handle);

    RCLCPP_INFO(this->get_logger(), "Request_id = %ld, %s Received, arm = %s",
                planner_request.request_id, request_handle->get_goal()->type.c_str(),
                request_handle->get_goal()->motion_group.c_str());

    LOG_INFO << "Request_id = " << planner_request.request_id
             << ", Received, type = " << request_handle->get_goal()->type
             << ", arm = " << request_handle->get_goal()->motion_group;
}

rclcpp_action::CancelResponse Ros2Node::HandleCancel(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<robot_server_interfaces::action::MoveRobot>>
        goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "MoveRobot request canceled");
    LOG_INFO << "MoveRobot request canceled";
    return rclcpp_action::CancelResponse::ACCEPT;
}

Int64 Ros2Node::MakeNewRequestID(RequestType type)
{
    if (type == RequestType::StartJointStreaming || type == RequestType::StartEEPoseStreaming)
    {
        if (latest_received_id_ > latest_report_id_.finished)
        {
            return -1;
        }
    }
    return ++latest_received_id_;
}

void Ros2Node::OnStreamingCommand(
    const robot_server_interfaces::msg::StreamingCommand::SharedPtr streaming_msg)
{
    RCLCPP_INFO(
        this->get_logger(),
        "Request_id = %ld, streaming command Received, data size = %zu, streaming_finished = %s",
        streaming_msg->request_id, streaming_msg->position.size(),
        streaming_msg->streaming_finished ? "true" : "false");
    std::ostringstream ss;
    for (const auto& v : streaming_msg->position)
    {
        ss << v << " ";
    }
    LOG_INFO << "Request_id = " << streaming_msg->request_id << " Streaming, "
             << "data size = " << streaming_msg->position.size() << ", "
             << "streaming_finished = "
             << (streaming_msg->streaming_finished ? "true, " : "false, ") << "data = " << ss.str();

    manager_buffers_.streaming_command_latest_buffer->AppendNewElement(
        ParseStreamingCommand(streaming_msg));
}

void Ros2Node::SetPropertyOnRequest(
    const std::shared_ptr<robot_server_interfaces::srv::SetProperty::Request> request,
    std::shared_ptr<robot_server_interfaces::srv::SetProperty::Response> response)
{
    PlannerRequest planner_request = ParseSetPropertyRequest(request);
    planner_request.request_id = MakeNewRequestID(planner_request.type);
    manager_buffers_.request_queue->EnQueue(planner_request);
    response->request_id = planner_request.request_id;

    RCLCPP_INFO(this->get_logger(), "Request_id = %ld, %s Received, value: %s",
                planner_request.request_id, request->property_type.c_str(), request->value.c_str());
    LOG_INFO << "Request_id = " << planner_request.request_id
             << ", Received, type = " << request->property_type << ", value = " << request->value;
}

PlannerRequest Ros2Node::ParseSetPropertyRequest(
    const std::shared_ptr<robot_server_interfaces::srv::SetProperty::Request> request)
{
    PlannerRequest planner_request;

    if (request->property_type == "TcpPrimary")
    {
        planner_request.type = RequestType::TcpPrimary;
        planner_request.data = utilities::StringToPoseQuaternion(request->value);
    }
    else if (request->property_type == "TcpSecondary")
    {
        planner_request.type = RequestType::TcpSecondary;
        planner_request.data = utilities::StringToPoseQuaternion(request->value);
    }
    else if (request->property_type == "SpeedRatio")
    {
        planner_request.type = RequestType::SpeedRatio;
        double speed = std::clamp(std::stod(request->value), 0.0, 1.0);
        planner_request.data = speed;
    }
    else if (request->property_type == "ClearError")
    {
        planner_request.type = RequestType::ClearError;
        planner_request.data =
            std::nullopt;  // Clear error request does not require additional data
    }
    else
    {
        planner_request.type = RequestType::Empty;
        LOG_CRIT << "Parse property: type not defined: " << request->property_type;
    }
    return planner_request;
}

StreamingCommand Ros2Node::ParseStreamingCommand(
    const std::shared_ptr<robot_server_interfaces::msg::StreamingCommand> msg)
{
    StreamingCommand steaming_command;

    steaming_command.request_id = msg->request_id;
    steaming_command.streaming_finished = msg->streaming_finished;
    if (msg->position.size() > 0)
    {
        steaming_command.command.resize(msg->position.size());
        for (size_t i = 0; i < msg->position.size(); ++i)
        {
            steaming_command.command[i] = msg->position[i];
        }
    }
    return steaming_command;
}

// 解析MoveRobot动作的命令
PlannerRequest Ros2Node::ParseMoveRequest(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<robot_server_interfaces::action::MoveRobot>>
        action_handle)
{
    const auto& input_request = action_handle->get_goal();

    PlannerRequest planner_request;
    // type/motion_group 转换为枚举
    if (input_request->type == "MoveJoint")
    {
        planner_request.type = RequestType::MoveJoint;
    }
    else if (input_request->type == "MoveLine")
    {
        planner_request.type = RequestType::MoveLine;
    }
    else if (input_request->type == "StartJointStreaming")
    {
        planner_request.type = RequestType::StartJointStreaming;
    }
    else if (input_request->type == "StartEEPoseStreaming")
    {
        planner_request.type = RequestType::StartEEPoseStreaming;
    }
    else
    {
        planner_request.type = RequestType::Unknown;
        return planner_request;
    }

    MoveRequest move_request;
    if (input_request->motion_group == "Primary")
    {
        move_request.motion_group = MotionGroup::Primary;
    }
    else if (input_request->motion_group == "Secondary")
    {
        move_request.motion_group = MotionGroup::Secondary;
    }
    else if (input_request->motion_group == "BothArms")
    {
        move_request.motion_group = MotionGroup::BothArms;
    }
    else if (input_request->motion_group == "WholeBody")
    {
        move_request.motion_group = MotionGroup::WholeBody;
    }
    else
    {
        move_request.motion_group = MotionGroup::Unknown;
    }

    Eigen::VectorXd value;
    if (input_request->target.size() > 0)
    {
        value.resize(input_request->target.size());
        for (size_t i = 0; i < input_request->target.size(); ++i)
        {
            value[i] = input_request->target[i];
        }
        move_request.value = value;
    }

    json input_config_json;
    if (!input_request->config.empty())
    {
        input_config_json = json::parse(input_request->config);

        if (input_config_json.contains("reference_frame"))
        {
            if (input_request->motion_group == "ArmBase")
            {
                move_request.reference_frame.emplace(ReferenceFrame::ArmBase);
            }
            else if (input_request->motion_group == "WorldBase")
            {
                move_request.reference_frame.emplace(ReferenceFrame::WorldBase);
            }
            else
            {
                move_request.reference_frame.emplace(ReferenceFrame::Unknown);
            }
        }

        if (input_config_json.contains("speed"))
        {
            move_request.speed.emplace(input_config_json["speed"].get<double>());
        }

        if (input_config_json.contains("blend_tolerance"))
        {
            move_request.blend_tolerance.emplace(
                input_config_json["blend_tolerance"].get<double>());
        }
    }
    planner_request.data = move_request;
    return planner_request;
}