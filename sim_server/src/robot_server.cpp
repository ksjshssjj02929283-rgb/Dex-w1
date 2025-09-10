#include "robot_server.hpp"

using namespace robot_server_lib;

RobotServer::RobotServer() : Node("sim_server")
{
    InitRobotServer();
    RCLCPP_INFO(this->get_logger(), "Initialized successfully.");
    LOG_INFO << "Initialized successfully.";
}

RobotServer::~RobotServer()
{
    if (publish_thread_.joinable())
    {
        publish_thread_.join();
    }
    if (simulation_thread_.joinable())
    {
        simulation_thread_.join();
    }
    if (manager_thread_.joinable())
    {
        manager_thread_.join();
    }
    if (torque_control_thread_.joinable())
    {
        torque_control_thread_.join();
    }
}

void RobotServer::InitRobotServer()
{
    std::filesystem::create_directories("log/");
    nanolog::initialize(nanolog::GuaranteedLogger(), "log/", "robot_server_sim", 10);
    nanolog::set_log_level(nanolog::LogLevel::INFO);  // options: DEBUG, INFO, WARN, CRIT 高频过程数据打印设置为DEBUG

    // 创建 /feedback/joint 发布器
    feedback_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/feedback/joint_sim", 10);

    // 创建 /feedback/robot_server_state_sim 发布器
    robot_server_state_publisher_ = this->create_publisher<std_msgs::msg::String>("/feedback/robot_server_state_sim", 10);

    // 创建 /feedback/robot_server_state 仿真专用订阅器用于与真实机器人关节同步, 部署到真实机器人时不需要
    robot_server_state_real_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "/feedback/robot_server_state", 10,
        [this](const std_msgs::msg::String::SharedPtr msg)
        {
            auto json_msg = nlohmann::json::parse(msg->data);
            if (json_msg.contains("joint_position"))
            {
                std::vector<double> joint_pos = json_msg["joint_position"].get<std::vector<double>>();
                auto temp = Eigen::Map<robot_server_lib::VectorDof>(joint_pos.data(), joint_pos.size());
                if (!this->real_robot_sync_.joint_position.isApprox(temp, 0.0001))
                {
                    this->real_robot_sync_.joint_position = temp;
                    this->real_robot_sync_.is_updated = true;
                }
            }
        });

    // 创建 /control/streaming_command_sim 订阅器
    streaming_command_subscriber_ = this->create_subscription<robot_server_interfaces::msg::StreamingCommand>(
        "/control/streaming_command_sim", 10,
        [this](const robot_server_interfaces::msg::StreamingCommand::SharedPtr streaming_msg)
        {
            manager_buffers_.streaming_command_latest_buffer->AppendNewElement(utils::ParseStreamingCommand(streaming_msg));

            RCLCPP_INFO(this->get_logger(), "Request_id = %ld, streaming command Received, data size = %zu, streaming_finished = %s",
                        streaming_msg->request_id, streaming_msg->position.size(), streaming_msg->streaming_finished ? "true" : "false");
            std::ostringstream ss;
            for (const auto& v : streaming_msg->position)
            {
                ss << v << " ";
            }
            LOG_INFO << "Request_id = " << streaming_msg->request_id << " Streaming, "
                     << "data size = " << streaming_msg->position.size() << ", "
                     << "streaming_finished = " << (streaming_msg->streaming_finished ? "true, " : "false, ") << "data = " << ss.str();
        });

    // 创建 /control/set_property_sim 服务
    set_property_srv_ = this->create_service<robot_server_interfaces::srv::SetProperty>(
        "/control/set_property_sim",
        [this](const std::shared_ptr<robot_server_interfaces::srv::SetProperty::Request> request,
               std::shared_ptr<robot_server_interfaces::srv::SetProperty::Response> response)
        {
            PlannerRequest planner_request = utils::ParseSetPropertyRequest(request);
            planner_request.request_id = MakeNewRequestID(planner_request.type);
            manager_buffers_.request_queue->EnQueue(planner_request);
            response->request_id = planner_request.request_id;

            RCLCPP_INFO(this->get_logger(), "Request_id = %ld, %s Received, value: %s", planner_request.request_id, request->property_type.c_str(),
                        request->value.c_str());
            LOG_INFO << "Request_id = " << planner_request.request_id << ", Received, type = " << request->property_type
                     << ", value = " << request->value;
        });

    // 创建 /control/move_robot_sim action server
    move_robot_action_server_ = rclcpp_action::create_server<robot_server_interfaces::action::MoveRobot>(
        this, "/control/move_robot_sim", std::bind(&RobotServer::HandleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&RobotServer::HandleCancel, this, std::placeholders::_1), std::bind(&RobotServer::HandleAccepted, this, std::placeholders::_1));

    const auto package_share_path = ament_index_cpp::get_package_share_directory("sim_server");
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot.cfg";

    // 初始化机器人模型
    robot_model_ = utils::GetRobotModelFromConfig(robot_cfg_path);

    // 初始化全局共享buffer
    manager_buffers_.Initialize();
    robot_buffers_.Initialize();

    // 创建仿真线程, 模拟执行torque_cmd生成joint raw measurement
#if TORQUE_CONTROL
    RCLCPP_INFO(this->get_logger(), "TORQUE_CONTROL is enabled");
    simulation_thread_ = std::thread(
        [this]() -> void
        {
            pinocchio::Data robot_data(*robot_model_.model_ptr);
            JointRawMeasurement cache{};
            // 初始更新关节测量数据
            assert(ndof_robot == robot_model_.model_ptr->nv && "Robot model DOF mismatch");
            VectorDof q, dq, ddq, torque_cmd;
            q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, -1.2, 0.0, -1.0, 0.0, -0.5, -0.5, -0.5, 1.2, 0.0, 1.0, 0.0, 0.5, 0.5;
            dq.setZero();
            ddq.setZero();
            torque_cmd.setZero();
            cache.position = q;
            cache.velocity = dq;
            cache.effort = torque_cmd;
            cache.time_point = TimePointSystem::Now();
            auto ret = robot_buffers_.joint_raw_measurement->AppendNewElement(cache);
            VectorDof temp_torque{};

            while (rclcpp::ok())
            {
                if (robot_buffers_.torque_command_vector && robot_buffers_.torque_command_vector->PeekLatest(&temp_torque))
                {
                    torque_cmd = temp_torque;
                }
                else
                {
                    torque_cmd.setZero();
                }
                auto t_start = TimePointSystem::Now();
                ddq = pinocchio::ForwardDynamics(*robot_model_.model_ptr, robot_data, q, dq, torque_cmd,
                                                 robot_model_.params_ptr->joint_command_sequence_by_name);
                int duration_micro = (t_start.time_in_ns() - TimePointSystem::Now().time_in_ns()) / 1e3;
                q += simulation_dt * dq;
                dq += simulation_dt * ddq;
                // 更新关节测量数据
                cache.position = q;
                cache.velocity = dq;
                cache.effort = torque_cmd;
                cache.time_point = TimePointSystem::Now();
                auto ret = robot_buffers_.joint_raw_measurement->AppendNewElement(cache);
                assert(ret);
                if (duration_micro < simulation_dt_micro)
                {
                    std::this_thread::sleep_for(std::chrono::microseconds(simulation_dt_micro - duration_micro));
                }
            }
        });

    // Init torque control thread
    torque_control_thread_ = std::thread(
        [this]() -> void
        {
            TorqueControllerManager torque_controller_manager(this->robot_model_, this->manager_buffers_, this->robot_buffers_);
            while (rclcpp::ok())
            {
                auto t_start = TimePointSystem::Now();
                torque_controller_manager.LoopOnce();
                int duration_micro = (t_start.time_in_ns() - TimePointSystem::Now().time_in_ns()) / 1e3;
                if (duration_micro < torque_ctrl_dt_micro)
                {
                    std::this_thread::sleep_for(std::chrono::microseconds(torque_ctrl_dt_micro - duration_micro));
                }
            }
        });
#else
    RCLCPP_INFO(this->get_logger(), "TORQUE_CONTROL is disabled");
    simulation_thread_ = std::thread(
        [this]() -> void
        {
            PlannerManagerCommand q_cmd{};
            JointMeasurement measurement{};
            while (rclcpp::ok())
            {
                if (!manager_buffers_.planner_manager_command->PeekLatest(&q_cmd))
                {
                    q_cmd.cmd.position << 0.5, -1.0, 0.5, 0.0, 0.0, 0.0, 0.5, -1.2, 0.0, -1.0, 0.0, -0.5, -0.5, -0.5, 1.2, 0.0, 1.0, 0.0, 0.5, 0.5;
                    q_cmd.cmd.velocity.setZero();
                    measurement.q_desired_pre = q_cmd.cmd.position;
                }
                // 更新关节测量数据
                measurement.position = q_cmd.cmd.position;
                measurement.velocity = q_cmd.cmd.velocity;
                if (this->real_robot_sync_.is_updated)
                {
                    measurement.position = measurement.q_desired_pre = this->real_robot_sync_.joint_position;
                    measurement.velocity.setZero();
                    this->real_robot_sync_.is_updated = false;
                }
                auto ret = manager_buffers_.joint_measurement->AppendNewElement(measurement);
                measurement.q_desired_pre = measurement.position;
                assert(ret);
                std::this_thread::sleep_for(std::chrono::microseconds(manager_dt_micro));
            }
        });

    // Init torque control thread
    torque_control_thread_ = std::thread(
        [this]() -> void
        {
            // 不开启力控输出时，只计算当前状态的理论力矩
            pinocchio::Data robot_data(*robot_model_.model_ptr);
            JointMeasurement measurement;
            VectorDof ddq_desired{};

            while (rclcpp::ok())
            {
                auto t_start = TimePointSystem::Now();

                if (manager_buffers_.joint_measurement->PeekLatest(&measurement))
                {
                    VectorDof feedback_q = measurement.position;
                    VectorDof feedback_dq = measurement.velocity;
                    VectorDof tau = pinocchio::InverseDynamics(*robot_model_.model_ptr, robot_data, feedback_q, feedback_dq, ddq_desired,
                                                     robot_model_.params_ptr->joint_command_sequence_by_name);
                    // std::cout << "Arm InverseDynamics computed: " << tau.tail(14).transpose() << std::endl;
                }

                int duration_micro = (TimePointSystem::Now().time_in_ns() - t_start.time_in_ns()) / 1e3;
                if (duration_micro < torque_ctrl_dt_micro)
                {
                    std::this_thread::sleep_for(std::chrono::microseconds(torque_ctrl_dt_micro - duration_micro));
                }
            }
        });
#endif
    manager_thread_ = std::thread(
        [this]() -> void
        {
            PlannerManager planner_manager(this->robot_model_, this->manager_buffers_, std::bind(&RobotServer::ReportRequestStatus, 
                                            this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

            while (rclcpp::ok())
            {
                auto t_start = TimePointSystem::Now();
                planner_manager.LoopOnce();
                int duration_micro = (TimePointSystem::Now().time_in_ns() - t_start.time_in_ns()) / 1e3;
                if (duration_micro < manager_dt_micro)
                {
                    std::this_thread::sleep_for(std::chrono::microseconds(manager_dt_micro - duration_micro));
                }
            }
        });

    publish_thread_ = std::thread(
        [this]() -> void
        {
            while (rclcpp::ok())
            {
                std_msgs::msg::String robot_server_state_msg;
                JointMeasurement measurement;
                Eigen::VectorXd ee_pose;
                if (!manager_buffers_.joint_measurement->PeekLatest(&measurement) || !manager_buffers_.ee_pose_measurement->PeekLatest(&ee_pose))
                {
                    robot_server_state_msg.data = std::string("{\"status\":\"") + ToString(manager_state_) + "\"}";
                    robot_server_state_publisher_->publish(robot_server_state_msg);
                    continue;
                }
                std::ostringstream joint_position_ss = utils::VectorToJsonArrayString(measurement.position);
                std::ostringstream joint_velocity_ss = utils::VectorToJsonArrayString(measurement.velocity);
                std::ostringstream joint_torque_ss = utils::VectorToJsonArrayString(measurement.effort);
                std::ostringstream ee_pose_ss = utils::VectorToJsonArrayString(ee_pose);
                
                // Publish /feedback/robot_server_state_sim
                uint64_t timestamp_ns = this->now().nanoseconds();
                robot_server_state_msg.data = std::string("{\"timestamp\":") + std::to_string(timestamp_ns) + ",\"status\":\"" +
                                              ToString(manager_state_) + "\"" + ",\"latest_queued_id\":" + std::to_string(latest_report_id_.queued) +
                                              ",\"latest_finished_id\":" + std::to_string(latest_report_id_.finished) +
                                              ",\"latest_errordrop_id\":" + std::to_string(latest_report_id_.error_drop) +
                                              ",\"ee_pose\":" + ee_pose_ss.str() + ",\"joint_position\":" + joint_position_ss.str() + 
                                              ",\"joint_velocity\":" + joint_velocity_ss.str() + ",\"actual_arm_torque\":" + joint_torque_ss.str() + "}";

                robot_server_state_publisher_->publish(robot_server_state_msg);

                // Publish /feedback/joint_sim
                sensor_msgs::msg::JointState feedback_msg;
                feedback_msg.header.stamp = this->get_clock()->now();
                feedback_msg.name.resize(ndof_robot);
                feedback_msg.position.resize(ndof_robot);
                feedback_msg.velocity.resize(ndof_robot);
                feedback_msg.effort.resize(ndof_robot);
                for (int i = 0; i < ndof_robot; ++i)
                {
                    feedback_msg.name[i] = robot_model_.params_ptr->joint_command_sequence_by_name[i];
                    feedback_msg.position[i] = measurement.position[i];
                    feedback_msg.velocity[i] = measurement.velocity[i];
                    feedback_msg.effort[i] = measurement.effort[i];
                }

                feedback_publisher_->publish(feedback_msg);
                std::this_thread::sleep_for(std::chrono::microseconds(publish_dt_micro));  // 100Hz
            }
        });
}

Int64 RobotServer::MakeNewRequestID(RequestType type)
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

void RobotServer::HandleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_server_interfaces::action::MoveRobot>> request_handle)
{
    PlannerRequest planner_request = utils::ParseMoveRequest(request_handle);
    auto request_id = MakeNewRequestID(planner_request.type);

    // Publish feedback with request id
    auto feedback = std::make_shared<robot_server_interfaces::action::MoveRobot::Feedback>();
    feedback->request_id = request_id;
    request_handle->publish_feedback(feedback);

    if (request_id < 0)
    {
        RCLCPP_WARN(this->get_logger(), "Still existed previous move request, failed to queue start streaming request");
        LOG_CRIT << "Still existed previous move request, failed to queue start streaming request";
        return;
    }
    planner_request.request_id = request_id;
    manager_buffers_.request_queue->EnQueue(planner_request);
    request_handle_queue_.EnQueue(request_handle);

    RCLCPP_INFO(this->get_logger(), "Request_id = %ld, %s Received, arm = %s", planner_request.request_id, request_handle->get_goal()->type.c_str(),
                request_handle->get_goal()->motion_group.c_str());

    LOG_INFO << "Request_id = " << planner_request.request_id << ", Received, type = " << request_handle->get_goal()->type
             << ", arm = " << request_handle->get_goal()->motion_group;
}

rclcpp_action::GoalResponse RobotServer::HandleGoal(const rclcpp_action::GoalUUID&,
                                                    std::shared_ptr<const robot_server_interfaces::action::MoveRobot::Goal> goal)
{
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotServer::HandleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_server_interfaces::action::MoveRobot>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "MoveRobot request canceled");
    LOG_INFO << "MoveRobot request canceled";
    return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotServer::ReportRequestStatus(Int64 request_id, RequestType request_type, RequestStatus request_status, Status manager_state)
{
    manager_state_ = manager_state;

    // Log the manager and request status
    if (request_id < 0)
    {
        RCLCPP_INFO(this->get_logger(), "manager state changed to %s", ToString(manager_state_));
        LOG_INFO << "Manager state changed to: " << ToString(manager_state_);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Request id = %ld, %s %s", request_id, ToString(request_type), ToString(request_status));
    LOG_INFO << "Request id = " << request_id << " " << ToString(request_status) << ", type = " << ToString(request_type);
    // Update the latest report id based on the request status
    switch (request_status)
    {
    case RequestStatus::Queued:
        utils::SetMax(latest_report_id_.queued, request_id);
        break;
    case RequestStatus::Executed:
        utils::SetMax(latest_report_id_.finished, request_id);
        if (request_type >= RequestType::MoveJoint)
        {
            std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_server_interfaces::action::MoveRobot>> handle_temp;
            if (request_handle_queue_.DeQueue(&handle_temp))
            {
                auto result_msg = std::make_shared<robot_server_interfaces::action::MoveRobot::Result>();
                result_msg->request_id = request_id;
                result_msg->success = true;
                handle_temp->succeed(result_msg);
            }
        }
        break;
    case RequestStatus::ErrorDrop:
        utils::SetMax(latest_report_id_.error_drop, request_id);
        if (request_type >= RequestType::MoveJoint)
        {
            std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_server_interfaces::action::MoveRobot>> handle_temp;
            if (request_handle_queue_.DeQueue(&handle_temp))
            {
                auto result_msg = std::make_shared<robot_server_interfaces::action::MoveRobot::Result>();
                result_msg->request_id = request_id;
                result_msg->success = false;
                handle_temp->succeed(result_msg);
            }
        }
        break;
    default:
        break;
    }
}