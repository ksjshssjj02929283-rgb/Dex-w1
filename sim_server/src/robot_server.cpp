#include "robot_server.hpp"

using namespace robot_server_lib;

RobotServer::RobotServer() : Node("sim_server")
{
    Init();
    RCLCPP_INFO(this->get_logger(), "Initialized successfully.");
    LOG_INFO << "Initialized successfully.";
}

RobotServer::~RobotServer()
{
    auto join_thread_if_joinable = [](std::thread &t)
    {
        if (t.joinable())
            t.join();
    };
    join_thread_if_joinable(this->publish_thread_);
    join_thread_if_joinable(this->simulation_thread_);
    join_thread_if_joinable(this->manager_thread_);
    join_thread_if_joinable(this->torque_control_thread_);
}

void RobotServer::Init()
{
    // 创建 /feedback/joint 发布器
    this->feedback_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/feedback/joint_sim", 10);

    // 创建 /feedback/robot_server_state_sim 发布器
    this->robot_server_state_publisher_ = this->create_publisher<std_msgs::msg::String>("/feedback/robot_server_state_sim", 10);

    // 创建 /feedback/robot_server_state 仿真专用订阅器用于与真实机器人关节同步, 部署到真实机器人时不需要
    this->robot_server_state_real_subscriber_ = this->create_subscription<std_msgs::msg::String>(
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
    this->streaming_command_subscriber_ = this->create_subscription<robot_server_interfaces::msg::StreamingCommand>(
        "/control/streaming_command_sim", 10,
        [this](const robot_server_interfaces::msg::StreamingCommand::SharedPtr streaming_msg)
        {
            this->manager_buffers_.streaming_command_latest_buffer->AppendNewElement(utils::ParseStreamingCommand(streaming_msg));

            RCLCPP_INFO(this->get_logger(), "Request_id = %ld, streaming command Received, data size = %zu, streaming_finished = %s",
                        streaming_msg->request_id, streaming_msg->position.size(), streaming_msg->streaming_finished ? "true" : "false");

            std::string pos_str;
            if (!streaming_msg->position.empty())
            {
                std::ostringstream oss;
                std::copy(streaming_msg->position.begin(), streaming_msg->position.end() - 1, std::ostream_iterator<double>(oss, " "));
                oss << streaming_msg->position.back();
                pos_str = oss.str();
            }
            LOG_INFO << "Request_id = " << streaming_msg->request_id << " Streaming, "
                     << "data size = " << streaming_msg->position.size() << ", "
                     << "streaming_finished = " << (streaming_msg->streaming_finished ? "true, " : "false, ") << "data = " << pos_str;
        });

    // 创建 /control/set_property_sim 服务
    this->set_property_srv_ = this->create_service<robot_server_interfaces::srv::SetProperty>(
        "/control/set_property_sim",
        [this](const std::shared_ptr<robot_server_interfaces::srv::SetProperty::Request> request,
               std::shared_ptr<robot_server_interfaces::srv::SetProperty::Response> response)
        {
            PlannerRequest planner_request = utils::ParseSetPropertyRequest(request);
            planner_request.request_id = MakeNewRequestID(planner_request.type);
            this->manager_buffers_.request_queue->EnQueue(planner_request);
            response->request_id = planner_request.request_id;

            RCLCPP_INFO(this->get_logger(), "Request_id = %ld, %s Received, value: %s", planner_request.request_id, request->property_type.c_str(),
                        request->value.c_str());
            LOG_INFO << "Request_id = " << planner_request.request_id << ", Received, type = " << request->property_type
                     << ", value = " << request->value;
        });

    // 创建 /control/move_robot_sim action server
    this->move_robot_action_server_ = rclcpp_action::create_server<robot_server_interfaces::action::MoveRobot>(
        this, "/control/move_robot_sim", std::bind(&RobotServer::HandleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&RobotServer::HandleCancel, this, std::placeholders::_1), std::bind(&RobotServer::HandleAccepted, this, std::placeholders::_1));

    const auto package_share_path = ament_index_cpp::get_package_share_directory("sim_server");
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot.cfg";

    // 初始化机器人模型
    this->robot_model_ = utils::GetRobotModelFromConfig(robot_cfg_path);

    // 初始化全局共享buffer
    this->manager_buffers_.Initialize();
    this->robot_buffers_.Initialize();

    // 创建仿真线程, 模拟执行torque_cmd生成joint raw measurement
#if TORQUE_CONTROL
    RCLCPP_INFO(this->get_logger(), "TORQUE_CONTROL is enabled");
    this->simulation_thread_ = std::thread(
        [this]() -> void
        {
            // 初始更新关节测量数据
            pinocchio::Data robot_data(*this->robot_model_.model_ptr);
            VectorDof torque_cmd{};
            JointRawMeasurement raw_measurement{};
            raw_measurement.position << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, -1.2, 0.0, -1.0, 0.0, -0.5, -0.5, -0.5, 1.2, 0.0, 1.0, 0.0, 0.5, 0.5;
            raw_measurement.velocity.setZero();
            raw_measurement.effort.setZero();
            raw_measurement.time_point = TimePointSystem::Now();
            auto ret = this->robot_buffers_.joint_raw_measurement->AppendNewElement(raw_measurement);

            auto next_cycle_time_point = std::chrono::steady_clock::now();
            while (rclcpp::ok())
            {
                next_cycle_time_point += std::chrono::microseconds(simulation_dt_micro);
                if (!this->robot_buffers_.torque_command_vector->PeekLatest(&torque_cmd))
                {
                    torque_cmd.setZero();
                }
                VectorDof ddq =
                    dynamics::ForwardDynamics(*this->robot_model_.model_ptr, robot_data, raw_measurement.position, raw_measurement.velocity,
                                              torque_cmd, this->robot_model_.params_ptr->joint_command_sequence_by_name);
                raw_measurement.position += simulation_dt * raw_measurement.velocity;
                raw_measurement.velocity += simulation_dt * ddq;
                raw_measurement.effort = torque_cmd;
                raw_measurement.time_point = TimePointSystem::Now();
                this->robot_buffers_.joint_raw_measurement->AppendNewElement(raw_measurement);
                std::this_thread::sleep_until(next_cycle_time_point);
            }
        });
#else
    RCLCPP_INFO(this->get_logger(), "TORQUE_CONTROL is disabled");
    this->simulation_thread_ = std::thread(
        [this]() -> void
        {
            PlannerManagerCommand q_cmd{};
            JointRawMeasurement raw_measurement{};
            raw_measurement.position << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, -1.2, 0.0, -1.0, 0.0, -0.5, -0.5, -0.5, 1.2, 0.0, 1.0, 0.0, 0.5, 0.5;
            raw_measurement.velocity.setZero();
            raw_measurement.effort.setZero();
            raw_measurement.time_point = TimePointSystem::Now();
            this->robot_buffers_.joint_raw_measurement->AppendNewElement(raw_measurement);
            auto next_cycle_time_point = std::chrono::steady_clock::now();
            while (rclcpp::ok())
            {
                next_cycle_time_point += std::chrono::microseconds(manager_dt_micro);
                if (this->manager_buffers_.planner_manager_command->PeekLatest(&q_cmd))
                {
                    // 更新关节测量数据
                    raw_measurement.position = q_cmd.cmd.position;
                    raw_measurement.velocity = q_cmd.cmd.velocity;
                    this->robot_buffers_.joint_raw_measurement->AppendNewElement(raw_measurement);
                }
                std::this_thread::sleep_until(next_cycle_time_point);
            }
        });
#endif
    // Init torque control thread
    this->torque_control_thread_ = std::thread(
        [this]() -> void
        {
            TorqueControllerManager torque_controller_manager(this->robot_model_, this->manager_buffers_, this->robot_buffers_);
            auto next_cycle_time_point = std::chrono::steady_clock::now();
            while (rclcpp::ok())
            {
                next_cycle_time_point += std::chrono::microseconds(torque_ctrl_dt_micro);
                torque_controller_manager.LoopOnce();
                std::this_thread::sleep_until(next_cycle_time_point);
            }
        });

    this->manager_thread_ = std::thread(
        [this]() -> void
        {
            PlannerManager planner_manager(this->robot_model_, this->manager_buffers_,
                                           std::bind(&RobotServer::ReportRequestStatus, this, std::placeholders::_1, std::placeholders::_2,
                                                     std::placeholders::_3, std::placeholders::_4));
            PlannerManagerCommand q_cmd{};
            auto next_cycle_time_point = std::chrono::steady_clock::now();
            while (rclcpp::ok())
            {
                next_cycle_time_point += std::chrono::microseconds(manager_dt_micro);
                planner_manager.LoopOnce();
                if (this->real_robot_sync_.is_updated && manager_buffers_.planner_manager_command->PeekLatest(&q_cmd))
                {
                    q_cmd.cmd.position = this->real_robot_sync_.joint_position;
                    q_cmd.cmd.velocity.setZero();
                    this->real_robot_sync_.is_updated = false;
                }
                std::this_thread::sleep_until(next_cycle_time_point);
            }
        });

    this->publish_thread_ = std::thread(
        [this]() -> void
        {
            JointMeasurement measurement;
            Eigen::VectorXd ee_pose;
            sensor_msgs::msg::JointState feedback_msg;
            feedback_msg.name.resize(ndof_robot);
            feedback_msg.position.resize(ndof_robot);
            feedback_msg.velocity.resize(ndof_robot);
            feedback_msg.effort.resize(ndof_robot);
            auto next_cycle_time_point = std::chrono::steady_clock::now();
            while (rclcpp::ok())
            {
                std_msgs::msg::String robot_server_state_msg;
                if (this->manager_buffers_.joint_measurement->PeekLatest(&measurement) &&
                    this->manager_buffers_.ee_pose_measurement->PeekLatest(&ee_pose))
                {
                    next_cycle_time_point += std::chrono::microseconds(publish_dt_micro);
                    std::ostringstream joint_position_ss = utils::VectorToJsonArrayString(measurement.position);
                    std::ostringstream joint_velocity_ss = utils::VectorToJsonArrayString(measurement.velocity);
                    std::ostringstream joint_torque_ss = utils::VectorToJsonArrayString(measurement.effort);
                    std::ostringstream ee_pose_ss = utils::VectorToJsonArrayString(ee_pose);
                    uint64_t timestamp_ns = this->now().nanoseconds();
                    robot_server_state_msg.data =
                        std::string("{\"timestamp\":") + std::to_string(timestamp_ns) + ",\"status\":\"" + ToString(this->manager_state_) + "\"" +
                        ",\"latest_queued_id\":" + std::to_string(this->latest_report_id_.queued) +
                        ",\"latest_finished_id\":" + std::to_string(this->latest_report_id_.finished) +
                        ",\"latest_errordrop_id\":" + std::to_string(this->latest_report_id_.error_drop) + ",\"ee_pose\":" + ee_pose_ss.str() +
                        ",\"joint_position\":" + joint_position_ss.str() + ",\"joint_velocity\":" + joint_velocity_ss.str() +
                        ",\"actual_arm_torque\":" + joint_torque_ss.str() + "}";
                }
                else
                {
                    uint64_t timestamp_ns = this->now().nanoseconds();
                    robot_server_state_msg.data =
                        std::string("{\"timestamp\":") + std::to_string(timestamp_ns) + ",\"status\":\"" + ToString(this->manager_state_) + "\"}";
                }
                this->robot_server_state_publisher_->publish(robot_server_state_msg);

                // Publish /feedback/joint_sim
                feedback_msg.header.stamp = this->get_clock()->now();
                for (int i = 0; i < ndof_robot; ++i)
                {
                    feedback_msg.name[i] = this->robot_model_.params_ptr->joint_command_sequence_by_name[i];
                    feedback_msg.position[i] = measurement.position[i];
                    feedback_msg.velocity[i] = measurement.velocity[i];
                    feedback_msg.effort[i] = measurement.effort[i];
                }

                this->feedback_publisher_->publish(feedback_msg);
                std::this_thread::sleep_until(next_cycle_time_point);
            }
        });
}

Int64 RobotServer::MakeNewRequestID(RequestType type)
{
    if (type == RequestType::StartJointStreaming || type == RequestType::StartEEPoseStreaming)
    {
        if (this->latest_received_id_ > this->latest_report_id_.finished)
        {
            return -1;
        }
    }
    return ++this->latest_received_id_;
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
    this->manager_buffers_.request_queue->EnQueue(planner_request);
    this->request_handle_queue_.EnQueue(request_handle);

    RCLCPP_INFO(this->get_logger(), "Request_id = %ld, %s Received, arm = %s", planner_request.request_id, request_handle->get_goal()->type.c_str(),
                request_handle->get_goal()->motion_group.c_str());

    LOG_INFO << "Request_id = " << planner_request.request_id << ", Received, type = " << request_handle->get_goal()->type
             << ", arm = " << request_handle->get_goal()->motion_group;
}

rclcpp_action::GoalResponse RobotServer::HandleGoal(const rclcpp_action::GoalUUID &,
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
    this->manager_state_ = manager_state;

    // Log the manager and request status
    if (request_id < 0)
    {
        RCLCPP_INFO(this->get_logger(), "manager state changed to %s", ToString(this->manager_state_));
        LOG_INFO << "Manager state changed to: " << ToString(this->manager_state_);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Request id = %ld, %s %s", request_id, ToString(request_type), ToString(request_status));
    LOG_INFO << "Request id = " << request_id << " " << ToString(request_status) << ", type = " << ToString(request_type);
    // Update the latest report id based on the request status
    switch (request_status)
    {
    case RequestStatus::Queued:
        utils::SetMax(this->latest_report_id_.queued, request_id);
        break;
    case RequestStatus::Executed:
        utils::SetMax(this->latest_report_id_.finished, request_id);
        if (request_type >= RequestType::MoveJoint)
        {
            std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_server_interfaces::action::MoveRobot>> handle_temp;
            if (this->request_handle_queue_.DeQueue(&handle_temp))
            {
                auto result_msg = std::make_shared<robot_server_interfaces::action::MoveRobot::Result>();
                result_msg->request_id = request_id;
                result_msg->success = true;
                handle_temp->succeed(result_msg);
            }
        }
        break;
    case RequestStatus::ErrorDrop:
        utils::SetMax(this->latest_report_id_.error_drop, request_id);
        if (request_type >= RequestType::MoveJoint)
        {
            std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_server_interfaces::action::MoveRobot>> handle_temp;
            if (this->request_handle_queue_.DeQueue(&handle_temp))
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