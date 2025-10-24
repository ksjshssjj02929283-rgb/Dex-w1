#include "robot_server_base.h"

#include <cstdlib>

namespace robot_server_lib
{

RobotServerBase::RobotServerBase(std::filesystem::path robot_cfg_path, bool is_sim) : Node("robot_server")
{
    std::string interface_suffix = (is_sim) ? "_sim" : "";

    // 初始化日志系统 Initialize the logging system, 固定路径为 ~/dexe_robot_server_data/logs
    std::filesystem::path log_dir = std::filesystem::path(std::getenv("HOME")) / "dexe_robot_server_data" / "logs";
    std::filesystem::create_directories(log_dir);
    nanolog::initialize(nanolog::GuaranteedLogger(), log_dir.string() + "/", "robot_server" + interface_suffix, 20,
                        10);                          // 每个日志文件最大20MB，最多保留10个日志文件
    nanolog::set_log_level(nanolog::LogLevel::INFO);  // options: DEBUG, INFO, WARN, CRIT 高频过程数据打印设置为DEBUG
    RCLCPP_INFO(this->get_logger(), "Logs directory: %s", (log_dir.string() + "/robot_server" + interface_suffix + ".txt").c_str());

    this->robot_server_state_publisher_ = this->create_publisher<std_msgs::msg::String>("/feedback/robot_server_state" + interface_suffix, 10);

    // 创建subscriber
    this->streaming_command_subscriber_ = this->create_subscription<StreamingCommandMsg>(
        "/control/streaming_command" + interface_suffix, 10, std::bind(&RobotServerBase::StreamingCommandCallback, this, std::placeholders::_1));

    // 创建service server
    this->set_property_srv_ =
        this->create_service<SetProperty>("/control/set_property" + interface_suffix,
                                          std::bind(&RobotServerBase::SetPropertyCallback, this, std::placeholders::_1, std::placeholders::_2));

    // 创建action server
    this->move_robot_action_server_ = rclcpp_action::create_server<MoveRobot>(
        this, "/control/move_robot" + interface_suffix,
        // Goal callback
        [](const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveRobot::Goal>) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
        // Cancel callback
        [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveRobot>>) { return rclcpp_action::CancelResponse::ACCEPT; },
        std::bind(&RobotServerBase::MoveRobotCallback, this, std::placeholders::_1));

    // 初始化机器人模型
    this->robot_model_ = utils::GetRobotModelFromConfig(robot_cfg_path);

    // 初始化全局共享buffer
    this->manager_buffers_.Initialize();
    this->robot_buffers_.Initialize();
}

RobotServerBase::~RobotServerBase()
{
    auto join_thread_if_joinable = [](std::thread &t)
    {
        if (t.joinable())
            t.join();
    };
    join_thread_if_joinable(this->publish_thread_);
    join_thread_if_joinable(this->manager_thread_);
    join_thread_if_joinable(this->torque_control_thread_);
    join_thread_if_joinable(this->robot_thread_);
}

void RobotServerBase::Run()
{
    this->robot_thread_ = std::thread(&RobotServerBase::RobotThreadFunc, this);
    this->torque_control_thread_ = std::thread(&RobotServerBase::TorqueControlThreadFunc, this);
    this->manager_thread_ = std::thread(&RobotServerBase::ManagerThreadFunc, this);
    this->publish_thread_ = std::thread(&RobotServerBase::PublishThreadFunc, this);
    RCLCPP_INFO(this->get_logger(), "Initialized successfully.");
    LOG_INFO << "Initialized successfully.";
}

void RobotServerBase::ManagerThreadFunc()
{
    PlannerManager planner_manager(this->robot_model_, this->manager_buffers_,
                                   std::bind(&RobotServerBase::ReportRequestStatus, this, std::placeholders::_1, std::placeholders::_2,
                                             std::placeholders::_3, std::placeholders::_4));
    PlannerManagerCommand q_cmd{};
    auto next_cycle_time_point = std::chrono::steady_clock::now();
    while (rclcpp::ok())
    {
        next_cycle_time_point += std::chrono::microseconds(manager_dt_micro);
        planner_manager.LoopOnce();
        std::this_thread::sleep_until(next_cycle_time_point);
    }
}

void RobotServerBase::TorqueControlThreadFunc()
{
    TorqueControllerManager torque_controller_manager(this->robot_model_, this->manager_buffers_, this->robot_buffers_);
    auto next_cycle_time_point = std::chrono::steady_clock::now();
    while (rclcpp::ok())
    {
        next_cycle_time_point += std::chrono::microseconds(torque_ctrl_dt_micro);
        torque_controller_manager.LoopOnce();
        std::this_thread::sleep_until(next_cycle_time_point);
    }
}

void RobotServerBase::PublishThreadFunc()
{
    JointMeasurement measurement;
    Eigen::VectorXd ee_pose;
    auto next_cycle_time_point = std::chrono::steady_clock::now();
    while (rclcpp::ok())
    {
        next_cycle_time_point += std::chrono::microseconds(publish_dt_micro);

        // Publish /feedback/robot_server_state
        std_msgs::msg::String robot_server_state_msg;
        if (is_robot_ready_ && this->manager_buffers_.joint_measurement->PeekLatest(&measurement) &&
            this->manager_buffers_.ee_pose_measurement->PeekLatest(&ee_pose))
        {
            std::ostringstream joint_position_ss = utils::VectorToJsonArrayString(measurement.position);
            std::ostringstream joint_velocity_ss = utils::VectorToJsonArrayString(measurement.velocity);
            std::ostringstream joint_torque_ss = utils::VectorToJsonArrayString(measurement.effort);
            std::ostringstream ee_pose_ss = utils::VectorToJsonArrayString(ee_pose);
            uint64_t timestamp_ns = this->now().nanoseconds();
            robot_server_state_msg.data = std::string("{\"timestamp\":") + std::to_string(timestamp_ns) + ",\"status\":\"" +
                                          ToString(this->manager_state_) + "\"" +
                                          ",\"latest_queued_id\":" + std::to_string(this->latest_report_id_.queued) +
                                          ",\"latest_finished_id\":" + std::to_string(this->latest_report_id_.finished) +
                                          ",\"latest_errordrop_id\":" + std::to_string(this->latest_report_id_.error_drop) +
                                          ",\"ee_pose\":" + ee_pose_ss.str() + ",\"joint_position\":" + joint_position_ss.str() +
                                          ",\"joint_velocity\":" + joint_velocity_ss.str() + ",\"joint_torque\":" + joint_torque_ss.str() + "}";
        }
        else
        {
            uint64_t timestamp_ns = this->now().nanoseconds();
            robot_server_state_msg.data =
                std::string("{\"timestamp\":") + std::to_string(timestamp_ns) + ",\"status\":\"" + ToString(this->manager_state_) + "\"}";
        }
        this->robot_server_state_publisher_->publish(robot_server_state_msg);
        std::this_thread::sleep_until(next_cycle_time_point);
    }
}

Int64 RobotServerBase::MakeNewRequestID(RequestType type)
{
    if (type == RequestType::StartJointStreaming || type == RequestType::StartEEPoseStreaming)
    {
        if (this->latest_received_id_ > this->latest_report_id_.finished)
        {
            RCLCPP_WARN(this->get_logger(), "Still existed previous move request, failed to queue start streaming request");
            LOG_CRIT << "Still existed previous move request, failed to queue start streaming request";
            return -1;
        }
    }
    return ++this->latest_received_id_;
}

void RobotServerBase::MoveRobotCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveRobot>> request_handle)
{
    PlannerRequest planner_request = utils::ParseMoveRequest(request_handle);
    auto request_id = MakeNewRequestID(planner_request.type);

    // Check if in Drag mode
    PlannerManagerCommand planner_cmd{};
    if (this->manager_buffers_.planner_manager_command->PeekLatest(&planner_cmd) && planner_cmd.robot_property.run_mode == RunMode::Drag)
    {
        RCLCPP_WARN(this->get_logger(), "In Drag mode, cannot accept new move robot request");
        LOG_CRIT << "In Drag mode, cannot accept new move robot request";
        request_id = -1;
    }
    // Publish feedback with request id
    auto feedback = std::make_shared<MoveRobot::Feedback>();
    feedback->request_id = request_id;
    request_handle->publish_feedback(feedback);

    if (request_id < 0)
    {
        RCLCPP_WARN(this->get_logger(), "MakeNewRequestID < 0, failed to queue move robot request");
        LOG_CRIT << "MakeNewRequestID < 0, failed to queue move robot request";
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

void RobotServerBase::StreamingCommandCallback(const StreamingCommandMsg::SharedPtr streaming_msg)
{
    RCLCPP_INFO(this->get_logger(), "Request_id = %ld, streaming command Received, data size = %zu, streaming_finished = %s",
                streaming_msg->request_id, streaming_msg->position.size(), streaming_msg->streaming_finished ? "true" : "false");
    std::ostringstream ss;
    for (const auto &v : streaming_msg->position)
    {
        ss << v << " ";
    }
    if (streaming_msg->streaming_finished)
    {
        LOG_INFO << "Request_id = " << streaming_msg->request_id << " Streaming, "
                 << "data size = " << streaming_msg->position.size() << ", "
                 << "streaming_finished = "
                 << "true, "
                 << "data = " << ss.str();
    }

    manager_buffers_.streaming_command_latest_buffer->AppendNewElement(utils::ParseStreamingCommand(streaming_msg));
}

void RobotServerBase::SetPropertyCallback(const std::shared_ptr<SetProperty::Request> request, std::shared_ptr<SetProperty::Response> response)
{
    PlannerRequest planner_request = utils::ParseSetPropertyRequest(request);
    planner_request.request_id = MakeNewRequestID(planner_request.type);
    this->manager_buffers_.request_queue->EnQueue(planner_request);
    response->request_id = planner_request.request_id;

    RCLCPP_INFO(this->get_logger(), "Request_id = %ld, %s Received, value: %s", planner_request.request_id, request->property_type.c_str(),
                request->value.c_str());
    LOG_INFO << "Request_id = " << planner_request.request_id << ", Received, type = " << request->property_type << ", value = " << request->value;
}

void RobotServerBase::ReportRequestStatus(Int64 request_id, RequestType request_type, RequestStatus request_status, Status manager_state)
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
            std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveRobot>> handle_temp;
            if (this->request_handle_queue_.DeQueue(&handle_temp))
            {
                auto result_msg = std::make_shared<MoveRobot::Result>();
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
            std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveRobot>> handle_temp;
            if (this->request_handle_queue_.DeQueue(&handle_temp))
            {
                auto result_msg = std::make_shared<MoveRobot::Result>();
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

}  // namespace robot_server_lib