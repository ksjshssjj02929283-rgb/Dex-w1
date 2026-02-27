#include "robot_server_base.h"

#include <cstdlib>

namespace robot_server_lib {

RobotServerBase::RobotServerBase(std::filesystem::path robot_cfg_path, bool is_sim)
    : Node("robot_server"), is_sim_(is_sim) {
    std::string interface_suffix = (is_sim_) ? "_sim" : "";

    // 初始化日志系统 Initialize the logging system, 固定路径为 ~/log
    std::filesystem::create_directories(home_dir / "log");
    nanolog::initialize(nanolog::GuaranteedLogger(), (home_dir / "log/").string(), "robot_server" + interface_suffix,
                        10,
                        10);  // 每个日志文件最大20MB，最多保留10个日志文件

    // 初始化记录日志实例（用于记录关节数据等高频数据）
    nanolog::LogManager::instance().get_or_create_logger("record",  // 日志实例名称
                                                         nanolog::GuaranteedLogger(), (home_dir / "log/").string(),
                                                         "robot_record" + interface_suffix,
                                                         20,        // 每个日志文件最大20MB
                                                         5,         // 保留5个日志文件
                                                         "jsonl");  // 文件后缀

    nanolog::set_log_level(nanolog::LogLevel::INFO);  // options: DEBUG, INFO, WARN, CRIT 高频过程数据打印设置为DEBUG
    RCLCPP_INFO(this->get_logger(), "Logs directory: %s, Records directory: %s",
                ((home_dir / "log").string() + "/robot_server" + interface_suffix + ".txt").c_str(),
                ((home_dir / "log").string() + "/robot_record" + interface_suffix + ".jsonl").c_str());
    LOG_INFO << "";
    LOG_INFO
        << "---------------------------------------Robot Server Initializing---------------------------------------";

    this->robot_server_state_publisher_ =
        this->create_publisher<std_msgs::msg::String>("/feedback/robot_server_state" + interface_suffix, 10);

    // 创建subscriber
    this->streaming_command_subscriber_ = this->create_subscription<StreamingCommandMsg>(
        "/control/streaming_command" + interface_suffix, 10,
        std::bind(&RobotServerBase::StreamingCommandCallback, this, std::placeholders::_1));

    // 创建service server
    this->set_property_srv_ = this->create_service<SetProperty>(
        "/control/set_property" + interface_suffix,
        std::bind(&RobotServerBase::SetPropertyCallback, this, std::placeholders::_1, std::placeholders::_2));

    // 创建action server
    this->move_robot_action_server_ = rclcpp_action::create_server<MoveRobot>(
        this, "/control/move_robot" + interface_suffix,
        // Goal callback
        [](const rclcpp_action::GoalUUID&, std::shared_ptr<const MoveRobot::Goal>) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        // Cancel callback
        [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveRobot>>) {
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        std::bind(&RobotServerBase::MoveRobotCallback, this, std::placeholders::_1));

    // 初始化机器人模型
    this->robot_model_ = utils::GetRobotModelFromConfig(robot_cfg_path);

    // 初始化全局共享buffer
    this->manager_buffers_.Initialize();
    this->robot_buffers_.Initialize();
}

RobotServerBase::~RobotServerBase() {
    auto join_thread_if_joinable = [](std::thread& t) {
        if (t.joinable()) t.join();
    };
    join_thread_if_joinable(this->publish_thread_);
    join_thread_if_joinable(this->manager_thread_);
    join_thread_if_joinable(this->torque_control_thread_);
    join_thread_if_joinable(this->robot_thread_);
}

void RobotServerBase::Run() {
    this->robot_thread_ = std::thread(&RobotServerBase::RobotThreadFunc, this);
    if (TORQUE_CONTROL || !is_sim_) {
        this->torque_control_thread_ = std::thread(&RobotServerBase::TorqueControlThreadFunc, this);
    }
    this->manager_thread_ = std::thread(&RobotServerBase::ManagerThreadFunc, this);
    this->publish_thread_ = std::thread(&RobotServerBase::PublishThreadFunc, this);
    RCLCPP_INFO(this->get_logger(), "Initialized successfully.");
    LOG_INFO << "Robot Server Initialized Successfully";
}

void RobotServerBase::ManagerThreadFunc() {
    PlannerManager planner_manager(this->robot_model_, this->manager_buffers_,
                                   std::bind(&RobotServerBase::ReportRequestStatus, this, std::placeholders::_1,
                                             std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
    auto next_cycle_time_point = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
        next_cycle_time_point += std::chrono::microseconds(manager_dt_micro);
        planner_manager.LoopOnce();
        std::this_thread::sleep_until(next_cycle_time_point);
    }
}

void RobotServerBase::TorqueControlThreadFunc() {
    if (!is_sim_) {
        Setup_Realtime();
    }
    TorqueControllerManager torque_controller_manager(this->robot_model_, this->manager_buffers_, this->robot_buffers_);
    auto next_cycle_time_point = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
        next_cycle_time_point += std::chrono::microseconds(torque_ctrl_dt_micro);
        torque_controller_manager.LoopOnce();
        std::this_thread::sleep_until(next_cycle_time_point);
    }
}

void RobotServerBase::PublishThreadFunc() {
    JointRawMeasurement raw_measurement;
    PlannerManagerCommand planner_cmd{};
    PlannerManagerInfo planner_info{};
    auto next_cycle_time_point = std::chrono::steady_clock::now();

    while (rclcpp::ok()) {
        next_cycle_time_point += std::chrono::microseconds(publish_dt_micro);

        std_msgs::msg::String robot_server_state_msg;
        uint64_t timestamp_ns = this->now().nanoseconds();

        if (this->robot_buffers_.joint_raw_measurement->PeekLatest(&raw_measurement) &&
            this->manager_buffers_.planner_manager_info->PeekLatest(&planner_info) &&
            this->manager_buffers_.planner_manager_command->PeekLatest(&planner_cmd)) {
            robot_server_state_msg.data = GetRobotServerStateJson(
                timestamp_ns,
                std::vector<double>(raw_measurement.position.data(),
                                    raw_measurement.position.data() + raw_measurement.position.size()),
                std::vector<double>(raw_measurement.velocity.data(),
                                    raw_measurement.velocity.data() + raw_measurement.velocity.size()),
                std::vector<double>(raw_measurement.effort.data(),
                                    raw_measurement.effort.data() + raw_measurement.effort.size()),
                std::vector<double>(raw_measurement.sensor_wrench.data(),
                                    raw_measurement.sensor_wrench.data() + raw_measurement.sensor_wrench.size()),
                std::vector<double>(planner_info.ee_pose_measurement.data(),
                                    planner_info.ee_pose_measurement.data() + planner_info.ee_pose_measurement.size()),
                planner_info);
        } else if (!TORQUE_CONTROL && is_sim_) {
            JointMeasurement joint_measurement;
            if (this->manager_buffers_.joint_measurement->PeekLatest(&joint_measurement) &&
                this->manager_buffers_.planner_manager_info->PeekLatest(&planner_info) &&
                this->manager_buffers_.planner_manager_command->PeekLatest(&planner_cmd)) {
                robot_server_state_msg.data = GetRobotServerStateJson(
                    timestamp_ns,
                    std::vector<double>(joint_measurement.position.data(),
                                        joint_measurement.position.data() + joint_measurement.position.size()),
                    std::vector<double>(joint_measurement.velocity.data(),
                                        joint_measurement.velocity.data() + joint_measurement.velocity.size()),
                    std::vector<double>(joint_measurement.effort.data(),
                                        joint_measurement.effort.data() + joint_measurement.effort.size()),
                    std::vector<double>(raw_measurement.sensor_wrench.data(),
                                        raw_measurement.sensor_wrench.data() + raw_measurement.sensor_wrench.size()),
                    std::vector<double>(
                        planner_info.ee_pose_measurement.data(),
                        planner_info.ee_pose_measurement.data() + planner_info.ee_pose_measurement.size()),
                    planner_info);
            } else {
                robot_server_state_msg.data = GetRobotServerStateJson(timestamp_ns);
            }
        } else {
            robot_server_state_msg.data = GetRobotServerStateJson(timestamp_ns);
        }

        this->robot_server_state_publisher_->publish(robot_server_state_msg);
        std::this_thread::sleep_until(next_cycle_time_point);
    }
}

std::string RobotServerBase::GetRobotServerStateJson(uint64_t timestamp_ns,
                                                     const std::optional<std::vector<double>>& position,
                                                     const std::optional<std::vector<double>>& velocity,
                                                     const std::optional<std::vector<double>>& effort,
                                                     const std::optional<std::vector<double>>& sensor_wrench,
                                                     const std::optional<std::vector<double>>& ee_pose,
                                                     const std::optional<PlannerManagerInfo>& planner_info) {
    std::string json = "{\"timestamp\":" + std::to_string(timestamp_ns) + ",\"status\":\"" +
                       ToString(this->manager_state_) + "\"" + ", \"robot_type\":\"" +
                       this->robot_model_.params_ptr->robot_name + "\"" + ", \"robot_version\":\"" +
                       this->robot_model_.params_ptr->robot_version + "\"";
    PlannerManagerInfo manager_info;
    this->manager_buffers_.planner_manager_info->PeekLatest(&manager_info);
    if (planner_info.has_value()) {
        std::ostringstream joint_position_ss = utils::VectorToJsonArrayString(position.value());
        std::ostringstream joint_velocity_ss = utils::VectorToJsonArrayString(velocity.value());
        std::ostringstream joint_torque_ss = utils::VectorToJsonArrayString(effort.value());
        std::ostringstream joint_sensor_wrench_ss = utils::VectorToJsonArrayString(sensor_wrench.value());
        std::ostringstream ee_pose_ss = utils::VectorToJsonArrayString(ee_pose.value());

        json += "\n ,\"run_mode\":\"" + std::string(ToString(planner_info->robot_property.run_mode)) + "\"" +
                ", \"server_error_code\":\"" + ToString(manager_info.robot_property.error_code) + "\"" +
                ", \"safe_level\":\"" + std::to_string(manager_info.robot_property.safe_level) + "\"" +
                ", \"global_speed_ratio\":\"" + std::to_string(manager_info.robot_property.global_speed_ratio) + "\"" +
                "\n ,\"latest_queued_id\":" + std::to_string(this->latest_report_id_.queued) +
                ",\"latest_finished_id\":" + std::to_string(this->latest_report_id_.finished) +
                ",\"latest_errordrop_id\":" + std::to_string(this->latest_report_id_.error_drop) +
                ",\n \"ee_pose\":" + ee_pose_ss.str() + ",\n \"force_sensor\":" + joint_sensor_wrench_ss.str() +
                ",\n \"joint_position\":" + joint_position_ss.str() +
                ",\n \"joint_velocity\":" + joint_velocity_ss.str() + ",\n \"joint_torque\":" + joint_torque_ss.str();
    }
    json += "}";
    return json;
}

Int64 RobotServerBase::MakeNewRequestID(const PlannerRequest& planner_request) {
    const RequestType& type = planner_request.type;
    if (type == RequestType::StartJointStreaming || type == RequestType::StartEEPoseStreaming ||
        type == RequestType::StartEEVelocityStreaming) {
        if (this->latest_received_id_ >
            (std::max(this->latest_report_id_.finished.load(), this->latest_report_id_.error_drop.load()))) {
            RCLCPP_WARN(this->get_logger(),
                        "Still existed previous move request, failed to queue start streaming request, "
                        "latest_received_id_: %ld, "
                        "latest_finished_id_: %ld, latest_errordrop_id_: %ld",
                        this->latest_received_id_, this->latest_report_id_.finished.load(),
                        this->latest_report_id_.error_drop.load());
            LOG_CRIT << "Still existed previous move request, failed to queue start streaming request"
                     << ", latest_received_id_: " << this->latest_received_id_
                     << ", latest_finished_id_: " << this->latest_report_id_.finished.load()
                     << ", latest_errordrop_id_: " << this->latest_report_id_.error_drop.load();
            return -1;
        }
    }
    if (type == RequestType::MoveTraj) {
        if (!std::holds_alternative<MoveRequest>(planner_request.data.value())) {
            RCLCPP_WARN(this->get_logger(), "Invalid MoveTraj request data, failed to queue move traj request");
            LOG_CRIT << "Invalid MoveTraj request data, failed to queue move traj request";
            return -1;
        }
        auto move_request = std::get<MoveRequest>(planner_request.data.value());
        if (move_request.interval.has_value()) {
            if (move_request.endpoint_index.has_value()) {
                std::vector<int> endpoint_index = move_request.endpoint_index.value();
                this->latest_received_id_ += endpoint_index.size();
            } else {
                this->latest_received_id_++;
            }
            return this->latest_received_id_;
        } else {
            RCLCPP_WARN(this->get_logger(),
                        "MoveTraj request missing interval or endpoint_index, failed to queue move traj request");
            LOG_CRIT << "MoveTraj request missing interval or endpoint_index, failed to queue move traj request";
            return -1;
        }
    }
    if (type == RequestType::EnableDrag && this->manager_state_ == Status::Running) {
        RCLCPP_WARN(this->get_logger(),
                    "Cannot change to Drag mode during Running state, request type: %s, manager state: %s",
                    ToString(type), ToString(this->manager_state_));
        LOG_CRIT << "Cannot change to Drag mode during Running state, request type: " << ToString(type)
                 << ", manager state: " << ToString(this->manager_state_);
        return -1;
    }
    return ++this->latest_received_id_;
}

void RobotServerBase::MoveRobotCallback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveRobot>> request_handle) {
    PlannerRequest planner_request = utils::ParseMoveRequest(request_handle);
    auto request_id = MakeNewRequestID(planner_request);

    // Publish feedback with request id
    auto feedback = std::make_shared<MoveRobot::Feedback>();
    feedback->request_id = request_id;
    request_handle->publish_feedback(feedback);

    if (request_id < 0) {
        RCLCPP_WARN(this->get_logger(), "MakeNewRequestID < 0, failed to queue move robot request");
        LOG_CRIT << "MakeNewRequestID < 0, failed to queue move robot request";
        return;
    }
    planner_request.request_id = request_id;
    planner_request.type >= RequestType::StopMove
        ? this->manager_buffers_.emergency_request_queue->EnQueue(planner_request)
        : this->manager_buffers_.request_queue->EnQueue(planner_request);

    this->request_handle_queue_.EnQueue(request_handle);
    RCLCPP_INFO(this->get_logger(), "Request_id = %ld, %s Received, arm = %s, is emergency request: %s",
                planner_request.request_id, request_handle->get_goal()->type.c_str(),
                request_handle->get_goal()->motion_group.c_str(),
                planner_request.type >= RequestType::StopMove ? "true" : "false");

    LOG_INFO << "Request_id = " << planner_request.request_id
             << ", Received, type = " << request_handle->get_goal()->type
             << ", arm = " << request_handle->get_goal()->motion_group
             << ", is emergency request: " << (planner_request.type >= RequestType::StopMove ? "true" : "false");
}

void RobotServerBase::StreamingCommandCallback(const StreamingCommandMsg::SharedPtr streaming_msg) {
    std::ostringstream ss;
    for (const auto& v : streaming_msg->position) {
        ss << v << " ";
    }
    if (streaming_msg->streaming_finished) {
        RCLCPP_INFO(this->get_logger(),
                    "Request_id = %ld, streaming command Received, data size = %zu, streaming_finished = %s",
                    streaming_msg->request_id, streaming_msg->position.size(),
                    streaming_msg->streaming_finished ? "true" : "false");
        LOG_INFO << "Request_id = " << streaming_msg->request_id << " Streaming, "
                 << "data size = " << streaming_msg->position.size() << ", "
                 << "streaming_finished = "
                 << "true, "
                 << "data = " << ss.str();
    }

    manager_buffers_.streaming_command_latest_buffer->AppendNewElement(utils::ParseStreamingCommand(streaming_msg));
}

void RobotServerBase::SetPropertyCallback(const std::shared_ptr<SetProperty::Request> request,
                                          std::shared_ptr<SetProperty::Response> response) {
    PlannerRequest planner_request = utils::ParseSetPropertyRequest(request);
    if (planner_request.type == RequestType::Empty) {
        RCLCPP_WARN(this->get_logger(), "Failed to parse set property request, type: %s, value: %s",
                    request->property_type.c_str(), request->value.c_str());
        LOG_CRIT << "Failed to parse set property request, type: " << request->property_type
                 << ", value: " << request->value;
        response->request_id = -1;
        return;
    }
    planner_request.request_id = MakeNewRequestID(planner_request);
    this->manager_buffers_.request_queue->EnQueue(planner_request);
    response->request_id = planner_request.request_id;

    RCLCPP_INFO(this->get_logger(), "Request_id = %ld, %s Received, value: %s", planner_request.request_id,
                request->property_type.c_str(), request->value.c_str());
    LOG_INFO << "Request_id = " << planner_request.request_id << ", Received, type = " << request->property_type
             << ", value = " << request->value;
}

void RobotServerBase::ReportRequestStatus(Int64 request_id,
                                          RequestType request_type,
                                          RequestStatus request_status,
                                          Status manager_state) {
    this->manager_state_ = manager_state;

    // Log the manager and request status
    if (request_id < 0) {
        RCLCPP_INFO(this->get_logger(), "manager state changed to %s", ToString(this->manager_state_));
        LOG_INFO << "Manager state changed to: " << ToString(this->manager_state_);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Request id = %ld, %s %s", request_id, ToString(request_type),
                ToString(request_status));
    LOG_INFO << "Request id = " << request_id << " " << ToString(request_status)
             << ", type = " << ToString(request_type);
    // Update the latest report id based on the request status
    switch (request_status) {
        case RequestStatus::Queued:
            utils::SetMax(this->latest_report_id_.queued, request_id);
            break;
        case RequestStatus::Executed:
            utils::SetMax(this->latest_report_id_.finished, request_id);
            if (request_type >= RequestType::MoveJoint) {
                std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveRobot>> handle_temp;
                if (this->request_handle_queue_.DeQueue(&handle_temp)) {
                    auto result_msg = std::make_shared<MoveRobot::Result>();
                    result_msg->request_id = request_id;
                    result_msg->success = true;
                    handle_temp->succeed(result_msg);
                }
            }
            break;
        case RequestStatus::PhaseFinished:
            utils::SetMax(this->latest_report_id_.finished, request_id);
            break;
        case RequestStatus::Stopped:
            utils::SetMax(this->latest_report_id_.finished, request_id);
            if (request_type >= RequestType::MoveJoint) {
                std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveRobot>> handle_temp;
                if (this->request_handle_queue_.DeQueue(&handle_temp)) {
                    auto result_msg = std::make_shared<MoveRobot::Result>();
                    result_msg->request_id = request_id;
                    result_msg->success = true;
                    handle_temp->succeed(result_msg);
                }
            }
            break;
        case RequestStatus::ErrorDrop:
            utils::SetMax(this->latest_report_id_.error_drop, request_id);
            if (request_type >= RequestType::MoveJoint) {
                std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveRobot>> handle_temp;
                if (this->request_handle_queue_.DeQueue(&handle_temp)) {
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

void RobotServerBase::Setup_Realtime() {
    // 锁内存，避免分页抖动
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        std::cerr << "mlockall failed: " << std::strerror(errno) << std::endl;
    }
    // 设置SCHED_FIFO实时调度
    sched_param sp{};
    sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        std::cerr << "sched_setscheduler failed: " << std::strerror(errno) << std::endl;
        LOG_CRIT << "sched_setscheduler failed, please run setcap!";
        assert(false && "sched_setscheduler failed, please run setcap!");
    }
}

}  // namespace robot_server_lib