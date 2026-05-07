#include "robot_server_sim.h"

namespace robot_server_lib {

RobotServer::RobotServer(std::filesystem::path robot_cfg_path, bool is_sim) : RobotServerBase(robot_cfg_path, is_sim) {
    // 创建publisher
    this->feedback_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/feedback/joint_sim", 10);

    // 创建 /control/joint 订阅器
    control_subscriber_ = this->create_subscription<JointControlMsg>(
        "/control/joint_sim", 10, std::bind(&RobotServer::ControlCallback, this, std::placeholders::_1));

    // 创建 /control/joint_position 订阅器
    position_control_subscriber_ = this->create_subscription<JointPositionControlMsg>(
        "/control/joint_position_sim", 10,
        std::bind(&RobotServer::PositionControlCallback, this, std::placeholders::_1));

    // 创建 /feedback/robot_server_state 仿真专用订阅器用于与真实机器人关节同步, 部署到真实机器人时不需要
    this->robot_server_state_real_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "/feedback/robot_server_state", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
            auto json_msg = nlohmann::json::parse(msg->data);
            if (json_msg.contains("joint_position")) {
                std::vector<double> joint_pos = json_msg["joint_position"].get<std::vector<double>>();
                auto temp = Eigen::Map<robot_server_lib::VectorDof>(joint_pos.data(), joint_pos.size());
                if (!this->real_robot_sync_.joint_position.isApprox(temp, 0.005)) {
                    this->real_robot_sync_.joint_position = temp;
                    this->real_robot_sync_.is_updated = true;
                }
            }
        });
    // 创建定时器，定时发布feedback 定时器回调函数
    this->Run();
    control_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() { PublishFeedback(); });
}

void RobotServer::RobotThreadFunc() {
#if TORQUE_CONTROL
    RCLCPP_INFO(this->get_logger(), "Torque control is enabled.");
    LOG_INFO << "Torque control is enabled.";

    TorqueCtrlCommand torque_cmd{};
    JointRawMeasurement raw_measurement{};
    raw_measurement.position << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, -1.2, 0.0, -1.0, 0.0, -0.5, -0.5, 1.0, 1.2, 0.0,
        1.0, 0.0, 0.5, 0.5;
    raw_measurement.velocity.setZero();
    raw_measurement.effort.setZero();
    raw_measurement.sensor_wrench = Eigen::VectorXd::Zero(12);
    raw_measurement.time_point = TimePointSystem::Now();
    this->robot_buffers_.joint_raw_measurement->AppendNewElement(raw_measurement);

    auto next_cycle_time_point = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
        next_cycle_time_point += std::chrono::microseconds(simulation_dt_micro);
        PlannerManagerCommand q_cmd{};
        if (this->robot_buffers_.torque_controller_command->PeekLatest(&torque_cmd) &&
            this->manager_buffers_.planner_manager_command->PeekLatest(&q_cmd) &&
            (manager_state_ == Status::Idle || manager_state_ == Status::Running))  // 有指令了才进行正动力学仿真
        {
            VectorDof ddq = dynamics::ForwardDynamics(*this->robot_model_.model_ptr, raw_measurement.position,
                                                      raw_measurement.velocity, torque_cmd.torque,
                                                      this->robot_model_.params_ptr->joint_command_sequence_by_name);
            raw_measurement.velocity += simulation_dt * ddq;
            raw_measurement.velocity.segment(0, 6) =
                q_cmd.cmd.velocity.segment(0, 6);  // 保持躯干和头部由位置控制器控制
            raw_measurement.position += simulation_dt * raw_measurement.velocity;
            raw_measurement.position.segment(0, 6) =
                q_cmd.cmd.position.segment(0, 6);  // 保持躯干和头部由位置控制器控制
            raw_measurement.effort = torque_cmd.torque;
            raw_measurement.time_point = TimePointSystem::Now();
            this->robot_buffers_.joint_raw_measurement->AppendNewElement(raw_measurement);
        }
        std::this_thread::sleep_until(next_cycle_time_point);
    }
#else
    RCLCPP_INFO(this->get_logger(), "Torque control is disabled.");
    LOG_INFO << "Torque control is disabled.";

    PlannerManagerCommand q_cmd{};
    JointMeasurement joint_measurement{};
    joint_measurement.position << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, -1.2, 0.0, -1.0, 0.0, -0.5, -0.5, -0.5, 1.2, 0.0,
        1.0, 0.0, 0.5, 0.5;
    joint_measurement.q_desired_pre = joint_measurement.position;
    joint_measurement.velocity.setZero();
    joint_measurement.effort.setZero();
    this->manager_buffers_.joint_measurement->AppendNewElement(joint_measurement);
    auto next_cycle_time_point = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
        next_cycle_time_point += std::chrono::microseconds(manager_dt_micro);
        if (this->manager_buffers_.planner_manager_command->PeekLatest(&q_cmd)) {
            // 更新关节测量数据
            joint_measurement.position = q_cmd.cmd.position;
            joint_measurement.q_desired_pre = q_cmd.cmd.position;
            joint_measurement.velocity = q_cmd.cmd.velocity;
            this->manager_buffers_.joint_measurement->AppendNewElement(joint_measurement);
        }
        std::this_thread::sleep_until(next_cycle_time_point);
    }
#endif
}

void RobotServer::ManagerThreadFunc() {
    PlannerManager planner_manager(this->robot_model_, this->manager_buffers_,
                                   std::bind(&RobotServer::ReportRequestStatus, this, std::placeholders::_1,
                                             std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
    PlannerManagerCommand q_cmd{};
    auto next_cycle_time_point = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
        next_cycle_time_point += std::chrono::microseconds(manager_dt_micro);
        if (is_robot_ready_) {
            planner_manager.LoopOnce();
            if (this->real_robot_sync_.is_updated &&
                this->manager_buffers_.planner_manager_command->PeekLatest(&q_cmd)) {
                q_cmd.cmd.position = this->real_robot_sync_.joint_position;
                q_cmd.cmd.velocity.setZero();
                this->real_robot_sync_.is_updated = false;
                this->manager_buffers_.planner_manager_command->AppendNewElement(q_cmd);
            }
        }
        std::this_thread::sleep_until(next_cycle_time_point);
    }
}

void RobotServer::PublishFeedback() {
    // 发布关节状态反馈
    sensor_msgs::msg::JointState feedback_msg;
    feedback_msg.header.stamp = this->get_clock()->now();

    feedback_msg.name.resize(ndof_robot);
    feedback_msg.position.resize(ndof_robot);
    feedback_msg.velocity.resize(ndof_robot);
    feedback_msg.effort.resize(ndof_robot);

#if TORQUE_CONTROL
    JointRawMeasurement raw_measurement{};
    if (this->robot_buffers_.joint_raw_measurement->PeekLatest(&raw_measurement)) {
        for (int i = 0; i < ndof_robot; ++i) {
            feedback_msg.name[i] = this->robot_model_.params_ptr->joint_names[i];
            feedback_msg.position[i] = raw_measurement.position[i];
            feedback_msg.velocity[i] = raw_measurement.velocity[i];
            feedback_msg.effort[i] = raw_measurement.effort[i];
        }
        feedback_publisher_->publish(feedback_msg);
    }
#else
    JointMeasurement joint_measurement{};
    if (this->manager_buffers_.joint_measurement->PeekLatest(&joint_measurement)) {
        for (int i = 0; i < ndof_robot; ++i) {
            feedback_msg.name[i] = this->robot_model_.params_ptr->joint_names[i];
            feedback_msg.position[i] = joint_measurement.position[i];
            feedback_msg.velocity[i] = joint_measurement.velocity[i];
            feedback_msg.effort[i] = joint_measurement.effort[i];
        }
        feedback_publisher_->publish(feedback_msg);
    }
#endif
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
    PlannerManagerCommand q_cmd{};
    if (!this->manager_buffers_.planner_manager_command->PeekLatest(&q_cmd)) {
        LOG_CRIT << "ControlCallback: Failed to peek latest planner_manager_command, robot not ready";
        return;
    }
    auto robot_control_temp = q_cmd.cmd;
    for (size_t i = 0; i < msg->name.size(); ++i) {
        for (int j = 0; j < ndof_robot; ++j) {
            if (this->robot_model_.params_ptr->joint_names[j] == msg->name[i]) {
                robot_control_temp.position[j] = msg->position[i];
                break;
            }
        }
    }
    if (manager_state_ == Status::Idle) {
        PlannerRequest planner_request;
        planner_request.type = RequestType::StartJointStreaming;
        MoveRequest move_request;
        move_request.motion_group = MotionGroup::WholeBody;
        planner_request.data = move_request;
        streaming_request_id_ = StartStreaming(planner_request);
    }
    if (streaming_request_id_ != -1) {
        StreamingCommandMsg::SharedPtr streaming_msg = std::make_shared<StreamingCommandMsg>();
        streaming_msg->request_id = streaming_request_id_;
        streaming_msg->streaming_finished = false;
        streaming_msg->position =
            std::vector<float>(robot_control_temp.position.begin(), robot_control_temp.position.end());
        this->StreamingCommandCallback(streaming_msg);
    }
}

void RobotServer::PositionControlCallback(const JointPositionControlMsg::SharedPtr msg) {
    PlannerManagerCommand q_cmd{};
    if (!this->manager_buffers_.planner_manager_command->PeekLatest(&q_cmd)) {
        LOG_CRIT << "ControlCallback: Failed to peek latest planner_manager_command, robot not ready";
        return;
    }
    auto robot_control_temp = q_cmd.cmd;
    for (size_t i = 0; i < msg->name.size(); ++i) {
        for (int j = 0; j < ndof_robot; ++j) {
            if (this->robot_model_.params_ptr->joint_names[j] == msg->name[i]) {
                robot_control_temp.position[j] = msg->position[i];
                break;
            }
        }
    }
    if (manager_state_ == Status::Idle) {
        PlannerRequest planner_request;
        planner_request.type = RequestType::StartJointStreaming;
        MoveRequest move_request;
        move_request.motion_group = MotionGroup::WholeBody;
        planner_request.data = move_request;
        streaming_request_id_ = StartStreaming(planner_request);
    }
    if (streaming_request_id_ != -1) {
        StreamingCommandMsg::SharedPtr streaming_msg = std::make_shared<StreamingCommandMsg>();
        streaming_msg->request_id = streaming_request_id_;
        streaming_msg->streaming_finished = false;
        streaming_msg->position =
            std::vector<float>(robot_control_temp.position.begin(), robot_control_temp.position.end());
        this->StreamingCommandCallback(streaming_msg);
    }
}
}  // namespace robot_server_lib
