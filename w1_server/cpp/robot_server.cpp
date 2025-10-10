#include "robot_server.h"

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

    // 创建 /control/cmd_vel 订阅器
    diff_chassis_control_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/control/cmd_vel", 10, std::bind(&RobotServer::DiffChassisControlCallback, this, std::placeholders::_1));

    // 创建定时器，定时发布feedback 定时器回调函数
    control_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() { PublishFeedback(); });
    
    this->Run(); 
}

void RobotServer::RobotThreadFunc()
{
    RCLCPP_INFO(this->get_logger(), "Torque control is disabled.");
    LOG_INFO << "Torque control is disabled.";
    // 初始化共享内存
    auto& feedback_shm_instance = FeedbackSharedMemory::GetInstance();
    auto& control_shm_instance = ControlSharedMemory::GetInstance();
    PlannerManagerCommand q_cmd{};
    JointRawMeasurement raw_measurement{};
    auto next_cycle_time_point = std::chrono::steady_clock::now();

    while (rclcpp::ok())
    {
        next_cycle_time_point += std::chrono::microseconds(torque_ctrl_dt_micro);
        raw_measurement.sensor_wrench = utils::GetSensorWrench(latest_wrench_primary_, latest_wrench_secondary_);
        // 读取反馈数据
        RobotFeedback feedback_data{};
        bool ret = feedback_shm_instance.PeekLatest(&feedback_data);
        bool is_robot_ready_ = feedback_data.robot_status_word & STATUSWORD_MOTOR_READY;
        robot_control_.robot_control_word = CTRLWORD_MOTOR_SWITCH_ON | CTRLWORD_MOTOR_CLEAR_FAULT;

        if (is_robot_ready_)
        {
            // 读取关节测量数据
            for (int i = 0; i < ndof_robot; ++i)
            {
                raw_measurement.position[i] = feedback_data.position[i];
                raw_measurement.velocity[i] = feedback_data.velocity[i];
                raw_measurement.effort[i] = feedback_data.torque[i];
                raw_measurement.time_point = TimePointSystem::Now();
            }
            this->robot_buffers_.joint_raw_measurement->AppendNewElement(raw_measurement);

            // 写入控制数据
            if (this->manager_buffers_.planner_manager_command->PeekLatest(&q_cmd))
            {
                for (int i = 0; i < ndof_robot; ++i)
                {
                    // 更新关节测量数据
                    robot_control_.position[i] = q_cmd.cmd.position[i];
                    robot_control_.velocity[i] = q_cmd.cmd.velocity[i];
                }
                robot_control_.robot_control_word = CTRLWORD_MOTOR_SWITCH_ON | CTRLWORD_MOTOR_ENABLE | CTRLWORD_MOTOR_CLEAR_FAULT;
            }
        }

        control_shm_instance.AppendNewElement(robot_control_);
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
            feedback_msg.name[i] = ecat_joint_name[i];
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
        limit_msg.name[i] = ecat_joint_name[i];
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
        RCLCPP_WARN(this->get_logger(), "Still existed previous move request, failed to queue start streaming request");
        LOG_CRIT << "Still existed previous move request, failed to queue start streaming request";
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
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        for (int j = 0; j < ndof_robot; ++j)
        {
            if (ecat_joint_name[j] == msg->name[i])
            {
                robot_control_temp.position[j] = msg->position[i];
                robot_control_.positive_velocity_limit[j] = msg->positive_velocity_limit[i];
                robot_control_.negative_velocity_limit[j] = msg->negative_velocity_limit[i];
                robot_control_.positive_torque_limit[j] = msg->positive_torque_limit[i];
                robot_control_.negative_torque_limit[j] = msg->negative_torque_limit[i];
                break;
            }
        }
    }
    if (manager_state_ == Status::Idle)
    {
        PlannerRequest planner_request;
        planner_request.type = RequestType::StartJointStreaming;
        MoveRequest move_request;
        move_request.motion_group = MotionGroup::WholeBody;
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

void RobotServer::PositionControlCallback(const JointPositionControlMsg::SharedPtr msg)
{
    auto robot_control_temp = robot_control_;
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        for (int j = 0; j < ndof_robot; ++j)
        {
            if (ecat_joint_name[j] == msg->name[i])
            {
                robot_control_temp.position[j] = msg->position[i];
                break;
            }
        }
    }
    if (manager_state_ == Status::Idle)
    {
        PlannerRequest planner_request;
        planner_request.type = RequestType::StartJointStreaming;
        MoveRequest move_request;
        move_request.motion_group = MotionGroup::WholeBody;
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

void RobotServer::OmniChassisControlCallback(const ChassisWholeControlMsg::SharedPtr msg)
{
    robot_control_.chassis_mode = 0;  // 设置为全向整车模式
    robot_control_.chassis_whole_mode = msg->mode;
    robot_control_.chassis_velocity = msg->speed;
    robot_control_.chassis_direction = msg->direction;
}

void RobotServer::DiffChassisControlCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    robot_control_.chassis_mode = 1;  // 设置为差速模式
    robot_control_.chassis_right_velocity = msg->linear.x + msg->angular.z * DiffChassisWidth * 0.5;
    robot_control_.chassis_left_velocity = msg->linear.x - msg->angular.z * DiffChassisWidth * 0.5;
}

}  // namespace robot_server_lib
