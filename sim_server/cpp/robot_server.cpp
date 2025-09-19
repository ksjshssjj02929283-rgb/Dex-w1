#include "robot_server.h"

namespace robot_server_lib
{

RobotServer::RobotServer(std::filesystem::path robot_cfg_path, bool is_sim) : RobotServerBase(robot_cfg_path, is_sim) { this->Run(); }

void RobotServer::InitExtraInterfaces()
{
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
}

void RobotServer::RobotThreadFunc()
{
#if TORQUE_CONTROL
    RCLCPP_INFO(this->get_logger(), "Torque control is enabled.");
    LOG_INFO << "Torque control is enabled.";

    pinocchio::Data robot_data(*this->robot_model_.model_ptr);
    VectorDof torque_cmd{};
    JointRawMeasurement raw_measurement{};
    raw_measurement.position << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, -1.2, 0.0, -1.0, 0.0, -0.5, -0.5, -0.5, 1.2, 0.0, 1.0, 0.0, 0.5, 0.5;
    raw_measurement.velocity.setZero();
    raw_measurement.effort.setZero();
    raw_measurement.sensor_wrench = Eigen::VectorXd::Zero(12);
    raw_measurement.time_point = TimePointSystem::Now();
    this->robot_buffers_.joint_raw_measurement->AppendNewElement(raw_measurement);

    auto next_cycle_time_point = std::chrono::steady_clock::now();
    while (rclcpp::ok())
    {
        next_cycle_time_point += std::chrono::microseconds(simulation_dt_micro);
        if (!this->robot_buffers_.torque_command_vector->PeekLatest(&torque_cmd))
        {
            torque_cmd.setZero();
        }
        VectorDof ddq = dynamics::ForwardDynamics(*this->robot_model_.model_ptr, robot_data, raw_measurement.position, raw_measurement.velocity,
                                                  torque_cmd, this->robot_model_.params_ptr->joint_command_sequence_by_name);
        raw_measurement.position += simulation_dt * raw_measurement.velocity;
        raw_measurement.velocity += simulation_dt * ddq;
        raw_measurement.effort = torque_cmd;
        raw_measurement.sensor_wrench = utils::GetSensorWrench(latest_wrench_primary_, latest_wrench_secondary_);
        raw_measurement.time_point = TimePointSystem::Now();
        this->robot_buffers_.joint_raw_measurement->AppendNewElement(raw_measurement);
        std::this_thread::sleep_until(next_cycle_time_point);
    }
#else
    RCLCPP_INFO(this->get_logger(), "Torque control is disabled.");
    LOG_INFO << "Torque control is disabled.";

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
#endif
}

void RobotServer::ManagerThreadFunc()
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
        if (this->real_robot_sync_.is_updated && this->manager_buffers_.planner_manager_command->PeekLatest(&q_cmd))
        {
            q_cmd.cmd.position = this->real_robot_sync_.joint_position;
            q_cmd.cmd.velocity.setZero();
            this->real_robot_sync_.is_updated = false;
        }
        std::this_thread::sleep_until(next_cycle_time_point);
    }
}
}  // namespace robot_server_lib
