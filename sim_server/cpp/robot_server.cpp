#include "robot_server.h"

namespace robot_server_lib
{

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
