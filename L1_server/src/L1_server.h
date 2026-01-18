#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>  // 新增

#include "joint_interfaces/msg/joint_command.hpp"
#include "robot_server_lib/robot_server_base.h"
#include "shared_memory_def.h"
#include "utils.h"

namespace robot_server_lib
{

using JointCommandMsg = joint_interfaces::msg::JointCommand;
using SetProperty = robot_server_interfaces::srv::SetProperty;


class RobotServer : public RobotServerBase
{
   public:
    RobotServer(std::filesystem::path robot_cfg_path, bool is_sim);

   private:
    void JointControlThreadFunc();

    void JointCommandCallback(const joint_interfaces::msg::JointCommand::SharedPtr msg);
    void JointStatePublisher();
    void WriteFeedbackToControl();
    void DebugCallback(const std_msgs::msg::String::SharedPtr msg);

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr debug_subscriber_;
    rclcpp::Subscription<joint_interfaces::msg::JointCommand>::SharedPtr joint_command_subscriber_;

    RobotControl robot_control_;
    RobotFeedback robot_feedback_;

    Int64 streaming_request_id_{-1};
    const int ndof_body_ = 16;                    // 机器人上半身自由度
    const int ndof_legs_ = 12;                      // 腿部自由度

    double legs_stiffness_[12] = {250.0};
    double legs_damping_[12] = {1.0};
    bool clear_error_flag_{false};
};

}  // namespace robot_server_lib