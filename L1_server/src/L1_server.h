#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>  // 新增

#include "joint_interfaces/msg/joint_position_control.hpp"
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
    void JointPositionControlCallback(const joint_interfaces::msg::JointPositionControl::SharedPtr msg);
    void JointStatePublisher();
    void WriteFeedbackToControl();
    void DebugCallback(const std_msgs::msg::String::SharedPtr msg);
    std::string GetRobotStatusString();

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr debug_subscriber_;
    rclcpp::Subscription<joint_interfaces::msg::JointCommand>::SharedPtr joint_command_subscriber_;
    rclcpp::Subscription<joint_interfaces::msg::JointPositionControl>::SharedPtr joint_position_control_subscriber_;

    RobotControl robot_control_;
    RobotFeedback robot_feedback_;

    Int64 streaming_request_id_{-1};
    const int ndof_body_ = 18;                    // 机器人上半身自由度
    const int ndof_legs_ = 12;                      // 腿部自由度

    std::string joint_name_[30] = {"BUTTOCK","WAIST","NECK1","NECK2",
        "LEFT_J1",  "LEFT_J2",  "LEFT_J3",  "LEFT_J4", "LEFT_J5",  "LEFT_J6",  "LEFT_J7",
        "RIGHT_J1", "RIGHT_J2", "RIGHT_J3", "RIGHT_J4", "RIGHT_J5", "RIGHT_J6", "RIGHT_J7",
        "LEFT_LEG_J1",  "LEFT_LEG_J2",  "LEFT_LEG_J3",  "LEFT_LEG_J4", "LEFT_LEG_J5",  "LEFT_LEG_J6",
        "RIGHT_LEG_J1", "RIGHT_LEG_J2", "RIGHT_LEG_J3", "RIGHT_LEG_J4", "RIGHT_LEG_J5", "RIGHT_LEG_J6"};

    double legs_stiffness_[12] = {25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0};
    double legs_damping_[12]   = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0};

    bool clear_error_flag_{false};
};

}  // namespace robot_server_lib