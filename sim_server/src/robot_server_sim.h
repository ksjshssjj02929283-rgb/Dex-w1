#pragma once

#include "robot_server_lib/robot_server_base.h"
#include "joint_interfaces/msg/joint_control.hpp"
#include "joint_interfaces/msg/joint_position_control.hpp"
#include "joint_interfaces/msg/joint_velocity_torque_limit_feedback.hpp"

namespace robot_server_lib
{

using JointControlMsg = joint_interfaces::msg::JointControl;
using JointPositionControlMsg = joint_interfaces::msg::JointPositionControl;

class RobotServer : public RobotServerBase
{
   public:
    RobotServer(std::filesystem::path robot_cfg_path, bool is_sim);

   protected:
    void RobotThreadFunc() override;
    void ManagerThreadFunc() override;
    void PublishFeedback();
    void ControlCallback(const JointControlMsg::SharedPtr msg);
    void PositionControlCallback(const JointPositionControlMsg::SharedPtr msg);
    Int64 StartStreaming(const PlannerRequest planner_request);

    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr feedback_publisher_;
    rclcpp::Subscription<JointControlMsg>::SharedPtr control_subscriber_;
    rclcpp::Subscription<JointPositionControlMsg>::SharedPtr position_control_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_server_state_real_subscriber_;
    struct
    {
        VectorDof joint_position;
        bool is_updated = false;
    } real_robot_sync_;
    Int64 streaming_request_id_{-1};
    const std::string ecat_joint_name_[ndof_robot] = {"ANKLE",    "KNEE",     "BUTTOCK",  "WAIST",    "NECK1",    "NECK2",   "LEFT_J1",
                                                    "LEFT_J2",  "LEFT_J3",  "LEFT_J4",  "LEFT_J5",  "LEFT_J6",  "LEFT_J7", "RIGHT_J1",
                                                    "RIGHT_J2", "RIGHT_J3", "RIGHT_J4", "RIGHT_J5", "RIGHT_J6", "RIGHT_J7"};
};

}  // namespace robot_server_lib