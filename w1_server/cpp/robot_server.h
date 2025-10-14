#pragma once

#include <geometry_msgs/msg/twist.hpp>

#include "chassis_interfaces/msg/chassis_whole_control.hpp"
#include "joint_interfaces/msg/joint_control.hpp"
#include "joint_interfaces/msg/joint_position_control.hpp"
#include "joint_interfaces/msg/joint_velocity_torque_limit_feedback.hpp"
#include "robot_server_lib/robot_server_base.h"
#include "shared_memory_def.h"

#define CTRLWORD_MOTOR_SWITCH_ON      (1U << 0)
#define CTRLWORD_MOTOR_ENABLE         (1U << 1)
#define CTRLWORD_MOTOR_QUICK_STOP     (1u << 2)
#define CTRLWORD_MOTOR_CLEAR_FAULT    (1U << 3)
#define CTRLWORD_MOTOR_CST_MODE       (1U << 4)

#define STATUSWORD_MOTOR_READY        (1U << 0)
#define STATUSWORD_MOTOR_ENABLED      (1U << 1)
#define STATUSWORD_MOTOR_FAULT        (1U << 2)
#define STATUSWORD_MOTOR_QUICK_STOP   (1U << 3)

namespace robot_server_lib
{

constexpr double DiffChassisWidth = 0.4390;
using JointControlMsg = joint_interfaces::msg::JointControl;
using JointPositionControlMsg = joint_interfaces::msg::JointPositionControl;
using ChassisWholeControlMsg = chassis_interfaces::msg::ChassisWholeControl;
using JointVelocityTorqueLimitFeedbackMsg = joint_interfaces::msg::JointVelocityTorqueLimitFeedback;

class RobotServer : public RobotServerBase
{
   public:
    RobotServer(std::filesystem::path robot_cfg_path, bool is_sim);

   protected:
    void RobotThreadFunc() override;
    Int64 StartStreaming(const PlannerRequest planner_request);

    void ControlCallback(const JointControlMsg::SharedPtr msg);
    void PositionControlCallback(const JointPositionControlMsg::SharedPtr msg);
    void OmniChassisControlCallback(const ChassisWholeControlMsg::SharedPtr msg);
    void DiffChassisControlCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    void PublishFeedback();

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr feedback_publisher_;
    rclcpp::Publisher<JointVelocityTorqueLimitFeedbackMsg>::SharedPtr velocity_torque_limit_feedback_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Subscription<JointControlMsg>::SharedPtr control_subscriber_;
    rclcpp::Subscription<JointPositionControlMsg>::SharedPtr position_control_subscriber_;
    rclcpp::Subscription<ChassisWholeControlMsg>::SharedPtr omni_chassis_control_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr diff_chassis_control_subscriber_;

    Int64 streaming_request_id_{-1};
    RobotControl robot_control_;
    const std::string ecat_joint_name[ndof_robot] = {"ANKLE",    "KNEE",     "BUTTOCK",  "WAIST",    "NECK1",    "NECK2",   "LEFT_J1",
                                                     "LEFT_J2",  "LEFT_J3",  "LEFT_J4",  "LEFT_J5",  "LEFT_J6",  "LEFT_J7", "RIGHT_J1",
                                                     "RIGHT_J2", "RIGHT_J3", "RIGHT_J4", "RIGHT_J5", "RIGHT_J6", "RIGHT_J7"};
};

}  // namespace robot_server_lib