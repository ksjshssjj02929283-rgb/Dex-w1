#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>  // 新增

#include "butterworth_filter.h"
#include "chassis_interfaces/msg/chassis_motor_control.hpp"
#include "chassis_interfaces/msg/chassis_whole_control.hpp"
#include "joint_interfaces/msg/joint_control.hpp"
#include "joint_interfaces/msg/joint_position_control.hpp"
#include "joint_interfaces/msg/joint_velocity_torque_limit_feedback.hpp"
#include "robot_server_lib/force_sensor_reader.h"
#include "robot_server_lib/robot_server_base.h"
#include "shared_memory_def.h"
#include "w1_utils.h"

namespace robot_server_lib
{

using JointControlMsg = joint_interfaces::msg::JointControl;
using JointPositionControlMsg = joint_interfaces::msg::JointPositionControl;
using ChassisWholeControlMsg = chassis_interfaces::msg::ChassisWholeControl;
using ChassisMotorControlMsg = chassis_interfaces::msg::ChassisMotorControl;
using JointVelocityTorqueLimitFeedbackMsg = joint_interfaces::msg::JointVelocityTorqueLimitFeedback;

class RobotServer : public RobotServerBase
{
   public:
    RobotServer(std::filesystem::path robot_cfg_path, bool is_sim);

   private:
    void TorqueControlThreadFunc() override;
    void PublishThreadFunc() override;
    Int64 StartStreaming(const PlannerRequest planner_request);
    void InitParamConfig();  // 初始化/读写 torque_limit_ 的 JSON 配置

    void ControlCallback(const JointControlMsg::SharedPtr msg);
    void PositionControlCallback(const JointPositionControlMsg::SharedPtr msg);
    void OmniChassisControlCallback(const ChassisWholeControlMsg::SharedPtr msg);
    void OmniChassisMotorControlCallback(const ChassisMotorControlMsg::SharedPtr msg);
    void DiffChassisControlCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void ForceArrayCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    void PublishFeedback();

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr feedback_publisher_;
    rclcpp::Publisher<JointVelocityTorqueLimitFeedbackMsg>::SharedPtr velocity_torque_limit_feedback_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Subscription<JointControlMsg>::SharedPtr control_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr force_subscriber_;
    rclcpp::Subscription<JointPositionControlMsg>::SharedPtr position_control_subscriber_;
    rclcpp::Subscription<ChassisWholeControlMsg>::SharedPtr omni_chassis_control_subscriber_;
    rclcpp::Subscription<ChassisMotorControlMsg>::SharedPtr omni_chassis_motor_control_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr diff_chassis_control_subscriber_;

    Int64 streaming_request_id_{-1};
    RobotControl robot_control_;
    RobotFeedback robot_feedback_;
    RobotProperty robot_property_;
    bool is_torque_limit_reached_[ndof_robot] = {false};  // 达到力矩限制标志位
    const std::string ecat_joint_name_[ndof_robot] = {"ANKLE",    "KNEE",     "BUTTOCK",  "WAIST",    "NECK1",    "NECK2",   "LEFT_J1",
                                                      "LEFT_J2",  "LEFT_J3",  "LEFT_J4",  "LEFT_J5",  "LEFT_J6",  "LEFT_J7", "RIGHT_J1",
                                                      "RIGHT_J2", "RIGHT_J3", "RIGHT_J4", "RIGHT_J5", "RIGHT_J6", "RIGHT_J7"};
    struct
    {
        double positive_limit_Nm[ndof_robot]{100.0, 100.0, 100.0, 100.0, 10.0, 10.0, 25.0, 25.0, 25.0, 25.0,
                                             10.0,  10.0,  10.0,  25.0,  25.0, 25.0, 25.0, 10.0, 10.0, 10.0};
        double negative_limit_Nm[ndof_robot]{100.0, 100.0, 100.0, 100.0, 10.0, 10.0, 25.0, 25.0, 25.0, 25.0,
                                             10.0,  10.0,  10.0,  25.0,  25.0, 25.0, 25.0, 10.0, 10.0, 10.0};
    } torque_limit_;  // 关节力矩限制参数默认值, 单位: Nm

    const int first_arm_joint_index_ = 6;           // 左臂第一个关节在整体关节数组中的索引
    const double position_diff_threshold_ = 0.525;  // /control/joint 位置控制命令阈值，超过该值认为是异常命令
    const double DiffChassisWidth = 0.4390;
    std::shared_ptr<ButterworthFilter> butterworth_filter_[ndof_robot];

    // 力传感器相关
    Vector3d sensor_force_;
    Vector12d sensor_wrench_;
    std::shared_ptr<ForceSensorReader> primary_sensor_;
    std::shared_ptr<ForceSensorReader> secondary_sensor_;
    bool primary_sensor_ready_ = false;
    bool secondary_sensor_ready_ = false;
};

}  // namespace robot_server_lib