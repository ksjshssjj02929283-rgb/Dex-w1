#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <numeric>
#include <std_msgs/msg/float64_multi_array.hpp>  // 新增

#include "joint_interfaces/msg/joint_command.hpp"
#include "joint_interfaces/msg/joint_position_control.hpp"
#include "parallel_ankle_calc.h"
#include "robot_server_lib/robot_server_base.h"
#include "shared_memory_def.h"
#include "utils.h"

namespace robot_server_lib {

using JointCommandMsg = joint_interfaces::msg::JointCommand;
using SetProperty = robot_server_interfaces::srv::SetProperty;

class RobotServer : public RobotServerBase {
public:
    RobotServer(std::filesystem::path robot_cfg_path, bool is_sim);

private:
    void JointControlThreadFunc();
    void JointStatePublisherThreadFunc();  // 100Hz关节状态发布线程

    // 判断关节name是否在joint_minus_中，如果在则返回true
    bool IsJointInMinusList(const std::string& joint_name) {
        for (const auto& name : joint_minus_) {
            if (name == joint_name) {
                return true;
            }
        }
        return false;
    }

    void JointCommandCallback(const joint_interfaces::msg::JointCommand::SharedPtr msg);
    void JointPositionControlCallback(const joint_interfaces::msg::JointPositionControl::SharedPtr msg);
    void JointStatePublisher();
    void WriteFeedbackToControl();
    RobotFeedback ConvertAnkleMotorsToPitchRoll(const RobotFeedback& robot_feedback);
    RobotControl ConvertPitchRollToAnkleMotors(const RobotControl& robot_control);
    void DebugCallback(const std_msgs::msg::String::SharedPtr msg);
    std::string InvertJointSignFromJsonString(std::string status_string);
    std::string GetRobotStatusString();

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_status_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr debug_subscriber_;
    rclcpp::Subscription<joint_interfaces::msg::JointCommand>::SharedPtr joint_command_subscriber_;
    rclcpp::Subscription<joint_interfaces::msg::JointPositionControl>::SharedPtr joint_position_control_subscriber_;

    RobotControl robot_control_;
    RobotFeedback robot_feedback_;
    double ankle_motor_torque_[4];

    builtin_interfaces::msg::Time last_command_timestamp_{};
    int command_timeout_ms_ = 200;

    Int64 streaming_request_id_{-1};
    const int ndof_body_ = 18;  // 机器人上半身自由度
    const int ndof_legs_ = 12;  // 腿部自由度

    std::string joint_name_[30] = {
        "HEAD_YAW_J1",        "HEAD_PITCH_J2",    "SHOULDER_PITCH_R_J1", "SHOULDER_ROLL_R_J2", "ELBOW_PITCH_R_J3",
        "ELBOW_YAW_R_J4",     "WRIST_PITCH_R_J5", "WRIST_YAW_R_J6",      "WRIST_ROLL_R_J7",    "SHOULDER_PITCH_L_J1",
        "SHOULDER_ROLL_L_J2", "ELBOW_PITCH_L_J3", "ELBOW_YAW_L_J4",      "WRIST_PITCH_L_J5",   "WRIST_YAW_L_J6",
        "WRIST_ROLL_L_J7",    "WAIST_YAW",        "WAIST_ROLL",          "HIP_PITCH_R_J1",     "HIP_ROLL_R_J2",
        "HIP_YAW_R_J3",       "KNEE_PITCH_R_J4",  "ANKLE_PITCH_R_J5",    "ANKLE_ROLL_R_J6",    "HIP_PITCH_L_J1",
        "HIP_ROLL_L_J2",      "HIP_YAW_L_J3",     "KNEE_PITCH_L_J4",     "ANKLE_PITCH_L_J5",   "ANKLE_ROLL_L_J6"};

    std::string joint_minus_[30] = {"HEAD_PITCH_J2",   "SHOULDER_PITCH_R_J1", "ELBOW_PITCH_R_J3", "WRIST_PITCH_R_J5",
                                    "WAIST_YAW",       "WAIST_ROLL",          "HIP_PITCH_L_J1",   "HIP_ROLL_L_J2",
                                    "KNEE_PITCH_L_J4", "ANKLE_PITCH_L_J5",    "HIP_ROLL_R_J2",    "ANKLE_PITCH_R_J5"};

    double torque_limit_Nm_[18] = {87.0, 87.0, 10.0, 10.0, 31.0, 31.0, 31.0, 31.0, 10.0,
                                   10.0, 10.0, 31.0, 31.0, 31.0, 31.0, 10.0, 10.0, 10.0};
    double legs_stiffness_[12]{0.0};
    double legs_damping_[12]{0.0};

    bool clear_error_flag_{false};

    std::thread joint_state_publisher_thread_;  // 100Hz关节状态发布线程

    utils::PMAnkle pm_ankle_left_{true};
    utils::PMAnkle pm_ankle_right_{false};
};

}  // namespace robot_server_lib
