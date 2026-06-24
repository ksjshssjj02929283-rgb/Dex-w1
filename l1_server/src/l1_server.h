#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <numeric>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "NanoLog.hpp"
#include "joint_interfaces/msg/joint_command.hpp"
#include "parallel_ankle_calc.h"
#include "rclcpp/rclcpp.hpp"
#include "shared_memory_def.h"
#include "utils.h"

namespace robot_server_lib {

using JointCommandMsg = joint_interfaces::msg::JointCommand;

class RobotServer : public rclcpp::Node {
public:
    RobotServer();

private:
    void JointControlThreadFunc();
    void JointStatePublisherThreadFunc();  // 100Hz关节状态发布线程

    // 获取关节符号：minus 关节返回 -1，其余返回 1
    int GetJointSign(int joint_index) const {
        for (const auto& name : joint_minus_) {
            if (std::strcmp(name, joint_name_[joint_index]) == 0) {
                return -1;
            }
        }
        return 1;
    }

    void JointCommandCallback(const joint_interfaces::msg::JointCommand::SharedPtr msg);
    void JointStatePublisher();
    void WriteFeedbackToControl();
    bool SortFeedbackByJointName(RobotFeedback& feedback);
    RobotFeedback ConvertAnkleMotorsToPitchRoll(const RobotFeedback& robot_feedback);
    RobotControl ConvertPitchRollToAnkleMotors(const RobotControl& robot_control);
    void DebugCallback(const std_msgs::msg::String::SharedPtr msg);
    std::string GetRobotStatusString();
    // 锁内存并设置 SCHED_FIFO 实时调度
    void SetupRealtime();

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_status_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr debug_subscriber_;
    rclcpp::Subscription<joint_interfaces::msg::JointCommand>::SharedPtr joint_command_subscriber_;

    RobotControl robot_control_;
    RobotFeedback robot_feedback_;
    double ankle_motor_torque_[4]{0.0};
    double ankle_motor_position_fb_[4]{0.0};
    double ankle_motor_position_cmd_[4]{0.0};

    builtin_interfaces::msg::Time last_command_timestamp_{};
    int command_timeout_ms_ = 200;

    Int64 streaming_request_id_{-1};
    const int ndof_body_ = 18;  // 机器人上半身自由度
    const int ndof_legs_ = 12;  // 腿部自由度
    const char* joint_name_[30] = {
        "HEAD_YAW_J1",        "HEAD_PITCH_J2",    "SHOULDER_PITCH_L_J1", "SHOULDER_ROLL_L_J2", "ELBOW_PITCH_L_J3",
        "ELBOW_YAW_L_J4",     "WRIST_PITCH_L_J5", "WRIST_YAW_L_J6",      "WRIST_ROLL_L_J7",    "SHOULDER_PITCH_R_J1",
        "SHOULDER_ROLL_R_J2", "ELBOW_PITCH_R_J3", "ELBOW_YAW_R_J4",      "WRIST_PITCH_R_J5",   "WRIST_YAW_R_J6",
        "WRIST_ROLL_R_J7",    "WAIST_YAW",        "WAIST_ROLL",          "HIP_PITCH_L_J1",     "HIP_ROLL_L_J2",
        "HIP_YAW_L_J3",       "KNEE_PITCH_L_J4",  "ANKLE_PITCH_L_J5",    "ANKLE_ROLL_L_J6",    "HIP_PITCH_R_J1",
        "HIP_ROLL_R_J2",      "HIP_YAW_R_J3",     "KNEE_PITCH_R_J4",     "ANKLE_PITCH_R_J5",   "ANKLE_ROLL_R_J6"};

    const char* joint_minus_[10] = {"HEAD_PITCH_J2",   "SHOULDER_PITCH_R_J1", "ELBOW_PITCH_R_J3", "WRIST_PITCH_R_J5",
                                    "WAIST_YAW",       "WAIST_ROLL",          "HIP_PITCH_L_J1",   "HIP_ROLL_L_J2",
                                    "KNEE_PITCH_L_J4", "HIP_ROLL_R_J2"};

    double torque_limit_Nm_[18] = {10.0, 10.0, 31.0, 31.0, 31.0, 31.0, 10.0, 10.0, 10.0,
                                   31.0, 31.0, 31.0, 31.0, 10.0, 10.0, 10.0, 87.0, 87.0};

    std::vector<double> joint_torque_feedforward_{ndof_body_ + ndof_legs_, 0.0};  // 实际下发到共享内存的力矩前馈

    bool clear_error_flag_{false};
    bool is_robot_ready_{false};
    int run_mode_{0};  // 上肢控制模式: 0=位置模式, 1=力矩模式(CST)

    std::thread torque_control_thread_;         // 关节控制线程
    std::thread joint_state_publisher_thread_;  // 100Hz关节状态发布线程

    utils::PMAnkle pm_ankle_left_{true};
    utils::PMAnkle pm_ankle_right_{false};
};

}  // namespace robot_server_lib
