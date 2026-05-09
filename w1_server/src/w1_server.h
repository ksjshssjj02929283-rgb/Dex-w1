#pragma once

#include <atomic>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>  // 新增

#include "chassis_interfaces/msg/chassis_motor_control.hpp"
#include "chassis_interfaces/msg/chassis_whole_control.hpp"
#include "joint_interfaces/msg/joint_control.hpp"
#include "joint_interfaces/msg/joint_position_control.hpp"
#include "joint_interfaces/msg/joint_velocity_torque_limit_feedback.hpp"
#include "robot_server_lib/NanoLog.hpp"
#include "robot_server_lib/butterworth_filter.h"
#include "robot_server_lib/ei_input_shaper.h"
#include "robot_server_lib/robot_server_base.h"
#include "robot_server_lib/sri_force_sensor.h"
#include "robot_server_lib/utils.h"
#include "shared_memory_def.h"
#include "w1_utils.h"

namespace robot_server_lib {

enum class RobotMode { W1, A1, SingleArm, DualArm, Unknown };

using JointControlMsg = joint_interfaces::msg::JointControl;
using JointPositionControlMsg = joint_interfaces::msg::JointPositionControl;
using ChassisWholeControlMsg = chassis_interfaces::msg::ChassisWholeControl;
using ChassisMotorControlMsg = chassis_interfaces::msg::ChassisMotorControl;
using JointVelocityTorqueLimitFeedbackMsg = joint_interfaces::msg::JointVelocityTorqueLimitFeedback;
using SetProperty = robot_server_interfaces::srv::SetProperty;

class RobotServer : public RobotServerBase {
public:
    RobotServer(std::filesystem::path robot_cfg_path, bool is_sim);

private:
    void TorqueControlThreadFunc() override;
    void PublishThreadFunc() override;
    Int64 StartStreaming(const PlannerRequest planner_request);
    void SwitchControlWord();

    void ControlCallback(const JointControlMsg::SharedPtr msg);
    void PositionControlCallback(const JointPositionControlMsg::SharedPtr msg);
    void OmniChassisControlCallback(const ChassisWholeControlMsg::SharedPtr msg);
    void OmniChassisMotorControlCallback(const ChassisMotorControlMsg::SharedPtr msg);
    void DiffChassisControlCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void ForceArrayCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void DebugCallback(const std_msgs::msg::String::SharedPtr msg);
    void PublishFeedback();
    std_msgs::msg::String GetRobotServerStateMsg();

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr feedback_publisher_;
    rclcpp::Publisher<JointVelocityTorqueLimitFeedbackMsg>::SharedPtr velocity_torque_limit_feedback_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr force_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr debug_subscriber_;
    rclcpp::Subscription<JointControlMsg>::SharedPtr control_subscriber_;
    rclcpp::Subscription<JointPositionControlMsg>::SharedPtr position_control_subscriber_;
    rclcpp::Subscription<ChassisWholeControlMsg>::SharedPtr omni_chassis_control_subscriber_;
    rclcpp::Subscription<ChassisMotorControlMsg>::SharedPtr omni_chassis_motor_control_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr diff_chassis_control_subscriber_;

    Int64 streaming_request_id_{-1};
    bool is_cmd_valid_{false};
    RobotControl robot_control_;
    RobotFeedback robot_feedback_;
    PlannerManagerInfo manager_info_{};
    PlannerManagerCommand planner_cmd_{};
    bool is_torque_limit_reached_[ndof_robot] = {false};  // 达到力矩限制标志位

    RunMode previous_mode_{RunMode::Position};
    int switch_counter_ = 0;
    const int max_switch_count_ = 200;              // 切换控制模式时的位置保持周期数
    const double position_diff_threshold_ = 1.578;  // /control/joint 位置控制命令阈值，超过该值认为是异常命令
    double streaming_timeout_ = 0.5;                // /control/joint_position streaming 超时时间
    VectorDof joint_position_before_filter_ = VectorDof::Zero();
    VectorDof joint_position_after_filter_ = VectorDof::Zero();

    // 机器人模式支持
    RobotMode robot_mode_ = RobotMode::W1;         // 当前机器人模式
    int ndof_config_;                              // 从配置读取的实际关节数 (W1=20, A1=14, Arm=7)
    bool robot_mode_auto_detected_ = false;        // 模式是否已自动检测
    bool has_logged_unknown_mode_ = false;         // 是否已记录未知模式警告
    bool has_logged_sort_feedback_error_ = false;  // 是否已记录反馈排序错误警告

    // 根据反馈数据自动检测机器人模式
    RobotMode DetectRobotModeFromFeedback(const RobotFeedback& feedback);

    // 判断是否为手臂关节
    bool IsArmJoint(int joint_index) const;

    // 获取当前模式下第一个手臂关节在全关节列表中的索引
    int GetFirstJointIndex() const;

    // 虚拟关节转换函数：根据模式从虚拟控制提取实际控制数据并写入关节名称
    void SortControlByJointName(RobotControl& control);

    // 根据关节名称重新排列反馈数据到标准顺序
    bool SortFeedbackByJointName(RobotFeedback& feedback);

    // 底盘控制目标变量（不直接写入共享内存）
    struct {
        int chassis_mode = 0;
        int chassis_whole_mode = 0;
        double chassis_velocity = 0.0;
        double chassis_direction = 0.0;
        double chassis_left_velocity = 0.0;
        double chassis_right_velocity = 0.0;
        double chassis_motor_velocity[OMNI_MOTOR_NUM] = {0};
        double chassis_motor_direction[OMNI_MOTOR_NUM] = {0};
        int chassis_estop = 0;
        int chassis_neutral = 0;
    } chassis_control_target_;

    // 力传感器相关
    Vector3d sensor_force_;
    Vector12d sensor_wrench_;
    std::shared_ptr<SriForceSensor> primary_sensor_;
    std::shared_ptr<SriForceSensor> secondary_sensor_;  // 暂未使用
    bool primary_sensor_ready_ = false;
    bool secondary_sensor_ready_ = false;

    // 位置指令线性插值器 (1kHz)
    utils::LinearInterpolator line_interpolator_;

    // 巴特沃斯滤波器数组 (1kHz 线程中应用)
    std::shared_ptr<ButterworthFilter> butterworth_filter_[ndof_robot];

    // 输入整形器
    std::shared_ptr<EIInputShaper> chassis_shaper_;
    EIInputShaper::Type chassis_shaper_type_ = EIInputShaper::EI;  // 输入整形器类型
    const double DiffChassisWidth = 0.4390;
    const double DiffChassisWheelD = 0.1730;    // 差速轮径，用于转RPM
    const double DiffChassisNatureFreq = 1.67;  // 底盘输入整形器自然频率，单位Hz
    // ramp率转换: 0.5m/s每周期
    const double DiffChassisMaxRampRpm = 0.5 / (M_PI * DiffChassisWheelD) * 60.0;
    std::shared_ptr<EIInputShaper> input_shapers_[ndof_robot];
    EIInputShaper::Type input_shaper_type_ = EIInputShaper::EI;  // 输入整形器类型
    const double input_shaper_max_ramp_rad_per_sec = 10 * M_PI;
};

}  // namespace robot_server_lib