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

enum class RobotMode { W1, A1, Arm };

using JointControlMsg = joint_interfaces::msg::JointControl;
using JointPositionControlMsg = joint_interfaces::msg::JointPositionControl;
using ChassisWholeControlMsg = chassis_interfaces::msg::ChassisWholeControl;
using ChassisMotorControlMsg = chassis_interfaces::msg::ChassisMotorControl;
using JointVelocityTorqueLimitFeedbackMsg = joint_interfaces::msg::JointVelocityTorqueLimitFeedback;
using SetProperty = robot_server_interfaces::srv::SetProperty;

class RobotServer : public RobotServerBase {
public:
    RobotServer(std::filesystem::path robot_cfg_path, bool is_sim, RobotMode robot_mode = RobotMode::W1);

private:
    void TorqueControlThreadFunc() override;
    void PublishThreadFunc() override;
    Int64 StartStreaming(const PlannerRequest planner_request);
    void InitParamConfig();  // 初始化/读写 torque_limit_ 的 JSON 配置
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
    // W1 模式下的默认关节名称，A1 模式下会动态调整
    std::string ecat_joint_name_[ndof_robot] = {"ANKLE",    "KNEE",     "BUTTOCK",  "WAIST",    "NECK1",
                                                "NECK2",    "LEFT_J1",  "LEFT_J2",  "LEFT_J3",  "LEFT_J4",
                                                "LEFT_J5",  "LEFT_J6",  "LEFT_J7",  "RIGHT_J1", "RIGHT_J2",
                                                "RIGHT_J3", "RIGHT_J4", "RIGHT_J5", "RIGHT_J6", "RIGHT_J7"};
    struct {
        double positive_limit_Nm[ndof_robot]{100.0, 100.0, 100.0, 100.0, 10.0, 10.0, 31.0, 31.0, 31.0, 31.0,
                                             10.0,  10.0,  10.0,  31.0,  31.0, 31.0, 31.0, 10.0, 10.0, 10.0};
        double negative_limit_Nm[ndof_robot]{100.0, 100.0, 100.0, 100.0, 10.0, 10.0, 31.0, 31.0, 31.0, 31.0,
                                             10.0,  10.0,  10.0,  31.0,  31.0, 31.0, 31.0, 10.0, 10.0, 10.0};
    } torque_limit_;  // 关节力矩限制参数默认值, 单位: Nm

    RunMode previous_mode_{RunMode::Position};
    int switch_counter_ = 0;
    const int max_switch_count_ = 200;                // 切换控制模式时的位置保持周期数
    bool enable_ruckig_ = true;                       // /control/joint接口是否使用 Ruckig 进行轨迹规划
    bool enable_butterworth_filter_[ndof_robot];      // 每个关节是否启用巴特沃斯滤波
    bool enable_record_ = false;                      // 是否启用调试日志
    std::atomic<bool> enable_chassis_shaper_{false};  // 是否启用底盘速度输入整形器
    std::atomic<bool> enable_torso_shaper_{false};    // 是否启用腿部 (ANKLE, KNEE) 输入整形器
    int first_arm_joint_index_ = 6;  // 左臂第一个关节在整体关节数组中的索引 (W1:6, A1:0)
    const double position_diff_threshold_ = 1.578;  // /control/joint 位置控制命令阈值，超过该值认为是异常命令
    double streaming_timeout_ = 0.5;                // /control/joint_position streaming 超时时间

    // 整形器开关状态追踪变量，用于检测状态变化以实现自动重置
    bool prev_chassis_shaper_enabled_ = false;
    bool prev_torso_shaper_enabled_ = false;

    // 机器人模式支持
    RobotMode robot_mode_ = RobotMode::W1;  // 当前机器人模式
    int ndof_config_;                       // 从配置读取的实际关节数 (W1=20, A1=14, Arm=7)

    // 虚拟关节转换函数（A1/Arm 模式使用）
    void Expand14ToVirtual20(const RobotFeedback& real_feedback, RobotFeedback& virtual_feedback);
    void Expand7ToVirtual20(const RobotFeedback& real_feedback, RobotFeedback& virtual_feedback);
    void ExtractVirtual20To14(const RobotControl& virtual_control, RobotControl& real_control);
    void ExtractVirtual20To7(const RobotControl& virtual_control, RobotControl& real_control);

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

    // 底盘速度输入整形器
    std::unique_ptr<EIInputShaper> chassis_shaper_;
    EIInputShaper::Type chassis_shaper_type_ = EIInputShaper::EI;  // 输入整形器类型
    const double DiffChassisWidth = 0.4390;
    const double DiffChassisWheelD = 0.1730;    // 差速轮径，用于转RPM
    const double DiffChassisNatureFreq = 1.67;  // 底盘输入整形器自然频率，单位Hz
    // ramp率转换: 0.5m/s每周期
    const double DiffChassisMaxRampRpm = 0.5 / (M_PI * DiffChassisWheelD) * 60.0;
    // 躯干输入整形器数组 (J1-J4)
    std::unique_ptr<EIInputShaper> torso_shapers_[4];
    EIInputShaper::Type torso_shaper_type_ = EIInputShaper::EI;
    const double TorsoShaperMaxRampRadPerSec = 0.5;  // 最大角速度 ramp率，单位 rad/s 每周期
    // 躯干频率参数 (可配置)
    double TorsoJ1NatureFreq = 3.0;  // torso J1 自然频率，单位Hz
    double TorsoJ2NatureFreq = 2.5;  // torso J2 自然频率，单位Hz
    double TorsoJ3NatureFreq = 2.5;  // torso J3 自然频率，单位Hz
    double TorsoJ4NatureFreq = 4.5;  // torso J4 自然频率，单位Hz

    // 力传感器相关
    Vector3d sensor_force_;
    Vector12d sensor_wrench_;
    std::shared_ptr<SriForceSensor> primary_sensor_;
    std::shared_ptr<SriForceSensor> secondary_sensor_;  // 暂未使用
    bool primary_sensor_ready_ = false;
    bool secondary_sensor_ready_ = false;

    // 位置指令线性插值器 (250Hz -> 1kHz)
    utils::LinearInterpolator line_interpolator_;

    // 巴特沃斯滤波器数组 (1kHz 线程中应用)
    std::shared_ptr<ButterworthFilter> butterworth_filter_[ndof_robot];
};

}  // namespace robot_server_lib