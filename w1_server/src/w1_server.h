#pragma once

#include <array>
#include <atomic>
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "chassis_command.h"
#include "chassis_safety_monitor.h"
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

/**
 * 机型模式，由 DetectRobotModeFromFeedback() 从反馈中的关节名集合自动识别。
 *
 *   Chassis  0 关节，仅底盘 —— 只装了下半身电机的整机调试场景。
 *            与 A1/SingleArm/DualArm 正好相反：后者是「有上半身没底盘」，
 *            它是「有底盘没上半身」。该模式下不向关节从站下发使能位
 *            （不进 SwitchControlWord），避免对不存在的电机做状态机切换；
 *            底盘控制与 chassis_node 照常工作。
 */
enum class RobotMode { W1, A1, SingleArm, DualArm, Chassis, Unknown };

using JointControlMsg = joint_interfaces::msg::JointControl;
using JointPositionControlMsg = joint_interfaces::msg::JointPositionControl;
using JointVelocityTorqueLimitFeedbackMsg = joint_interfaces::msg::JointVelocityTorqueLimitFeedback;
using SetProperty = robot_server_interfaces::srv::SetProperty;

class RobotServer : public RobotServerBase {
public:
    RobotServer(std::filesystem::path robot_cfg_path, bool is_sim);

    /// 底盘目标通道：main.cpp 取出后交给 ChassisNode，两者共用同一实例
    std::shared_ptr<SingleWriterLatestBuffer<chassis_node::ChassisTarget>> chassis_target_buffer() const {
        return chassis_target_buffer_;
    }

private:
    void TorqueControlThreadFunc() override;
    void PublishThreadFunc() override;
    Int64 StartStreaming(const PlannerRequest planner_request);
    void SwitchControlWord();

    void ControlCallback(const JointControlMsg::SharedPtr msg);
    void PositionControlCallback(const JointPositionControlMsg::SharedPtr msg);
    void ForceArrayCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void DebugCallback(const std_msgs::msg::String::SharedPtr msg);
    void PublishFeedback();
    void PublishImu();
    std_msgs::msg::String GetRobotServerStateMsg();

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr feedback_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<JointVelocityTorqueLimitFeedbackMsg>::SharedPtr velocity_torque_limit_feedback_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr imu_timer_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr force_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr debug_subscriber_;
    rclcpp::Subscription<JointControlMsg>::SharedPtr control_subscriber_;
    rclcpp::Subscription<JointPositionControlMsg>::SharedPtr position_control_subscriber_;

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
    RobotMode robot_mode_ = RobotMode::W1;  // 当前机器人模式
    utils::ChassisType chassis_type_ = utils::ChassisType::DiffDrive;
    int ndof_config_;                        // 从配置读取的实际关节数 (W1=20, A1=14, Arm=7)
    bool robot_mode_auto_detected_ = false;  // 模式是否已自动检测

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

    // 底盘目标通道：由 chassis_node 写入、本节点 1 kHz 线程读取。
    // 单写者无锁缓冲（与 robot_buffers_ 同一范式），避免实时线程与普通优先级线程争互斥量。
    using ChassisTargetBuffer = SingleWriterLatestBuffer<chassis_node::ChassisTarget>;
    std::shared_ptr<ChassisTargetBuffer> chassis_target_buffer_;

    // 底盘安全速度监测：监测器与清错沿检测只由 1 kHz 线程读写；上报状态经原子量给发布线程，实时侧不加锁。
    // 人工复位与电机清错共用 SetProperty ClearError（XWiz「清错恢复」）。
    chassis_node::ChassisSafetyMonitor chassis_safety_;
    bool chassis_clear_error_prev_ = false;
    bool chassis_restart_hold_warned_ = false;  // 复位后待零速的告警每次只打一条
    std::atomic<bool> chassis_safety_latched_{false};
    std::atomic<uint8_t> chassis_safety_fault_{0};
    std::atomic<bool> chassis_safety_awaiting_zero_{false};

    // 上下半身一体：上半身就绪过之后再出现故障或掉线，底盘一起进安全状态，ClearError 一起清。
    // 解耦模式只用于底盘调试：上半身异常不停底盘、底盘不要求关节反馈；
    // robot_config.yaml 的 chassis_config.decouple_upper_lower_body，默认关。
    bool decouple_upper_lower_body_ = false;
    bool upper_body_ready_seen_ = false;

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
    const double DiffChassisNatureFreq = 1.67;                     // 底盘输入整形器自然频率，单位Hz
    // 整形器的 ramp 上限：0.5 m/s 每周期。轮径是执行层整形自己的标定量，
    // 与 chassis_node 的 ChassisGeometry 无关（那边管运动学，这边管滤波器时间常数）。
    static constexpr double kShaperWheelDiameter = 0.1730;  // m
    const double DiffChassisMaxRampRpm = 0.5 / (M_PI * kShaperWheelDiameter) * 60.0;
    std::shared_ptr<EIInputShaper> input_shapers_[ndof_robot];
    EIInputShaper::Type input_shaper_type_ = EIInputShaper::EI;  // 输入整形器类型
    const double input_shaper_max_ramp_rad_per_sec = 10 * M_PI;
    uint32_t imu_timestamp_ = 0;
};

}  // namespace robot_server_lib
