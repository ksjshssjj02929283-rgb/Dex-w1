// ----------------------------------------------------------------------------
// Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
//
// All rights reserved.
// ----------------------------------------------------------------------------

/*----------------------------------------------------------------------------
enum for various states and command types
机器人状态类和指令类枚举
----------------------------------------------------------------------------*/
#pragma once

#include <Eigen/Dense>
#include <array>
#include <Eigen/Geometry>
#include <array>
#include <filesystem>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "bounded_queue.h"
#include "pinocchio_api.h"
#include "single_rw_latest_queue.h"
#include "time_utility.h"
#include "tool_compensation.h"
#include "typedef.h"

namespace robot_server_lib {

/*-------------------------------------------------------------------------------
robot state and request data structure class
机器人状态和指令数据结构
-------------------------------------------------------------------------------*/

struct JointMeasurement {
    VectorDof position;                          // 关节位置
    VectorDof velocity;                          // 关节速度
    VectorDof effort;                            // 关节力矩
    VectorDof q_desired_pre;                     // 预期关节位置
    Vector12d sensor_wrench{Vector12d::Zero()};  // 六维力传感器数据

    std::string ToString() const {
        std::stringstream ss;
        ss << "Position: " << position.transpose() << ", Velocity: " << velocity.transpose()
           << ", Effort: " << effort.transpose() << ", Previous Q Desired: " << q_desired_pre.transpose()
           << ", Sensor_wrench: " << sensor_wrench.transpose();
        return ss.str();
    }
};

struct JointRawMeasurement {
    VectorDof position;                          // 关节位置
    VectorDof velocity;                          // 关节速度
    VectorDof torque;                            // 关节力矩
    VectorDof sensor_torque{VectorDof::Zero()};  // 关节力矩传感器读数(用于外力估算)
    Vector12d sensor_wrench{Vector12d::Zero()};  // 六维力传感器数据
    TimePointSystem time_point;

    std::string ToString() const {
        std::stringstream ss;
        ss << "Position: " << position.transpose() << ", Velocity: " << velocity.transpose()
           << ", Torque: " << torque.transpose() << ", Sensor Torque: " << sensor_torque.transpose()
           << ", Sensor Wrench: " << sensor_wrench.transpose();
        return ss.str();
    }
};

struct JointCommand {
    VectorDof position;
    VectorDof velocity;
    VectorDof acceleration;
    VectorDof torque_feedforward;
    VectorDof joint_stiffness;
    VectorDof joint_damping;
    VectorDof joint_integral_ratio;

    std::string ToString() const {
        std::stringstream ss;
        ss << "Position: " << position.transpose() << ", Velocity: " << velocity.transpose()
           << ", Acceleration: " << acceleration.transpose()
           << ", Torque Feedforward: " << torque_feedforward.transpose()
           << ", Integral Ratio: " << joint_integral_ratio.transpose();
        return ss.str();
    }
};

// 安全检查配置结构体
struct SafetyCheckConfig {
    bool check_position_limit{true};  // 是否检查位置限位
    bool check_velocity_limit{true};  // 是否检查速度限位
    bool check_torque_limit{true};    // 是否检查力矩限位
};

struct TrajectoryBounds {
    double max_velocity{1.0};
    double max_acceleration{2.0};
    double max_jerk{100000.0};

    std::string ToString() const {
        std::stringstream ss;
        ss << "[" << max_velocity << ", " << max_acceleration << ", " << max_jerk << "]";
        return ss.str();
    }
};

// 关节导纳控制的单关节配置: 描述虚拟二阶系统 (ddq + D·dq + K·q = torque_ext, M=1) 的参数
struct JointAdmittanceJointConfig {
    bool enabled{false};               // 是否对该关节启用导纳
    double natural_frequency_hz{1.5};  // 自然频率 fn (Hz), 决定刚度 K = (2π·fn)²
    double damping_ratio{1.2};         // 阻尼比 ζ, 决定阻尼 D = 2·ζ·(2π·fn), 1.0=临界阻尼
    double offset_limit{0.05};         // 位置偏移限幅 (rad), 防止偏移过大
    double velocity_limit{0.4};        // 速度偏移限幅 (rad/s)
    double acceleration_limit{5.0};    // 加速度偏移限幅 (rad/s²)
};

// 关节导纳控制全局配置
struct JointAdmittanceConfig {
    std::array<JointAdmittanceJointConfig, ndof_robot> joints{};
};

struct RobotProperty {
    pinocchio::SE3 tcp_frame{Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()};
    pinocchio::SE3 tcp_frame_secondary{Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()};
    RunMode run_mode{RunMode::Position};
    double global_speed_ratio{1.0};
    double safe_level{0.5};
    bool clear_error_flag{false};
    bool allow_torque_control{false};
    bool enable_cartesian_impedance{false};
    SafetyCheckConfig safety_check_config;  // 安全检查配置
    VectorDof joint_stiffness = VectorDof::Constant(200.0);
    VectorDof joint_damping = VectorDof::Constant(20.0);
    VectorDof joint_integral_ratio = VectorDof::Constant(0.01);
    JointAdmittanceConfig joint_admittance;  // 关节导纳控制配置
    Vector12d cart_stiffness = Vector12d::Constant(200.0);
    Vector12d cart_damping_ratio = Vector12d::Constant(1.0);  // 阻尼比 范围0.5 ~ 1.5
    std::array<ErrorCode, ndof_robot> error_codes{};
    TrajectoryBounds joint_streaming_bounds;     // joint streaming轨迹限制
    bool enable_streaming_ruckig{true};          // /control/joint接口是否使用Ruckig轨迹规划
    bool enable_chassis_shaper{false};           // 是否启用底盘速度输入整形器
    bool enable_record{false};                   // 是否启用调试日志
    bool enable_butterworth_filter[ndof_robot];  // 是否使用Butterworth滤波器
    bool enable_input_shaper[ndof_robot];        // 是否使用输入整形器
    VectorDof butterworth_filter_frequency = VectorDof::Constant(3.0);  // 输入滤波器频率
    VectorDof input_shaper_frequency = VectorDof::Constant(6.0);        // 输入整形器频率
};

struct PlannerManagerInfo {
    Eigen::VectorXd ee_pose_measurement;
    Eigen::VectorXd torso_pose_measurement;
    RobotProperty robot_property;
};

struct PoseQuaternion {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};

    PoseQuaternion() = default;
    explicit PoseQuaternion(const Eigen::Matrix4d& mat) {
        position = mat.block<3, 1>(0, 3);
        orientation = Eigen::Quaterniond(mat.block<3, 3>(0, 0));
    }
    explicit PoseQuaternion(const Eigen::Vector3d& pos, const Eigen::Quaterniond& ori)
        : position(pos), orientation(ori) {}

    std::string ToString() const {
        std::stringstream ss;
        ss << "Pose: (" << position.x() << ", " << position.y() << ", " << position.z() << ", " << orientation.w()
           << ", " << orientation.x() << ", " << orientation.y() << ", " << orientation.z() << ")";
        return ss.str();
    }

    Eigen::VectorXd ToVectorXd() const {
        Eigen::VectorXd vec(7);
        vec << position, orientation.w(), orientation.x(), orientation.y(), orientation.z();
        return vec;
    }

    // 组合两个位姿：等价于 4x4 变换矩阵相乘 (T = T1 * T2)
    PoseQuaternion operator*(const PoseQuaternion& other) const {
        PoseQuaternion res;
        res.orientation = orientation * other.orientation;
        res.position = position + orientation * other.position;
        return res;
    }

    PoseQuaternion& operator*=(const PoseQuaternion& other) {
        position = position + orientation * other.position;
        orientation = orientation * other.orientation;
        return *this;
    }
};

struct PlannerManagerCommand {
    std::int64_t sequence_number{-1};
    JointCommand cmd;
};

struct TorqueCtrlCommand {
    VectorDof torque;              // 关节tracking力控模式下的力矩指令
    VectorDof torque_feedforward;  // 力矩前馈(抵消重力, 科氏力, 摩擦力)
    VectorDof torque_impedance;    // 笛卡尔阻抗控制模式下的力矩
    VectorDof q_desired;
};

// 移动指令
struct MoveRequest {
    MotionGroup motion_group;
    // 目标位置
    Eigen::VectorXd value;

    std::optional<ReferenceFrame> reference_frame;
    std::optional<double> speed;
    std::optional<double> acceleration;  // 加速度比例，范围 0.01 ~ 1.0
    std::optional<std::vector<int>> endpoint_index;
    std::optional<double> interval;
    std::optional<double> streaming_timeout;
    std::optional<double> streaming_velocity_timeout;  // Ruckig 速度超时时间，比 streaming_timeout 更短
    std::optional<bool> use_velocity;                  // 是否使用目标速度输入
    std::optional<bool> use_acceleration;              // 是否使用目标加速度输入
};

using PlannerData = std::variant<MoveRequest, PoseQuaternion, Vector3d, Vector6d, Eigen::VectorXd, bool, int, double>;

struct PlannerRequest {
    Int64 request_id{-1};
    RequestType type{RequestType::Empty};
    std::optional<PlannerData> data;  // start streaming指令无data
};

struct StreamingCommand {
    Int64 request_id{-1};
    bool streaming_finished{false};
    Eigen::VectorXd command;              // 用于存储流式数据
    Eigen::VectorXd target_velocity;      // 目标速度
    Eigen::VectorXd target_acceleration;  // 目标加速度
    TimePointSystem received_time{TimePointSystem::Invalid()};
};

struct TimeMetaInfo {
    TimePointSystem current_time{};
    TimePointSystem measurement_time{};
    Int64 episode_index{-1};
    TimePointSystem episode_start_time{};
};

// 线程间通信数据结构
struct PlannerManagerBuffers {
    std::shared_ptr<SingleWriterLatestBuffer<JointMeasurement>> joint_measurement;
    std::shared_ptr<SingleWriterLatestBuffer<PlannerManagerInfo>> planner_manager_info;
    std::shared_ptr<SingleWriterLatestBuffer<PlannerManagerCommand>> planner_manager_command;
    std::shared_ptr<SingleWriterLatestBuffer<StreamingCommand>> streaming_command_latest_buffer;
    std::shared_ptr<BoundedQueue<PlannerRequest>> request_queue;
    std::shared_ptr<BoundedQueue<PlannerRequest>> emergency_request_queue;

    PlannerManagerBuffers() = default;
    ~PlannerManagerBuffers() = default;

    void Initialize() {
        joint_measurement = std::make_shared<SingleWriterLatestBuffer<JointMeasurement>>();
        planner_manager_info = std::make_shared<SingleWriterLatestBuffer<PlannerManagerInfo>>();
        planner_manager_command = std::make_shared<SingleWriterLatestBuffer<PlannerManagerCommand>>();
        streaming_command_latest_buffer = std::make_shared<SingleWriterLatestBuffer<StreamingCommand>>();
        request_queue = std::make_shared<BoundedQueue<PlannerRequest>>(32);
        emergency_request_queue = std::make_shared<BoundedQueue<PlannerRequest>>(32);
    }
};

struct TorqueControlBuffer {
    std::shared_ptr<SingleWriterLatestBuffer<TorqueCtrlCommand>> torque_controller_command;
    std::shared_ptr<SingleWriterLatestBuffer<JointRawMeasurement>> joint_raw_measurement;

    TorqueControlBuffer() = default;
    ~TorqueControlBuffer() = default;

    void Initialize() {
        torque_controller_command = std::make_shared<SingleWriterLatestBuffer<TorqueCtrlCommand>>();
        joint_raw_measurement = std::make_shared<SingleWriterLatestBuffer<JointRawMeasurement>>();
    }
};

// 机器人配置文件参数
struct RobotParams {
    std::string robot_type;
    std::string robot_version;
    std::string robot_key;
    std::string left_hand_type;
    std::string right_hand_type;
    Eigen::VectorXd joint_velocity_limit;  // 关节最大速度限制（正负对称）
    Eigen::VectorXd joint_torque_limit;    // 关节最大力矩限制（正负对称）
    VectorDof joint_torque_ext_deadzone = VectorDof::Constant(TorqueExtDeadzone);  // 外力矩死区 (N·m), 仅手臂可配置
    Eigen::VectorXd joint_limit_margin_deg;  // 关节位置限位安全阈值（度），用于在硬限位基础上缩小范围
    std::vector<std::string> joints_primary_names;
    std::vector<std::string> joints_secondary_names;
    std::vector<std::string> joints_torso_names;
    std::string ee_primary_name;           // 主臂末端执行器名称
    std::string ee_secondary_name;         // 从臂末端执行器名称
    std::vector<std::string> joint_names;  // 所有关节名称（URDF顺序，不含universe）
    std::unordered_map<std::string, std::string> ros2_to_urdf_map;  // 逻辑名称到URDF名称的映射
    std::vector<std::string> joints_a1;                             // A1模式关节名称
    std::vector<std::string> joints_single_arm;                     // 单臂模式关节名称
    std::vector<std::string> joints_dual_arm;                       // 双臂模式关节名称
    std::optional<Eigen::VectorXd> dyn_base_params_primary;         // 动力学基参数矩阵(非方阵)
    std::optional<Eigen::VectorXd> dyn_base_params_secondary;       // 动力学基参数矩阵(非方阵)
    std::array<double, 7> fv_ratio_primary{0.6, 0.6, 0.6, 0.6, 0.6, 0.5, 0.5};
    std::array<double, 7> fv_ratio_secondary{0.6, 0.6, 0.6, 0.6, 0.6, 0.5, 0.5};
    std::array<double, 7> fc_ratio_primary{0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
    std::array<double, 7> fc_ratio_secondary{0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
    std::string friction_model = "smooth";
    double friction_velocity_scale = 0.1;
    double stribeck_velocity = 0.1;
    double stribeck_alpha = 2.0;
    dynamics::tool_compensation::ToolGravityCompensationConfig tool_dynamic_primary;  // 主臂末端工具重力补偿参数
    dynamics::tool_compensation::ToolGravityCompensationConfig tool_dynamic_secondary;  // 从臂末端工具重力补偿参数
};

struct RobotModel {
    std::shared_ptr<const RobotParams> params_ptr;
    std::shared_ptr<const pinocchio::Model> model_ptr;
    std::shared_ptr<const pinocchio::Model> primary_arm_model_ptr;
    std::shared_ptr<const pinocchio::Model> secondary_arm_model_ptr;
    RobotProperty robot_property_default;

    RobotModel() = default;
    ~RobotModel() = default;

    bool IsDynamicParamsAvailable() const {
        return params_ptr->dyn_base_params_primary.has_value() && params_ptr->dyn_base_params_secondary.has_value();
    }
};

}  // namespace robot_server_lib
