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
#include <Eigen/Geometry>
#include <array>
#include <filesystem>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "bounded_queue.h"
#include "dynamics_utils.h"
#include "single_rw_latest_queue.h"
#include "time_utility.h"
#include "typedef.h"

namespace robot_server_lib
{

/*-------------------------------------------------------------------------------
robot state and request data structure class
机器人状态和指令数据结构
-------------------------------------------------------------------------------*/

struct JointMeasurement
{
    VectorDof position;                          // 关节位置
    VectorDof velocity;                          // 关节速度
    VectorDof effort;                            // 关节力矩
    VectorDof q_desired_pre;                     // 预期关节位置
    Vector12d sensor_wrench{Vector12d::Zero()};  // 六维力传感器数据

    std::string ToString() const
    {
        std::stringstream ss;
        ss << "Position: " << position.transpose() << ", Velocity: " << velocity.transpose() << ", Effort: " << effort.transpose()
           << ", Previous Q Desired: " << q_desired_pre.transpose() << ", Sensor_wrench: " << sensor_wrench.transpose();
        return ss.str();
    }
};

struct JointRawMeasurement
{
    VectorDof position;                          // 关节位置
    VectorDof velocity;                          // 关节速度
    VectorDof effort;                            // 关节力矩
    Vector12d sensor_wrench{Vector12d::Zero()};  // 六维力传感器数据
    TimePointSystem time_point;

    std::string ToString() const
    {
        std::stringstream ss;
        ss << "Position: " << position.transpose() << ", Velocity: " << velocity.transpose() << ", Effort: " << effort.transpose()
           << ", Sensor Wrench: " << sensor_wrench.transpose();
        return ss.str();
    }
};

struct JointCommand
{
    VectorDof position;
    VectorDof velocity;
    VectorDof acceleration;
    VectorDof compliance_position;
    VectorDof torque_feedforward;

    std::string ToString() const
    {
        std::stringstream ss;
        ss << "Position: " << position.transpose() << ", Velocity: " << velocity.transpose() << ", Acceleration: " << acceleration.transpose()
           << ", Compliance Position: " << compliance_position.transpose() << ", Torque Feedforward: " << torque_feedforward.transpose();
        return ss.str();
    }
};

struct RobotProperty
{
    pinocchio::SE3 tcp_frame{Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()};
    pinocchio::SE3 tcp_frame_secondary{Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()};
    RunMode run_mode{RunMode::Position};
    double global_speed_ratio{0.5};
    double safe_level{0.5};
    bool clear_error_flag{false};
    ErrorCode error_code{ErrorCode::NoError};
};

struct PlannerManagerInfo
{
    Eigen::VectorXd ee_pose_measurement;
    Eigen::VectorXd torso_pose_measurement;
    RobotProperty robot_property;
};

struct PoseQuaternion
{
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};

    PoseQuaternion() = default;
    explicit PoseQuaternion(const Eigen::Matrix4d &mat)
    {
        position = mat.block<3, 1>(0, 3);
        orientation = Eigen::Quaterniond(mat.block<3, 3>(0, 0));
    }
    explicit PoseQuaternion(const Eigen::Vector3d &pos, const Eigen::Quaterniond &ori) : position(pos), orientation(ori) {}

    std::string ToString() const
    {
        std::stringstream ss;
        ss << "Pose: (" << position.x() << ", " << position.y() << ", " << position.z() << ", " << orientation.w() << ", " << orientation.x() << ", "
           << orientation.y() << ", " << orientation.z() << ")";
        return ss.str();
    }

    Eigen::VectorXd ToVectorXd() const
    {
        Eigen::VectorXd vec(7);
        vec << position, orientation.w(), orientation.x(), orientation.y(), orientation.z();
        return vec;
    }

    // 组合两个位姿：等价于 4x4 变换矩阵相乘 (T = T1 * T2)
    PoseQuaternion operator*(const PoseQuaternion &other) const
    {
        PoseQuaternion res;
        res.orientation = orientation * other.orientation;
        res.position = position + orientation * other.position;
        return res;
    }

    PoseQuaternion &operator*=(const PoseQuaternion &other)
    {
        position = position + orientation * other.position;
        orientation = orientation * other.orientation;
        return *this;
    }
};

struct PlannerManagerCommand
{
    std::int64_t sequence_number{-1};
    JointCommand cmd;
};

struct TorqueCtrlCommand
{
    VectorDof torque;
    VectorDof torque_external;
    VectorDof q_desired;
};

// 移动指令
struct MoveRequest
{
    MotionGroup motion_group;
    // 目标位置
    Eigen::VectorXd value;

    std::optional<ReferenceFrame> reference_frame;
    std::optional<double> speed;
    std::optional<double> blend_tolerance;
    std::optional<bool> optional_mode;
    std::optional<std::vector<int>> endpoint_index;
    std::optional<double> interval;
};

using PlannerData = std::variant<MoveRequest, PoseQuaternion, Vector3d, bool, double>;

struct PlannerRequest
{
    Int64 request_id{-1};
    RequestType type{RequestType::Empty};
    std::optional<PlannerData> data;  // start streaming指令无data
};

struct StreamingCommand
{
    Int64 request_id{-1};
    bool streaming_finished{false};
    Eigen::VectorXd command;  // 用于存储流式数据
    TimePointSystem received_time{TimePointSystem::Invalid()};
};

struct TimeMetaInfo
{
    TimePointSystem current_time{};
    TimePointSystem measurement_time{};
    Int64 episode_index{-1};
    TimePointSystem episode_start_time{};
};

// 线程间通信数据结构
struct PlannerManagerBuffers
{
    std::shared_ptr<SingleWriterLatestBuffer<JointMeasurement>> joint_measurement;
    std::shared_ptr<SingleWriterLatestBuffer<PlannerManagerInfo>> planner_manager_info;
    std::shared_ptr<SingleWriterLatestBuffer<PlannerManagerCommand>> planner_manager_command;
    std::shared_ptr<SingleWriterLatestBuffer<StreamingCommand>> streaming_command_latest_buffer;
    std::shared_ptr<BoundedQueue<PlannerRequest>> request_queue;
    std::shared_ptr<BoundedQueue<PlannerRequest>> emergency_request_queue;

    PlannerManagerBuffers() = default;
    ~PlannerManagerBuffers() = default;

    void Initialize()
    {
        joint_measurement = std::make_shared<SingleWriterLatestBuffer<JointMeasurement>>();
        planner_manager_info = std::make_shared<SingleWriterLatestBuffer<PlannerManagerInfo>>();
        planner_manager_command = std::make_shared<SingleWriterLatestBuffer<PlannerManagerCommand>>();
        streaming_command_latest_buffer = std::make_shared<SingleWriterLatestBuffer<StreamingCommand>>();
        request_queue = std::make_shared<BoundedQueue<PlannerRequest>>(32);
        emergency_request_queue = std::make_shared<BoundedQueue<PlannerRequest>>(32);
    }
};

struct TorqueControlBuffer
{
    std::shared_ptr<SingleWriterLatestBuffer<TorqueCtrlCommand>> torque_controller_command;
    std::shared_ptr<SingleWriterLatestBuffer<JointRawMeasurement>> joint_raw_measurement;

    TorqueControlBuffer() = default;
    ~TorqueControlBuffer() = default;

    void Initialize()
    {
        torque_controller_command = std::make_shared<SingleWriterLatestBuffer<TorqueCtrlCommand>>();
        joint_raw_measurement = std::make_shared<SingleWriterLatestBuffer<JointRawMeasurement>>();
    }
};

// 机器人配置文件参数
struct RobotParams
{
    std::filesystem::path urdf_path;
    std::filesystem::path left_arm_urdf_path;
    std::filesystem::path right_arm_urdf_path;
    std::string robot_name;
    std::string left_arm_name;
    std::string right_arm_name;
    std::vector<std::string> joint_command_sequence_by_name;  // 关节指令顺序
    Eigen::VectorXd joint_max_velocity_limit_positive;      // 关节正向最大速度限制
    Eigen::VectorXd joint_max_velocity_limit_negative;      // 关节负向最大速度限制
    Eigen::VectorXd joint_max_torque_limit_positive;        // 关节正向最大力矩限制
    Eigen::VectorXd joint_max_torque_limit_negative;        // 关节负向最大力矩限制
    Eigen::VectorXd joints_primary;
    Eigen::VectorXd joints_secondary;
    Eigen::VectorXd joints_torso;
};

struct RobotModel
{
    std::shared_ptr<const pinocchio::Model> model_ptr;
    std::shared_ptr<const RobotParams> params_ptr;
    std::shared_ptr<const pinocchio::Model> left_arm_model_ptr;
    std::shared_ptr<const pinocchio::Model> right_arm_model_ptr;

    RobotModel() = default;
    ~RobotModel() = default;
};

}  // namespace robot_server_lib
