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
#include "pinocchio_include.h"
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
    VectorDof position;  // 关节位置
    VectorDof velocity;  // 关节速度
    VectorDof effort;    // 关节力矩

    VectorDof q_desired_pre;  // 预期关节位置

    std::string ToString() const
    {
        std::stringstream ss;
        ss << "Position: " << position.transpose() << ", Velocity: " << velocity.transpose() << ", Effort: " << effort.transpose()
           << ", Previous Q Desired: " << q_desired_pre.transpose();
        return ss.str();
    }
};

struct JointRawMeasurement
{
    VectorDof position;  // 关节位置
    VectorDof velocity;  // 关节速度
    VectorDof effort;    // 关节力矩
    TimePointSystem time_point;

    std::string ToString() const
    {
        std::stringstream ss;
        ss << "Position: " << position.transpose() << ", Velocity: " << velocity.transpose() << ", Effort: " << effort.transpose();
        return ss.str();
    }
};

struct JointCommand
{
    VectorDof position;
    VectorDof velocity;
    VectorDof acceleration;
    VectorDof torque_feedforward;

    std::string ToString() const
    {
        std::stringstream ss;
        ss << "Position: " << position.transpose() << ", Velocity: " << velocity.transpose() << ", Acceleration: " << acceleration.transpose()
           << ", Torque Feedforward: " << torque_feedforward.transpose();
        return ss.str();
    }
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
};

struct PlannerManagerCommand
{
    std::int64_t sequence_number{-1};
    JointCommand cmd;
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
};

using PlannerData = std::variant<MoveRequest, PoseQuaternion, bool, double>;

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
    TimePointSystem received_time{};
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
    std::shared_ptr<SingleWriterLatestBuffer<Eigen::VectorXd>> ee_pose_measurement;
    std::shared_ptr<SingleWriterLatestBuffer<PlannerManagerCommand>> planner_manager_command;
    std::shared_ptr<SingleWriterLatestBuffer<StreamingCommand>> streaming_command_latest_buffer;
    std::shared_ptr<BoundedQueue<PlannerRequest>> request_queue;
    std::shared_ptr<BoundedQueue<PlannerRequest>> emergency_request_queue;

    PlannerManagerBuffers() = default;
    ~PlannerManagerBuffers() = default;

    void Initialize()
    {
        joint_measurement = std::make_shared<SingleWriterLatestBuffer<JointMeasurement>>();
        ee_pose_measurement = std::make_shared<SingleWriterLatestBuffer<Eigen::VectorXd>>();
        planner_manager_command = std::make_shared<SingleWriterLatestBuffer<PlannerManagerCommand>>();
        streaming_command_latest_buffer = std::make_shared<SingleWriterLatestBuffer<StreamingCommand>>();
        request_queue = std::make_shared<BoundedQueue<PlannerRequest>>(32);
        emergency_request_queue = std::make_shared<BoundedQueue<PlannerRequest>>(32);
    }
};

struct TorqueControlBuffer
{
    std::shared_ptr<SingleWriterLatestBuffer<VectorDof>> torque_command_vector;
    std::shared_ptr<SingleWriterLatestBuffer<JointRawMeasurement>> joint_raw_measurement;

    TorqueControlBuffer() = default;
    ~TorqueControlBuffer() = default;

    void Initialize()
    {
        torque_command_vector = std::make_shared<SingleWriterLatestBuffer<VectorDof>>();
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
    Eigen::VectorXd joints_primary;
    Eigen::VectorXd joints_secondary;
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

struct RobotProperty
{
    pinocchio::SE3 tcp_frame{Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()};
    pinocchio::SE3 tcp_frame_secondary{Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()};
    double global_speed_ratio{0.5};
};

}  // namespace robot_server_lib
