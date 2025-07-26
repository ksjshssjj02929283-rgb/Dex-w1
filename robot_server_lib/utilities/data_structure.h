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

struct PoseTrajectory
{
    using Vector3d = Eigen::Matrix<double, 3, 1>;
    using Vector7d = Eigen::Matrix<double, 7, 1>;

    std::vector<Vector7d> poses_;        // poses
    std::vector<double> segment_times_;  // normalized times 0~1 at poses
    double total_length_;                // total length of trajectory

    void InitializePosesAndTimes(std::vector<Vector7d> &&pose_list)
    {
        poses_ = std::move(pose_list);

        int N = poses_.size() - 1;
        assert(N > 0);
        std::vector<double> segment_lengths_(N);
        total_length_ = 0.0;
        const double q_weight = 0.5;  // weight of quaternion distance
        for (int i = 0; i < N; ++i)
        {
            Vector3d p1 = poses_[i].head<3>();
            Vector3d p2 = poses_[i + 1].head<3>();
            double pos_dist = (p2 - p1).norm();

            Eigen::Quaterniond q1(poses_[i](3), poses_[i](4), poses_[i](5), poses_[i](6));
            Eigen::Quaterniond q2(poses_[i + 1](3), poses_[i + 1](4), poses_[i + 1](5), poses_[i + 1](6));
            q1.normalize();
            q2.normalize();
            double ang_dist = q1.angularDistance(q2);

            double seg_len = pos_dist + q_weight * ang_dist;
            segment_lengths_[i] = seg_len;
            total_length_ += seg_len;
        }

        segment_times_.resize(N + 1);
        segment_times_[0] = 0.0;
        for (int i = 0; i < N; ++i)
        {
            double dt = segment_lengths_[i] / total_length_;
            segment_times_[i + 1] = segment_times_[i] + dt;
        }
        assert(segment_times_.back() == 1.0);
    }

    Eigen::Matrix4d operator()(const double phase) const  // scaled time of 0~1
    {
        Eigen::Matrix4d result = Matrix4d::Identity();
        if (phase <= 0.0)
        {
            result.block<3, 3>(0, 0) =
                Eigen::Quaterniond(poses_.front()(3), poses_.front()(4), poses_.front()(5), poses_.front()(6)).toRotationMatrix();
            result.block<3, 1>(0, 3) = poses_.front().head(3);
            return result;
        }
        if (phase >= 1.0)
        {
            result.block<3, 3>(0, 0) = Eigen::Quaterniond(poses_.back()(3), poses_.back()(4), poses_.back()(5), poses_.back()(6)).toRotationMatrix();
            result.block<3, 1>(0, 3) = poses_.back().head(3);
            return result;
        }

        int seg = 0;
        while (seg + 1 < segment_times_.size() && phase >= segment_times_[seg + 1])
        {
            ++seg;
        }
        double alpha = (phase - segment_times_[seg]) / (segment_times_[seg + 1] - segment_times_[seg]);
        if (phase - segment_times_[seg] < 0.005)  // corresponds to 200 milestones on trajectory
        {
            alpha = 0.0;  // ensure all poses in poses_ can be sampled
        }

        // interpolate position
        Vector3d p0 = poses_[seg].head<3>();
        Vector3d p1 = poses_[seg + 1].head<3>();
        result.block<3, 1>(0, 3) = (1 - alpha) * p0 + alpha * p1;
        // interpolate orientation
        Eigen::Quaterniond q0(poses_[seg](3), poses_[seg](4), poses_[seg](5), poses_[seg](6));
        Eigen::Quaterniond q1(poses_[seg + 1](3), poses_[seg + 1](4), poses_[seg + 1](5), poses_[seg + 1](6));
        q0.normalize();
        q1.normalize();
        result.block<3, 3>(0, 0) = (q0.slerp(alpha, q1)).toRotationMatrix();
        return result;
    }

    double totalLength() const { return total_length_; }

    void clear()
    {
        poses_.clear();
        segment_times_.clear();
        total_length_ = 0.0;
    }

    std::string ToString()
    {
        std::stringstream ss;
        for (const auto &pose : poses_)
        {
            ss << "Pose: (";
            for (int i = 0; i < 7; ++i)
            {
                ss << pose(i);
                if (i < 6)
                    ss << ",";
            }
            ss << ")\n";
        }
        return ss.str();
    }
};

template <int NDoF>
struct JointTrajectory
{
    static constexpr double VLIMIT = 1.0;
    static constexpr double ALIMIT = 2.0;

    std::vector<Eigen::Matrix<double, NDoF, 1>> positions;
    std::vector<Eigen::Matrix<double, NDoF, 1>> velocities;
    std::vector<Eigen::Matrix<double, NDoF, 1>> accelerations;
    Eigen::Matrix<double, NDoF, 1> max_velocity = Eigen::Matrix<double, NDoF, 1>::Constant(VLIMIT);
    Eigen::Matrix<double, NDoF, 1> min_velocity = Eigen::Matrix<double, NDoF, 1>::Constant(-VLIMIT);
    Eigen::Matrix<double, NDoF, 1> max_acceleration = Eigen::Matrix<double, NDoF, 1>::Constant(ALIMIT);
    Eigen::Matrix<double, NDoF, 1> min_acceleration = Eigen::Matrix<double, NDoF, 1>::Constant(-ALIMIT);

    void SetJointTrajectory(std::vector<Eigen::Matrix<double, NDoF, 1>> &&_trajectory) { positions = std::move(_trajectory); }

    void clear()
    {
        positions.clear();
        velocities.clear();
        accelerations.clear();
    }

    bool has_value() { return !positions.empty(); }

    int size() const
    {
        return positions.size();
    }

    Eigen::Matrix<double, NDoF, 1> operator()(const double phase) const
    {
        if(phase <= 0.0)
        {
            return positions.front();
        }
        if(phase >= 1.0)
        {
            return positions.back();
        }

        int index = static_cast<int>(phase * (size() - 1));
        double alpha = phase * (size() - 1) - index;
        // linear interpolation
        return (1 - alpha) * positions[index] + alpha * positions[index + 1];
    }

    std::string ToString()
    {
        std::stringstream ss;
        ss << "Positions:\n";
        for (int i = 0; i < positions.size(); ++i)
        {
            ss << "  (";
            const auto &p = positions[i];
            for (int i = 0; i < 7; ++i)
            {
                ss << p(i);
                if (i < 6)
                    ss << ",";
            }
            ss << ")\n";
        }
        ss << "Velocities:\n";
        for (int i = 0; i < velocities.size(); ++i)
        {
            ss << "  (";
            const auto &v = velocities[i];
            for (int i = 0; i < 7; ++i)
            {
                ss << v(i);
                if (i < 6)
                    ss << ",";
            }
            ss << ")\n";
        }
        return ss.str();
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
    // 目标位置/路径点
    Eigen::VectorXd value;

    std::optional<ReferenceFrame> reference_frame;
    std::optional<double> speed;
    std::optional<double> blend_tolerance;
};

using PlannerData = std::variant<MoveRequest, PoseQuaternion, double>;

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
    std::shared_ptr<BoundedQueue<PlannerRequest>> request_queue;
    std::shared_ptr<SingleWriterLatestBuffer<StreamingCommand>> streaming_command_latest_buffer;

    PlannerManagerBuffers() = default;
    ~PlannerManagerBuffers() = default;

    void Initialize()
    {
        joint_measurement = std::make_shared<SingleWriterLatestBuffer<JointMeasurement>>();
        ee_pose_measurement = std::make_shared<SingleWriterLatestBuffer<Eigen::VectorXd>>();
        planner_manager_command = std::make_shared<SingleWriterLatestBuffer<PlannerManagerCommand>>();
        request_queue = std::make_shared<BoundedQueue<PlannerRequest>>(32);
        streaming_command_latest_buffer = std::make_shared<SingleWriterLatestBuffer<StreamingCommand>>();
    }
};

struct TorqueControlBuffer
{
    std::shared_ptr<SingleWriterLatestBuffer<VectorDof>> torque_command_vector;
    std::shared_ptr<SingleWriterLatestBuffer<JointRawMeasurement>> joint_raw_measurement;

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
    Eigen::VectorXd joints_primary;
    Eigen::VectorXd joints_secondary;
    // std::vector<double> zero_position {};
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
