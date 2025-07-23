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
        VectorDof position; // 关节位置
        VectorDof velocity; // 关节速度
        VectorDof effort;   // 关节力矩

        VectorDof q_desired_pre; // 预期关节位置

        std::string ToString() const
        {
            std::stringstream ss;
            ss << "Position: " << position.transpose() << ", Velocity: " << velocity.transpose()
               << ", Effort: " << effort.transpose()
               << ", Previous Q Desired: " << q_desired_pre.transpose();
            return ss.str();
        }
    };

    struct JointRawMeasurement
    {
        VectorDof position; // 关节位置
        VectorDof velocity; // 关节速度
        VectorDof effort;   // 关节力矩
        TimePointSystem time_point;

        std::string ToString() const
        {
            std::stringstream ss;
            ss << "Position: " << position.transpose() << ", Velocity: " << velocity.transpose()
               << ", Effort: " << effort.transpose();
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
            ss << "Position: " << position.transpose() << ", Velocity: " << velocity.transpose()
               << ", Acceleration: " << acceleration.transpose()
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
            ss << "Pose: (" << position.x() << ", " << position.y() << ", " << position.z() << ", "
               << orientation.w() << ", " << orientation.x() << ", " << orientation.y() << ", "
               << orientation.z() << ")";
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

        PoseTrajectory(const std::vector<Vector7d> &pose_list) : poses_(pose_list)
        {
            int N = poses_.size() - 1;
            assert(N > 0);
            std::vector<double> segment_lengths_(N);
            double total_length = 0.0;
            const double q_weight = 0.5; // weight of quaternion distance
            for (int i = 0; i < N; ++i)
            {
                Vector3d p1 = poses_[i].head<3>();
                Vector3d p2 = poses_[i + 1].head<3>();
                double pos_dist = (p2 - p1).norm();

                Eigen::Quaterniond q1(poses_[i].tail<4>());
                Eigen::Quaterniond q2(poses_[i + 1].tail<4>());
                q1.normalize();
                q2.normalize();
                double ang_dist = q1.angularDistance(q2);

                double seg_len = pos_dist + q_weight * ang_dist;
                segment_lengths_[i] = seg_len;
                total_length += seg_len;
            }

            segment_times_.resize(N + 1);
            segment_times_[0] = 0.0;
            for (int i = 0; i < N; ++i)
            {
                double dt = segment_lengths_[i] / total_length;
                segment_times_[i + 1] = segment_times_[i] + dt;
            }
            assert(segment_times_.back() == 1.0);
        }

        Eigen::Matrix4d operator()(double t) const // scaled time of 0~1
        {
            Eigen::Matrix4d result = Matrix4d::Identity();
            if (t <= 0.0)
            {
                result.block<3, 3>(0, 0) = Eigen::Quaterniond(Eigen::Vector4d(poses_.front().head(4))).toRotationMatrix();
                result.block<3, 1>(0, 3) = poses_.front().tail(3);
                return result;
            }
            if (t >= 1.0)
            {
                result.block<3, 3>(0, 0) = Eigen::Quaterniond(Eigen::Vector4d(poses_.back().head(4))).toRotationMatrix();
                result.block<3, 1>(0, 3) = poses_.back().tail(3);
                return result;
            }

            int seg = 0;
            while (seg + 1 < segment_times_.size() && t > segment_times_[seg + 1])
            {
                ++seg;
            }
            double alpha = (t - segment_times_[seg]) / (segment_times_[seg + 1] - segment_times_[seg]);

            // interpolate position
            Vector3d p0 = poses_[seg].head<3>();
            Vector3d p1 = poses_[seg + 1].head<3>();
            result.block<3, 1>(0, 3) = (1 - alpha) * p0 + alpha * p1;
            // interpolate orientation
            Eigen::Quaterniond q0(Eigen::Vector4d(poses_[seg].tail<4>()));
            Eigen::Quaterniond q1(Eigen::Vector4d(poses_[seg + 1].tail<4>()));
            q0.normalize();
            q1.normalize();
            result.block<3, 3>(0, 0) = (q0.slerp(alpha, q1)).toRotationMatrix();
            return result;
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

    private:
        std::vector<Vector7d> poses_;       // poses
        std::vector<double> segment_times_; // segment length
    };

    struct PlannerManagerCommand
    {
        std::int64_t sequence_number{-1};
        JointCommand cmd;
    };

    template <int nDoF>
    struct JointTrajectory
    {
        struct JointConstraints
        {
            double max_velocity{1.0};
            double max_acceleration{2.0};
            double max_jerk{100000.0};
        };

        void ComputeTimeParametrization()
        {
            const auto &q_list = joint_space_trajectory_;
            const int N = q_list.size();
            assert(N >= 2 && "JointTrajectory must have at least 2 points for time parametrization");

            times_.resize(N);
            times_[0] = 0.0;

            std::vector<double> durations(N - 1);
            for (int i = 0; i < N - 1; ++i)
            {
                double max_duration = 0.0;
                for (int j = 0; j < nDoF; ++j)
                {
                    double q0 = q_list[i](j);
                    double q1 = q_list[i + 1](j);
                    double delta = std::abs(q1 - q0);
                    // 估算当前段的速度 v_i ≈ Δq / Δt，用前后段近似加速度
                    double v0 = (i == 0) ? 0 : (q_list[i](j) - q_list[i - 1](j)) / durations[i - 1];
                    double v1 = (i == N - 2) ? 0 : (q_list[i + 2](j) - q_list[i + 1](j)) / durations[i + 1];
                    double avg_v = std::abs(v1 - v0);                       // 估算速度变化量
                    double avg_a = avg_v / std::max(1e-3, durations[i]);    // 估算加速度
                    double est_jerk = avg_a / std::max(1e-3, durations[i]); // 估算jerk
                    double t_j = std::cbrt(6.0 * delta / constraints.max_jerk); // 极限估算
                    double t_a = std::sqrt(delta / constraints.max_acceleration);
                    double t_v = delta / constraints.max_velocity;

                    double t = std::max({t_j * 3, t_a * 2, t_v}); // 保守估计
                    max_duration = std::max(max_duration, t);
                }
                durations[i] = max_duration;
            }

            for (int i = 1; i < N; ++i)
            {
                times_[i] = times_[i - 1] + durations[i - 1];
            }
        }

        std::vector<Eigen::Matrix<double, nDoF, 1>> &GetJointTrajectory()
        {
            return joint_space_trajectory_;
        }

    private:
        std::vector<Eigen::Matrix<double, nDoF, 1>> joint_space_trajectory_;
        std::vector<double> times_;
        JointConstraints constraints;
    };

    // 移动指令
    struct MoveRequest
    {
        MotionGroup motion_group;
        // 目标位置
        Eigen::VectorXd value;
        // 路径点
        std::vector<Eigen::VectorXd> value_vec;

        std::optional<ReferenceFrame> reference_frame;
        std::optional<double> speed;
        std::optional<double> blend_tolerance;
    };

    using PlannerData = std::variant<MoveRequest, PoseQuaternion, double>;

    struct PlannerRequest
    {
        Int64 request_id{-1};
        RequestType type{RequestType::Empty};
        std::optional<PlannerData> data; // start streaming指令无data
    };

    struct StreamingCommand
    {
        Int64 request_id{-1};
        bool streaming_finished{false};
        Eigen::VectorXd command; // 用于存储流式数据
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

} // namespace robot_server_lib
