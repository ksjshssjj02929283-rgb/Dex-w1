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
#include <unsupported/Eigen/Splines>

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
            ss << "Position: " << position.transpose() << ", Velocity: " << velocity.transpose() << ", Effort: " << effort.transpose()
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

        static PoseQuaternion FromVector7d(const Eigen::Matrix<double, 7, 1> &vec)
        {
            PoseQuaternion pose;
            pose.position = vec.head(3);
            pose.orientation = Eigen::Quaterniond(vec(3), vec(4), vec(5), vec(6));
            return pose;
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
        using Spline3d = Eigen::Spline<double, 3>;

        std::vector<PoseQuaternion> poses; // poses
        std::vector<double> segment_times; // normalized times 0~1 at poses
        Spline3d pos_spline;
        double total_length; // total length of trajectory
        int degree;          // order of interpolation

        static PoseTrajectory FromPoses(std::vector<PoseQuaternion> &&pose_list, const int interp_degree = 1)
        {
            PoseTrajectory pose_trajectory;
            auto &poses = pose_trajectory.poses;
            auto &segment_times = pose_trajectory.segment_times;
            auto &total_length = pose_trajectory.total_length;
            auto &pos_spline = pose_trajectory.pos_spline;
            auto &degree = pose_trajectory.degree;

            poses = std::move(pose_list);
            const int N = poses.size();
            assert(N >= 2);
            degree = std::clamp(interp_degree, 1, N - 1);
            // Estimate length
            std::vector<double> segment_lengths_(N - 1);
            total_length = 0.0;
            const double q_weight = 0.5; // weight of quaternion distance
            for (int i = 0; i < N - 1; ++i)
            {
                const auto &p1 = poses[i].position;
                const auto &p2 = poses[i + 1].position;
                const double pos_dist = (p2 - p1).norm();
                const auto &q1 = poses[i].orientation;
                const auto &q2 = poses[i + 1].orientation;
                const double ang_dist = q1.angularDistance(q2);
                const double seg_len = pos_dist + q_weight * ang_dist;
                segment_lengths_[i] = seg_len;
                total_length += seg_len;
            }
            // Estimate time stamps
            segment_times.resize(N);
            segment_times[0] = 0.0;
            for (int i = 0; i < N - 1; ++i)
                segment_times[i + 1] = segment_times[i] + segment_lengths_[i] / total_length;
            assert(segment_times.back() == 1.0);
            // Construct position spline
            Eigen::MatrixXd ctrl_points(3, N);
            for (int i = 0; i < N; ++i)
                ctrl_points.col(i) = poses[i].position;
            Eigen::RowVectorXd knots(N);
            for (int i = 0; i < N; ++i)
                knots(i) = segment_times[i];
            pos_spline = Spline3d(Eigen::SplineFitting<Spline3d>::Interpolate(ctrl_points, degree, knots));

            return pose_trajectory;
        }

        // Spline Interpolation
        Eigen::Matrix4d operator()(const double phase) const // scaled time of 0~1
        {
            Eigen::Matrix4d result = Eigen::Matrix4d::Identity();

            double t = std::clamp(phase, 0.0, 1.0);

            // --- 位置 ---
            Eigen::Vector3d pos = pos_spline(t);
            result.block<3, 1>(0, 3) = pos;

            // --- 旋转 (用 SLERP 做段内线性插值) ---
            int seg = 0;
            while (seg + 1 < segment_times.size() && t >= segment_times[seg + 1])
                ++seg;
            double t0 = segment_times[seg];
            double t1 = segment_times[seg + 1];
            double alpha = (t1 - t0 > 1e-6) ? (t - t0) / (t1 - t0) : 0.0;
            Eigen::Quaterniond q = poses[seg].orientation.slerp(alpha, poses[seg + 1].orientation);
            result.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();

            return result;
        }

        double totalLength() const { return total_length; }

        void clear()
        {
            poses.clear();
            segment_times.clear();
            total_length = 0.0;
            degree = 0;
        }

        std::string ToString()
        {
            std::stringstream ss;
            ss << "PoseTrajectory:\n"
               << "  Total Length: " << std::to_string(total_length) << "\n"
               << "  Poses:\n";
            for (int i = 0; i < poses.size(); ++i)
            {
                ss << "    Time: " << segment_times[i] << ",\t" << poses[i].ToString() << "\n";
            }
            return ss.str();
        }
    };

    template <int NDoF>
    struct JointTrajectory
    {
        static constexpr double VLIMIT = 1.0;
        static constexpr double ALIMIT = 10.0;
        static constexpr double JLIMIT = 100.0;

        std::vector<Eigen::Matrix<double, NDoF, 1>> positions;
        std::vector<Eigen::Matrix<double, NDoF, 1>> velocities;
        std::vector<Eigen::Matrix<double, NDoF, 1>> accelerations;
        Eigen::Matrix<double, NDoF, 1> max_velocity = Eigen::Matrix<double, NDoF, 1>::Constant(VLIMIT);
        Eigen::Matrix<double, NDoF, 1> min_velocity = Eigen::Matrix<double, NDoF, 1>::Constant(-VLIMIT);
        Eigen::Matrix<double, NDoF, 1> max_acceleration = Eigen::Matrix<double, NDoF, 1>::Constant(ALIMIT);
        Eigen::Matrix<double, NDoF, 1> min_acceleration = Eigen::Matrix<double, NDoF, 1>::Constant(-ALIMIT);
        Eigen::Matrix<double, NDoF, 1> max_jerk = Eigen::Matrix<double, NDoF, 1>::Constant(JLIMIT);
        Eigen::Matrix<double, NDoF, 1> min_jerk = Eigen::Matrix<double, NDoF, 1>::Constant(-JLIMIT);

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
            if (phase <= 0.0)
            {
                return positions.front();
            }
            if (phase >= 1.0)
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
        std::optional<int> interp_order;
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
