#pragma once

#include "move_planner_base.h"
#include "trajectory_time_parameterization.hpp"

namespace robot_server_lib
{
    static constexpr int ndof_arms_ = 14;
    static constexpr int ndof_left_ = 7;
    static constexpr int ndof_right_ = 7;

    class MoveTrajectoryPlanner : public MotionPlannerBase
    {
    public:
        MoveTrajectoryPlanner(RobotModel robot_model, double controller_dt, bool wait_trajectory_upon_finish)
            : MotionPlannerBase(robot_model), controller_dt_(controller_dt), wait_trajectory_upon_finish_(wait_trajectory_upon_finish)
        {
            current_phase_ = 0.0;
            total_time_ = 0.0;
            time_primary_ = 0.0;
            time_secondary_ = 0.0;

            kine_ptr_primary_ = std::dynamic_pointer_cast<kinematics::YaikKinematics>(kinematics_ptr_);
            kine_ptr_secondary_ = std::dynamic_pointer_cast<kinematics::YaikKinematics>(kinematics_secondary_ptr_);
        }

        ~MoveTrajectoryPlanner() override = default;

        ComputeCommandStatus ComputeCommand(const JointMeasurement &measurement, const TimeMetaInfo &time_meta, JointCommand &joint_cmd) override
        {
            if (current_phase_ < 1.0)
                current_phase_ += controller_dt_ / total_time_;
            JointPositionDof q_ndof = measurement.q_desired_pre;
            JointPositionDof dq_ndof = measurement.velocity;
            Eigen::VectorXd q_arms, dq_arms;
            GetArmFromWholeBody(q_ndof, q_arms, MotionGroup::BothArms);
            GetArmFromWholeBody(dq_ndof, dq_arms, MotionGroup::BothArms);
            Eigen::Matrix<double, ndof_left_, 1> q_left(q_arms.head<ndof_left_>());
            Eigen::Matrix<double, ndof_left_, 1> dq_left(dq_arms.head<ndof_left_>());
            Eigen::Matrix<double, ndof_right_, 1> q_right(q_arms.tail<ndof_right_>());
            Eigen::Matrix<double, ndof_right_, 1> dq_right(dq_arms.tail<ndof_right_>());

            trajectory_spline_primary_.evaluate(current_phase_ * time_primary_, q_left, dq_left);
            trajectory_spline_secondary_.evaluate(current_phase_ * time_secondary_, q_right, dq_right);
            
            if (refine_cmd)
            {
                kine_ptr_primary_->ProjectToPose(pose_trajectory_primary_(current_phase_), q_left);
                kine_ptr_secondary_->ProjectToPose(pose_trajectory_secondary_(current_phase_), q_right);
            }

            const bool goal_reached =
                current_phase_ >= 1.0 && ((q_left - q_arms.head<ndof_left_>()).norm() < 1e-4 && (q_right - q_arms.tail<ndof_right_>()).norm() < 1e-4);

            q_arms.head<ndof_left_>() << q_left;
            dq_arms.head<ndof_left_>() << dq_left;
            q_arms.tail<ndof_right_>() << q_right;
            dq_arms.tail<ndof_right_>() << dq_right;
            MapArmToWholeBody(q_arms, q_ndof, MotionGroup::BothArms);
            MapArmToWholeBody(dq_arms, dq_ndof, MotionGroup::BothArms);

            Eigen::VectorXd q_left_cur, pose_left_ref(7);
            GetArmFromWholeBody(q_ndof, q_left_cur, MotionGroup::Primary);
            auto pose_left = kine_ptr_primary_->GetFK(q_left_cur, robot_property_.tcp_frame).ToVectorXd();
            pose_left_ref << -0.3, 0.363727, 0.262795, -0.229399, -0.440151, 0.599072, 0.628296;
            auto norm = (pose_left - pose_left_ref).norm();
            LOG_INFO << "Desired ref reached: " << std::to_string(norm) << "\n";

            if (goal_reached)
            {
                ClearCache();
                return ComputeCommandStatus::TaskFinished;
            }
            else
            {
                for (auto i = 0; i < ndof_robot; i++)
                {
                    joint_cmd.position[i] = q_ndof[i];
                    // joint_cmd.velocity[i] = 0.0;
                    joint_cmd.velocity[i] = dq_ndof[i];
                    joint_cmd.acceleration[i] = 0.0;
                    joint_cmd.torque_feedforward[i] = 0.0;
                }
                return ComputeCommandStatus::TaskOngoing;
            }
        }

        bool ResetForNewTrajectory(const PlannerRequest &request, const JointMeasurement &measurement, const TimeMetaInfo &time_meta) override
        {
            assert(request.data.has_value());
            assert(std::holds_alternative<MoveRequest>(request.data.value()));
            ClearCache();

            const auto &move_request = std::get<MoveRequest>(request.data.value());
            SetMotionGroup(move_request.motion_group);
            double speed_ratio = robot_property_.global_speed_ratio;
            if (move_request.speed.has_value())
            {
                speed_ratio = speed_ratio * std::clamp(move_request.speed.value(), 0.01, 1.0);
            }
            int interp_order_ = 1;
            if(move_request.interp_order.has_value())
            {
                LOG_INFO << "12345: \n";
                interp_order_ = move_request.interp_order.value();
            }

            JointTrajectory<ndof_left_> joint_trajectory_primary_;
            JointTrajectory<ndof_right_> joint_trajectory_secondary_;
            // const double max_speed = speed_ratio * joint_bounds_.max_velocity;
            // const double max_acceleration = speed_ratio * joint_bounds_.max_acceleration;
            // const double max_jerk = speed_ratio * joint_bounds_.max_jerk;
            const double max_speed = 0.5;
            const double max_acceleration = 5.0;
            const double max_jerk = 20.0;
            joint_trajectory_primary_.max_velocity.setConstant(max_speed);
            joint_trajectory_primary_.min_velocity.setConstant(-max_speed);
            joint_trajectory_primary_.max_acceleration.setConstant(max_acceleration);
            joint_trajectory_primary_.min_acceleration.setConstant(-max_acceleration);
            joint_trajectory_primary_.max_jerk.setConstant(max_jerk);
            joint_trajectory_primary_.min_jerk.setConstant(-max_jerk);
            joint_trajectory_secondary_.max_velocity.setConstant(max_speed);
            joint_trajectory_secondary_.min_velocity.setConstant(-max_speed);
            joint_trajectory_secondary_.max_acceleration.setConstant(max_acceleration);
            joint_trajectory_secondary_.min_acceleration.setConstant(-max_acceleration);
            joint_trajectory_secondary_.max_jerk.setConstant(max_jerk);
            joint_trajectory_secondary_.min_jerk.setConstant(-max_jerk);

            Eigen::VectorXd q_start;
            GetArmFromWholeBody(measurement.q_desired_pre, q_start);

            switch (move_request.motion_group)
            {
            case MotionGroup::BothArms:
            {
                assert(q_start.size() == ndof_arms_);
                // 获取起点
                PoseQuaternion pose_start_primary = kine_ptr_primary_->GetFK(q_start.head(ndof_left_), robot_property_.tcp_frame);
                PoseQuaternion pose_start_secondary = kine_ptr_secondary_->GetFK(q_start.tail(ndof_right_), robot_property_.tcp_frame_secondary);

                // 获取目标点
                std::vector<Eigen::Matrix<double, 7, 1>> poses_primary, poses_secondary;
                poses_primary.emplace_back(pose_start_primary.ToVectorXd());
                poses_secondary.emplace_back(pose_start_secondary.ToVectorXd());
                assert(move_request.value.size() % (ndof_arms_) == 0);
                const int num_points = move_request.value.size() / (ndof_arms_);
                assert(num_points > 0);
                for (int i = 0; i < num_points; ++i)
                {
                    const auto &value = move_request.value.segment(i * ndof_arms_, ndof_arms_);
                    poses_primary.emplace_back(value.head(ndof_left_));
                    poses_secondary.emplace_back(value.tail(ndof_right_));
                }
                pose_trajectory_primary_.Initialize(std::move(poses_primary), interp_order_);
                pose_trajectory_secondary_.Initialize(std::move(poses_secondary), interp_order_);
                LOG_INFO << "Primary arm motion plan: trajectory: \n"
                         << pose_trajectory_primary_.ToString();
                LOG_INFO << "Secondary arm motion plan: trajectory: \n"
                         << pose_trajectory_secondary_.ToString();

                std::vector<Eigen::Matrix<double, ndof_left_, 1>> trajectory_primary;
                std::vector<Eigen::Matrix<double, ndof_right_, 1>> trajectory_secondary;
                auto trajectory_ok_primary = kine_ptr_primary_->RunGlobalRedundancyResolutionForJointTrajectory(
                    q_start.head(ndof_left_), pose_trajectory_primary_, trajectory_primary, robot_property_.tcp_frame);
                auto trajectory_ok_secondary = kine_ptr_secondary_->RunGlobalRedundancyResolutionForJointTrajectory(
                    q_start.tail(ndof_right_), pose_trajectory_secondary_, trajectory_secondary, robot_property_.tcp_frame_secondary);
                joint_trajectory_primary_.SetJointTrajectory(std::move(trajectory_primary));
                joint_trajectory_secondary_.SetJointTrajectory(std::move(trajectory_secondary));
                if (!trajectory_ok_primary || !trajectory_ok_secondary)
                {
                    LOG_CRIT << "Failed to compute joint trajectory for both arms.";
                    return false;
                }
                break;
            }
            case MotionGroup::Primary:
            {
                assert(q_start.size() == ndof_left_);
                PoseQuaternion pose_start_primary = kine_ptr_primary_->GetFK(q_start, robot_property_.tcp_frame);

                std::vector<Eigen::Matrix<double, 7, 1>> poses_primary;
                poses_primary.emplace_back(pose_start_primary.ToVectorXd());
                assert(move_request.value.size() % (ndof_left_) == 0);
                const int num_points = move_request.value.size() / (ndof_left_);
                assert(num_points > 0);
                for (int i = 0; i < num_points; ++i)
                {
                    const auto &value = move_request.value.segment(i * ndof_left_, ndof_left_);
                    poses_primary.emplace_back(value);
                }
                pose_trajectory_primary_.Initialize(std::move(poses_primary), interp_order_);
                LOG_INFO << "Primary arm motion plan: trajectory: \n"
                         << pose_trajectory_primary_.ToString();

                std::vector<Eigen::Matrix<double, ndof_left_, 1>> trajectory_primary;
                auto trajectory_ok = kine_ptr_primary_->RunGlobalRedundancyResolutionForJointTrajectory(q_start, pose_trajectory_primary_,
                                                                                                        trajectory_primary, robot_property_.tcp_frame);

                joint_trajectory_primary_.SetJointTrajectory(std::move(trajectory_primary));
                if (!trajectory_ok)
                {
                    LOG_CRIT << "Failed to compute joint trajectory for primary arm.";
                    return false;
                }
                break;
            }
            case MotionGroup::Secondary:
            {
                assert(q_start.size() == ndof_right_);
                PoseQuaternion pose_start_secondary = kine_ptr_secondary_->GetFK(q_start, robot_property_.tcp_frame_secondary);

                std::vector<Eigen::Matrix<double, 7, 1>> poses_secondary;
                poses_secondary.emplace_back(pose_start_secondary.ToVectorXd());
                assert(move_request.value.size() % (ndof_right_) == 0);
                const int num_points = move_request.value.size() / (ndof_right_);
                assert(num_points > 0);
                for (int i = 0; i < num_points; ++i)
                {
                    const auto &value = move_request.value.segment(i * ndof_right_, ndof_right_);
                    poses_secondary.emplace_back(value);
                }
                pose_trajectory_secondary_.Initialize(std::move(poses_secondary), interp_order_);
                LOG_INFO << "Secondary arm motion plan: trajectory: \n"
                         << pose_trajectory_secondary_.ToString();

                std::vector<Eigen::Matrix<double, ndof_right_, 1>> trajectory_secondary;
                auto trajectory_ok = kine_ptr_secondary_->RunGlobalRedundancyResolutionForJointTrajectory(
                    q_start, pose_trajectory_secondary_, trajectory_secondary, robot_property_.tcp_frame_secondary);

                joint_trajectory_secondary_.SetJointTrajectory(std::move(trajectory_secondary));
                if (!trajectory_ok)
                {
                    LOG_CRIT << "Failed to compute joint trajectory for secondary arm.";
                    return false;
                }
                break;
            }
            default:
                assert(false && "Unknown MotionGroup type");
                break;
            }

            if (!joint_trajectory_primary_.has_value() && !joint_trajectory_secondary_.has_value())
            {
                LOG_WARN << "No joint trajectory available for time parameterization.";
            }
            if (joint_trajectory_primary_.has_value())
            {
                std::vector<double> time_diff;
                parameterizeJointTrajectory<ndof_left_>(joint_trajectory_primary_, time_diff, &time_primary_);
                LOG_INFO << "Primary arm planned joint trajectory: \n"
                         << joint_trajectory_primary_.ToString();
                trajectory_spline_primary_.fromJointTrajectory(joint_trajectory_primary_, time_diff);
            }
            if (joint_trajectory_secondary_.has_value())
            {
                std::vector<double> time_diff;
                parameterizeJointTrajectory<ndof_right_>(joint_trajectory_secondary_, time_diff, &time_secondary_);
                LOG_INFO << "Secondary arm planned joint trajectory: \n"
                         << joint_trajectory_secondary_.ToString();
                trajectory_spline_secondary_.fromJointTrajectory(joint_trajectory_secondary_, time_diff);
            }
            total_time_ = std::max(time_primary_, time_secondary_);

            return true;
        }

        void ClearCache()
        {
            current_phase_ = 0.0;
            total_time_ = 0.0;
            time_primary_ = 0.0;
            time_secondary_ = 0.0;
            pose_trajectory_primary_.clear();
            pose_trajectory_secondary_.clear();
            trajectory_spline_primary_.segments.clear();
            trajectory_spline_secondary_.segments.clear();
        }

    protected:
        const bool wait_trajectory_upon_finish_;
        const double controller_dt_{0.01};
        const bool refine_cmd{true}; // refine joint command to reduce error from given trajectory

        // JointTrajectory<ndof_left_> joint_trajectory_primary_;
        // JointTrajectory<ndof_right_> joint_trajectory_secondary_;
        TrajectorySpline<ndof_left_> trajectory_spline_primary_;
        TrajectorySpline<ndof_right_> trajectory_spline_secondary_;
        PoseTrajectory pose_trajectory_primary_;
        PoseTrajectory pose_trajectory_secondary_;

        double current_phase_;
        double time_primary_, time_secondary_, total_time_;

        std::shared_ptr<kinematics::YaikKinematics> kine_ptr_primary_;
        std::shared_ptr<kinematics::YaikKinematics> kine_ptr_secondary_;
    };

} // namespace robot_server_lib
