#pragma once

#include "move_planner_base.h"

namespace robot_server_lib
{
    static constexpr double step_forward_threshold = 0.75;

    class MoveTrajectoryPlanner : public MotionPlannerBase
    {
    public:
        MoveTrajectoryPlanner(RobotModel robot_model, double controller_dt, bool wait_trajectory_upon_finish)
            : MotionPlannerBase(robot_model),
              controller_dt_(controller_dt),
              ruckig_otg_(controller_dt),
              wait_trajectory_upon_finish_(wait_trajectory_upon_finish)
        {
            for (int i = 0; i < ndof_robot; ++i)
            {
                runtime_.input.max_velocity[i] = joint_bounds_.max_velocity;
                runtime_.input.max_acceleration[i] = joint_bounds_.max_acceleration;
                runtime_.input.max_jerk[i] = joint_bounds_.max_jerk;
            }
            current_phase_ = 0.0;

            kine_ptr_primary_ = std::dynamic_pointer_cast<kinematics::YaikKinematics>(kinematics_ptr_);
            kine_ptr_secondary_ = std::dynamic_pointer_cast<kinematics::YaikKinematics>(kinematics_secondary_ptr_);
        }

        ~MoveTrajectoryPlanner() override = default;

        ComputeCommandStatus ComputeCommand(const JointMeasurement &measurement, const TimeMetaInfo &time_meta, JointCommand &joint_cmd) override
        {
            JointPositionDof q_ndof = measurement.q_desired_pre;
            Eigen::VectorXd q_arms;
            GetArmFromWholeBody(q_ndof, q_arms, MotionGroup::BothArms);
            // update ruckig target
            double ratio_left = 1.0, ratio_right = 1.0;
            if (joint_trajectory_primary_.has_value())
            {
                auto q_target = joint_trajectory_primary_(current_phase_ + phase_gap_);
                auto q_prev = joint_trajectory_primary_(current_phase_);
                auto gap = (q_target - q_prev).norm();
                ratio_left = gap > 1e-4 ? (q_arms.head(ndof_primary_) - q_prev).norm() / gap : 1.0;
            }
            if (joint_trajectory_secondary_.has_value())
            {
                auto q_target = joint_trajectory_secondary_(current_phase_ + phase_gap_);
                auto q_prev = joint_trajectory_secondary_(current_phase_);
                auto gap = (q_target - q_prev).norm();
                ratio_right = gap > 1e-4 ? (q_arms.tail(ndof_secondary_) - q_prev).norm() / gap : 1.0;
            }

            // Current target almost reached, update the next target
            if (ratio_left > step_forward_threshold && ratio_right > step_forward_threshold)
            {
                if (current_phase_ + phase_gap_ < 1.0)
                {
                    current_phase_ += phase_gap_;
                    // update targets
                    SetRuckigTarget(measurement);
                }
            }
            LOG_INFO << "current_phase_ : " << std::to_string(current_phase_) << "\n";

            auto status = ruckig_otg_.update(runtime_.input, runtime_.output);
            for (auto i = 0; i < ndof_robot; i++)
            {
                q_ndof[i] = runtime_.output.new_position[i];
            }
            // if(refine_q_cmd)
            // {
            //     Eigen::VectorXd q_primary, q_secondary;
            //     GetArmFromWholeBody(q_ndof, q_primary, MotionGroup::Primary);
            //     GetArmFromWholeBody(q_ndof, q_secondary, MotionGroup::Secondary);
            //     Eigen::VectorXd q_primary, q_secondary;
            //     kine_ptr_primary_
            // }

            switch (status)
            {
            case ruckig::Working:
            {
                for (auto i = 0; i < ndof_robot; i++)
                {
                    joint_cmd.position[i] = q_ndof[i];
                    joint_cmd.velocity[i] = 0.0;
                    joint_cmd.acceleration[i] = 0;
                    joint_cmd.torque_feedforward[i] = 0;
                }

                // Done
                runtime_.output.pass_to_input(runtime_.input);
                return ComputeCommandStatus::TaskOngoing;
            };
            case ruckig::Finished:
            {
                for (auto i = 0; i < ndof_robot; i++)
                {
                    joint_cmd.position[i] = q_ndof[i];
                    joint_cmd.velocity[i] = 0.0;
                    joint_cmd.acceleration[i] = 0;
                    joint_cmd.torque_feedforward[i] = 0;
                }
                if (current_phase_ + phase_gap_ >= 1.0)
                {
                    // Done
                    const bool robot_stopped = (measurement.velocity.norm() < IdleVelocityThreshold);
                    if (!wait_trajectory_upon_finish_ || robot_stopped)
                    {
                        if (robot_stopped)
                        {
                            ClearCache();
                            return ComputeCommandStatus::TaskFinished;
                        }
                    }
                }
                runtime_.output.pass_to_input(runtime_.input);
                return ComputeCommandStatus::TaskOngoing;
            };
            default:
                return ComputeCommandStatus::Error;
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
            for (int i = 0; i < ndof_robot; ++i)
            {
                runtime_.input.max_velocity[i] = joint_bounds_.max_velocity * speed_ratio;
                runtime_.input.max_acceleration[i] = joint_bounds_.max_acceleration * speed_ratio;
            }

            Eigen::VectorXd q_start;
            GetArmFromWholeBody(measurement.q_desired_pre, q_start);

            switch (move_request.motion_group)
            {
            case MotionGroup::BothArms:
            {
                int ndof_arms_ = ndof_primary_ + ndof_secondary_;
                assert(q_start.size() == ndof_arms_);
                // 获取起点
                PoseQuaternion pose_start_primary = kine_ptr_primary_->GetFK(q_start.head(ndof_primary_), robot_property_.tcp_frame);
                PoseQuaternion pose_start_secondary = kine_ptr_secondary_->GetFK(q_start.tail(ndof_secondary_), robot_property_.tcp_frame_secondary);

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
                    poses_primary.emplace_back(value.head(ndof_primary_));
                    poses_secondary.emplace_back(value.tail(ndof_secondary_));
                }
                pose_trajectory_primary_.InitializePosesAndTimes(std::move(poses_primary));
                pose_trajectory_secondary_.InitializePosesAndTimes(std::move(poses_secondary));
                LOG_INFO << "Primary arm motion plan: trajectory: \n"
                         << pose_trajectory_primary_.ToString();
                LOG_INFO << "Secondary arm motion plan: trajectory: \n"
                         << pose_trajectory_secondary_.ToString();

                std::vector<Eigen::Matrix<double, ndof_primary_, 1>> trajectory_primary;
                std::vector<Eigen::Matrix<double, ndof_secondary_, 1>> trajectory_secondary;
                auto trajectory_ok_primary = kine_ptr_primary_->RunGlobalRedundancyResolutionForJointTrajectory(
                    q_start.head(ndof_primary_), pose_trajectory_primary_, trajectory_primary, robot_property_.tcp_frame);
                auto trajectory_ok_secondary = kine_ptr_secondary_->RunGlobalRedundancyResolutionForJointTrajectory(
                    q_start.tail(ndof_secondary_), pose_trajectory_secondary_, trajectory_secondary, robot_property_.tcp_frame_secondary);
                if (!trajectory_ok_primary || !trajectory_ok_secondary)
                {
                    LOG_CRIT << "Failed to compute joint trajectory for both arms.";
                    return false;
                }

                joint_trajectory_primary_.SetJointTrajectory(std::move(trajectory_primary));
                joint_trajectory_secondary_.SetJointTrajectory(std::move(trajectory_secondary));
                LOG_INFO << "Primary arm joint trajectory: \n"
                         << joint_trajectory_primary_.ToString()
                         << "Secondary arm joint trajectory: \n"
                         << joint_trajectory_secondary_.ToString();
                phase_gap_ = std::min(1.0 / (joint_trajectory_primary_.size() - 1), 1.0 / (joint_trajectory_secondary_.size() - 1));
                break;
            }
            case MotionGroup::Primary:
            {
                assert(q_start.size() == ndof_primary_);
                PoseQuaternion pose_start_primary = kine_ptr_primary_->GetFK(q_start, robot_property_.tcp_frame);

                std::vector<Eigen::Matrix<double, 7, 1>> poses_primary;
                poses_primary.emplace_back(pose_start_primary.ToVectorXd());
                assert(move_request.value.size() % (ndof_primary_) == 0);
                const int num_points = move_request.value.size() / (ndof_primary_);
                assert(num_points > 0);
                for (int i = 0; i < num_points; ++i)
                {
                    const auto &value = move_request.value.segment(i * ndof_primary_, ndof_primary_);
                    poses_primary.emplace_back(value);
                }
                pose_trajectory_primary_.InitializePosesAndTimes(std::move(poses_primary));
                LOG_INFO << "Primary arm motion plan: trajectory: \n"
                         << pose_trajectory_primary_.ToString();

                std::vector<Eigen::Matrix<double, ndof_primary_, 1>> trajectory_primary;
                auto trajectory_ok =
                    kine_ptr_primary_->RunGlobalRedundancyResolutionForJointTrajectory(q_start, pose_trajectory_primary_, trajectory_primary, robot_property_.tcp_frame);
                if (!trajectory_ok)
                {
                    LOG_CRIT << "Failed to compute joint trajectory for primary arm.";
                    return false;
                }

                joint_trajectory_primary_.SetJointTrajectory(std::move(trajectory_primary));
                LOG_INFO << "Primary arm joint trajectory: \n"
                         << joint_trajectory_primary_.ToString();
                phase_gap_ = 1.0 / (joint_trajectory_primary_.size() - 1);
                break;
            }
            case MotionGroup::Secondary:
            {
                assert(q_start.size() == ndof_secondary_);
                PoseQuaternion pose_start_secondary = kine_ptr_secondary_->GetFK(q_start, robot_property_.tcp_frame_secondary);

                std::vector<Eigen::Matrix<double, 7, 1>> poses_secondary;
                poses_secondary.emplace_back(pose_start_secondary.ToVectorXd());
                assert(move_request.value.size() % (ndof_secondary_) == 0);
                const int num_points = move_request.value.size() / (ndof_secondary_);
                assert(num_points > 0);
                for (int i = 0; i < num_points; ++i)
                {
                    const auto &value = move_request.value.segment(i * ndof_secondary_, ndof_secondary_);
                    poses_secondary.emplace_back(value);
                }
                pose_trajectory_secondary_.InitializePosesAndTimes(std::move(poses_secondary));
                LOG_INFO << "Secondary arm motion plan: trajectory: \n"
                         << pose_trajectory_secondary_.ToString();

                std::vector<Eigen::Matrix<double, ndof_secondary_, 1>> trajectory_secondary;
                auto trajectory_ok =
                    kine_ptr_secondary_->RunGlobalRedundancyResolutionForJointTrajectory(q_start, pose_trajectory_secondary_, trajectory_secondary, robot_property_.tcp_frame_secondary);
                if (!trajectory_ok)
                {
                    LOG_CRIT << "Failed to compute joint trajectory for secondary arm.";
                    return false;
                }

                joint_trajectory_secondary_.SetJointTrajectory(std::move(trajectory_secondary));
                LOG_INFO << "Secondary arm joint trajectory: \n"
                         << joint_trajectory_secondary_.ToString();
                phase_gap_ = 1.0 / (joint_trajectory_secondary_.size() - 1);
                break;
            }
            default:
                assert(false && "Unknown MotionGroup type");
                break;
            }

            for (int i = 0; i < ndof_robot; i++)
            {
                runtime_.input.current_position[i] = measurement.q_desired_pre[i];
                runtime_.input.current_velocity[i] = measurement.velocity[i];
                runtime_.input.current_acceleration[i] = 0.0;
            }
            // target initialization
            SetRuckigTarget(measurement);
            return true;
        }

    protected:
        void SetRuckigTarget(const JointMeasurement &measurement)
        {
            JointPositionDof q_dof = measurement.q_desired_pre;
            if (joint_trajectory_primary_.has_value())
            {
                auto q_target_left = joint_trajectory_primary_(current_phase_ + phase_gap_);
                MapArmToWholeBody(q_target_left, q_dof, MotionGroup::Primary);
            }
            if (joint_trajectory_secondary_.has_value())
            {
                auto q_target_right = joint_trajectory_secondary_(current_phase_ + phase_gap_);
                MapArmToWholeBody(q_target_right, q_dof, MotionGroup::Secondary);
            }
            for (int i = 0; i < ndof_robot; i++)
            {
                runtime_.input.target_position[i] = q_dof(i);
                runtime_.input.target_velocity[i] = 0.0;
                runtime_.input.target_acceleration[i] = 0.0;
            }
        }

        void ClearCache()
        {
            current_phase_ = 0.0;
            pose_trajectory_primary_.clear();
            pose_trajectory_secondary_.clear();
            joint_trajectory_primary_.clear();
            joint_trajectory_secondary_.clear();
        }

        const bool wait_trajectory_upon_finish_;
        const double controller_dt_{0.01};
        ruckig::Ruckig<ndof_robot> ruckig_otg_;
        // cache
        struct
        {
            ruckig::InputParameter<ndof_robot> input;
            ruckig::OutputParameter<ndof_robot> output;
        } runtime_;

        JointTrajectory<ndof_primary_> joint_trajectory_primary_;
        JointTrajectory<ndof_secondary_> joint_trajectory_secondary_;
        PoseTrajectory pose_trajectory_primary_;
        PoseTrajectory pose_trajectory_secondary_;

        double current_phase_;
        double phase_gap_;

        std::shared_ptr<kinematics::YaikKinematics> kine_ptr_primary_;
        std::shared_ptr<kinematics::YaikKinematics> kine_ptr_secondary_;
    };

} // namespace robot_server_lib
