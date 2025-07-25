#pragma once

#include "move_planner_base.h"

namespace robot_server_lib
{
    static constexpr int ndof_waist_ = 4;
    static constexpr int ndof_arms_ = 14;
    static constexpr int ndof_head_ = 2;
    static constexpr int ndof_left_ = 7;
    static constexpr int ndof_right_ = 7;
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
            for (int i = 0; i < ndof_arms_; ++i)
            {
                runtime_.input.max_velocity[i] = joint_bounds_.max_velocity;
                runtime_.input.max_acceleration[i] = joint_bounds_.max_acceleration;
                runtime_.input.max_jerk[i] = joint_bounds_.max_jerk;
            }
            current_index_ = 0;
            trajectory_size_ = 10;

            kine_ptr_primary_ = std::dynamic_pointer_cast<kinematics::YaikKinematics>(kinematics_ptr_);
            kine_ptr_secondary_ = std::dynamic_pointer_cast<kinematics::YaikKinematics>(kinematics_secondary_ptr_);
        }

        ~MoveTrajectoryPlanner() override = default;

        ComputeCommandStatus ComputeCommand(const JointMeasurement &measurement,
                                            const TimeMetaInfo &time_meta,
                                            JointCommand &joint_cmd) override
        {
            auto status = ruckig_otg_.update(runtime_.input, runtime_.output);

            JointPositionDof q_ndof = measurement.q_desired_pre;
            Eigen::VectorXd q_arms;
            GetArmFromWholeBody(q_ndof, q_arms, MotionGroup::BothArms);
            for (auto i = 0; i < ndof_arms_; i++)
            {
                q_arms[i] = runtime_.output.new_position[i];
            }
            MapArmToWholeBody(q_arms, q_ndof, MotionGroup::BothArms);

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
                if (current_index_ == trajectory_size_ - 1)
                {
                    // Done
                    const bool robot_stopped = (measurement.velocity.norm() < IdleVelocityThreshold);
                    if (!wait_trajectory_upon_finish_ || robot_stopped)
                    {
                        if (robot_stopped)
                        {
                            joint_trajectory_primary_.clear();
                            joint_trajectory_secondary_.clear();
                            return ComputeCommandStatus::TaskFinished;
                        }
                    }
                    else
                    {
                        runtime_.output.pass_to_input(runtime_.input);
                        return ComputeCommandStatus::TaskOngoing;
                    }
                }
                else
                {
                    for (auto i = 0; i < ndof_robot; i++)
                    {
                        joint_cmd.position[i] = q_ndof[i];
                        joint_cmd.velocity[i] = 0.0;
                        joint_cmd.acceleration[i] = 0;
                        joint_cmd.torque_feedforward[i] = 0;
                    }
                }
            };
            default:
                return ComputeCommandStatus::Error;
            }

            // update ruckig target
            double ratio_left = 1.0, ratio_right = 1.0;
            if (joint_trajectory_primary_.has_value())
            {
                auto &q_target = joint_trajectory_primary_(current_index_ + 1);
                auto &q_prev = joint_trajectory_primary_(current_index_);
                auto gap = (q_target - q_prev).norm();
                ratio_left = gap > 1e-4 ? (q_arms.head(ndof_left_) - q_prev).norm() / gap : 1.0;
            }
            if (joint_trajectory_secondary_.has_value())
            {
                auto &q_target = joint_trajectory_secondary_(current_index_ + 1);
                auto &q_prev = joint_trajectory_secondary_(current_index_);
                auto gap = (q_target - q_prev).norm();
                ratio_right = gap > 1e-4 ? (q_arms.tail(ndof_right_) - q_prev).norm() / gap : 1.0;
            }

            // Current target almost reached, update the next target
            if (ratio_left > step_forward_threshold && ratio_right > step_forward_threshold)
            {
                if (current_index_ + 1 < trajectory_size_ - 1)
                {
                    ++current_index_;
                    UpdateTarget();
                }
            }
        }

        bool
        ResetForNewTrajectory(const PlannerRequest &request, const JointMeasurement &measurement,
                              const TimeMetaInfo &time_meta) override
        {
            assert(request.data.has_value());
            assert(std::holds_alternative<MoveRequest>(request.data.value()));

            const auto &move_request = std::get<MoveRequest>(request.data.value());
            SetMotionGroup(move_request.motion_group);
            double speed_ratio = robot_property_.global_speed_ratio;
            if (move_request.speed.has_value())
            {
                speed_ratio = speed_ratio * std::clamp(move_request.speed.value(), 0.01, 1.0);
            }
            for (int i = 0; i < ndof_arms_; ++i)
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
                assert(q_start.size() == ndof_arms_);
                // 获取起点
                PoseQuaternion pose_start_primary =
                    kine_ptr_primary_->GetFK(q_start.head(ndof_left_), robot_property_.tcp_frame);
                PoseQuaternion pose_start_secondary = kine_ptr_secondary_->GetFK(
                    q_start.tail(ndof_right_), robot_property_.tcp_frame_secondary);

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
                auto pose_trajectory_primary = PoseTrajectory(poses_primary);
                auto pose_trajectory_secondary = PoseTrajectory(poses_secondary);
                LOG_INFO << "Primary arm motion plan: trajectory: \n"
                         << pose_trajectory_primary.ToString();
                LOG_INFO << "Secondary arm motion plan: trajectory: \n"
                         << pose_trajectory_secondary.ToString();

                std::vector<Eigen::Matrix<double, ndof_left_, 1>> trajectory_primary;
                std::vector<Eigen::Matrix<double, ndof_right_, 1>> trajectory_secondary;
                auto trajectory_ok_primary = kine_ptr_primary_->RunGlobalRedundancyResolutionForJointTrajectory(q_start.head(ndof_left_), pose_trajectory_primary, trajectory_primary);
                auto trajectory_ok_secondary = kine_ptr_secondary_->RunGlobalRedundancyResolutionForJointTrajectory(q_start.tail(ndof_right_), pose_trajectory_secondary, trajectory_secondary);
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
                auto pose_trajectory_primary = PoseTrajectory(poses_primary);
                LOG_INFO << "Primary arm motion plan: trajectory: \n"
                         << pose_trajectory_primary.ToString();

                std::vector<Eigen::Matrix<double, ndof_left_, 1>> trajectory_primary;
                auto trajectory_ok = kine_ptr_primary_->RunGlobalRedundancyResolutionForJointTrajectory(q_start, pose_trajectory_primary, trajectory_primary);

                joint_trajectory_primary_.SetJointTrajectory(std::move(trajectory_primary));
                joint_trajectory_secondary_.clear();
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
                PoseQuaternion pose_start_secondary =
                    kine_ptr_secondary_->GetFK(q_start, robot_property_.tcp_frame_secondary);

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
                auto poses_trajectory_secondary = PoseTrajectory(poses_secondary);
                LOG_INFO << "Secondary arm motion plan: trajectory: \n"
                         << poses_trajectory_secondary.ToString();

                std::vector<Eigen::Matrix<double, ndof_right_, 1>> trajectory_secondary;
                auto trajectory_ok = kine_ptr_secondary_->RunGlobalRedundancyResolutionForJointTrajectory(q_start, poses_trajectory_secondary, trajectory_secondary);

                joint_trajectory_primary_.clear();
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

            current_index_ = 0;
            InitializeRuckig(measurement);
            return true;
        }

    protected:
        void InitializeRuckig(const JointMeasurement &measurement)
        {
            Eigen::VectorXd q_start, qd_start;
            GetArmFromWholeBody(measurement.q_desired_pre, q_start, MotionGroup::BothArms);
            GetArmFromWholeBody(measurement.velocity, qd_start, MotionGroup::BothArms);
            assert(q_start.size() == ndof_arms_);
            assert(qd_start.size() == ndof_arms_);
            for (int i = 0; i < ndof_arms_; i++)
            {
                runtime_.input.current_position[i] = q_start(i);
                runtime_.input.current_velocity[i] = qd_start(i);
                runtime_.input.current_acceleration[i] = 0.0;

                runtime_.input.target_position[i] = q_start(i);
                runtime_.input.target_velocity[i] = 0.0;
                runtime_.input.target_acceleration[i] = 0.0;
            }
            if (joint_trajectory_primary_.has_value())
            {
                LOG_INFO << "Primary arm joint trajectory: \n"
                         << joint_trajectory_primary_.ToString();
                auto &q_target_left = joint_trajectory_primary_(0);
                for (int i = 0; i < ndof_left_; i++)
                {
                    runtime_.input.target_position[i] = q_target_left(i);
                }
            }
            if (joint_trajectory_secondary_.has_value())
            {
                LOG_INFO << "Secondary arm joint trajectory: \n"
                         << joint_trajectory_secondary_.ToString();
                auto &q_target_right = joint_trajectory_secondary_(0);
                for (int i = 0; i < ndof_right_; i++)
                {
                    runtime_.input.target_position[i + ndof_left_] = q_target_right(i);
                }
            }
        }

        void UpdateTarget()
        {
            if (joint_trajectory_primary_.has_value())
            {
                auto &q_target_left = joint_trajectory_primary_(current_index_);
                for (int i = 0; i < ndof_left_; i++)
                {
                    runtime_.input.target_position[i] = q_target_left(i);
                }
            }
            if (joint_trajectory_secondary_.has_value())
            {
                auto &q_target_right = joint_trajectory_secondary_(current_index_);
                for (int i = 0; i < ndof_right_; i++)
                {
                    runtime_.input.target_position[i + ndof_left_] = q_target_right(i);
                }
            }
        }

        const bool wait_trajectory_upon_finish_;
        const double controller_dt_{0.01};
        ruckig::Ruckig<ndof_arms_> ruckig_otg_;
        // cache
        struct
        {
            ruckig::InputParameter<ndof_arms_> input;
            ruckig::OutputParameter<ndof_arms_> output;
        } runtime_;

        JointTrajectory<ndof_left_> joint_trajectory_primary_;
        JointTrajectory<ndof_right_> joint_trajectory_secondary_;
        int current_index_;
        int trajectory_size_;

        std::shared_ptr<kinematics::YaikKinematics> kine_ptr_primary_;
        std::shared_ptr<kinematics::YaikKinematics> kine_ptr_secondary_;
    };

} // namespace robot_server_lib
