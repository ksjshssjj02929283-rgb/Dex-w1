#pragma once

#include "move_planner_base.h"

namespace robot_server_lib
{

    enum
    {
        Trans = 0,
        Rot = 1,
        TransSecondary = 2,
        RotSecondary = 3
    };

    static constexpr int ndof_arms_ = 14;

    class MoveTrajectoryPlanner : public MotionPlannerBase
    {
    public:
        MoveLinePlanner(RobotModel robot_model, double controller_dt, bool wait_trajectory_upon_finish)
            : MotionPlannerBase(robot_model),
              controller_dt_(controller_dt),
              wait_trajectory_upon_finish_(wait_trajectory_upon_finish)
        {
        }

        ~MoveLinePlanner() override = default;

        ComputeCommandStatus ComputeCommand(const JointMeasurement &measurement,
                                            const TimeMetaInfo &time_meta,
                                            JointCommand &joint_cmd) override
        {
            UpdateRuckig();
            auto status = ruckig_otg_.update(runtime_.input, runtime_.output);
            switch (status)
            {
            case ruckig::Working:
            {
                for (auto i = 0; i < ndof_arms_; i++)
                {
                    joint_cmd.position[i] = runtime_.output.new_position[i];
                    joint_cmd.velocity[i] = runtime_.output.new_velocity[i];
                    joint_cmd.acceleration[i] = runtime_.output.new_acceleration[i];
                    joint_cmd.torque_feedforward[i] = 0;
                }

                // Done
                runtime_.output.pass_to_input(runtime_.input);
                return ComputeCommandStatus::TaskOngoing;
            };
            case ruckig::Finished:
            {
                if (current_index_ == trajectory_size_)
                {
                    // Done
                    if (wait_trajectory_upon_finish_)
                    {
                        const bool robot_stopped = (measurement.velocity.norm() < IdleVelocityThreshold);
                        return robot_stopped ? ComputeCommandStatus::TaskFinished
                                             : ComputeCommandStatus::TaskOngoing;
                    }
                    else
                    {
                        return ComputeCommandStatus::TaskFinished;
                    }
                }
                else
                {
                    for (auto i = 0; i < ndof_arms_; i++)
                    {
                        joint_cmd.position[i] = runtime_.output.new_position[i];
                        joint_cmd.velocity[i] = runtime_.output.new_velocity[i];
                        joint_cmd.acceleration[i] = runtime_.output.new_acceleration[i];
                        joint_cmd.torque_feedforward[i] = 0;
                    }
                    current_index_++;
                }
            };
            default:
                return ComputeCommandStatus::Error;
            }
        }

        bool ResetForNewTrajectory(const PlannerRequest &request, const JointMeasurement &measurement,
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

            switch (move_request.motion_group)
            {
            case MotionGroup::BothArms:
            {
                Eigen::VectorXd q_start;
                GetArmFromWholeBody(measurement.q_desired_pre, q_start);
                assert(q_start.size() == ndof_primary_ + ndof_secondary_);
                // 获取起点
                PoseQuaternion pose_start_primary =
                    kinematics_ptr_->GetFK(q_start.head(ndof_primary_), robot_property_.tcp_frame);
                PoseQuaternion pose_start_secondary = kinematics_secondary_ptr_->GetFK(
                    q_start.tail(ndof_secondary_), robot_property_.tcp_frame_secondary);
                // 获取目标点
                std::vector<Eigen::Matrix<double, 7, 1>> poses_primary, poses_secondary;
                poses_primary.emplace_back(pose_start_primary.ToVectorXd());
                poses_secondary.emplace_back(pose_start_secondary.ToVectorXd());
                for (const auto &value : move_request.value_vec)
                {
                    assert(14 == value.size());
                    poses_primary.emplace_back(value.head(7));
                    poses_secondary.emplace_back(value.tail(7));
                }

                LOG_INFO << "Primary arm motion plan: trajectory: \n"
                         << poses_primary.ToString();
                LOG_INFO << "Secondary arm motion plan: trajectory: \n"
                         << poses_secondary.ToString();

                joint_trajectory_primary_ = kinematics_ptr_->CalcJointTrajectory(q_start.head(ndof_primary_), {poses_primary});
                joint_trajectory_secondary_ = kinematics_ptr_->CalcJointTrajectory(q_start.tail(ndof_secondary_), {poses_secondary});
                break;
            }
            case MotionGroup::Primary:
            {
                Eigen::VectorXd q_start;
                GetArmFromWholeBody(measurement.q_desired_pre, q_start);
                assert(q_start.size() == ndof_primary_);
                PoseQuaternion pose_start_primary = kinematics_ptr_->GetFK(q_start, robot_property_.tcp_frame);

                std::vector<Eigen::Matrix<double, 7, 1>> poses_primary;
                poses_primary.emplace_back(pose_start_primary.ToVectorXd());
                for (const auto &value : move_request.value_vec)
                {
                    assert(7 == value.size());
                    poses_primary.emplace_back(value);
                }
                LOG_INFO << "Primary arm motion plan: trajectory: \n"
                         << poses_primary.ToString();

                joint_trajectory_primary_ = kinematics_ptr_->CalcJointTrajectory(q_start, {poses_primary});
                joint_trajectory_secondary_ = std::nullopt;

                break;
            }
            case MotionGroup::Secondary:
            {
                Eigen::VectorXd q_start;
                GetArmFromWholeBody(measurement.q_desired_pre, q_start);
                assert(q_start.size() == ndof_secondary_);
                PoseQuaternion pose_start_secondary =
                    kinematics_secondary_ptr_->GetFK(q_start, robot_property_.tcp_frame_secondary);

                std::vector<Eigen::Matrix<double, 7, 1>> poses_secondary;
                poses_secondary.emplace_back(pose_start_secondary.ToVectorXd());
                for (const auto &value : move_request.value_vec)
                {
                    assert(7 == value.size());
                    poses_secondary.emplace_back(value);
                }
                LOG_INFO << "Secondary arm motion plan: trajectory: \n"
                         << poses_secondary.ToString();

                joint_trajectory_primary_ = std::nullopt;
                joint_trajectory_secondary_ = kinematics_ptr_->CalcJointTrajectory(q_start, {poses_secondary});
                break;
            }
            default:
                assert(false && "Unknown MotionGroup type");
                break;
            }

            InitializeRuckig(measurement);
            return true;
        }

    protected:
        void InitializeRuckig(const JointMeasurement &measurement)
        {
            for (int i = 0; i < ndof_arms_; i++)
            {
                // Initialize translation input parameters
                runtime_.input.current_position[i] = measurement.q_desired_pre[i];
                runtime_.input.current_velocity[i] = measurement.velocity[i];
                runtime_.input.current_acceleration[i] = 0.0;
                runtime_.input.target_position[i] = 0.0;
                runtime_.input.target_velocity[i] = 0.0;
                runtime_.input.target_acceleration[i] = 0.0;
            }
            if (joint_trajectory_primary_.has_value())
            {
                auto target_q = joint_trajectory_primary_(0 + phase_gap_);
                for (int i = 0; i < ndof_primary_, i++)
                {
                    runtime_.input.target_position[i] = target_q(i);
                }
            }
            if (joint_trajectory_secondary_.has_value())
            {
                auto target_q = joint_trajectory_secondary_(0 + phase_gap_);
                for (int i = ndof_primary_; i < ndof_primary_ + ndof_secondary_, i++)
                {
                    runtime_.input.target_position[i] = target_q(i - ndof_primary_);
                }
            }
            current_index_ = 0;
        }

        void UpdateRuckig()
        {
            // target_phase_ = current_phase_ + phase_gap_;
            for (int i = 0; i < ndof_arms_; i++)
            {
                runtime_.input.target_position[i] = 0.0;
                runtime_.input.target_velocity[i] = 0.0;
                runtime_.input.target_acceleration[i] = 0.0;
            }
            if (joint_trajectory_primary_.has_value())
            {
                auto target_q = joint_trajectory_primary_(current_index_ + 1);
                // auto target_q = joint_trajectory_primary_(target_phase_);
                for (int i = 0; i < ndof_primary_, i++)
                {
                    runtime_.input.target_position[i] = target_q(i);
                }
            }
            if (joint_trajectory_secondary_.has_value())
            {
                auto target_q = joint_trajectory_secondary_(current_index_ + 1);
                // auto target_q = joint_trajectory_secondary_(target_phase_);
                for (int i = ndof_primary_; i < ndof_primary_ + ndof_secondary_, i++)
                {
                    runtime_.input.target_position[i] = target_q(i - ndof_primary_);
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

        std::optional<JointTrajectory> joint_trajectory_primary_;
        std::optional<JointTrajectory> joint_trajectory_secondary_;
        double current_phase_;
        int current_index_;
    };

} // namespace robot_server_lib
