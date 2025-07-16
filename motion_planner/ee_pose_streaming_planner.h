#pragma once

#include "move_planner_base.h"

namespace robot_server_lib
{

class EEPoseStreamingPlanner : public MotionPlannerBase
{
   public:
    EEPoseStreamingPlanner(
        RobotModel robot_model,
        std::shared_ptr<SingleWriterLatestBuffer<StreamingCommand>> streaming_buffer,
        double controller_dt)
        : MotionPlannerBase(robot_model),
          streaming_buffer_(streaming_buffer),
          controller_dt_(controller_dt),
          ruckig_otg_(controller_dt)
    {
        for (auto i = 0; i < ndof_robot; i++)
        {
            runtime_.input.max_velocity[i] = joint_bounds_.max_velocity;
            runtime_.input.max_acceleration[i] = joint_bounds_.max_acceleration;
            runtime_.input.max_jerk[i] = joint_bounds_.max_jerk;
        }
    }

    ~EEPoseStreamingPlanner() override = default;

    ComputeCommandStatus ComputeCommand(const JointMeasurement &measurement,
                                        const TimeMetaInfo &time_meta,
                                        JointCommand &output_joint_cmd) override
    {
        if (!runtime_.streaming_finished)
        {
            // 尝试从缓存中获取最新的StreamingCommand
            StreamingCommand cmd_buffer;
            bool received = streaming_buffer_->PeekLatest(&cmd_buffer);
            if (received && cmd_buffer.request_id == runtime_.request_id)
            {
                assert(cmd_buffer.command.size() == 7 || cmd_buffer.command.size() == 14);
                // Convert command to joint positions
                if (!ToJointStreamingCommand(cmd_buffer, measurement))
                {
                    LOG_CRIT << "EEPoseStreamingPlanner: Solve IK failed";
                    return ComputeCommandStatus::Error;
                }
                runtime_.latest_cmd.emplace(cmd_buffer);
            }
        }
        if (!runtime_.latest_cmd.has_value())
        {
            output_joint_cmd.position = measurement.q_desired_pre;
            output_joint_cmd.velocity.setZero();
            output_joint_cmd.acceleration.setZero();
            output_joint_cmd.torque_feedforward.setZero();
            return ComputeCommandStatus::TaskOngoing;
        }

        const StreamingCommand &latest_cmd = runtime_.latest_cmd.value();
        assert(latest_cmd.request_id == runtime_.request_id);
        if (latest_cmd.streaming_finished)
            runtime_.streaming_finished = true;

        bool update_ruckig = true;
        if (runtime_.previous_cmd.has_value())
        {
            auto norm_diff = (runtime_.previous_cmd.value().command - latest_cmd.command).norm();
            if (norm_diff < PositionThreshold)
            {
                update_ruckig = false;
            }
        }

        if (update_ruckig)
        {
            for (auto i = 0; i < ndof_robot; i++)
            {
                runtime_.input.current_position[i] = measurement.position[i];
                runtime_.input.current_velocity[i] = measurement.velocity[i];
                runtime_.input.current_acceleration[i] = 0.0;
                runtime_.input.target_position[i] = latest_cmd.command[i];
                runtime_.input.target_velocity[i] = 0.0;
                runtime_.input.target_acceleration[i] = 0.0;
            }
        }
        runtime_.previous_cmd = latest_cmd;

        auto status = ruckig_otg_.update(runtime_.input, runtime_.output);
        if (status == ruckig::Finished)
        {
            for (auto i = 0; i < ndof_robot; i++)
            {
                output_joint_cmd.position[i] = runtime_.output.new_position[i];
                output_joint_cmd.velocity[i] = 0.0;
                output_joint_cmd.acceleration[i] = 0.0;
                output_joint_cmd.torque_feedforward[i] = 0.0;
            }

            return runtime_.streaming_finished ? ComputeCommandStatus::TaskFinished
                                               : ComputeCommandStatus::TaskOngoing;
        }
        if (status == ruckig::Working)
        {
            for (auto i = 0; i < ndof_robot; i++)
            {
                output_joint_cmd.position[i] = runtime_.output.new_position[i];
                output_joint_cmd.velocity[i] = runtime_.output.new_velocity[i];
                output_joint_cmd.acceleration[i] = runtime_.output.new_acceleration[i];
                output_joint_cmd.torque_feedforward[i] = 0;
            }

            runtime_.output.pass_to_input(runtime_.input);
            return ComputeCommandStatus::TaskOngoing;
        }
        return ComputeCommandStatus::TaskOngoing;
    }

    bool ResetForNewTrajectory(const PlannerRequest &request, const JointMeasurement &measurement,
                               const TimeMetaInfo &time_meta) override
    {
        assert(request.data.has_value());
        assert(std::holds_alternative<MoveRequest>(request.data.value()));
        auto move_request = std::get<MoveRequest>(request.data.value());
        SetMotionGroup(move_request.motion_group);

        runtime_.request_id = request.request_id;
        runtime_.streaming_finished = false;
        runtime_.latest_cmd = std::nullopt;
        runtime_.previous_cmd = std::nullopt;
        // Reset buffer
        streaming_buffer_->AppendNewElement(StreamingCommand{});

        return true;
    }

   protected:
    bool ToJointStreamingCommand(StreamingCommand &cmd, const JointMeasurement &measurement)
    {
        Eigen::VectorXd joint_cmd;
        GetArmFromWholeBody(measurement.q_desired_pre, joint_cmd);
        JointPositionDof joint_cmd_ndof = measurement.q_desired_pre;
        switch (motion_group_)
        {
        case MotionGroup::BothArm:
        {
            assert(14 == cmd.command.size());
            auto pose_cmd_primary =
                utilities::VectorXdToPoseQuaternion(Eigen::VectorXd(cmd.command.head(7)));
            auto pose_cmd_secondary =
                utilities::VectorXdToPoseQuaternion(Eigen::VectorXd(cmd.command.tail(7)));

            assert(joint_cmd.size() == ndof_primary_ + ndof_secondary_);
            Eigen::VectorXd q_primary = joint_cmd.head(ndof_primary_);
            Eigen::VectorXd q_secondary = joint_cmd.tail(ndof_secondary_);

            if (!kinematics_ptr_->GetIK(pose_cmd_primary, q_primary, robot_property_.tcp_frame) ||
                !kinematics_secondary_ptr_->GetIK(pose_cmd_secondary, q_secondary,
                                                  robot_property_.tcp_frame_secondary))
            {
                LOG_CRIT << "Inverse Kinematics failed";
                return false;
            }
            MapArmToWholeBody(q_primary, joint_cmd_ndof, MotionGroup::Primary);
            MapArmToWholeBody(q_secondary, joint_cmd_ndof, MotionGroup::Secondary);
            cmd.command = joint_cmd_ndof;
            break;
        }
        case MotionGroup::Primary:
        {
            assert(7 == cmd.command.size());
            auto pose_cmd = utilities::VectorXdToPoseQuaternion(cmd.command);
            assert(joint_cmd.size() == ndof_primary_);
            if (!kinematics_ptr_->GetIK(pose_cmd, joint_cmd, robot_property_.tcp_frame))
            {
                LOG_CRIT << "Inverse Kinematics failed";
                return false;
            }
            MapArmToWholeBody(joint_cmd, joint_cmd_ndof);
            cmd.command = joint_cmd_ndof;

            break;
        }
        case MotionGroup::Secondary:
        {
            assert(7 == cmd.command.size());
            auto pose_cmd = utilities::VectorXdToPoseQuaternion(cmd.command);

            assert(joint_cmd.size() == ndof_secondary_);
            if (!kinematics_secondary_ptr_->GetIK(pose_cmd, joint_cmd,
                                                  robot_property_.tcp_frame_secondary))
            {
                LOG_CRIT << "Inverse Kinematics failed";
                return false;
            }
            MapArmToWholeBody(joint_cmd, joint_cmd_ndof);
            cmd.command = joint_cmd_ndof;
            break;
        }
        default:
            assert(false && "Unknown MotionGroup type");
            break;
        }
        return true;
    }

    std::shared_ptr<SingleWriterLatestBuffer<StreamingCommand>> streaming_buffer_;
    const double controller_dt_{0.01};
    ruckig::Ruckig<ndof_robot> ruckig_otg_;

    // cache
    struct
    {
        Int64 request_id{-1};
        bool streaming_finished{false};
        std::optional<StreamingCommand> latest_cmd;
        std::optional<StreamingCommand> previous_cmd;

        ruckig::InputParameter<ndof_robot> input;
        ruckig::OutputParameter<ndof_robot> output;
    } runtime_;
};

}  // namespace robot_server_lib