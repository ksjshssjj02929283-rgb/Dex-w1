#pragma once

#include "move_planner_base.h"

namespace robot_server_lib
{

class JointStreamingPlanner : public MotionPlannerBase
{
   public:
    JointStreamingPlanner(RobotModel robot_model, std::shared_ptr<SingleWriterLatestBuffer<StreamingCommand>> streaming_buffer, double controller_dt)
        : MotionPlannerBase(robot_model), streaming_buffer_(streaming_buffer), controller_dt_(controller_dt), ruckig_otg_(controller_dt)
    {
        for (auto i = 0; i < ndof_robot; i++)
        {
            runtime_.input.max_velocity[i] = joint_bounds_.max_velocity;
            runtime_.input.max_acceleration[i] = joint_bounds_.max_acceleration;
            runtime_.input.max_jerk[i] = joint_bounds_.max_jerk;
        }
    }

    ~JointStreamingPlanner() override = default;

    ComputeCommandStatus ComputeCommand(const JointMeasurement &measurement, const TimeMetaInfo &time_meta, JointCommand &output_joint_cmd) override
    {
        // 尝试从缓存中获取最新的StreamingCommand
        if (!runtime_.streaming_finished)
        {
            StreamingCommand cmd_buffer;
            bool received = streaming_buffer_->PeekLatest(&cmd_buffer);
            if (received && cmd_buffer.request_id == runtime_.request_id)
            {
                runtime_.latest_cmd.emplace(cmd_buffer);
            }
        }
        // streaming_buffer_没有值时的初始化
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
            if (norm_diff < latest_cmd.command.size() * PositionThreshold)
            {
                update_ruckig = false;
            }
        }
        JointPositionDof command_ndof = measurement.q_desired_pre;
        // latest_cmd.command按照joints_map映射到ndof_command
        MapArmToWholeBody(latest_cmd.command, command_ndof);
        if (update_ruckig)
        {
            for (auto i = 0; i < ndof_robot; i++)
            {
                runtime_.input.current_position[i] = measurement.q_desired_pre[i];
                runtime_.input.current_velocity[i] = measurement.velocity[i];
                runtime_.input.current_acceleration[i] = runtime_.output.new_acceleration[i];
                runtime_.input.target_position[i] = command_ndof[i];
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
            return runtime_.streaming_finished ? ComputeCommandStatus::TaskFinished : ComputeCommandStatus::TaskOngoing;
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
        else
        {
            return ComputeCommandStatus::Error;
        }
    }

    bool ResetForNewTrajectory(const PlannerRequest &request, const JointMeasurement &measurement, const TimeMetaInfo &time_meta) override
    {
        assert(request.data.has_value());
        assert(std::holds_alternative<MoveRequest>(request.data.value()));
        auto move_request = std::get<MoveRequest>(request.data.value());
        SetMotionGroup(move_request.motion_group);

        runtime_.request_id = request.request_id;
        runtime_.streaming_finished = false;
        runtime_.latest_cmd = std::nullopt;
        runtime_.previous_cmd = std::nullopt;

        return true;
    }

   protected:
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