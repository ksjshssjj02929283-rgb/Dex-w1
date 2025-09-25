#include "joint_streaming_planner.h"

namespace robot_server_lib
{

JointStreamingPlanner::JointStreamingPlanner(RobotModel robot_model, std::shared_ptr<SingleWriterLatestBuffer<StreamingCommand>> streaming_buffer, double controller_dt)
    : MotionPlannerBase(robot_model), streaming_buffer_(streaming_buffer), controller_dt_(controller_dt)
{
}

JointStreamingPlanner::~JointStreamingPlanner() = default;

ComputeCommandStatus JointStreamingPlanner::ComputeCommand(const JointMeasurement &measurement, const TimeMetaInfo &time_meta, JointCommand &output_joint_cmd)
{
    if (!runtime_.streaming_finished)
    {
        StreamingCommand cmd_buffer;
        bool received = streaming_buffer_->PeekLatest(&cmd_buffer);
        if (received && cmd_buffer.request_id == runtime_.request_id)
        {
            JointPositionDof target_ndof = measurement.q_desired_pre;
            MapArmToWholeBody(cmd_buffer.command, target_ndof);
            cmd_buffer.command = target_ndof;
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
    bool is_timeout = false;
    if (latest_cmd.received_time.is_valid())
    {
        auto time_interval = (time_meta.current_time.time_in_ns() - latest_cmd.received_time.time_in_ns()) * 1e-9;
        is_timeout = time_interval > StreamingCommandTimeout;
    }
    if (latest_cmd.streaming_finished || is_timeout)
        runtime_.streaming_finished = true;

    for (auto i = 0; i < ndof_robot; i++)
    {
        output_joint_cmd.position[i] = latest_cmd.command[i];
        output_joint_cmd.velocity[i] = 0.0;
        output_joint_cmd.acceleration[i] = 0.0;
        output_joint_cmd.torque_feedforward[i] = 0.0;
    }
    if (!runtime_.streaming_finished)
    {
        return ComputeCommandStatus::TaskOngoing;
    }
    const bool robot_stopped = (measurement.velocity.norm() < IdleVelocityThreshold);
    return robot_stopped ? ComputeCommandStatus::TaskFinished : ComputeCommandStatus::TaskOngoing;
}

bool JointStreamingPlanner::ResetForNewTrajectory(const PlannerRequest &request, const JointMeasurement &measurement, const TimeMetaInfo &time_meta)
{
    assert(request.data.has_value());
    assert(std::holds_alternative<MoveRequest>(request.data.value()));
    auto move_request = std::get<MoveRequest>(request.data.value());
    SetMotionGroup(move_request.motion_group);

    runtime_.request_id = request.request_id;
    runtime_.streaming_finished = false;
    runtime_.latest_cmd = std::nullopt;

    return true;
}

} // namespace robot_server_lib
