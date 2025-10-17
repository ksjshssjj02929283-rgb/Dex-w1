#include "ee_pose_streaming_planner.h"

namespace robot_server_lib
{

EEPoseStreamingPlanner::EEPoseStreamingPlanner(RobotModel robot_model, std::shared_ptr<SingleWriterLatestBuffer<StreamingCommand>> streaming_buffer,
                                               double controller_dt)
    : MotionPlannerBase(robot_model), streaming_buffer_(streaming_buffer), controller_dt_(controller_dt)
{
}

EEPoseStreamingPlanner::~EEPoseStreamingPlanner() = default;

ComputeCommandStatus EEPoseStreamingPlanner::ComputeCommand(const JointMeasurement &measurement, const TimeMetaInfo &time_meta,
                                                            JointCommand &output_joint_cmd)
{
    if (!runtime_.streaming_finished)
    {
        StreamingCommand cmd_buffer;
        bool received = streaming_buffer_->PeekLatest(&cmd_buffer);
        if (received && cmd_buffer.request_id == runtime_.request_id)
        {
            assert(cmd_buffer.command.size() == 7 || cmd_buffer.command.size() == 14);
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
        auto time_interval = (time_meta.current_time.time_in_ns() - request_start_time_.time_in_ns()) * 1e-9;
        output_joint_cmd.position = measurement.q_desired_pre;
        output_joint_cmd.velocity.setZero();
        output_joint_cmd.acceleration.setZero();
        output_joint_cmd.torque_feedforward.setZero();
        return time_interval < StreamingCommandTimeout ? ComputeCommandStatus::TaskOngoing : ComputeCommandStatus::TaskFinished;
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
        output_joint_cmd.position[i] = is_timeout ? measurement.q_desired_pre[i] : latest_cmd.command[i];
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

bool EEPoseStreamingPlanner::ResetForNewTrajectory(const PlannerRequest &request, const JointMeasurement &measurement, const TimeMetaInfo &time_meta)
{
    assert(request.data.has_value());
    assert(std::holds_alternative<MoveRequest>(request.data.value()));
    auto move_request = std::get<MoveRequest>(request.data.value());
    SetMotionGroup(move_request.motion_group);

    runtime_.request_id = request.request_id;
    runtime_.streaming_finished = false;
    runtime_.latest_cmd = std::nullopt;

    // 记录请求开始时间（用于判断没有收到任何流式命令时的超时）
    request_start_time_ = TimePointSystem::Now();
    return true;
}

bool EEPoseStreamingPlanner::ToJointStreamingCommand(StreamingCommand &cmd, const JointMeasurement &measurement)
{
    Eigen::VectorXd joint_cmd;
    utils::GetArmFromWholeBody(this->robot_model_, measurement.q_desired_pre, joint_cmd, this->motion_group_);
    JointPositionDof joint_cmd_ndof = measurement.q_desired_pre;
    switch (motion_group_)
    {
    case MotionGroup::BothArms:
    {
        assert(14 == cmd.command.size());
        auto pose_cmd_primary = utils::VectorXdToPoseQuaternion(Eigen::VectorXd(cmd.command.head(7)));
        auto pose_cmd_secondary = utils::VectorXdToPoseQuaternion(Eigen::VectorXd(cmd.command.tail(7)));

        assert(joint_cmd.size() == ndof_primary_ + ndof_secondary_);
        Eigen::VectorXd q_primary = joint_cmd.head(ndof_primary_);
        Eigen::VectorXd q_secondary = joint_cmd.tail(ndof_secondary_);

        if (!kinematics_ptr_->GetIK(pose_cmd_primary, q_primary, robot_property_.tcp_frame) ||
            !kinematics_secondary_ptr_->GetIK(pose_cmd_secondary, q_secondary, robot_property_.tcp_frame_secondary))
        {
            LOG_CRIT << "Inverse Kinematics failed";
            return false;
        }
        utils::MapArmToWholeBody(this->robot_model_, q_primary, joint_cmd_ndof, MotionGroup::Primary);
        utils::MapArmToWholeBody(this->robot_model_, q_secondary, joint_cmd_ndof, MotionGroup::Secondary);
        cmd.command = joint_cmd_ndof;
        break;
    }
    case MotionGroup::Primary:
    {
        assert(7 == cmd.command.size());
        auto pose_cmd = utils::VectorXdToPoseQuaternion(cmd.command);
        assert(joint_cmd.size() == ndof_primary_);
        if (!kinematics_ptr_->GetIK(pose_cmd, joint_cmd, robot_property_.tcp_frame))
        {
            LOG_CRIT << "Inverse Kinematics failed";
            return false;
        }
        utils::MapArmToWholeBody(this->robot_model_, joint_cmd, joint_cmd_ndof, this->motion_group_);
        cmd.command = joint_cmd_ndof;
        break;
    }
    case MotionGroup::Secondary:
    {
        assert(7 == cmd.command.size());
        auto pose_cmd = utils::VectorXdToPoseQuaternion(cmd.command);
        assert(joint_cmd.size() == ndof_secondary_);
        if (!kinematics_secondary_ptr_->GetIK(pose_cmd, joint_cmd, robot_property_.tcp_frame_secondary))
        {
            LOG_CRIT << "Inverse Kinematics failed";
            return false;
        }
        utils::MapArmToWholeBody(this->robot_model_, joint_cmd, joint_cmd_ndof, this->motion_group_);
        cmd.command = joint_cmd_ndof;
        break;
    }
    default:
        assert(false && "Unknown MotionGroup type");
        break;
    }
    return true;
}

}  // namespace robot_server_lib
