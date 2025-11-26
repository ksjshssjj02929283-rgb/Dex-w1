#include "ee_pose_streaming_planner.h"

namespace robot_server_lib
{

EEPoseStreamingPlanner::EEPoseStreamingPlanner(RobotModel robot_model, std::shared_ptr<SingleWriterLatestBuffer<StreamingCommand>> streaming_buffer,
                                               double controller_dt)
    : MotionPlannerBase(robot_model), streaming_buffer_(streaming_buffer), ruckig_otg_(controller_dt)
{
    controller_dt_ = controller_dt;
    for (auto i = 0; i < ndof_robot; i++)
    {
        runtime_.input.max_velocity[i] = joint_bounds_.max_velocity;
        runtime_.input.max_acceleration[i] = joint_bounds_.max_acceleration;
        runtime_.input.max_jerk[i] = joint_bounds_.max_jerk;
    }
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
        output_joint_cmd.position = measurement.q_desired_pre;
        output_joint_cmd.velocity.setZero();
        output_joint_cmd.acceleration.setZero();
        output_joint_cmd.torque_feedforward.setZero();
        auto time_interval = (time_meta.current_time.time_in_ns() - request_start_time_.time_in_ns()) * 1e-9;
        return time_interval < StreamingCommandTimeout ? ComputeCommandStatus::TaskOngoing : ComputeCommandStatus::TaskFinished;
    }

    const StreamingCommand &latest_cmd = runtime_.latest_cmd.value();

    // 检查是否超时, 如果超时则认为流式命令结束
    bool is_timeout = false;
    if (latest_cmd.received_time.is_valid())
    {
        auto time_interval = (time_meta.current_time.time_in_ns() - latest_cmd.received_time.time_in_ns()) * 1e-9;
        is_timeout = time_interval > StreamingCommandTimeout;
    }
    runtime_.streaming_finished = latest_cmd.streaming_finished || is_timeout;

    // 检测输入是否更新
    bool target_updated = true;
    if (runtime_.previous_cmd.has_value())
    {
        target_updated = ((runtime_.previous_cmd.value().command - latest_cmd.command).norm()) > PositionThreshold;
    }

    // 如果输入更新了则更新Ruckig输入
    if (target_updated)
    {
        for (auto i = 0; i < ndof_robot; i++)
        {
            runtime_.input.current_position[i] = measurement.q_desired_pre[i];
            runtime_.input.current_velocity[i] = runtime_.output.new_velocity[i];
            runtime_.input.current_acceleration[i] = runtime_.output.new_acceleration[i];
            runtime_.input.target_position[i] = latest_cmd.command[i];
            runtime_.input.target_velocity[i] = 0.0;
            runtime_.input.target_acceleration[i] = 0.0;
        }
    }
    runtime_.previous_cmd = latest_cmd;

    // Ruckig进行轨迹计算
    auto status = ruckig_otg_.update(runtime_.input, runtime_.output);

    // Ruckig输出
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
    else if (status == ruckig::Finished)
    {
        bool robot_stopped = true;
        for (auto i = 0; i < ndof_robot; i++)
        {
            output_joint_cmd.position[i] = runtime_.output.new_position[i];
            output_joint_cmd.velocity[i] = 0.0;
            output_joint_cmd.acceleration[i] = 0.0;
            output_joint_cmd.torque_feedforward[i] = 0.0;
            robot_stopped = robot_stopped && measurement.velocity[i] < IdleVelocityThreshold;
        }
        // 需同时满足机器人静止和streaming_finished为true，才能结束本次streaming
        if (runtime_.streaming_finished && robot_stopped)
        {
            LOG_INFO << "EEPoseStreamingPlanner: TaskFinished, measurement velocity: " << measurement.velocity.transpose();
            return ComputeCommandStatus::TaskFinished;
        }
        else
        {
            return ComputeCommandStatus::TaskOngoing;
        }    
    }
    else
    {
        return ComputeCommandStatus::Error;
    }
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
    runtime_.previous_cmd = std::nullopt;

    // 初始化Ruckig输出
    for (auto i = 0; i < ndof_robot; i++)
    {
        runtime_.output.new_position[i] = measurement.q_desired_pre[i];
        runtime_.output.new_velocity[i] = 0.0;
        runtime_.output.new_acceleration[i] = 0.0;
    }

    // 记录请求开始时间（用于判断没有收到任何流式命令时的超时）
    request_start_time_ = TimePointSystem::Now();
    return true;
}

bool EEPoseStreamingPlanner::ToJointStreamingCommand(StreamingCommand &cmd, const JointMeasurement &measurement)
{
    Eigen::VectorXd joint_cmd;
    utils::GetJointFromWholeBody(this->robot_model_, measurement.q_desired_pre, joint_cmd, this->motion_group_);
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

        if (!kinematics_primary_ptr_->GetIK(pose_cmd_primary, q_primary, robot_property_.tcp_frame) ||
            !kinematics_secondary_ptr_->GetIK(pose_cmd_secondary, q_secondary, robot_property_.tcp_frame_secondary))
        {
            LOG_CRIT << "Inverse Kinematics failed";
            return false;
        }
        utils::MapJointToWholeBody(this->robot_model_, q_primary, joint_cmd_ndof, MotionGroup::Primary);
        utils::MapJointToWholeBody(this->robot_model_, q_secondary, joint_cmd_ndof, MotionGroup::Secondary);
        cmd.command = joint_cmd_ndof;
        break;
    }
    case MotionGroup::Primary:
    {
        assert(7 == cmd.command.size());
        auto pose_cmd = utils::VectorXdToPoseQuaternion(cmd.command);
        assert(joint_cmd.size() == ndof_primary_);
        if (!kinematics_primary_ptr_->GetIK(pose_cmd, joint_cmd, robot_property_.tcp_frame))
        {
            LOG_CRIT << "Inverse Kinematics failed";
            return false;
        }
        utils::MapJointToWholeBody(this->robot_model_, joint_cmd, joint_cmd_ndof, this->motion_group_);
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
        utils::MapJointToWholeBody(this->robot_model_, joint_cmd, joint_cmd_ndof, this->motion_group_);
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
