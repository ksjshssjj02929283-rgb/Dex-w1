#include "joint_streaming_planner.h"

namespace robot_server_lib {

JointStreamingPlanner::JointStreamingPlanner(
    RobotModel robot_model,
    std::shared_ptr<SingleWriterLatestBuffer<StreamingCommand>> streaming_buffer,
    double controller_dt)
    : MotionPlannerBase(robot_model), streaming_buffer_(streaming_buffer), ruckig_otg_(controller_dt_) {
    controller_dt_ = controller_dt;
    for (auto i = 0; i < ndof_robot; i++) {
        runtime_.input.max_velocity[i] = streaming_bounds_.max_velocity;
        runtime_.input.max_acceleration[i] = streaming_bounds_.max_acceleration;
        runtime_.input.max_jerk[i] = streaming_bounds_.max_jerk;
    }
}

JointStreamingPlanner::~JointStreamingPlanner() = default;

ComputeCommandStatus JointStreamingPlanner::ComputeCommand(const JointMeasurement &measurement,
                                                           const TimeMetaInfo &time_meta,
                                                           JointCommand &output_joint_cmd) {
    if (!runtime_.streaming_finished) {
        StreamingCommand cmd_buffer;
        bool received = streaming_buffer_->PeekLatest(&cmd_buffer);
        // 如果收到了新的命令，并且请求ID匹配，则更新最新命令
        if (received && cmd_buffer.request_id == runtime_.request_id) {
            JointPositionDof target_ndof = measurement.q_desired_pre;
            utils::MapJointToWholeBody(this->robot_model_, cmd_buffer.command, target_ndof, this->motion_group_);
            cmd_buffer.command = target_ndof;
            runtime_.latest_cmd.emplace(cmd_buffer);
        }
    }

    // 如果从startstreaming后没有收到任何流式命令，检查是否超时, 超时则结束任务
    if (!runtime_.latest_cmd.has_value()) {
        output_joint_cmd.position = measurement.q_desired_pre;
        output_joint_cmd.velocity.setZero();
        output_joint_cmd.acceleration.setZero();
        output_joint_cmd.torque_feedforward.setZero();
        auto time_interval = (time_meta.current_time.time_in_ns() - request_start_time_.time_in_ns()) * 1e-9;
        return time_interval < StreamingCommandTimeout ? ComputeCommandStatus::TaskOngoing
                                                       : ComputeCommandStatus::TaskFinished;
    }

    const StreamingCommand &latest_cmd = runtime_.latest_cmd.value();

    // 检查是否超时, 如果超时则认为流式命令结束
    bool is_timeout = false;
    if (latest_cmd.received_time.is_valid()) {
        auto time_interval = (time_meta.current_time.time_in_ns() - latest_cmd.received_time.time_in_ns()) * 1e-9;
        is_timeout = time_interval > StreamingCommandTimeout;
    }
    runtime_.streaming_finished = latest_cmd.streaming_finished || is_timeout;

    if (no_ruckig_mode_) {
        // 直接输出最新命令作为关节命令
        bool robot_stopped = true;
        for (auto i = 0; i < ndof_robot; i++) {
            // 如果超时则保持当前指令位置
            output_joint_cmd.position[i] = is_timeout ? measurement.q_desired_pre[i] : latest_cmd.command[i];
            output_joint_cmd.velocity[i] = 0.0;
            output_joint_cmd.acceleration[i] = 0.0;
            output_joint_cmd.torque_feedforward[i] = 0.0;
            robot_stopped = robot_stopped && measurement.velocity[i] < IdleVelocityThreshold;
        }
        if (!command_finished_time_.is_valid()) {
            command_finished_time_ = TimePointSystem::Now();
        }
        robot_stopped =
            robot_stopped || ((TimePointSystem::Now().time_in_ns() - command_finished_time_.time_in_ns()) * 1e-9 >
                              StreamingCommandTimeout);
        // 需同时满足机器人静止和streaming_finished为true，才能结束本次streaming
        return runtime_.streaming_finished && robot_stopped ? ComputeCommandStatus::TaskFinished
                                                            : ComputeCommandStatus::TaskOngoing;
    }

    // 检测输入是否更新
    bool target_updated = true;
    if (runtime_.previous_cmd.has_value()) {
        target_updated = ((runtime_.previous_cmd.value().command - latest_cmd.command).norm()) > PositionThreshold;
    }
    // 如果输入更新了且没有超时则更新Ruckig输入
    if (target_updated && !is_timeout) {
        for (auto i = 0; i < ndof_robot; i++) {
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
    if (status == ruckig::Working) {
        for (auto i = 0; i < ndof_robot; i++) {
            output_joint_cmd.position[i] = runtime_.output.new_position[i];
            output_joint_cmd.velocity[i] = runtime_.output.new_velocity[i];
            output_joint_cmd.acceleration[i] = runtime_.output.new_acceleration[i];
            output_joint_cmd.torque_feedforward[i] = 0;
        }
        runtime_.output.pass_to_input(runtime_.input);
        return ComputeCommandStatus::TaskOngoing;
    } else if (status == ruckig::Finished) {
        bool robot_stopped = true;
        for (auto i = 0; i < ndof_robot; i++) {
            output_joint_cmd.position[i] = runtime_.output.new_position[i];
            output_joint_cmd.velocity[i] = 0.0;
            output_joint_cmd.acceleration[i] = 0.0;
            output_joint_cmd.torque_feedforward[i] = 0.0;
            robot_stopped = robot_stopped && measurement.velocity[i] < IdleVelocityThreshold;
        }
        if (!command_finished_time_.is_valid()) {
            command_finished_time_ = TimePointSystem::Now();
        }
        robot_stopped =
            robot_stopped || ((TimePointSystem::Now().time_in_ns() - command_finished_time_.time_in_ns()) * 1e-9 >
                              StreamingCommandTimeout);
        // 需同时满足机器人静止和streaming_finished为true，才能结束本次streaming
        if (runtime_.streaming_finished && robot_stopped) {
            LOG_INFO << "JointStreamingPlanner: TaskFinished, measurement velocity: "
                     << measurement.velocity.transpose()
                     << ", measurement position: " << measurement.position.transpose();
            return ComputeCommandStatus::TaskFinished;
        } else {
            return ComputeCommandStatus::TaskOngoing;
        }
    } else {
        return ComputeCommandStatus::Error;
    }
}

bool JointStreamingPlanner::ResetForNewTrajectory(const PlannerRequest &request,
                                                  const JointMeasurement &measurement,
                                                  const TimeMetaInfo &time_meta) {
    assert(request.data.has_value());
    assert(std::holds_alternative<MoveRequest>(request.data.value()));

    // 解析MoveRequest
    auto move_request = std::get<MoveRequest>(request.data.value());
    SetMotionGroup(move_request.motion_group);

    // 是否为无Ruckig模式
    if (move_request.optional_mode.has_value()) {
        no_ruckig_mode_ = (bool)move_request.optional_mode.value();
    } else {
        no_ruckig_mode_ = false;
    }

    runtime_.request_id = request.request_id;
    runtime_.streaming_finished = false;
    runtime_.latest_cmd = std::nullopt;
    runtime_.previous_cmd = std::nullopt;

    // 初始化Ruckig输出
    for (auto i = 0; i < ndof_robot; i++) {
        runtime_.output.new_position[i] = measurement.q_desired_pre[i];
        runtime_.output.new_velocity[i] = 0.0;
        runtime_.output.new_acceleration[i] = 0.0;
    }

    // 记录请求开始时间（用于判断没有收到任何流式命令时的超时）
    request_start_time_ = TimePointSystem::Now();

    // 记录请求开始时间（用于判断没有收到任何流式命令时的超时）, 重置命令完成时间
    request_start_time_ = TimePointSystem::Now();
    command_finished_time_ = TimePointSystem::Invalid();

    return true;
}

}  // namespace robot_server_lib
