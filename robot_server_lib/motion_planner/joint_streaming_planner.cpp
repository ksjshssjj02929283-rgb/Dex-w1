#include "joint_streaming_planner.h"

namespace robot_server_lib {

JointStreamingPlanner::JointStreamingPlanner(
    RobotModel robot_model,
    std::shared_ptr<SingleWriterLatestBuffer<StreamingCommand>> streaming_buffer,
    double controller_dt)
    : MotionPlannerBase(robot_model), streaming_buffer_(streaming_buffer), ruckig_otg_(controller_dt_) {
    controller_dt_ = controller_dt;
    // 从配置读取流式运动限制参数
    streaming_bounds_ = robot_model.params_ptr->streaming_bounds;
    LOG_INFO << "JointStreamingPlanner initialized with streaming bounds: Velocity: " << streaming_bounds_.max_velocity
             << ", Acceleration: " << streaming_bounds_.max_acceleration << ", Jerk: " << streaming_bounds_.max_jerk;
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
            int joint_count = utils::GetJointCount(this->robot_model_, this->motion_group_);
            // 检查数据长度
            auto cmd_size = static_cast<int>(cmd_buffer.command.size());
            if (use_acceleration_ && cmd_size != 3 * joint_count) {
                LOG_CRIT << "JointStreamingPlanner: use_acceleration enabled but command size " << cmd_size
                         << " != 3 * joint_count " << 3 * joint_count;
                return ComputeCommandStatus::Error;
            }
            if (use_velocity_ && !use_acceleration_ && cmd_size != 2 * joint_count) {
                LOG_CRIT << "JointStreamingPlanner: use_velocity enabled but command size " << cmd_size
                         << " != 2 * joint_count " << 2 * joint_count;
                return ComputeCommandStatus::Error;
            }
            if (!use_velocity_ && !use_acceleration_ && cmd_size != joint_count) {
                LOG_CRIT << "JointStreamingPlanner: Neither use_velocity nor use_acceleration, "
                         << "but command size " << cmd_size << " != joint_count " << joint_count;
                return ComputeCommandStatus::Error;
            }

            // 提取位置、速度、加速度向量（部分关节维度）
            Eigen::VectorXd position_part(joint_count);
            Eigen::VectorXd velocity_part(joint_count);
            Eigen::VectorXd acceleration_part(joint_count);
            for (int i = 0; i < joint_count; i++) {
                position_part[i] = cmd_buffer.command[i];
            }
            if (use_velocity_) {
                for (int i = 0; i < joint_count; i++) {
                    velocity_part[i] = cmd_buffer.command[joint_count + i];
                }
            }
            if (use_acceleration_) {
                for (int i = 0; i < joint_count; i++) {
                    acceleration_part[i] = cmd_buffer.command[2 * joint_count + i];
                }
            }

            // 分别映射到全身关节
            JointPositionDof target_ndof = measurement.q_desired_pre;
            JointPositionDof velocity_ndof = JointPositionDof::Zero();
            JointPositionDof acceleration_ndof = JointPositionDof::Zero();

            utils::MapJointToWholeBody(this->robot_model_, position_part, target_ndof, this->motion_group_);
            if (use_velocity_) {
                utils::MapJointToWholeBody(this->robot_model_, velocity_part, velocity_ndof, this->motion_group_);
            }
            if (use_acceleration_) {
                utils::MapJointToWholeBody(this->robot_model_, acceleration_part, acceleration_ndof,
                                           this->motion_group_);
            }

            // 将映射后的指令重新存入 cmd_buffer（用于后续处理）
            cmd_buffer.command = target_ndof;
            cmd_buffer.target_velocity = velocity_ndof;
            cmd_buffer.target_acceleration = acceleration_ndof;
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
        return time_interval < streaming_timeout_ ? ComputeCommandStatus::TaskOngoing
                                                  : ComputeCommandStatus::TaskFinished;
    }

    const StreamingCommand &latest_cmd = runtime_.latest_cmd.value();

    // 如果有有效输入指令则检查指令是否超时, 如果超时则认为流式命令结束
    bool is_timeout = false;
    bool is_velocity_timeout = false;  // 速度超时，使用更短的时间
    if (latest_cmd.received_time.is_valid()) {
        auto time_interval = (time_meta.current_time.time_in_ns() - latest_cmd.received_time.time_in_ns()) * 1e-9;
        is_timeout = time_interval > streaming_timeout_;
        is_velocity_timeout = time_interval > streaming_velocity_timeout_;  // 更快触发速度停止
    }
    runtime_.streaming_finished = latest_cmd.streaming_finished || is_timeout;

    if (!use_ruckig_) {
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
        // 如果没有收到新的命令且机器人已经静止，则结束本次streaming
        robot_stopped =
            robot_stopped &&
            ((TimePointSystem::Now().time_in_ns() - command_finished_time_.time_in_ns()) * 1e-9 > streaming_timeout_);
        // 需同时满足机器人静止和streaming_finished为true，才能结束本次streaming
        return runtime_.streaming_finished && robot_stopped ? ComputeCommandStatus::TaskFinished
                                                            : ComputeCommandStatus::TaskOngoing;
    }

    // 检测输入是否更新（使用 FloatEqual 比较）
    bool target_updated = true;
    if (runtime_.previous_cmd.has_value()) {
        target_updated = false;
        for (auto i = 0; i < ndof_robot; i++) {
            if (!utils::FloatEqual(runtime_.previous_cmd.value().command[i], latest_cmd.command[i])) {
                target_updated = true;
                break;
            }
            if (use_velocity_ &&
                !utils::FloatEqual(runtime_.previous_cmd.value().target_velocity[i], latest_cmd.target_velocity[i])) {
                target_updated = true;
                break;
            }
            if (use_acceleration_ && !utils::FloatEqual(runtime_.previous_cmd.value().target_acceleration[i],
                                                        latest_cmd.target_acceleration[i])) {
                target_updated = true;
                break;
            }
        }
    }
    // 如果输入更新了且没有超时则更新Ruckig输入
    if (target_updated && !is_timeout) {
        for (auto i = 0; i < ndof_robot; i++) {
            runtime_.input.current_position[i] = runtime_.output.new_position[i];
            runtime_.input.current_velocity[i] = runtime_.output.new_velocity[i];
            runtime_.input.current_acceleration[i] = runtime_.output.new_acceleration[i];
            runtime_.input.target_position[i] = latest_cmd.command[i];
            runtime_.input.target_velocity[i] =
                (use_velocity_ && !is_velocity_timeout) ? latest_cmd.target_velocity[i] : 0.0;
            runtime_.input.target_acceleration[i] =
                (use_acceleration_ && !is_velocity_timeout) ? latest_cmd.target_acceleration[i] : 0.0;
        }
        // 每秒打印一次日志，内容包括接收到的命令帧数、位置、速度、加速度等信息
        if (!last_log_time_.is_valid() ||
            (time_meta.current_time.time_in_ns() - last_log_time_.time_in_ns()) * 1e-9 >= 1.0) {
            LOG_INFO << "Streaming received " << frame_count_ << " frames in last period, "
                     << "position = " << latest_cmd.command.transpose()
                     << ", velocity = " << latest_cmd.target_velocity.transpose()
                     << ", acceleration = " << latest_cmd.target_acceleration.transpose();
            frame_count_ = 0;
            last_log_time_ = time_meta.current_time;
        }
        frame_count_++;
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
            robot_stopped &&
            ((TimePointSystem::Now().time_in_ns() - command_finished_time_.time_in_ns()) * 1e-9 > streaming_timeout_);
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
        LOG_CRIT << "JointStreamingPlanner: Ruckig error, status = " << static_cast<int>(status);
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

    // 是否启用Ruckig
    if (move_request.use_ruckig.has_value()) {
        use_ruckig_ = move_request.use_ruckig.value();
    } else {
        use_ruckig_ = true;
    }

    // 超时时间设置
    if (move_request.streaming_timeout.has_value()) {
        streaming_timeout_ = move_request.streaming_timeout.value();
    } else {
        streaming_timeout_ = DefaultStreamingTimeout;
    }

    // Ruckig 速度超时时间设置（比 streaming_timeout 更短，用于快速停止）
    if (move_request.streaming_velocity_timeout.has_value()) {
        streaming_velocity_timeout_ = move_request.streaming_velocity_timeout.value();
    }

    // 目标速度和加速度配置
    if (move_request.use_velocity.has_value()) {
        use_velocity_ = move_request.use_velocity.value();
    } else {
        use_velocity_ = false;
    }
    if (move_request.use_acceleration.has_value()) {
        use_acceleration_ = move_request.use_acceleration.value();
    } else {
        use_acceleration_ = false;
    }
    runtime_.request_id = request.request_id;
    runtime_.streaming_finished = false;
    runtime_.stopping = false;
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

    // 重置日志计时器和帧计数器
    last_log_time_ = TimePointSystem::Invalid();
    frame_count_ = 0;

    return true;
}

}  // namespace robot_server_lib
