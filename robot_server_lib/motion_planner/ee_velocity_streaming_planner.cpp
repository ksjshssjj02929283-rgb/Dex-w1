#include "ee_velocity_streaming_planner.h"

namespace robot_server_lib {

EEVelocityStreamingPlanner::EEVelocityStreamingPlanner(
    RobotModel robot_model,
    std::shared_ptr<SingleWriterLatestBuffer<StreamingCommand>> streaming_buffer,
    double controller_dt)
    : MotionPlannerBase(robot_model), streaming_buffer_(streaming_buffer) {
    controller_dt_ = controller_dt;
}

EEVelocityStreamingPlanner::~EEVelocityStreamingPlanner() = default;

ComputeCommandStatus EEVelocityStreamingPlanner::ComputeCommand(const JointMeasurement& measurement,
                                                                const TimeMetaInfo& time_meta,
                                                                JointCommand& output_joint_cmd) {
    if (!runtime_.streaming_finished) {
        StreamingCommand cmd_buffer;
        bool received = streaming_buffer_->PeekLatest(&cmd_buffer);
        if (received && cmd_buffer.request_id == runtime_.request_id) {
            if (motion_group_ == MotionGroup::BothArms) {
                if (cmd_buffer.command.size() != 12) {
                    std::stringstream ss;
                    ss << "EEVelocityStreamingPlanner: BothArms data size error: " << cmd_buffer.command.size()
                       << ", expected 12.";
                    std::cout << ss.str() << "\n";
                    LOG_CRIT << ss.str();
                    return ComputeCommandStatus::Error;
                }
            } else {
                if (cmd_buffer.command.size() != 6) {
                    LOG_CRIT << "EEVelocityStreamingPlanner: Single-arm data size error: " << cmd_buffer.command.size()
                             << ", expected 6.";
                    return ComputeCommandStatus::Error;
                }
            }
            runtime_.latest_cmd.emplace(cmd_buffer);
        }
    }
    if (!runtime_.latest_cmd.has_value()) {
        output_joint_cmd.position = measurement.q_desired_pre;
        output_joint_cmd.velocity.setZero();
        output_joint_cmd.acceleration.setZero();
        output_joint_cmd.torque_feedforward.setZero();
        auto time_interval = (time_meta.current_time.time_in_ns() - request_start_time_.time_in_ns()) * 1e-9;
        return time_interval < streaming_timeout_ ? ComputeCommandStatus::TaskOngoing
                                                  : ComputeCommandStatus::TaskFinished;
    }

    const StreamingCommand& latest_cmd = runtime_.latest_cmd.value();

    // 检查是否超时
    bool is_timeout = false;
    if (latest_cmd.received_time.is_valid()) {
        auto time_interval = (time_meta.current_time.time_in_ns() - latest_cmd.received_time.time_in_ns()) * 1e-9;
        is_timeout = time_interval > streaming_timeout_;
    }
    runtime_.streaming_finished = latest_cmd.streaming_finished || is_timeout;

    // Initialize output joint velocity
    auto last_cmd_qdot = output_joint_cmd.velocity;
    output_joint_cmd.velocity.setZero();

    // Handle both single-arm and dual-arm cases
    Eigen::VectorXd desired_q_primary, desired_q_secondary;
    utils::GetJointFromWholeBody(this->robot_model_, measurement.q_desired_pre, desired_q_primary,
                                 MotionGroup::Primary);
    utils::GetJointFromWholeBody(this->robot_model_, measurement.q_desired_pre, desired_q_secondary,
                                 MotionGroup::Secondary);

    if (motion_group_ == MotionGroup::BothArms) {
        // Dual-arm mode: handle 12D command (6D for each arm)
        Vector6d input_velocity_primary, input_velocity_secondary;
        if (!runtime_.streaming_finished) {
            if (latest_cmd.command.size() != 12) {
                return ComputeCommandStatus::Error;
            }
            input_velocity_primary = latest_cmd.command.head<6>();
            input_velocity_secondary = latest_cmd.command.tail<6>();

            // Apply reference frame transformations for both arms
            if (reference_frame_ == ReferenceFrame::EEFrame) {
                // Primary arm transformation
                Eigen::Affine3d transform_primary(
                    pinocchio::GetFK(*this->robot_model_.primary_arm_model_ptr, desired_q_primary));
                input_velocity_primary.head<3>() = transform_primary.linear() * input_velocity_primary.head<3>();
                input_velocity_primary.tail<3>() = transform_primary.linear() * input_velocity_primary.tail<3>();

                // Secondary arm transformation
                Eigen::Affine3d transform_secondary(
                    pinocchio::GetFK(*this->robot_model_.secondary_arm_model_ptr, desired_q_secondary));
                input_velocity_secondary.head<3>() = transform_secondary.linear() * input_velocity_secondary.head<3>();
                input_velocity_secondary.tail<3>() = transform_secondary.linear() * input_velocity_secondary.tail<3>();
            } else if (reference_frame_ == ReferenceFrame::WorldBase) {
                // Primary arm transformation
                auto primary_armbase_pose_in_worldbase = pinocchio::GetFkByFrameId(
                    *this->robot_model_.primary_arm_model_ptr, desired_q_primary, this->primary_arm_base_id_);
                input_velocity_primary.head<3>() =
                    primary_armbase_pose_in_worldbase.rotation().transpose() * input_velocity_primary.head<3>();
                input_velocity_primary.tail<3>() =
                    primary_armbase_pose_in_worldbase.rotation().transpose() * input_velocity_primary.tail<3>();

                // Secondary arm transformation
                auto secondary_armbase_pose_in_worldbase = pinocchio::GetFkByFrameId(
                    *this->robot_model_.secondary_arm_model_ptr, desired_q_secondary, this->secondary_arm_base_id_);
                input_velocity_secondary.head<3>() =
                    secondary_armbase_pose_in_worldbase.rotation().transpose() * input_velocity_secondary.head<3>();
                input_velocity_secondary.tail<3>() =
                    secondary_armbase_pose_in_worldbase.rotation().transpose() * input_velocity_secondary.tail<3>();
            }
        } else {
            input_velocity_primary.setZero();
            input_velocity_secondary.setZero();
        }

        // Compute Jacobians for both arms
        Eigen::MatrixXd jacobian_primary =
            pinocchio::GetJacobiMatrix(*this->robot_model_.primary_arm_model_ptr, desired_q_primary);
        Eigen::MatrixXd jacobian_secondary =
            pinocchio::GetJacobiMatrix(*this->robot_model_.secondary_arm_model_ptr, desired_q_secondary);

        // Compute pseudo-inverses
        Eigen::MatrixXd jacobian_pinv_primary, jacobian_pinv_secondary;
        pinocchio::PseudoInverse(jacobian_primary, jacobian_pinv_primary);
        pinocchio::PseudoInverse(jacobian_secondary, jacobian_pinv_secondary);

        // Solve joint velocities for both arms
        runtime_.qdot_solved_primary = jacobian_pinv_primary * input_velocity_primary;
        runtime_.qdot_solved_secondary = jacobian_pinv_secondary * input_velocity_secondary;

        // Map both arm velocities to whole body
        utils::MapJointToWholeBody(this->robot_model_, runtime_.qdot_solved_primary, output_joint_cmd.velocity,
                                   MotionGroup::Primary);
        utils::MapJointToWholeBody(this->robot_model_, runtime_.qdot_solved_secondary, output_joint_cmd.velocity,
                                   MotionGroup::Secondary);

        // Maintain backward compatibility
        runtime_.qdot_solved = output_joint_cmd.velocity;
    } else {
        // Single-arm mode: handle 6D command
        Vector6d input_velocity;
        if (!runtime_.streaming_finished) {
            if (6 != latest_cmd.command.size()) {
                return ComputeCommandStatus::Error;
            }
            input_velocity = latest_cmd.command;

            // Apply reference frame transformation
            if (reference_frame_ == ReferenceFrame::EEFrame) {
                Eigen::Affine3d transform(pinocchio::GetFK(this->pinocchio_model_, desired_q_primary));
                input_velocity.head<3>() = transform.linear() * input_velocity.head<3>();
                input_velocity.tail<3>() = transform.linear() * input_velocity.tail<3>();
            } else if (reference_frame_ == ReferenceFrame::WorldBase) {
                int arm_base_id =
                    (motion_group_ == MotionGroup::Primary) ? this->primary_arm_base_id_ : this->secondary_arm_base_id_;
                auto armbase_pose_in_worldbase =
                    pinocchio::GetFkByFrameId(*this->robot_model_.model_ptr, measurement.q_desired_pre, arm_base_id);
                input_velocity.head<3>() = armbase_pose_in_worldbase.rotation().transpose() * input_velocity.head<3>();
                input_velocity.tail<3>() = armbase_pose_in_worldbase.rotation().transpose() * input_velocity.tail<3>();
            }
        } else {
            input_velocity.setZero();
        }

        // Compute Jacobian for the current arm
        Eigen::MatrixXd jacobian = pinocchio::GetJacobiMatrix(this->pinocchio_model_, desired_q_primary);

        // Compute pseudo-inverse
        Eigen::MatrixXd jacobian_pinv;
        pinocchio::PseudoInverse(jacobian, jacobian_pinv);

        // Solve joint velocity
        runtime_.qdot_solved = jacobian_pinv * input_velocity;

        // Map to whole body
        utils::MapJointToWholeBody(this->robot_model_, runtime_.qdot_solved, output_joint_cmd.velocity,
                                   this->motion_group_);
    }

    output_joint_cmd.position = measurement.q_desired_pre + output_joint_cmd.velocity * controller_dt_;
    output_joint_cmd.acceleration = (output_joint_cmd.velocity - last_cmd_qdot) / controller_dt_;
    output_joint_cmd.torque_feedforward.setZero();

    // Check robot stopped status - include both arm velocities in dual-arm mode
    bool robot_stopped = (measurement.velocity.array() < IdleVelocityThreshold).all();
    if (motion_group_ == MotionGroup::BothArms) {
        robot_stopped = robot_stopped && (runtime_.qdot_solved_primary.array().abs() < IdleVelocityThreshold).all() &&
                        (runtime_.qdot_solved_secondary.array().abs() < IdleVelocityThreshold).all();
    } else {
        robot_stopped = robot_stopped && (runtime_.qdot_solved.array().abs() < IdleVelocityThreshold).all();
    }

    // 需同时满足机器人静止和streaming_finished为true，才能结束本次streaming
    if (runtime_.streaming_finished && robot_stopped) {
        LOG_INFO << "EEVelocityStreamingPlanner: TaskFinished, measurement velocity: "
                 << measurement.velocity.transpose();
        return ComputeCommandStatus::TaskFinished;
    } else {
        return ComputeCommandStatus::TaskOngoing;
    }
}

bool EEVelocityStreamingPlanner::ResetForNewTrajectory(const PlannerRequest& request,
                                                       const JointMeasurement& measurement,
                                                       const TimeMetaInfo& time_meta) {
    assert(request.data.has_value());
    assert(std::holds_alternative<MoveRequest>(request.data.value()));
    auto move_request = std::get<MoveRequest>(request.data.value());
    SetMotionGroup(move_request.motion_group);

    // 超时时间设置
    if (move_request.streaming_timeout.has_value()) {
        streaming_timeout_ = move_request.streaming_timeout.value();
    } else {
        streaming_timeout_ = DefaultStreamingTimeout;
    }

    runtime_.request_id = request.request_id;
    runtime_.streaming_finished = false;
    runtime_.latest_cmd = std::nullopt;

    if (motion_group_ == MotionGroup::Primary) {
        runtime_.qdot_solved_primary.setZero(ndof_primary_);
        runtime_.qdot_solved = runtime_.qdot_solved_primary;  // Backward compatibility
        this->pinocchio_model_ = *this->robot_model_.primary_arm_model_ptr;
    } else if (motion_group_ == MotionGroup::Secondary) {
        runtime_.qdot_solved_secondary.setZero(ndof_secondary_);
        runtime_.qdot_solved = runtime_.qdot_solved_secondary;  // Backward compatibility
        this->pinocchio_model_ = *this->robot_model_.secondary_arm_model_ptr;
    } else if (motion_group_ == MotionGroup::BothArms) {
        runtime_.qdot_solved_primary.setZero(ndof_primary_);
        runtime_.qdot_solved_secondary.setZero(ndof_secondary_);
        runtime_.qdot_solved.setZero(ndof_primary_ + ndof_secondary_);  // Combined
    } else {
        LOG_CRIT << "EEVelocityStreamingPlanner: Unsupported motion group.";
        return false;
    }

    this->primary_arm_base_id_ =
        robot_model_.model_ptr->getFrameId(this->robot_model_.params_ptr->joints_primary_names[0]) - 1;
    this->secondary_arm_base_id_ =
        robot_model_.model_ptr->getFrameId(this->robot_model_.params_ptr->joints_secondary_names[0]) - 1;

    // 默认为eeframe速度控制, 默认单臂控制
    if (move_request.reference_frame.has_value()) {
        reference_frame_ = move_request.reference_frame.value();
    }

    // 记录请求开始时间（用于判断没有收到任何流式命令时的超时）
    request_start_time_ = TimePointSystem::Now();
    return true;
}

}  // namespace robot_server_lib
