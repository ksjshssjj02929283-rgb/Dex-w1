#include "torque_controller_manager.h"

namespace robot_server_lib {

TorqueControllerManager::TorqueControllerManager(RobotModel robot_model,
                                                 PlannerManagerBuffers manager_buffers,
                                                 TorqueControlBuffer robot_buffers)
    : robot_model_(std::move(robot_model)),
      manager_buffers_(std::move(manager_buffers)),
      robot_buffers_(std::move(robot_buffers)) {
    Initialize();
}

TorqueControllerManager::~TorqueControllerManager() = default;

void TorqueControllerManager::Initialize() {
    // Initialize the runtime data
    runtime_data_.loop_counter = -1;

    runtime_data_.time_meta.current_time = TimePointSystem::Now();
    runtime_data_.time_meta.measurement_time = TimePointSystem::Invalid();
    runtime_data_.time_meta.episode_index = 0;
    runtime_data_.time_meta.episode_start_time = runtime_data_.time_meta.current_time;

    // Initialize torque controller
    torque_controller_ = std::make_shared<TorqueControllerInvDynamic>(robot_model_);
}

void TorqueControllerManager::LoopOnce(const PlannerManagerCommand* planner_command,
                                       const JointCommand& control_reference) {
    const auto t_start = std::chrono::steady_clock::now();
    runtime_data_.loop_counter++;

    // 尝试获取最新的关节测量值 Try to peek the latest joint raw_measurement
    runtime_data_.time_meta.current_time = TimePointSystem::Now();
    if (!robot_buffers_.joint_raw_measurement->PeekLatest(&runtime_data_.raw_measurement)) {
        runtime_data_.time_meta.measurement_time = TimePointSystem::Invalid();
        runtime_data_.raw_measurement = JointRawMeasurement{};
    } else {
        runtime_data_.time_meta.measurement_time = TimePointSystem::Now();
    }
    if (planner_command == nullptr) {
        runtime_data_.input_joint_cmd.time_point = TimePointSystem::Invalid();
    } else {
        runtime_data_.input_joint_cmd.cmd = *planner_command;
        runtime_data_.input_joint_cmd.cmd.cmd = control_reference;
        runtime_data_.input_joint_cmd.time_point = TimePointSystem::Now();
    }
    const auto t_input = std::chrono::steady_clock::now();

    ProcessResponse response;
    response.re_run = true;  // first run
    while (response.re_run) {
        switch (controller_state_) {
            case TorqueControllerState::Init:
                response = InitProcess();
                break;
            case TorqueControllerState::Idle:
                response = IdleProcess();
                break;
            case TorqueControllerState::Decelerating:
                response = DeceleratingProcess();
                break;
            case TorqueControllerState::Operating:
                response = OperatingProcess();
                break;
            default:
                assert(false && "Unknown state in TorqueControllerManager");
        }
    }
    const auto t_state = std::chrono::steady_clock::now();

    // Publish
    auto t_compute = t_state;
    if (TimePointSystem::IsValid(runtime_data_.time_meta.measurement_time)) {
        torque_controller_->ComputeCommand(runtime_data_.raw_measurement, runtime_data_.input_joint_cmd.cmd,
                                           runtime_data_.output_cmd);
        t_compute = std::chrono::steady_clock::now();
        JointMeasurement manager_measurement{};
        manager_measurement.position = runtime_data_.raw_measurement.position;
        manager_measurement.velocity = runtime_data_.raw_measurement.velocity;
        manager_measurement.effort = runtime_data_.raw_measurement.torque;
        // 规划续接使用原始目标，避免将整形延迟反馈给规划器。
        manager_measurement.q_desired_pre =
            controller_state_ == TorqueControllerState::Operating && planner_command != nullptr
                ? planner_command->cmd.position
                : runtime_data_.output_cmd.q_desired;
        manager_measurement.sensor_wrench = runtime_data_.raw_measurement.sensor_wrench;
        manager_buffers_.joint_measurement->AppendNewElement(manager_measurement);
        robot_buffers_.torque_controller_command->AppendNewElement(runtime_data_.output_cmd);
    } else {
        // No valid raw measurement, reset the manager_measurement and torque command buffers
        manager_buffers_.joint_measurement->Reset();
        robot_buffers_.torque_controller_command->Reset();
        if (runtime_data_.loop_counter % 3000 == 0) {
            LOG_CRIT << "No Raw joint measurement available, skipping loop.";
        }
    }
    const auto t_end = std::chrono::steady_clock::now();
    const auto total_us = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();
    if (total_us > torque_ctrl_dt_micro) {
        LOG_CRIT_THROTTLE(1000)
            << "[TorqueControllerManager] Total=" << total_us
            << "us | Input=" << std::chrono::duration_cast<std::chrono::microseconds>(t_input - t_start).count()
            << " State=" << std::chrono::duration_cast<std::chrono::microseconds>(t_state - t_input).count()
            << " Compute=" << std::chrono::duration_cast<std::chrono::microseconds>(t_compute - t_state).count()
            << " Publish=" << std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_compute).count();
    }
}

ProcessResponse TorqueControllerManager::InitProcess() {
    ProcessResponse response;
    last_cmd_sequence_number_ = runtime_data_.input_joint_cmd.cmd.sequence_number;
    if (TimePointSystem::IsValid(runtime_data_.time_meta.measurement_time)) {
        ChangeState(TorqueControllerState::Idle);
        response.re_run = true;
        return response;
    }
    return response;
}

ProcessResponse TorqueControllerManager::IdleProcess() {
    ProcessResponse response;
    if (TimePointSystem::IsInvalid(runtime_data_.time_meta.measurement_time)) {
        ChangeState(TorqueControllerState::Init);
        response.re_run = true;
        return response;
    }
    assert(TimePointSystem::IsValid(runtime_data_.raw_measurement.time_point));
    Int64 received_cmd_sequence = runtime_data_.input_joint_cmd.cmd.sequence_number;
    if (current_episode_index_ == runtime_data_.time_meta.episode_index) {
        if (received_cmd_sequence >= 0 && received_cmd_sequence > last_cmd_sequence_number_) {
            last_cmd_sequence_number_ = received_cmd_sequence;
            ChangeState(TorqueControllerState::Operating);
            response.re_run = true;
            return response;
        }
    } else {
        // A new idle episode
        current_episode_index_ = runtime_data_.time_meta.episode_index;
        last_cmd_sequence_number_ = received_cmd_sequence;
        torque_controller_->ResetForNewEpisode(runtime_data_.raw_measurement, controller_state_);
    }

    return response;
}

ProcessResponse TorqueControllerManager::DeceleratingProcess() {
    ProcessResponse response;
    if (TimePointSystem::IsInvalid(runtime_data_.time_meta.measurement_time)) {
        ChangeState(TorqueControllerState::Init);
        response.re_run = true;
        return response;
    }
    assert(TimePointSystem::IsValid(runtime_data_.raw_measurement.time_point));
    Int64 received_cmd_sequence = runtime_data_.input_joint_cmd.cmd.sequence_number;

    if (current_episode_index_ == runtime_data_.time_meta.episode_index) {
        if (received_cmd_sequence >= 0 && received_cmd_sequence > last_cmd_sequence_number_) {
            last_cmd_sequence_number_ = received_cmd_sequence;
            ChangeState(TorqueControllerState::Operating);
            response.re_run = true;
            return response;
        }

        // Speed is slow enough for idle
        if (runtime_data_.raw_measurement.velocity.norm() < IdleVelocityThreshold) {
            ChangeState(TorqueControllerState::Idle);
            response.re_run = true;
            return response;
        }
    } else {
        // A new episode
        current_episode_index_ = runtime_data_.time_meta.episode_index;
        last_cmd_sequence_number_ = received_cmd_sequence;
        torque_controller_->ResetForNewEpisode(runtime_data_.raw_measurement, controller_state_);
    }
    return response;
}

ProcessResponse TorqueControllerManager::OperatingProcess() {
    ProcessResponse response;
    if (TimePointSystem::IsInvalid(runtime_data_.time_meta.measurement_time)) {
        ChangeState(TorqueControllerState::Init);
        response.re_run = true;
        return response;
    }
    assert(TimePointSystem::IsValid(runtime_data_.raw_measurement.time_point));
    const auto& received_cmd_time = runtime_data_.input_joint_cmd.time_point;
    const Int64 received_cmd_seq = runtime_data_.input_joint_cmd.cmd.sequence_number;

    if (current_episode_index_ == runtime_data_.time_meta.episode_index) {
        // In this episode, check if the cmd is too old
        const auto now = TimePointSystem::Now();
        const bool cmd_delayed = now.time_in_ns() > received_cmd_time.time_in_ns() + MaxDelayInNs;
        if (received_cmd_seq < 0 || cmd_delayed) {
            last_cmd_sequence_number_ = received_cmd_seq;
            ChangeState(TorqueControllerState::Decelerating);
            response.re_run = true;
            return response;
        }
    } else {
        // A new episode
        current_episode_index_ = runtime_data_.time_meta.episode_index;
        last_cmd_sequence_number_ = received_cmd_seq;
        torque_controller_->ResetForNewEpisode(runtime_data_.raw_measurement, controller_state_);
    }

    return response;
}

void TorqueControllerManager::ChangeState(TorqueControllerState new_state) {
    assert(controller_state_ != new_state);
    runtime_data_.time_meta.episode_index += 1;
    runtime_data_.time_meta.episode_start_time = runtime_data_.time_meta.current_time;
    controller_state_ = new_state;
    LOG_INFO << "TorqueControllerManager state changed to: " << ToString(controller_state_);
}
}  // namespace robot_server_lib
