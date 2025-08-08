#include "torque_controller_manager.h"

namespace robot_server_lib
{

TorqueControllerManager::TorqueControllerManager(RobotModel robot_model, PlannerManagerBuffers manager_buffers, TorqueControlBuffer robot_buffers)
    : robot_model_(std::move(robot_model)), manager_buffers_(std::move(manager_buffers)), robot_buffers_(std::move(robot_buffers))
{
    Initialize();
}

TorqueControllerManager::~TorqueControllerManager() = default;

void TorqueControllerManager::Initialize()
{
    // Initialize the runtime data
    runtime_data_.loop_counter = -1;

    auto &time_meta = runtime_data_.time_meta;
    time_meta.current_time = TimePointSystem::Now();
    time_meta.measurement_time = TimePointSystem::Invalid();
    time_meta.episode_index = 0;
    time_meta.episode_start_time = time_meta.current_time;

    // Initialize torque controller todo: 不用inv dynamic时的初始化
    idle_torque_controller_ = std::make_shared<IdleControllerInvDynamic>(robot_model_);
    deceleration_torque_controller_ = std::make_shared<DecelerationControllerInvDynamic>(robot_model_);
    operating_torque_controller_ = std::make_shared<OperatingControllerInvDynamic>(robot_model_);
}

void TorqueControllerManager::LoopOnce()
{
    auto &time_meta = runtime_data_.time_meta;
    auto &raw_measurement = runtime_data_.raw_measurement;
    auto &input_joint_cmd = runtime_data_.input_joint_cmd;
    auto &output_cmd = runtime_data_.output_cmd;
    runtime_data_.loop_counter++;

    // 尝试获取最新的关节测量值 Try to peek the latest joint raw_measurement
    time_meta.current_time = TimePointSystem::Now();
    if (!robot_buffers_.joint_raw_measurement->PeekLatest(&raw_measurement))
    {
        time_meta.measurement_time = TimePointSystem::Invalid();
    }
    else
    {
        time_meta.measurement_time = TimePointSystem::Now();
        // LOG_DEBUG << "Raw Measurement received:  " << raw_measurement.ToString();
    }

    // 获取最新的位置控制下发的joint command
    if (!manager_buffers_.planner_manager_command->PeekLatest(&input_joint_cmd.cmd))
    {
        input_joint_cmd.time_point = TimePointSystem::Invalid();
    }
    else
    {
        input_joint_cmd.time_point = TimePointSystem::Now();
    }

    ProcessResponse response;
    response.re_run = true;  // first run
    while (response.re_run)
    {
        switch (controller_state_)
        {
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

    assert(current_controller_ != nullptr);

    // Publish
    if (current_controller_->ComputeCommand(raw_measurement, input_joint_cmd.cmd, output_cmd))
    {
        JointMeasurement measurement_publish{};
        measurement_publish.position = raw_measurement.position;
        measurement_publish.velocity = raw_measurement.velocity;
        measurement_publish.effort = raw_measurement.effort;
        measurement_publish.q_desired_pre = output_cmd.q_desired;
        manager_buffers_.joint_measurement->AppendNewElement(measurement_publish);
        robot_buffers_.torque_command_vector->AppendNewElement(output_cmd.torque);
        std::ostringstream oss;
        oss << output_cmd.torque.transpose();
        // LOG_DEBUG << "Torque controller output: " << oss.str();
    }
}

ProcessResponse TorqueControllerManager::InitProcess()
{
    ProcessResponse response;
    last_cmd_sequence_number_ = runtime_data_.input_joint_cmd.cmd.sequence_number;
    // Measurement is valid
    if (TimePointSystem::IsValid(runtime_data_.raw_measurement.time_point))
    {
        ChangeState(TorqueControllerState::Idle);
        response.re_run = true;
        return response;
    }
    return response;
}

ProcessResponse TorqueControllerManager::IdleProcess()
{
    assert(TimePointSystem::IsValid(runtime_data_.raw_measurement.time_point));
    const auto &time_meta = runtime_data_.time_meta;
    const auto &raw_measurement = runtime_data_.raw_measurement;

    // 切换controller
    current_controller_ = idle_torque_controller_;

    ProcessResponse response;
    Int64 received_cmd_sequence = runtime_data_.input_joint_cmd.cmd.sequence_number;
    if (time_meta.episode_index == current_episode_index_)
    {
        if (received_cmd_sequence >= 0 && received_cmd_sequence > last_cmd_sequence_number_)
        {
            last_cmd_sequence_number_ = received_cmd_sequence;
            ChangeState(TorqueControllerState::Operating);
            response.re_run = true;
            return response;
        }
    }
    else
    {
        // A new idle episode
        current_episode_index_ = time_meta.episode_index;
        last_cmd_sequence_number_ = received_cmd_sequence;
        current_controller_->ResetForNewEpisode(raw_measurement);
    }

    return response;
}

ProcessResponse TorqueControllerManager::DeceleratingProcess()
{
    assert(TimePointSystem::IsValid(runtime_data_.raw_measurement.time_point));
    ProcessResponse response;
    const auto &time_meta = runtime_data_.time_meta;
    const auto &raw_measurement = runtime_data_.raw_measurement;
    Int64 received_cmd_sequence = runtime_data_.input_joint_cmd.cmd.sequence_number;

    // 切换controller
    current_controller_ = deceleration_torque_controller_;

    if (time_meta.episode_index == current_episode_index_)
    {
        if (received_cmd_sequence >= 0 && received_cmd_sequence > last_cmd_sequence_number_)
        {
            last_cmd_sequence_number_ = received_cmd_sequence;
            ChangeState(TorqueControllerState::Operating);
            response.re_run = true;
            return response;
        }

        // Speed is slow enough for idle
        if (raw_measurement.velocity.norm() < IdleVelocityThreshold)
        {
            ChangeState(TorqueControllerState::Idle);
            response.re_run = true;
            return response;
        }
    }
    else
    {
        // A new episode
        current_episode_index_ = time_meta.episode_index;
        last_cmd_sequence_number_ = received_cmd_sequence;
        current_controller_->ResetForNewEpisode(raw_measurement);
    }
    return response;
}

ProcessResponse TorqueControllerManager::OperatingProcess()
{
    assert(TimePointSystem::IsValid(runtime_data_.raw_measurement.time_point));
    ProcessResponse response;
    const auto &received_cmd_time = runtime_data_.input_joint_cmd.time_point;
    const Int64 received_cmd_seq = runtime_data_.input_joint_cmd.cmd.sequence_number;
    const auto &time_meta = runtime_data_.time_meta;
    const auto &raw_measurement = runtime_data_.raw_measurement;

    // 切换controller
    current_controller_ = operating_torque_controller_;

    if (time_meta.episode_index == current_episode_index_)
    {
        // In this episode, check if the cmd is too old
        const auto now = TimePointSystem::Now();
        const bool cmd_delayed = now.time_in_ns() > received_cmd_time.time_in_ns() + MaxDelayInNs;
        if (received_cmd_seq < 0 || cmd_delayed)
        {
            last_cmd_sequence_number_ = received_cmd_seq;
            ChangeState(TorqueControllerState::Decelerating);
            response.re_run = true;
            return response;
        }
    }
    else
    {
        // A new episode
        current_episode_index_ = time_meta.episode_index;
        last_cmd_sequence_number_ = received_cmd_seq;
        current_controller_->ResetForNewEpisode(raw_measurement);
    }

    return response;
}

void TorqueControllerManager::ChangeState(TorqueControllerState new_state)
{
    assert(controller_state_ != new_state);
    auto &time_meta = runtime_data_.time_meta;
    time_meta.episode_index += 1;
    time_meta.episode_start_time = time_meta.current_time;
    controller_state_ = new_state;
    LOG_INFO << "TorqueControllerManager state changed to: " << ToString(controller_state_);
}
}  // namespace robot_server_lib