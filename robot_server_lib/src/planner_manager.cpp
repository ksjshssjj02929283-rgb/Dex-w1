#include "planner_manager.h"

namespace robot_server_lib
{

PlannerManager::PlannerManager(RobotModel robot_model, PlannerManagerBuffers buffer,
                               std::function<void(Int64, RequestType, RequestStatus, Status)> report_request_status)
    : robot_model_(robot_model), manager_buffer_(buffer), ReportRequestStatus(report_request_status)
{
    Initialize();
}

PlannerManager::~PlannerManager() = default;

void PlannerManager::Initialize()
{
    // Initialize the runtime data
    runtime_data_.loop_counter = -1;
    runtime_data_.planner_request.request_id = -1;

    auto &time_meta = runtime_data_.time_meta;
    time_meta.current_time = TimePointSystem::Now();
    time_meta.measurement_time = TimePointSystem::Invalid();
    time_meta.episode_index = 0;
    time_meta.episode_start_time = time_meta.current_time;

    idle_planner_ = std::make_shared<IdlePlanner>(robot_model_);
    move_joint_planner_ = std::make_shared<MoveJointPlanner>(robot_model_, manager_dt, false);
    move_line_planner_ = std::make_shared<MoveLinePlanner>(robot_model_, manager_dt, false);
    joint_streaming_planner_ = std::make_shared<JointStreamingPlanner>(robot_model_, manager_buffer_.streaming_command_latest_buffer, manager_dt);
    ee_pose_streaming_planner_ = std::make_shared<EEPoseStreamingPlanner>(robot_model_, manager_buffer_.streaming_command_latest_buffer, manager_dt);
}

void PlannerManager::LoopOnce()
{
    auto &time_meta = runtime_data_.time_meta;
    auto &request = runtime_data_.planner_request;
    auto &measurement = runtime_data_.measurement;
    auto &joint_cmd = runtime_data_.joint_cmd;
    runtime_data_.loop_counter++;

    // DEBUG下换行并输出episode信息 Print episode information
    if (Status::Running == manager_state_)
    {
        LOG_DEBUG << "";
        LOG_DEBUG << "Loop counter: " << runtime_data_.loop_counter << " Episode " << time_meta.episode_index
                  << " current state: " << ToString(manager_state_);
    }

    // 尝试获取最新的关节测量值 Try to peek the latest joint measurement
    time_meta.current_time = TimePointSystem::Now();
    if (!manager_buffer_.joint_measurement->PeekLatest(&measurement))
    {
        time_meta.measurement_time = TimePointSystem::Invalid();
        LOG_CRIT << "No joint measurement available, skipping loop.";
    }
    else
    {
        time_meta.measurement_time = TimePointSystem::Now();

        Eigen::VectorXd joint_primary, joint_secondary;
        PoseQuaternion ee_pose_primary, ee_pose_secondary;
        idle_planner_->GetArmFromWholeBody(measurement.position, joint_primary, MotionGroup::Primary);
        idle_planner_->GetArmFromWholeBody(measurement.position, joint_secondary, MotionGroup::Secondary);
        ee_pose_primary = idle_planner_->Kinematics()->GetFK(joint_primary, robot_property_.tcp_frame);
        ee_pose_secondary = idle_planner_->KinematicsSecondary()->GetFK(joint_secondary, robot_property_.tcp_frame_secondary);
        Eigen::VectorXd ee_pose(14);
        ee_pose << ee_pose_primary.ToVectorXd(), ee_pose_secondary.ToVectorXd();
        manager_buffer_.ee_pose_measurement->AppendNewElement(ee_pose);
    }

    // 如果内部缓存没有指令则尝试从队列中获取新命令, 有指令则跳过获取, 继续执行缓存指令
    // if there is no internal request, try to get a new request from the queue
    if (request.request_id < 0)
    {
        if (!manager_buffer_.request_queue->DeQueue(&request))
        {
            request.type = RequestType::Empty;  // 设置为空命令 Set to empty request
        }
        else
        {
            LOG_INFO << "New request received with request ID: " << request.request_id << " Type: " << ToString(request.type);
        }
    }

    // 处理新命令和测量值 Process the new request and measurement
    RequestResponse response = OnRequest();
    if (response.request_consumed)
        ConsumeRequest();  // 如果命令被消费则重置命令ID If the request is consumed, reset the
                           // request ID

    // 如果设置属性或Idle状态下接受新命令则重新运行 rerun if set property or accept new request in
    // Idle status
    while (response.re_run)
    {
        // 如果没有新命令则尝试获取新命令 Peek new request if empty
        if (request.request_id < 0)
        {
            if (!manager_buffer_.request_queue->DeQueue(&request))
            {
                request.type = RequestType::Empty;  // 设置为空命令 Set to empty request
            }
        }
        response = OnRequest();
        // 如果命令被消费则重置命令ID, 没被消费则不会再次获取指令 If the request is consumed, reset
        // the request ID
        if (response.request_consumed)
            ConsumeRequest();
    }

    // 更新关节命令基于当前规划器 Update the joint command based on the current planner
    auto status(ComputeCommandStatus::Error);
    if (current_planner_ != nullptr)
    {
        status = current_planner_->ComputeCommand(measurement, time_meta, joint_cmd);
    }

    // 发布关节命令到缓冲区 publish the joint command to the buffer
    if (status != ComputeCommandStatus::Error)
    {
        PlannerManagerCommand temp_planner_command{};
        temp_planner_command.sequence_number = runtime_data_.loop_counter;
        temp_planner_command.cmd = joint_cmd;
        manager_buffer_.planner_manager_command->AppendNewElement(temp_planner_command);
        if (Status::Running == manager_state_)
            LOG_DEBUG << "Current Planner: " << typeid(*current_planner_).name() << " output: " << joint_cmd.ToString();
    }

    // 根据规划器状态切换状态 Switch state based on the planner status
    switch (status)
    {
    case ComputeCommandStatus::TaskFinished:
        ReportRequestStatus(runtime_data_.latest_processing_request_id, runtime_data_.latest_processing_request_type, RequestStatus::Executed,
                            manager_state_);
        ChangeState(Status::Idle);
        return;
    case ComputeCommandStatus::Error:
        if (manager_state_ == Status::Init || manager_state_ == Status::Error)
        {
            // 如果处于Init状态说明暂未接收到measurement不报错继续等待,
            // 已经处于Error则不再切换状态和报告错误
        }
        else
        {
            ReportRequestStatus(runtime_data_.latest_processing_request_id, runtime_data_.latest_processing_request_type, RequestStatus::ErrorDrop,
                                manager_state_);
            ChangeState(Status::Error);
        }
        return;
    default:
        return;
    }
}

RequestResponse PlannerManager::OnRequestStateInit()
{
    RequestResponse response{};
    const auto &time_meta = runtime_data_.time_meta;
    const auto &request = runtime_data_.planner_request;
    auto &measurement = runtime_data_.measurement;
    auto &joint_cmd = runtime_data_.joint_cmd;
    assert(response.re_run == false);
    assert(response.request_consumed == false);

    // 切换 current_planner_ switch to nullptr
    current_planner_ = nullptr;

    // 如果measurement_time无效则说明还未初始化完成, 还不能切换Idle
    // If the measurement time is invalid, the new measurement is not here yet
    if (time_meta.measurement_time.is_invalid())
    {
        if (request.type != RequestType::Empty)
        {
            ReportRequestStatus(request.request_id, request.type, RequestStatus::ErrorDrop, manager_state_);
            response.request_consumed = true;
        }

        return response;
    }
    // New measurement is ready
    assert(time_meta.measurement_time.is_valid());
    // 初始化关节命令 Initialize the joint command
    joint_cmd.position = measurement.position;
    joint_cmd.velocity.setZero();
    joint_cmd.acceleration.setZero();
    joint_cmd.torque_feedforward.setZero();
    response.re_run = true;  // Re run at Idle state
    ChangeState(Status::Idle);

    return response;
}

RequestResponse PlannerManager::OnRequestStateIdle()
{
    RequestResponse response{};
    const auto &time_meta = runtime_data_.time_meta;
    const auto &request = runtime_data_.planner_request;
    const auto &measurement = runtime_data_.measurement;
    assert(response.re_run == false);
    assert(response.request_consumed == false);
    assert(time_meta.measurement_time.is_valid());
    // 切换到idle planner switch to idle planner
    current_planner_ = idle_planner_;

    // 如果episode索引不同，则重置idle planner If the episode index has changed, reset the idle
    // planner
    if (current_planner_->episode_index != time_meta.episode_index)
    {
        current_planner_->SetRobotProperty(robot_property_);
        current_planner_->ResetForNewTrajectory(PlannerRequest{}, measurement, time_meta);
        current_planner_->episode_index = time_meta.episode_index;
    }
    switch (request.type)
    {
    case RequestType::Empty:
        return response;
    case RequestType::TcpPrimary:
    case RequestType::TcpSecondary:
    case RequestType::SpeedRatio:
    case RequestType::ClearError:
    {
        SetProperty(request);
        response.re_run = true;
        response.request_consumed = true;
        runtime_data_.latest_processing_request_id = request.request_id;  // 更新最新处理的命令ID Update the latest processing request ID
        runtime_data_.latest_processing_request_type = request.type;
        ReportRequestStatus(request.request_id, request.type, RequestStatus::Executed, manager_state_);
        return response;
    }
    // 其他为运动命令类型 Other types are motion tcp_frame_secondary_requests
    case RequestType::MoveJoint:
    case RequestType::MoveLine:
    case RequestType::StartJointStreaming:
    case RequestType::StartEEPoseStreaming:
    {
        ChangeState(Status::Running);
        response.re_run = true;
        response.request_consumed = false;
        return response;
    }
    default:
        ChangeState(Status::Error);
        return response;
    }
}

RequestResponse PlannerManager::OnRequestStateRunning()
{
    RequestResponse response{};
    const auto &time_meta = runtime_data_.time_meta;
    const auto &request = runtime_data_.planner_request;
    const auto &measurement = runtime_data_.measurement;
    assert(response.re_run == false);
    assert(response.request_consumed == false);
    assert(time_meta.measurement_time.is_valid());

    // 如果当前规划器episode索引与time_meta的episode索引不同，则说明state改变过,
    // 需要重新选取并重置当前规划器 If the current planner's episode index is different from the
    // time meta's episode index, reset the current planner
    if (current_planner_->episode_index == time_meta.episode_index)
    {
        // 如果episode索引相同则说明与上个周期同处于一段轨迹, 可直接return, todo:
        // 后期可以考虑轨迹合并 if the episode index is the same, it means we are still in the same
        // trajectory as the last cycle, can return directly
        assert(current_planner_ != nullptr);
        assert(current_planner_->episode_index >= 0);
        return response;
    }
    // 如果不同则需要重置当前规划器
    // If the episode index is different, reset the current planner
    switch (request.type)
    {
    case RequestType::MoveJoint:
    {
        current_planner_ = move_joint_planner_;
        break;
    }
    case RequestType::MoveLine:
    {
        current_planner_ = move_line_planner_;
        break;
    }
    case RequestType::StartJointStreaming:
    {
        current_planner_ = joint_streaming_planner_;
        break;
    }
    case RequestType::StartEEPoseStreaming:
    {
        current_planner_ = ee_pose_streaming_planner_;
        break;
    }
    default:
    {
        assert(false);  // should never reach here
    }
    }
    current_planner_->episode_index = time_meta.episode_index;
    current_planner_->SetRobotProperty(robot_property_);
    current_planner_->ResetForNewTrajectory(request, measurement, time_meta);
    response.re_run = false;
    response.request_consumed = true;
    runtime_data_.latest_processing_request_id = request.request_id;  // 更新最新处理的命令ID Update the latest processing request ID
    runtime_data_.latest_processing_request_type = request.type;
    ReportRequestStatus(request.request_id, request.type, RequestStatus::Queued, manager_state_);

    return response;
}

RequestResponse PlannerManager::OnRequestStateError()
{
    RequestResponse response{};
    const auto &request = runtime_data_.planner_request;
    assert(response.re_run == false);
    assert(response.request_consumed == false);
    // current_planner_ switch to nullptr
    current_planner_ = nullptr;

    // 处理错误状态，重置或清除错误 Handle error state, reset or clear errors
    if (request.type == RequestType::ClearError)
    {
        //清除错误状态 Clear error state
        response.re_run = true;
        response.request_consumed = true;
        ReportRequestStatus(request.request_id, request.type, RequestStatus::Executed, manager_state_);
        ChangeState(Status::Idle);
        return response;
    }
    if (request.type == RequestType::Empty)
    {
        // 如果是空命令则直接返回 If the request is empty, return directly
        return response;
    }
    // 忽略其他命令 Ignore other requests
    ReportRequestStatus(request.request_id, request.type, RequestStatus::ErrorDrop, manager_state_);
    // 其他情况保持在Error状态 Other cases remain in Error state
    response.request_consumed = true;
    return response;
}

RequestResponse PlannerManager::OnRequest()
{
    RequestResponse response{};
    switch (manager_state_)
    {
    case Status::Init:
        response = OnRequestStateInit();
        break;

    case Status::Idle:
        response = OnRequestStateIdle();
        break;

    case Status::Running:
        response = OnRequestStateRunning();
        break;

    case Status::Error:
        response = OnRequestStateError();
        break;

    default:
        assert(false);  // should never reach here
    }

    return response;
}

void PlannerManager::ChangeState(Status new_state)
{
    assert(manager_state_ != new_state);
    auto &time_meta = runtime_data_.time_meta;
    time_meta.episode_index += 1;
    time_meta.episode_start_time = time_meta.current_time;
    manager_state_ = new_state;
    ReportRequestStatus(-1, RequestType::Empty, RequestStatus::Unknown, manager_state_);
}

void PlannerManager::SetProperty(const PlannerRequest &request)
{
    // todo: 数据格式错误转error的处理
    switch (request.type)
    {
    case RequestType::TcpPrimary:
    {
        assert(request.data.has_value() && std::holds_alternative<PoseQuaternion>(request.data.value()));
        auto tcp_temp = std::get<PoseQuaternion>(request.data.value());
        robot_property_.tcp_frame = pinocchio::SE3(tcp_temp.orientation, tcp_temp.position);
        return;
    }
    case RequestType::TcpSecondary:
    {
        assert(request.data.has_value() && std::holds_alternative<PoseQuaternion>(request.data.value()));
        auto tcp_temp = std::get<PoseQuaternion>(request.data.value());
        robot_property_.tcp_frame_secondary = pinocchio::SE3(tcp_temp.orientation, tcp_temp.position);
        return;
    }
    case RequestType::SpeedRatio:
    {
        assert(request.data.has_value() && std::holds_alternative<double>(request.data.value()));
        robot_property_.global_speed_ratio = std::get<double>(request.data.value());
        return;
    }
    case RequestType::ClearError:
    {
        assert(!request.data.has_value());
        return;
    }
    default:
        assert(false);
    }
}
}  // namespace robot_server_lib