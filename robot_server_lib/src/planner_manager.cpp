#include "planner_manager.h"

namespace robot_server_lib {

PlannerManager::PlannerManager(RobotModel robot_model,
                               PlannerManagerBuffers buffer,
                               std::function<void(Int64, RequestType, RequestStatus, Status)> report_request_status)
    : robot_model_(robot_model), manager_buffer_(buffer), ReportRequestStatus(report_request_status) {
    Initialize();
}

PlannerManager::~PlannerManager() = default;

void PlannerManager::Initialize() {
    // 从 robot_model 中加载机器人属性配置
    robot_property_ = robot_model_.robot_property;

    // Initialize the runtime data
    runtime_data_.loop_counter = -1;
    runtime_data_.planner_request.request_id = -1;

    // Initialize pre-allocated variables for PublishPlannerInfo
    runtime_data_.publish_measurement_primary.resize(robot_model_.params_ptr->joints_primary.size());
    runtime_data_.publish_measurement_secondary.resize(robot_model_.params_ptr->joints_secondary.size());
    runtime_data_.publish_measurement_torso.resize(robot_model_.params_ptr->joints_torso.size());
    runtime_data_.publish_ee_pose.resize(14);
    runtime_data_.publish_torso_X_Z_RY.resize(3);

    auto &time_meta = runtime_data_.time_meta;
    time_meta.current_time = TimePointSystem::Now();
    time_meta.measurement_time = TimePointSystem::Invalid();
    time_meta.episode_index = 0;
    time_meta.episode_start_time = time_meta.current_time;

    idle_planner_ = std::make_shared<IdlePlanner>(robot_model_);
    move_joint_planner_ = std::make_shared<MoveJointPlanner>(robot_model_, manager_dt, true);
    move_line_planner_ = std::make_shared<MoveLinePlanner>(robot_model_, manager_dt, true);
    move_traj_planner_ = std::make_shared<MoveTrajPlanner>(robot_model_, manager_dt, true);
    joint_streaming_planner_ = std::make_shared<JointStreamingPlanner>(
        robot_model_, manager_buffer_.streaming_command_latest_buffer, manager_dt);
    ee_pose_streaming_planner_ = std::make_shared<EEPoseStreamingPlanner>(
        robot_model_, manager_buffer_.streaming_command_latest_buffer, manager_dt);
    ee_velocity_streaming_planner_ = std::make_shared<EEVelocityStreamingPlanner>(
        robot_model_, manager_buffer_.streaming_command_latest_buffer, manager_dt);
    impedance_ctrl_primary_ =
        std::make_shared<dynamics::CartesianImpedanceController>(robot_model_, MotionGroup::Primary);
    impedance_ctrl_secondary_ =
        std::make_shared<dynamics::CartesianImpedanceController>(robot_model_, MotionGroup::Secondary);
}

void PlannerManager::LoopOnce() {
    auto &time_meta = runtime_data_.time_meta;
    auto &request = runtime_data_.planner_request;
    auto &emergency_request = runtime_data_.emergency_request;
    auto &measurement = runtime_data_.measurement;
    runtime_data_.loop_counter++;

    // 尝试获取最新的关节测量值 Try to peek the latest joint measurement
    time_meta.current_time = TimePointSystem::Now();
    if (!manager_buffer_.joint_measurement->PeekLatest(&measurement)) {
        time_meta.measurement_time = TimePointSystem::Invalid();
    } else {
        time_meta.measurement_time = TimePointSystem::Now();
        // 拖动模式下跳过位置限位检测（允许超出位置限位），但仍检测速度和力矩限位
        SafetyCheckConfig safety_config = robot_property_.safety_check_config;
        safety_config.check_position_limit =
            robot_property_.safety_check_config.check_position_limit &&
            !(robot_property_.run_mode == RunMode::Drag || robot_property_.run_mode == RunMode::DragImpedance);
        if (!CheckSafety(measurement, robot_model_, robot_property_.error_code, safety_config) &&
            manager_state_ != Status::Error) {
            runtime_data_.last_finished_request_id = runtime_data_.latest_processing_request_id;
            ReportRequestStatus(runtime_data_.latest_processing_request_id,
                                runtime_data_.latest_processing_request_type, RequestStatus::ErrorDrop, manager_state_);
            ChangeState(Status::Error);
        }
    }
    PublishPlannerInfo();

    // 尝试处理紧急队列 request Try to process emergency queue
    if (emergency_request.request_id < 0) {
        if (!manager_buffer_.emergency_request_queue->DeQueue(&emergency_request)) {
            emergency_request.type = RequestType::Empty;  // 设置为空命令 Set to empty request
        } else {
            OnEmergencyRequest();
        }
    }

    // 处理新命令和测量值 Process the new request and measurement
    RequestResponse response;
    response.re_run = true;

    // 根据状态机处理指令, 如果设置属性或Idle状态下接受新命令则重新运行 rerun if set property or accept new request in
    // Idle status
    while (response.re_run) {
        // 如果没有新命令则尝试获取新命令 Peek new request if empty
        if (request.request_id < 0) {
            if (manager_buffer_.request_queue->DeQueue(&request)) {
                if (request.request_id < runtime_data_.terminated_request_id) {
                    ReportRequestStatus(request.request_id, request.type, RequestStatus::Stopped, manager_state_);
                    ConsumeRequest(&request);  // 丢弃stop之前的指令 Discard outdated request
                } else {
                    LOG_INFO << "New request received with request ID: " << request.request_id
                             << " Type: " << ToString(request.type);
                }
            } else {
                request.type = RequestType::Empty;  // 设置为空命令 Set to empty request
            }
        }
        response = OnRequest();
        // 如果命令被消费则重置命令ID, 没被消费则不会再次获取指令 If the request is consumed, reset
        // the request ID
        if (response.request_consumed) ConsumeRequest(&request);
    }

    // 更新关节命令基于当前规划器 Update the joint command based on the current planner
    auto status(ComputeCommandStatus::Error);
    if (current_planner_ != nullptr) {
        if (this->runtime_data_.latest_processing_request_type == RequestType::StopMove) {
            status = current_planner_->ComputeStopCommand(measurement, time_meta, this->runtime_data_.joint_cmd);
        } else {
            status = current_planner_->ComputeCommand(measurement, time_meta, this->runtime_data_.joint_cmd);
        }
        this->runtime_data_.joint_cmd.joint_stiffness = this->robot_property_.joint_stiffness;
        this->runtime_data_.joint_cmd.joint_damping = this->robot_property_.joint_damping;
        this->runtime_data_.joint_cmd.joint_integral_ratio = this->robot_property_.joint_integral_ratio;
        // 如果提供了动力学参数, 则计算双臂前馈阻抗控制力 Compute the feedforward impedance control torque for both arms
        const auto &dyn_base_params_primary = this->robot_model_.params_ptr->dyn_base_params_primary;
        const auto &dyn_base_params_secondary = this->robot_model_.params_ptr->dyn_base_params_secondary;
        if (dyn_base_params_primary.has_value()) {
            Eigen::VectorXd tau_ff_pri, q_cmd_primary, q_feedback_primary, dq_feedback_primary, dq_cmd_primary;
            Vector6d F_imp_pri;
            utils::GetJointFromWholeBody(this->robot_model_, measurement.position, q_feedback_primary,
                                         MotionGroup::Primary);
            utils::GetJointFromWholeBody(this->robot_model_, measurement.velocity, dq_feedback_primary,
                                         MotionGroup::Primary);
            utils::GetJointFromWholeBody(this->robot_model_, this->runtime_data_.joint_cmd.position, q_cmd_primary,
                                         MotionGroup::Primary);
            utils::GetJointFromWholeBody(this->robot_model_, this->runtime_data_.joint_cmd.velocity, dq_cmd_primary,
                                         MotionGroup::Primary);

            auto ret_pri = impedance_ctrl_primary_->UpdateTorqueCommand(
                q_feedback_primary, dq_feedback_primary, q_cmd_primary, dq_cmd_primary, tau_ff_pri, F_imp_pri);
            utils::MapJointToWholeBody(this->robot_model_, tau_ff_pri, this->runtime_data_.joint_cmd.torque_feedforward,
                                       MotionGroup::Primary);
            this->robot_property_.cart_impedance_force.head(6) = F_imp_pri;
        }
        if (dyn_base_params_secondary.has_value()) {
            Eigen::VectorXd tau_ff_sec, q_cmd_secondary, q_feedback_secondary, dq_feedback_secondary, dq_cmd_secondary;
            Vector6d F_imp_sec;
            utils::GetJointFromWholeBody(this->robot_model_, measurement.position, q_feedback_secondary,
                                         MotionGroup::Secondary);
            utils::GetJointFromWholeBody(this->robot_model_, measurement.velocity, dq_feedback_secondary,
                                         MotionGroup::Secondary);
            utils::GetJointFromWholeBody(this->robot_model_, this->runtime_data_.joint_cmd.position, q_cmd_secondary,
                                         MotionGroup::Secondary);
            utils::GetJointFromWholeBody(this->robot_model_, this->runtime_data_.joint_cmd.velocity, dq_cmd_secondary,
                                         MotionGroup::Secondary);
            auto ret_sec = impedance_ctrl_secondary_->UpdateTorqueCommand(
                q_feedback_secondary, dq_feedback_secondary, q_cmd_secondary, dq_cmd_secondary, tau_ff_sec, F_imp_sec);
            utils::MapJointToWholeBody(this->robot_model_, tau_ff_sec, this->runtime_data_.joint_cmd.torque_feedforward,
                                       MotionGroup::Secondary);
            this->robot_property_.cart_impedance_force.tail(6) = F_imp_sec;
        }
    }

    // 发布关节命令到缓冲区 publish the joint command to the buffer
    if (status != ComputeCommandStatus::Error) {
        PlannerManagerCommand planner_command{};
        planner_command.sequence_number = runtime_data_.loop_counter;
        planner_command.cmd = runtime_data_.joint_cmd;
        // 位置模式下使用简化动力学计算（不加载动力学参数），提升性能
        planner_command.use_simple_dynamics = (robot_property_.run_mode == RunMode::Position);
        manager_buffer_.planner_manager_command->AppendNewElement(planner_command);
    }

    // 根据规划器状态切换状态机和报告轨迹信息 Switch state based on the planner status
    switch (status) {
        case ComputeCommandStatus::TaskOngoing:
            return;
        case ComputeCommandStatus::TaskStopped:
            LOG_INFO << "StopMove request finished.";
            ReportRequestStatus(runtime_data_.latest_processing_request_id,
                                runtime_data_.latest_processing_request_type, RequestStatus::Executed, manager_state_);
            ChangeState(Status::Idle);
            return;
        case ComputeCommandStatus::PhaseFinished:
            runtime_data_.last_finished_request_id++;
            ReportRequestStatus(runtime_data_.last_finished_request_id, runtime_data_.latest_processing_request_type,
                                RequestStatus::PhaseFinished, manager_state_);
            return;
        case ComputeCommandStatus::TaskFinished:
            runtime_data_.last_finished_request_id = runtime_data_.latest_processing_request_id;
            ReportRequestStatus(runtime_data_.latest_processing_request_id,
                                runtime_data_.latest_processing_request_type, RequestStatus::Executed, manager_state_);
            ChangeState(Status::Idle);
            return;
        case ComputeCommandStatus::Error:
            if (manager_state_ == Status::Init || manager_state_ == Status::Error) {
                // 如果处于Init状态说明暂未接收到measurement不报错继续等待, 已经处于Error则不再切换状态和报告错误
            } else {
                runtime_data_.last_finished_request_id = runtime_data_.latest_processing_request_id;
                ReportRequestStatus(runtime_data_.latest_processing_request_id,
                                    runtime_data_.latest_processing_request_type, RequestStatus::ErrorDrop,
                                    manager_state_);
                ChangeState(Status::Error);
            }
            return;
        default:
            return;
    }
}

RequestResponse PlannerManager::OnRequestStateInit() {
    RequestResponse response{};
    const auto &time_meta = runtime_data_.time_meta;
    const auto &request = runtime_data_.planner_request;
    auto &measurement = runtime_data_.measurement;
    auto &joint_cmd = runtime_data_.joint_cmd;

    // 切换 current_planner_ switch to nullptr
    current_planner_ = nullptr;

    // 如果measurement_time无效则说明还未初始化完成, 还不能切换Idle
    // If the measurement time is invalid, the new measurement is not here yet
    if (time_meta.measurement_time.is_invalid()) {
        manager_buffer_.planner_manager_command->Reset();  // 重置命令缓冲区 Reset the command buffer
        switch (request.type) {
            case RequestType::ClearError: {
                // 清除错误状态 Clear error state
                response.re_run = true;
                response.request_consumed = true;
                robot_property_.clear_error_flag = true;
                robot_property_.error_code = ErrorCode::NoError;  // 尝试清除错误码 Clear error code
                runtime_data_.latest_processing_request_id =
                    request.request_id;  // 更新最新处理的命令ID Update the latest processing request ID
                runtime_data_.latest_processing_request_type = request.type;
                runtime_data_.last_finished_request_id = runtime_data_.latest_processing_request_id;
                ReportRequestStatus(request.request_id, request.type, RequestStatus::Executed, manager_state_);
                break;
            }
            case RequestType::EnableSleep: {
                // Sleep 模式下在 Init 状态也可以取消休眠
                assert(request.data.has_value() && std::holds_alternative<bool>(request.data.value()));
                bool enable_sleep = std::get<bool>(request.data.value());
                if (!enable_sleep) {
                    // 取消 Sleep 模式，恢复到 Position
                    robot_property_.run_mode = RunMode::Position;
                    response.re_run = true;
                    response.request_consumed = true;
                    runtime_data_.latest_processing_request_id = request.request_id;
                    runtime_data_.latest_processing_request_type = request.type;
                    runtime_data_.last_finished_request_id = runtime_data_.latest_processing_request_id;
                    ReportRequestStatus(request.request_id, request.type, RequestStatus::Executed, manager_state_);
                    LOG_INFO << "Cancel Sleep mode in Init state, switched to Position mode";
                } else {
                    response.request_consumed = true;
                    runtime_data_.latest_processing_request_id = request.request_id;
                    runtime_data_.latest_processing_request_type = request.type;
                    runtime_data_.last_finished_request_id = runtime_data_.latest_processing_request_id;
                    ReportRequestStatus(request.request_id, request.type, RequestStatus::Executed, manager_state_);
                }
                break;
            }
            case RequestType::EnablePosLimitCheck: {
                SetProperty(request);
                response.re_run = true;
                response.request_consumed = true;
                runtime_data_.latest_processing_request_id = request.request_id;
                runtime_data_.latest_processing_request_type = request.type;
                runtime_data_.last_finished_request_id = runtime_data_.latest_processing_request_id;
                ReportRequestStatus(request.request_id, request.type, RequestStatus::Executed, manager_state_);
                break;
            }
            case RequestType::Empty:
                // 空命令不处理
                break;
            default:
                // 其他命令在 Init 状态下被拒绝
                runtime_data_.last_finished_request_id = runtime_data_.latest_processing_request_id;
                ReportRequestStatus(request.request_id, request.type, RequestStatus::ErrorDrop, manager_state_);
                response.request_consumed = true;
                break;
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

RequestResponse PlannerManager::OnRequestStateIdle() {
    RequestResponse response{};
    const auto &time_meta = runtime_data_.time_meta;
    const auto &request = runtime_data_.planner_request;
    const auto &measurement = runtime_data_.measurement;
    if (time_meta.measurement_time.is_invalid())  // 如果measurement_time无效则说明测量数据错误, 不能继续运行
    {
        ChangeState(Status::Init);
        LOG_CRIT << "Measurement time invalid in Running state, changing to Init state.";
        return response;
    }
    // 切换到idle planner switch to idle planner
    current_planner_ = idle_planner_;

    // 如果episode索引不同，则重置idle planner If the episode index has changed, reset the idle
    // planner
    if (current_planner_->episode_index != time_meta.episode_index) {
        current_planner_->SetRobotProperty(robot_property_);
        current_planner_->ResetForNewTrajectory(PlannerRequest{}, measurement, time_meta);
        current_planner_->episode_index = time_meta.episode_index;
    }
    if (request.type == RequestType::Empty) {
        return response;
    } else if (request.type < RequestType::MoveJoint)  // 设置属性命令 Set property commands
    {
        SetProperty(request);
        response.re_run = true;
        response.request_consumed = true;
        runtime_data_.latest_processing_request_id =
            request.request_id;  // 更新最新处理的命令ID Update the latest processing request ID
        runtime_data_.latest_processing_request_type = request.type;
        runtime_data_.last_finished_request_id = runtime_data_.latest_processing_request_id;
        ReportRequestStatus(request.request_id, request.type, RequestStatus::Executed, manager_state_);
        return response;
    } else  // 运动命令 Motion commands
    {
        ChangeState(Status::Running);
        response.re_run = true;
        response.request_consumed =
            false;  // 不消费命令，交给Running状态处理 Do not consume the request, let the Running state handle it
        return response;
    }
}

RequestResponse PlannerManager::OnRequestStateRunning() {
    RequestResponse response{};
    const auto &time_meta = runtime_data_.time_meta;
    const auto &request = runtime_data_.planner_request;
    const auto &measurement = runtime_data_.measurement;
    if (time_meta.measurement_time.is_invalid())  // 如果measurement_time无效则说明测量数据错误, 不能继续运行
    {
        ChangeState(Status::Init);
        runtime_data_.last_finished_request_id = runtime_data_.latest_processing_request_id;
        ReportRequestStatus(runtime_data_.latest_processing_request_id, runtime_data_.latest_processing_request_type,
                            RequestStatus::ErrorDrop, manager_state_);
        LOG_CRIT << "Measurement time invalid in Running state, changing to Init state.";
        return response;
    }

    // 如果当前规划器episode索引与time_meta的episode索引不同，则说明state改变过,
    // 需要重新选取并重置当前规划器 If the current planner's episode index is different from the
    // time meta's episode index, reset the current planner
    if (current_planner_->episode_index == time_meta.episode_index) {
        // 如果episode索引相同则说明与上个周期同处于一段轨迹, 可直接return, todo:
        // 后期可以考虑轨迹合并 if the episode index is the same, it means we are still in the same
        // trajectory as the last cycle, can return directly
        assert(current_planner_ != nullptr);
        assert(current_planner_->episode_index >= 0);
        return response;
    }
    // 如果不同则需要重置当前规划器
    // If the episode index is different, reset the current planner
    switch (request.type) {
        case RequestType::MoveJoint: {
            current_planner_ = move_joint_planner_;
            break;
        }
        case RequestType::MoveLine: {
            current_planner_ = move_line_planner_;
            break;
        }
        case RequestType::MoveTraj: {
            current_planner_ = move_traj_planner_;
            break;
        }
        case RequestType::StartJointStreaming: {
            current_planner_ = joint_streaming_planner_;
            break;
        }
        case RequestType::StartEEPoseStreaming: {
            current_planner_ = ee_pose_streaming_planner_;
            break;
        }
        case RequestType::StartEEVelocityStreaming: {
            current_planner_ = ee_velocity_streaming_planner_;
            break;
        }
        default: {
            assert(false);  // should never reach here
        }
    }
    current_planner_->episode_index = time_meta.episode_index;
    // 如果进入Running时为拖动模式, 强制将拖拽模式切换为位置模式 Force switch drag mode to position mode
    if (robot_property_.run_mode == RunMode::Drag) {
        robot_property_.run_mode = RunMode::Position;
    }
    current_planner_->SetRobotProperty(robot_property_);
    current_planner_->ResetForNewTrajectory(request, measurement, time_meta);
    response.re_run = false;
    response.request_consumed = true;
    runtime_data_.latest_processing_request_id =
        request.request_id;  // 更新最新处理的命令ID Update the latest processing request ID
    runtime_data_.latest_processing_request_type = request.type;
    ReportRequestStatus(request.request_id, request.type, RequestStatus::Queued, manager_state_);

    return response;
}

RequestResponse PlannerManager::OnRequestStateError() {
    RequestResponse response{};
    const auto &request = runtime_data_.planner_request;
    if (runtime_data_.time_meta.measurement_time
            .is_invalid())  // 如果measurement_time无效则说明测量数据错误, 不能继续运行
    {
        ChangeState(Status::Init);
        LOG_CRIT << "Measurement time invalid in Running state, changing to Init state.";
        return response;
    }
    // current_planner_ switch to nullptr
    current_planner_ = nullptr;
    robot_property_.run_mode = RunMode::Position;  // 错误模式下强制切换为位置模式 In error mode, force to position mode

    // 处理错误状态，重置或清除错误 Handle error state, reset or clear errors
    if (request.type == RequestType::ClearError) {
        //清除错误状态 Clear error state
        response.re_run = true;
        response.request_consumed = true;
        robot_property_.clear_error_flag = true;
        robot_property_.error_code = ErrorCode::NoError;  // 尝试清除错误码 Clear error code
        runtime_data_.latest_processing_request_id =
            request.request_id;  // 更新最新处理的命令ID Update the latest processing request ID
        runtime_data_.latest_processing_request_type = request.type;
        runtime_data_.last_finished_request_id = runtime_data_.latest_processing_request_id;
        ReportRequestStatus(request.request_id, request.type, RequestStatus::Executed, manager_state_);
        ChangeState(Status::Idle);
        return response;
    }
    if (request.type == RequestType::EnablePosLimitCheck) {
        SetProperty(request);
        response.re_run = true;
        response.request_consumed = true;
        runtime_data_.latest_processing_request_id =
            request.request_id;  // 更新最新处理的命令ID Update the latest processing request ID
        runtime_data_.latest_processing_request_type = request.type;
        runtime_data_.last_finished_request_id = runtime_data_.latest_processing_request_id;
        ReportRequestStatus(request.request_id, request.type, RequestStatus::Executed, manager_state_);
        return response;
    }
    if (request.type == RequestType::Empty) {
        // 如果是空命令则直接返回 If the request is empty, return directly
        return response;
    }
    // 忽略其他命令 Ignore other requests
    runtime_data_.latest_processing_request_id =
        request.request_id;  // 更新最新处理的命令ID Update the latest processing request ID
    runtime_data_.latest_processing_request_type = request.type;
    runtime_data_.last_finished_request_id = runtime_data_.latest_processing_request_id;
    ReportRequestStatus(request.request_id, request.type, RequestStatus::ErrorDrop, manager_state_);
    // 其他情况保持在Error状态 Other cases remain in Error state
    response.request_consumed = true;
    return response;
}

RequestResponse PlannerManager::OnRequest() {
    RequestResponse response{};
    switch (manager_state_) {
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

void PlannerManager::ChangeState(Status new_state) {
    assert(manager_state_ != new_state);
    auto &time_meta = runtime_data_.time_meta;
    time_meta.episode_index += 1;
    time_meta.episode_start_time = time_meta.current_time;
    manager_state_ = new_state;
    ReportRequestStatus(-1, RequestType::Empty, RequestStatus::Unknown, manager_state_);
}

void PlannerManager::SetProperty(const PlannerRequest &request) {
    // todo: 数据格式错误转error的处理
    switch (request.type) {
        case RequestType::TcpPrimary: {
            assert(request.data.has_value() && std::holds_alternative<PoseQuaternion>(request.data.value()));
            auto tcp_temp = std::get<PoseQuaternion>(request.data.value());
            robot_property_.tcp_frame = pinocchio::SE3(tcp_temp.orientation, tcp_temp.position);
            return;
        }
        case RequestType::TcpSecondary: {
            assert(request.data.has_value() && std::holds_alternative<PoseQuaternion>(request.data.value()));
            auto tcp_temp = std::get<PoseQuaternion>(request.data.value());
            robot_property_.tcp_frame_secondary = pinocchio::SE3(tcp_temp.orientation, tcp_temp.position);
            return;
        }
        case RequestType::SpeedRatio: {
            assert(request.data.has_value() && std::holds_alternative<double>(request.data.value()));
            robot_property_.global_speed_ratio = std::get<double>(request.data.value());
            return;
        }
        case RequestType::SwitchRunMode: {
            assert(request.data.has_value() && std::holds_alternative<int>(request.data.value()));
            int mode = std::get<int>(request.data.value());
            switch (mode) {
                case 0:
                    robot_property_.run_mode = RunMode::Position;
                    break;
                case 1:
                    if (this->robot_model_.IsDynamicParamsAvailable())  // 无动力学参数不能开启力控
                    {
                        robot_property_.run_mode = RunMode::Torque;
                    } else {
                        std::cout << "[WARN] Dynamic parameters not available, cannot enable torque control mode.\n";
                        LOG_WARN << "Dynamic parameters not available, cannot enable torque control.";
                    }
                    break;
                case 2:
                    robot_property_.run_mode = RunMode::Drag;  // 在server层处理开启哪种拖动模式
                    break;
                case 3:
                    if (this->robot_model_.IsDynamicParamsAvailable())  // 无动力学参数不能开启阻抗控制
                    {
                        robot_property_.run_mode = RunMode::CartesianImpedance;
                    } else {
                        std::cout << "[WARN] Dynamic parameters not available, cannot enable impedance control mode.\n";
                        LOG_WARN << "Dynamic parameters not available, cannot enable impedance control mode.";
                    }
                    break;
                case 4:
                    if (this->robot_model_.IsDynamicParamsAvailable())  // 无动力学参数不能开启阻抗控制
                    {
                        robot_property_.run_mode = RunMode::DragImpedance;
                    } else {
                        std::cout
                            << "[WARN] Dynamic parameters not available, cannot enable drag impedance control mode.\n";
                        LOG_WARN << "Dynamic parameters not available, cannot enable drag impedance control mode.";
                    }
                    break;
                default:
                    LOG_WARN << "Invalid run mode: " << mode << ", switch failed.";
                    break;
            }
            std::cout << "[INFO] Set run mode to " << ToString(robot_property_.run_mode) << "\n";
            LOG_INFO << "Set run mode to " << ToString(robot_property_.run_mode);
            return;
        }
        case RequestType::EnableDrag: {
            assert(request.data.has_value() && std::holds_alternative<bool>(request.data.value()));
            bool enable_drag = std::get<bool>(request.data.value());
            static auto previous_mode = robot_property_.run_mode;
            if (enable_drag) {
                if (robot_property_.run_mode != RunMode::Drag) {
                    previous_mode = robot_property_.run_mode;  // 保存之前的模式 Save the previous mode
                }
                robot_property_.run_mode = RunMode::Drag;
            } else {
                robot_property_.run_mode = previous_mode;
            }
            LOG_INFO << "Set run mode to " << ToString(robot_property_.run_mode);
            return;
        }
        case RequestType::EnableSleep: {
            assert(request.data.has_value() && std::holds_alternative<bool>(request.data.value()));
            bool enable_sleep = std::get<bool>(request.data.value());
            if (enable_sleep) {
                robot_property_.run_mode = RunMode::Sleep;
                LOG_INFO << "Enable Sleep mode";
            } else {
                robot_property_.run_mode = RunMode::Position;
                LOG_INFO << "Disable Sleep mode";
            }
            return;
        }
        case RequestType::JointStiffness: {
            auto stiffness = std::get<Eigen::VectorXd>(request.data.value());
            LOG_INFO << "Received joint stiffness command: " << stiffness.transpose();
            auto ndof_arm = this->robot_model_.params_ptr->joints_primary.size() +
                            this->robot_model_.params_ptr->joints_secondary.size();
            if (stiffness.size() == 1) {
                if (stiffness[0] < -FloatZeroTol) {
                    std::stringstream ss;
                    ss << "[WARN] Invalid joint stiffness value: " << stiffness[0] << ", must >= 0. Command ignored.\n";
                    std::cout << ss.str();
                    LOG_WARN << ss.str();
                    return;
                }
                robot_property_.joint_stiffness = Eigen::VectorXd::Constant(ndof_robot, stiffness[0]);
            } else {
                if (stiffness.size() == ndof_arm) {
                    if ((stiffness.array() < -FloatZeroTol).any()) {
                        std::stringstream ss;
                        ss << "[WARN] Invalid joint stiffness value in: " << stiffness.transpose()
                           << ", all values must >= 0. Command ignored.\n";
                        std::cout << ss.str();
                        LOG_WARN << ss.str();
                        return;
                    }
                    // 按顺序赋值给tail部分关节 Assign to the tail joints in order
                    robot_property_.joint_stiffness.tail(stiffness.size()) = stiffness;
                } else {
                    std::stringstream ss;
                    ss << "[WARN] Invalid joint stiffness size: " << stiffness.size() << ", expected size: " << ndof_arm
                       << " or 1. Command ignored.\n";
                    std::cout << ss.str();
                    LOG_WARN << ss.str();
                }
            }
            std::cout << "[INFO] Set joint stiffness to " << robot_property_.joint_stiffness.tail(ndof_arm).transpose()
                      << "\n";
            LOG_INFO << "Set joint stiffness to " << robot_property_.joint_stiffness.tail(ndof_arm).transpose();
            return;
        }
        case RequestType::JointDamping: {
            auto damping = std::get<Eigen::VectorXd>(request.data.value());
            LOG_INFO << "Received joint damping command: " << damping.transpose();
            auto ndof_arm = this->robot_model_.params_ptr->joints_primary.size() +
                            this->robot_model_.params_ptr->joints_secondary.size();
            if (damping.size() == 1) {
                if (damping[0] < -FloatZeroTol) {
                    std::stringstream ss;
                    ss << "[WARN] Invalid joint damping value: " << damping[0] << ", must >= 0. Command ignored.\n";
                    std::cout << ss.str();
                    LOG_WARN << ss.str();
                    return;
                }
                robot_property_.joint_damping = Eigen::VectorXd::Constant(ndof_robot, damping[0]);
            } else {
                if (damping.size() == ndof_arm) {
                    if ((damping.array() < -FloatZeroTol).any()) {
                        std::stringstream ss;
                        ss << "[WARN] Invalid joint damping value in: " << damping.transpose()
                           << ", all values must >= 0. Command ignored.\n";
                        std::cout << ss.str();
                        LOG_WARN << ss.str();
                        return;
                    }
                    // 按顺序赋值给tail部分关节 Assign to the tail joints in order
                    robot_property_.joint_damping.tail(damping.size()) = damping;
                } else {
                    std::stringstream ss;
                    ss << "[WARN] Invalid joint damping size: " << damping.size() << ", expected size: " << ndof_arm
                       << " or 1. Command ignored.\n";
                    std::cout << ss.str();
                    LOG_WARN << ss.str();
                }
            }
            std::cout << "[INFO] Set joint damping to " << robot_property_.joint_damping.tail(ndof_arm).transpose()
                      << "\n";
            LOG_INFO << "Set joint damping to " << robot_property_.joint_damping.tail(ndof_arm).transpose();
            return;
        }
        case RequestType::JointIntegralRatio: {
            auto k_i_ratio = std::get<double>(request.data.value());
            if (k_i_ratio < 0.0 || k_i_ratio > 1.0) {
                std::stringstream ss;
                ss << "[WARN] Invalid k_i_ratio value: " << k_i_ratio
                   << ", must be in range [0, 1]. Command ignored.\n";
                std::cout << ss.str();
                LOG_WARN << ss.str();
                return;
            }
            robot_property_.joint_integral_ratio = k_i_ratio;
            std::cout << "[INFO] Set k_i_ratio to " << robot_property_.joint_integral_ratio << "\n";
            LOG_INFO << "Set k_i_ratio to " << robot_property_.joint_integral_ratio;
            return;
        }
        case RequestType::CartStiffness: {
            auto stiffness = std::get<Eigen::VectorXd>(request.data.value());
            if (stiffness.size() == 1) {
                if (stiffness[0] < -FloatZeroTol) {
                    std::stringstream ss;
                    ss << "[WARN] Invalid cart stiffness value: " << stiffness[0] << ", must >= 0. Command ignored.\n";
                    std::cout << ss.str();
                    LOG_WARN << ss.str();
                    return;
                }
                robot_property_.cart_stiffness = Vector12d::Constant(stiffness[0]);
            } else {
                if (stiffness.size() == 12) {
                    if ((stiffness.array() < -FloatZeroTol).any()) {
                        std::stringstream ss;
                        ss << "[WARN] Invalid cart stiffness value in: " << stiffness.transpose()
                           << ", all values must >= 0. Command ignored.\n";
                        std::cout << ss.str();
                        LOG_WARN << ss.str();
                        return;
                    }
                    robot_property_.cart_stiffness = stiffness;
                } else {
                    std::stringstream ss;
                    ss << "[WARN] Invalid cart stiffness size: " << stiffness.size() << ", expected size: 12"
                       << " or 1. Command ignored.\n";
                    std::cout << ss.str();
                    LOG_WARN << ss.str();
                }
            }
            impedance_ctrl_primary_->SetStiffness(robot_property_.cart_stiffness.head(6));
            impedance_ctrl_secondary_->SetStiffness(robot_property_.cart_stiffness.tail(6));
            std::stringstream ss;
            ss << "[INFO] Set cartisian impedance stiffness to " << robot_property_.cart_stiffness.transpose() << "\n";
            std::cout << ss.str();
            LOG_INFO << ss.str();
            return;
        }
        case RequestType::CartDampingRatio: {
            auto damping_ratio = std::get<Eigen::VectorXd>(request.data.value());
            if (damping_ratio.size() == 1) {
                if (damping_ratio[0] < 0.5 - FloatZeroTol || damping_ratio[0] > 1.5 + FloatZeroTol) {
                    std::stringstream ss;
                    ss << "[WARN] Invalid cart damping ratio value: " << damping_ratio[0]
                       << ", must >= 0.5 and <= 1.5. Command ignored.\n";
                    std::cout << ss.str();
                    LOG_WARN << ss.str();
                    return;
                }
                robot_property_.cart_damping_ratio = Vector12d::Constant(damping_ratio[0]);
            } else {
                if (damping_ratio.size() == 12) {
                    if ((damping_ratio.array() < 0.7 - FloatZeroTol).any() ||
                        (damping_ratio.array() > 1.3 + FloatZeroTol).any()) {
                        std::stringstream ss;
                        ss << "[WARN] Invalid cart damping ratio value in: " << damping_ratio.transpose()
                           << ", all values must >= 0.5 and <= 1.5. Command ignored.\n";
                        std::cout << ss.str();
                        LOG_WARN << ss.str();
                        return;
                    }
                    robot_property_.cart_damping_ratio = damping_ratio;
                } else {
                    std::stringstream ss;
                    ss << "[WARN] Invalid cart damping ratio size: " << damping_ratio.size() << ", expected size: 12"
                       << " or 1. Command ignored.\n";
                    std::cout << ss.str();
                    LOG_WARN << ss.str();
                }
            }
            impedance_ctrl_primary_->SetDampingRatio(robot_property_.cart_damping_ratio.head(6));
            impedance_ctrl_secondary_->SetDampingRatio(robot_property_.cart_damping_ratio.tail(6));
            std::stringstream ss;
            ss << "[INFO] Set cartisian impedance damping ratio to " << robot_property_.cart_damping_ratio.transpose()
               << "\n";
            std::cout << ss.str();
            LOG_INFO << ss.str();
            return;
        }

        case RequestType::SafeLevel: {
            assert(request.data.has_value() && std::holds_alternative<double>(request.data.value()));
            double safe_level = std::get<double>(request.data.value());
            robot_property_.safe_level = std::clamp(safe_level, 0.1, 1.0);
            LOG_INFO << "Set safe level to " << robot_property_.safe_level;
            return;
        }
        case RequestType::EnablePosLimitCheck: {
            assert(request.data.has_value() && std::holds_alternative<bool>(request.data.value()));
            bool enable = std::get<bool>(request.data.value());
            robot_property_.safety_check_config.check_position_limit = enable;
            LOG_INFO << "Set enable_position_limit_check to " << enable;
            return;
        }
        case RequestType::ClearError: {
            assert(!request.data.has_value());
            robot_property_.clear_error_flag = true;
            robot_property_.error_code = ErrorCode::NoError;  // 尝试清除错误码 Clear error code
            LOG_INFO << "Set clear error flag to true, error code reset to: " << ToString(robot_property_.error_code);
            return;
        }
        default:
            assert(false);
    }
}

void PlannerManager::ConsumeRequest(PlannerRequest *request) {
    request->request_id = -1;            // Reset request ID
    request->type = RequestType::Empty;  // Reset request type
}

void PlannerManager::OnEmergencyRequest() {
    auto &request = runtime_data_.planner_request;
    auto &emergency_request = runtime_data_.emergency_request;
    switch (emergency_request.type) {
        case RequestType::StopMove:
            LOG_INFO << "New emergency request received with request ID: " << emergency_request.request_id
                     << " Type: " << ToString(emergency_request.type);
            runtime_data_.terminated_request_id = emergency_request.request_id;
            if (Status::Running != manager_state_) {
                runtime_data_.latest_processing_request_id = emergency_request.request_id;
                runtime_data_.latest_processing_request_type = emergency_request.type;
                runtime_data_.last_finished_request_id = runtime_data_.latest_processing_request_id;
                ReportRequestStatus(emergency_request.request_id, emergency_request.type, RequestStatus::Executed,
                                    manager_state_);
                ConsumeRequest(&emergency_request);
                break;
            }

            // 清除当前处理中的的请求 Clear the currently processing request
            ReportRequestStatus(runtime_data_.latest_processing_request_id,
                                runtime_data_.latest_processing_request_type, RequestStatus::Stopped, manager_state_);
            // 改写当前处理的请求ID和类型 Update the currently processing request ID and type
            runtime_data_.latest_processing_request_id = emergency_request.request_id;
            runtime_data_.latest_processing_request_type = emergency_request.type;
            runtime_data_.last_finished_request_id = runtime_data_.latest_processing_request_id;
            ConsumeRequest(&emergency_request);
            // 清除当前缓存请求 Clear cached request
            if (request.type != RequestType::Empty) {
                ReportRequestStatus(request.request_id, request.type, RequestStatus::Stopped, manager_state_);
                ConsumeRequest(&request);  // 清除当前缓存请求 Clear cached request
            }
            break;
        default:
            LOG_WARN << "Unknown emergency request type received: " << ToString(emergency_request.type);
            ConsumeRequest(&emergency_request);
            break;
    }
}

void PlannerManager::PublishPlannerInfo() {
    PlannerManagerInfo info{};
    auto &measurement = runtime_data_.measurement;
    // Use pre-allocated member variables instead of creating temporary vectors
    auto &measurement_primary = runtime_data_.publish_measurement_primary;
    auto &measurement_secondary = runtime_data_.publish_measurement_secondary;
    auto &measurement_torso = runtime_data_.publish_measurement_torso;
    PoseQuaternion ee_pose_primary, ee_pose_secondary, torso_pose;
    utils::GetJointFromWholeBody(this->robot_model_, measurement.position, measurement_primary, MotionGroup::Primary);
    utils::GetJointFromWholeBody(this->robot_model_, measurement.position, measurement_secondary,
                                 MotionGroup::Secondary);
    utils::GetJointFromWholeBody(this->robot_model_, measurement.position, measurement_torso, MotionGroup::Torso);
    if (this->manager_state_ == Status::Idle || this->manager_state_ == Status::Running) {
        ee_pose_primary = idle_planner_->KinematicsPrimary()->GetFK(measurement_primary, robot_property_.tcp_frame);
        ee_pose_secondary =
            idle_planner_->KinematicsSecondary()->GetFK(measurement_secondary, robot_property_.tcp_frame_secondary);
        torso_pose = idle_planner_->KinematicsTorso()->GetFK(measurement_torso);
    }
    auto &ee_pose = runtime_data_.publish_ee_pose;
    ee_pose << ee_pose_primary.ToVectorXd(), ee_pose_secondary.ToVectorXd();
    ee_pose = (ee_pose.array() * 1e4).round() / 1e4;  // 精确到小数点后四位
    info.ee_pose_measurement = ee_pose;
    auto &torso_X_Z_RY = runtime_data_.publish_torso_X_Z_RY;
    torso_X_Z_RY << torso_pose.position[0], torso_pose.position[2],
        measurement_torso[0] + measurement_torso[1] + measurement_torso[2];
    info.torso_pose_measurement = torso_X_Z_RY;
    // 记录当前property Record current property
    info.robot_property = robot_property_;
    manager_buffer_.planner_manager_info->AppendNewElement(info);
    robot_property_.clear_error_flag = false;  // 每周期重置error_flag
}
}  // namespace robot_server_lib
