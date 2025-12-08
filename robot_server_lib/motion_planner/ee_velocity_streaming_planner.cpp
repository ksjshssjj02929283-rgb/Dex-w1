#include "ee_velocity_streaming_planner.h"

namespace robot_server_lib
{

EEVelocityStreamingPlanner::EEVelocityStreamingPlanner(RobotModel robot_model, std::shared_ptr<SingleWriterLatestBuffer<StreamingCommand>> streaming_buffer,
                                               double controller_dt)
    : MotionPlannerBase(robot_model), streaming_buffer_(streaming_buffer)
{
    controller_dt_ = controller_dt;
}

EEVelocityStreamingPlanner::~EEVelocityStreamingPlanner() = default;

ComputeCommandStatus EEVelocityStreamingPlanner::ComputeCommand(const JointMeasurement &measurement, const TimeMetaInfo &time_meta,
                                                            JointCommand &output_joint_cmd)
{
    if (!runtime_.streaming_finished)
    {
        StreamingCommand cmd_buffer;
        bool received = streaming_buffer_->PeekLatest(&cmd_buffer);
        if (received && cmd_buffer.request_id == runtime_.request_id)
        {
            if(cmd_buffer.command.size() != 6 && cmd_buffer.command.size() != 12)
            {
                LOG_CRIT << "EEVelocityStreamingPlanner: data size error: " << cmd_buffer.command.size() << ", expected 6 or 12.";
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

    // 检查是否超时
    bool is_timeout = false;
    if (latest_cmd.received_time.is_valid())
    {
        auto time_interval = (time_meta.current_time.time_in_ns() - latest_cmd.received_time.time_in_ns()) * 1e-9;
        is_timeout = time_interval > StreamingCommandTimeout;
    }
    runtime_.streaming_finished = latest_cmd.streaming_finished || is_timeout;

    Eigen::VectorXd desired_q;
    utils::GetJointFromWholeBody(this->robot_model_, measurement.q_desired_pre, desired_q, this->motion_group_);
    Vector6d input_ee_velocity;
    if (!runtime_.streaming_finished)
    {
        // 处理输入更新对应motion group的关节角
        if(6 != latest_cmd.command.size())
        {
            return ComputeCommandStatus::Error;
        }
        input_ee_velocity = latest_cmd.command;
        // 根据参考系转换速度
        if (reference_frame_ == ReferenceFrame::EEFrame)
        {
            Eigen::Affine3d transform(pinocchio::GetFK(this->pinocchio_model_, desired_q));
            input_ee_velocity.head<3>() = transform.linear() * input_ee_velocity.head<3>();
            input_ee_velocity.tail<3>() = transform.linear() * input_ee_velocity.tail<3>();
        }
    }
    else
    {
        input_ee_velocity.setZero();
    }
    // 计算当前位置的雅可比矩阵
	Eigen::MatrixXd jacobian = pinocchio::GetJacobiMatrix(this->pinocchio_model_, desired_q);
    // 计算雅可比伪逆矩阵
    Eigen::MatrixXd jacobian_pinv;
    pinocchio::PseudoInverse(jacobian, jacobian_pinv);
    // 计算关节速度解
    runtime_.qdot_solved = jacobian_pinv * input_ee_velocity;

    //映射回whole body关节速度
    auto last_cmd_qdot = output_joint_cmd.velocity;
    utils::MapJointToWholeBody(this->robot_model_, runtime_.qdot_solved, output_joint_cmd.velocity, this->motion_group_);

    output_joint_cmd.position = measurement.q_desired_pre + output_joint_cmd.velocity * controller_dt_;
    output_joint_cmd.acceleration = (output_joint_cmd.velocity - last_cmd_qdot) / controller_dt_;
    output_joint_cmd.torque_feedforward.setZero();

    bool robot_stopped = (measurement.velocity.array() < IdleVelocityThreshold).all();

    // 需同时满足机器人静止和streaming_finished为true，才能结束本次streaming
    if (runtime_.streaming_finished && robot_stopped)
    {
        LOG_INFO << "EEVelocityStreamingPlanner: TaskFinished, measurement velocity: " << measurement.velocity.transpose();
        return ComputeCommandStatus::TaskFinished;
    }
    else
    {
        return ComputeCommandStatus::TaskOngoing;
    }
}

bool EEVelocityStreamingPlanner::ResetForNewTrajectory(const PlannerRequest &request, const JointMeasurement &measurement, const TimeMetaInfo &time_meta)
{
    assert(request.data.has_value());
    assert(std::holds_alternative<MoveRequest>(request.data.value()));
    auto move_request = std::get<MoveRequest>(request.data.value());
    SetMotionGroup(move_request.motion_group);

    runtime_.request_id = request.request_id;
    runtime_.streaming_finished = false;
    runtime_.latest_cmd = std::nullopt;

    if (motion_group_ == MotionGroup::Primary)
    {
        runtime_.qdot_solved.setZero(ndof_primary_);
		this->pinocchio_model_ = *this->robot_model_.primary_arm_model_ptr;
    }
    else if (motion_group_ == MotionGroup::Secondary)
    {
        runtime_.qdot_solved.setZero(ndof_secondary_);
		this->pinocchio_model_ = *this->robot_model_.secondary_arm_model_ptr;
    }
    else
    {
        LOG_CRIT << "EEVelocityStreamingPlanner: Unsupported motion group.";
        return false;
    }

    // 默认为eeframe速度控制, 默认单臂控制
    if (move_request.reference_frame.has_value())
    {
        reference_frame_ = move_request.reference_frame.value();
        if (reference_frame_ != ReferenceFrame::EEFrame && reference_frame_ != ReferenceFrame::ArmBase)
        {
            LOG_CRIT << "EEVelocityStreamingPlanner: Unsupported reference frame.";
            return false;
        }
    }

    // 记录请求开始时间（用于判断没有收到任何流式命令时的超时）
    request_start_time_ = TimePointSystem::Now();
    return true;
}

}  // namespace robot_server_lib
