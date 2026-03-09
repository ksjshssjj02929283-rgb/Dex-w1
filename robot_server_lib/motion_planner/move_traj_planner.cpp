#include "move_traj_planner.h"

#include <cassert>
#include <iostream>

namespace robot_server_lib {

MoveTrajPlanner::MoveTrajPlanner(RobotModel model, double controller_dt, bool wait_trajectory_upon_finish)
    : MotionPlannerBase(model), wait_trajectory_upon_finish_(wait_trajectory_upon_finish) {
    controller_dt_ = controller_dt;
}

MoveTrajPlanner::~MoveTrajPlanner() = default;

ComputeCommandStatus MoveTrajPlanner::ComputeCommand(const JointMeasurement &measurement,
                                                     const TimeMetaInfo &time_meta,
                                                     JointCommand &output_joint_cmd) {
    // 推进时间
    runtime_.timepoint += controller_dt_;
    ComputeCommandStatus ret = GetInterpolatedPosition(output_joint_cmd, runtime_.timepoint);
    if (wait_trajectory_upon_finish_) {
        bool robot_stopped = true;
        for (auto i = 0; i < ndof_robot; i++) {
            robot_stopped = robot_stopped && measurement.velocity[i] < IdleVelocityThreshold;
        }
        if (ret == ComputeCommandStatus::TaskFinished) {
            if (robot_stopped) {
                LOG_INFO << "MoveTrajPlanner: Reached the end of trajectory at time " << runtime_.total_duration
                         << "s, point index " << (runtime_.traj_npoints - 1)
                         << " measurement velocity: " << measurement.velocity.transpose();
                return ComputeCommandStatus::TaskFinished;
            } else {
                return ComputeCommandStatus::TaskOngoing;
            }
        }
    }
    return ret;
}

ComputeCommandStatus MoveTrajPlanner::GetInterpolatedPosition(JointCommand &joint_cmd, double timepoint) {
    double t = std::max(0.0, std::min(timepoint, runtime_.total_duration));

    int point_idx = static_cast<int>(t / runtime_.interval);
    if (point_idx >= runtime_.traj_npoints - 1) point_idx = runtime_.traj_npoints - 2;

    double t_point_start = point_idx * runtime_.interval;
    double local_t = t - t_point_start;
    double alpha = runtime_.interval > 0.0 ? (local_t / runtime_.interval) : 0.0;
    alpha = std::clamp(alpha, 0.0, 1.0);

    const Eigen::VectorXd &p0 = runtime_.target_traj[point_idx];
    const Eigen::VectorXd &p1 = runtime_.target_traj[point_idx + 1];

    Eigen::VectorXd position = (1.0 - alpha) * p0 + alpha * p1;
    Eigen::VectorXd velocity = (p1 - p0) / runtime_.interval;
    Eigen::VectorXd acceleration = Eigen::VectorXd::Zero(runtime_.traj_dof);

    utils::MapJointToWholeBody(this->robot_model_, position, joint_cmd.position, this->motion_group_);
    utils::MapJointToWholeBody(this->robot_model_, velocity, joint_cmd.velocity, this->motion_group_);
    utils::MapJointToWholeBody(this->robot_model_, acceleration, joint_cmd.acceleration, this->motion_group_);

    if (utils::FloatCompare(timepoint, runtime_.total_duration) >= 0) {
        position = runtime_.target_traj.back();
        velocity.setZero();
        acceleration.setZero();
        utils::MapJointToWholeBody(this->robot_model_, position, joint_cmd.position, this->motion_group_);
        utils::MapJointToWholeBody(this->robot_model_, velocity, joint_cmd.velocity, this->motion_group_);
        utils::MapJointToWholeBody(this->robot_model_, acceleration, joint_cmd.acceleration, this->motion_group_);
        return ComputeCommandStatus::TaskFinished;
    }

    double prev_t = timepoint - controller_dt_;
    bool phase_reached = false;
    for (int end_id : runtime_.endpoint_index) {
        if (end_id < 0 || end_id >= runtime_.traj_npoints) continue;
        double end_time = end_id * runtime_.interval;
        if (utils::FloatCompare(end_time, runtime_.total_duration) >= 0) continue;

        bool reached = (utils::FloatCompare(prev_t, end_time) < 0) && (utils::FloatCompare(t, end_time) >= 0);
        if (utils::FloatEqual(t, end_time)) reached = true;

        if (reached) {
            const Eigen::VectorXd &pend = runtime_.target_traj[end_id];
            Eigen::VectorXd vnext = Eigen::VectorXd::Zero(runtime_.traj_dof);
            if (end_id + 1 < runtime_.traj_npoints)
                vnext = (runtime_.target_traj[end_id + 1] - runtime_.target_traj[end_id]) / runtime_.interval;

            utils::MapJointToWholeBody(this->robot_model_, pend, joint_cmd.position, this->motion_group_);
            utils::MapJointToWholeBody(this->robot_model_, vnext, joint_cmd.velocity, this->motion_group_);
            acceleration.setZero();
            utils::MapJointToWholeBody(this->robot_model_, acceleration, joint_cmd.acceleration, this->motion_group_);
            LOG_INFO << "MoveTrajPlanner: Reached trajectory phase endpoint at time " << timepoint << "s, point index "
                     << end_id;
            phase_reached = true;
            break;
        }
    }

    if (phase_reached) return ComputeCommandStatus::PhaseFinished;

    return ComputeCommandStatus::TaskOngoing;
}

bool MoveTrajPlanner::ResetForNewTrajectory(const PlannerRequest &request,
                                            const JointMeasurement &measurement,
                                            const TimeMetaInfo &time_meta) {
    assert(request.data.has_value());
    assert(std::holds_alternative<MoveRequest>(request.data.value()));

    runtime_.Reset();

    auto move_request = std::get<MoveRequest>(request.data.value());
    SetMotionGroup(move_request.motion_group);

    switch (motion_group_) {
        case MotionGroup::Primary:
            runtime_.traj_dof = ndof_primary_;
            break;
        case MotionGroup::Secondary:
            runtime_.traj_dof = ndof_secondary_;
            break;
        case MotionGroup::BothArms:
            runtime_.traj_dof = ndof_primary_ + ndof_secondary_;
            break;
        case MotionGroup::WholeBody:
            runtime_.traj_dof = ndof_robot;
            break;
        case MotionGroup::Torso:
            runtime_.traj_dof = ndof_torso_;
            break;
        default:
            LOG_CRIT << "MoveTrajPlanner: Unknown MotionGroup";
            return false;
    }

    Eigen::VectorXd q_start(runtime_.traj_dof);
    utils::GetJointFromWholeBody(this->robot_model_, measurement.q_desired_pre, q_start, this->motion_group_);

    if (move_request.value.size() % runtime_.traj_dof != 0) {
        LOG_CRIT << "MoveTrajPlanner: target size " << move_request.value.size() << " is not multiple of traj_dof "
                 << runtime_.traj_dof;
        return false;
    }

    int user_npoints = static_cast<int>(move_request.value.size() / runtime_.traj_dof);
    if (user_npoints <= 0) {
        LOG_CRIT << "MoveTrajPlanner::ResetForNewTrajectory: empty user trajectory points";
        return false;
    }

    if (move_request.interval.has_value()) {
        runtime_.interval = move_request.interval.value();
    } else {
        LOG_CRIT << "MoveTrajPlanner: missing trajectory interval";
        return false;
    }
    if (move_request.endpoint_index.has_value()) {
        runtime_.endpoint_index = move_request.endpoint_index.value();
        // 校验传的最后一个index是否等于点位数
        if (!runtime_.endpoint_index.empty() && runtime_.endpoint_index.back() != user_npoints) {
            LOG_CRIT << "MoveTrajPlanner: endpoint_index's last index " << runtime_.endpoint_index.back()
                     << " is not equal to user_npoints " << user_npoints;
            return false;
        }
    } else {
        runtime_.endpoint_index.push_back(user_npoints);  // 默认整个轨迹为一个阶段
    }
    // 插入起点：轨迹点总数 = 起点 + 用户点
    runtime_.traj_npoints = user_npoints + 1;
    runtime_.total_duration = (runtime_.traj_npoints - 1) * runtime_.interval;  // = user_npoints * interval
    runtime_.target_traj.reserve(runtime_.traj_npoints);

    // 第 0 个点：当前关节状态 q_start, 加入起点为第0个点, 正好让轨迹点对齐用户输入的endpoint_index
    runtime_.target_traj.emplace_back(q_start);
    for (int i = 0; i < user_npoints; ++i) {
        runtime_.target_traj.emplace_back(move_request.value.segment(i * runtime_.traj_dof, runtime_.traj_dof));
    }
    LOG_INFO << "MoveTrajPlanner: New trajectory loaded: traj_dof = " << runtime_.traj_dof
             << ", traj_npoints = " << user_npoints << ", interval = " << runtime_.interval
             << ", endpoint_index = " << runtime_.endpoint_index << ", total_duration = " << runtime_.total_duration
             << "s"
             << "\ncurrent posision:   " << q_start.transpose()
             << "\nfirst target point: " << runtime_.target_traj[1].transpose()
             << "\nend target point:   " << runtime_.target_traj.back().transpose();

    return true;
}

}  // namespace robot_server_lib
