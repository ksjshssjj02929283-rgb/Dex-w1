#include "move_joint_planner.h"

namespace robot_server_lib {

MoveJointPlanner::MoveJointPlanner(RobotModel robot_model, double controller_dt, bool wait_trajectory_upon_finish)
    : MotionPlannerBase(robot_model),
      ruckig_otg_(controller_dt),
      wait_trajectory_upon_finish_(wait_trajectory_upon_finish) {
    controller_dt_ = controller_dt;
    for (int i = 0; i < ndof_robot; ++i) {
        runtime_.input.max_velocity[i] = joint_bounds_.max_velocity;
        runtime_.input.max_acceleration[i] = joint_bounds_.max_acceleration;
        runtime_.input.max_jerk[i] = joint_bounds_.max_jerk;
    }
}

MoveJointPlanner::~MoveJointPlanner() = default;

ComputeCommandStatus MoveJointPlanner::ComputeCommand(const JointMeasurement &measurement,
                                                      const TimeMetaInfo &time_meta,
                                                      JointCommand &joint_cmd) {
    auto status = ruckig_otg_.update(runtime_.input, runtime_.output);
    switch (status) {
        case ruckig::Working: {
            for (auto i = 0; i < ndof_robot; i++) {
                joint_cmd.position[i] = runtime_.output.new_position[i];
                joint_cmd.velocity[i] = runtime_.output.new_velocity[i];
                joint_cmd.acceleration[i] = runtime_.output.new_acceleration[i];
                joint_cmd.torque_feedforward[i] = 0;
            }
            runtime_.output.pass_to_input(runtime_.input);
            return ComputeCommandStatus::TaskOngoing;
        }
        case ruckig::Finished: {
            bool robot_stopped = true;
            for (auto i = 0; i < ndof_robot; i++) {
                joint_cmd.position[i] = runtime_.output.new_position[i];
                joint_cmd.velocity[i] = 0;
                joint_cmd.acceleration[i] = 0;
                joint_cmd.torque_feedforward[i] = 0;
                robot_stopped = robot_stopped && measurement.velocity[i] < IdleVelocityThreshold;
            }
            if (wait_trajectory_upon_finish_) {
                if (robot_stopped) {
                    LOG_INFO << "MoveJointPlanner: TaskFinished, measurement velocity: "
                             << measurement.velocity.transpose();
                    return ComputeCommandStatus::TaskFinished;
                } else {
                    return ComputeCommandStatus::TaskOngoing;
                }
            } else {
                return ComputeCommandStatus::TaskFinished;
            }
        }
        default:
            return ComputeCommandStatus::Error;
    }
}

bool MoveJointPlanner::ResetForNewTrajectory(const PlannerRequest &request,
                                             const JointMeasurement &measurement,
                                             const TimeMetaInfo &time_meta) {
    assert(request.data.has_value());
    assert(std::holds_alternative<MoveRequest>(request.data.value()));

    const auto &move_request = std::get<MoveRequest>(request.data.value());
    const auto &target_q = move_request.value;
    double speed_ratio = robot_property_.global_speed_ratio;
    double acc_ratio = 1.0;  // 加速度比例
    if (move_request.speed.has_value()) {
        speed_ratio = speed_ratio * std::clamp(move_request.speed.value(), 0.01, 2.0);
    }
    if (move_request.acceleration.has_value()) {
        acc_ratio = std::clamp(move_request.acceleration.value(), 0.01, 2.0);
    }
    for (int i = 0; i < ndof_robot; ++i) {
        runtime_.input.max_velocity[i] = joint_bounds_.max_velocity * speed_ratio;
        runtime_.input.max_acceleration[i] = joint_bounds_.max_acceleration * acc_ratio;
    }

    std::ostringstream oss;
    oss << target_q.transpose();
    LOG_INFO << "";
    LOG_INFO << "MoveJoint Request: id: " << request.request_id
             << " motion_group: " << ToString(move_request.motion_group) << " target_q: " << oss.str();
    LOG_INFO << "MoveJoint speed: " << speed_ratio
             << " (movejoint speed: " << (move_request.speed.has_value() ? move_request.speed.value() : 1.0)
             << ", global speed ratio: " << robot_property_.global_speed_ratio << ")"
             << ", acceleration: " << acc_ratio;
    JointPositionDof target_ndof = measurement.q_desired_pre;
    SetMotionGroup(move_request.motion_group);
    utils::MapJointToWholeBody(this->robot_model_, target_q, target_ndof, this->motion_group_);

    for (auto i = 0; i < ndof_robot; i++) {
        runtime_.input.current_position[i] = measurement.q_desired_pre[i];
        runtime_.input.current_velocity[i] = measurement.velocity[i];
        runtime_.input.current_acceleration[i] = 0.0;
        runtime_.input.target_position[i] = target_ndof[i];
        runtime_.input.target_velocity[i] = 0.0;
        runtime_.input.target_acceleration[i] = 0.0;
    }
    return true;
}

}  // namespace robot_server_lib
