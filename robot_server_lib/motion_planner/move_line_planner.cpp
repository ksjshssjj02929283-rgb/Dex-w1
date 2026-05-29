#include "move_line_planner.h"

namespace robot_server_lib {
// 构造函数实现
MoveLinePlanner::MoveLinePlanner(RobotModel robot_model, double controller_dt, bool wait_trajectory_upon_finish)
    : MotionPlannerBase(robot_model),
      ruckig_otg_(controller_dt),
      wait_trajectory_upon_finish_(wait_trajectory_upon_finish) {
    controller_dt_ = controller_dt;
    for (int idx : {Trans, TransSecondary, TransTorso}) {
        runtime_.input.max_velocity[idx] = cartesian_bounds_.translation.max_velocity;
        runtime_.input.max_acceleration[idx] = cartesian_bounds_.translation.max_acceleration;
        runtime_.input.max_jerk[idx] = cartesian_bounds_.translation.max_jerk;
    }
    for (int idx : {Rot, RotSecondary, RotTorso}) {
        runtime_.input.max_velocity[idx] = cartesian_bounds_.rotation.max_velocity;
        runtime_.input.max_acceleration[idx] = cartesian_bounds_.rotation.max_acceleration;
        runtime_.input.max_jerk[idx] = cartesian_bounds_.rotation.max_jerk;
    }
}

MoveLinePlanner::~MoveLinePlanner() = default;

ComputeCommandStatus MoveLinePlanner::ComputeCommand(const JointMeasurement& measurement,
                                                     const TimeMetaInfo& time_meta,
                                                     JointCommand& joint_cmd) {
    const double alpha = 0.1;  // Low-pass filter coefficient for velocity
    for (int i = 0; i < ndof_robot; ++i) {
        filtered_velocity_[i] = alpha * measurement.velocity[i] + (1.0 - alpha) * filtered_velocity_[i];
    }

    auto status = ruckig_otg_.update(runtime_.input, runtime_.output);
    JointPositionDof q_ndof = measurement.q_desired_pre;
    if (!ComputeOutputJoint(q_ndof)) {
        LOG_CRIT << "MoveLine: Solve IK failed";
        return ComputeCommandStatus::Error;
    }
    switch (status) {
        case ruckig::Working: {
            runtime_.output.pass_to_input(runtime_.input);
            for (auto i = 0; i < ndof_robot; i++) {
                joint_cmd.position[i] = q_ndof[i];
                joint_cmd.velocity[i] = 0.0;
                joint_cmd.acceleration[i] = 0.0;
                joint_cmd.torque_feedforward[i] = 0.0;
            }
            return ComputeCommandStatus::TaskOngoing;
        };
        case ruckig::Finished: {
            bool robot_stopped = true;
            for (auto i = 0; i < ndof_robot; i++) {
                joint_cmd.position[i] = q_ndof[i];
                joint_cmd.velocity[i] = 0.0;
                joint_cmd.acceleration[i] = 0.0;
                joint_cmd.torque_feedforward[i] = 0.0;
                robot_stopped = robot_stopped && std::abs(filtered_velocity_[i]) < IdleVelocityThreshold;
            }
            if (wait_trajectory_upon_finish_) {
                if (robot_stopped) {
                    LOG_INFO << "MoveLinePlanner: TaskFinished, measurement velocity: "
                             << measurement.velocity.transpose();
                    return ComputeCommandStatus::TaskFinished;
                } else {
                    return ComputeCommandStatus::TaskOngoing;
                }
            } else {
                return ComputeCommandStatus::TaskFinished;
            }
        };
        default:
            LOG_CRIT << "MoveLinePlanner: Ruckig OTG returned: " << static_cast<int>(status);
            return ComputeCommandStatus::Error;
    }
}

bool MoveLinePlanner::ResetForNewTrajectory(const PlannerRequest& request,
                                            const JointMeasurement& measurement,
                                            const TimeMetaInfo& time_meta) {
    filtered_velocity_ = measurement.velocity;
    assert(request.data.has_value());
    assert(std::holds_alternative<MoveRequest>(request.data.value()));
    const auto& move_request = std::get<MoveRequest>(request.data.value());
    SetMotionGroup(move_request.motion_group);
    double speed_ratio = robot_property_.global_speed_ratio;
    if (move_request.speed.has_value()) {
        speed_ratio = speed_ratio * std::clamp(move_request.speed.value(), 0.01, 1.0);
    }
    for (int idx : {Trans, TransSecondary, TransTorso}) {
        runtime_.input.max_velocity[idx] = cartesian_bounds_.translation.max_velocity * speed_ratio;
        runtime_.input.max_acceleration[idx] = cartesian_bounds_.translation.max_acceleration * speed_ratio;
    }
    for (int idx : {Rot, RotSecondary, RotTorso}) {
        runtime_.input.max_velocity[idx] = cartesian_bounds_.rotation.max_velocity * speed_ratio;
        runtime_.input.max_acceleration[idx] = cartesian_bounds_.rotation.max_acceleration * speed_ratio;
    }
    switch (move_request.motion_group) {
        case MotionGroup::BothArms: {
            if (14 != move_request.value.size()) {
                LOG_CRIT << "MoveRequest value size mismatch for BothArms! Expected: 14, Actual: "
                         << move_request.value.size();
                std::cout << "[Error] MoveRequest value size mismatch for BothArms! Expected: 14, Actual: "
                          << move_request.value.size() << "\n";
                exit(EXIT_FAILURE);
            }
            Eigen::VectorXd q_start;
            utils::GetJointFromWholeBody(this->robot_model_, measurement.q_desired_pre, q_start, this->motion_group_);
            if (q_start.size() != ndof_primary_ + ndof_secondary_) {
                LOG_CRIT << "q_start size mismatch for BothArms! Expected: " << (ndof_primary_ + ndof_secondary_)
                         << ", Actual: " << q_start.size();
                std::cout << "[Error] q_start size mismatch for BothArms! Expected: "
                          << (ndof_primary_ + ndof_secondary_) << ", Actual: " << q_start.size() << "\n";
                exit(EXIT_FAILURE);
            }
            PoseQuaternion pose_start =
                kinematics_primary_ptr_->GetFK(q_start.head(ndof_primary_), robot_property_.tcp_frame);
            PoseQuaternion pose_start_secondary =
                kinematics_secondary_ptr_->GetFK(q_start.tail(ndof_secondary_), robot_property_.tcp_frame_secondary);
            PoseQuaternion pose_target = utils::VectorXdToPoseQuaternion(move_request.value.head(7));
            PoseQuaternion pose_target_secondary = utils::VectorXdToPoseQuaternion(move_request.value.tail(7));
            LOG_INFO << "Primary arm motion plan: start: " << pose_start.ToString()
                     << ", target: " << pose_target.ToString();
            LOG_INFO << "Secondary arm motion plan: start: " << pose_start_secondary.ToString()
                     << ", target: " << pose_target_secondary.ToString();
            line_primary_.emplace(CalcLineGeometry(pose_start, pose_target));
            line_secondary_.emplace(CalcLineGeometry(pose_start_secondary, pose_target_secondary));
            line_torso_ = std::nullopt;
            break;
        }
        case MotionGroup::Primary: {
            if (7 != move_request.value.size()) {
                LOG_CRIT << "MoveRequest value size mismatch for Primary! Expected: 7, Actual: "
                         << move_request.value.size();
                std::cout << "[Error] MoveRequest value size mismatch for Primary! Expected: 7, Actual: "
                          << move_request.value.size() << "\n";
                exit(EXIT_FAILURE);
            }
            Eigen::VectorXd q_start;
            utils::GetJointFromWholeBody(this->robot_model_, measurement.q_desired_pre, q_start, this->motion_group_);
            if (q_start.size() != ndof_primary_) {
                LOG_CRIT << "q_start size mismatch for Primary! Expected: " << ndof_primary_
                         << ", Actual: " << q_start.size();
                std::cout << "[Error] q_start size mismatch for Primary! Expected: " << ndof_primary_
                          << ", Actual: " << q_start.size() << "\n";
                exit(EXIT_FAILURE);
            }
            PoseQuaternion pose_start = kinematics_primary_ptr_->GetFK(q_start, robot_property_.tcp_frame);
            PoseQuaternion pose_target = utils::VectorXdToPoseQuaternion(move_request.value);
            LOG_INFO << "Primary arm motion plan: start: " << pose_start.ToString()
                     << ", target: " << pose_target.ToString();
            line_primary_.emplace(CalcLineGeometry(pose_start, pose_target));
            line_secondary_ = std::nullopt;
            line_torso_ = std::nullopt;
            break;
        }
        case MotionGroup::Secondary: {
            if (7 != move_request.value.size()) {
                LOG_CRIT << "MoveRequest value size mismatch for Secondary! Expected: 7, Actual: "
                         << move_request.value.size();
                std::cout << "[Error] MoveRequest value size mismatch for Secondary! Expected: 7, Actual: "
                          << move_request.value.size() << "\n";
                exit(EXIT_FAILURE);
            }
            Eigen::VectorXd q_start;
            utils::GetJointFromWholeBody(this->robot_model_, measurement.q_desired_pre, q_start, this->motion_group_);
            if (q_start.size() != ndof_secondary_) {
                LOG_CRIT << "q_start size mismatch for Secondary! Expected: " << ndof_secondary_
                         << ", Actual: " << q_start.size();
                std::cout << "[Error] q_start size mismatch for Secondary! Expected: " << ndof_secondary_
                          << ", Actual: " << q_start.size() << "\n";
                exit(EXIT_FAILURE);
            }
            PoseQuaternion pose_start = kinematics_secondary_ptr_->GetFK(q_start, robot_property_.tcp_frame_secondary);
            PoseQuaternion pose_target = utils::VectorXdToPoseQuaternion(move_request.value);
            LOG_INFO << "Secondary arm motion plan: start: " << pose_start.ToString()
                     << ", target: " << pose_target.ToString();
            line_primary_ = std::nullopt;
            line_secondary_.emplace(CalcLineGeometry(pose_start, pose_target));
            line_torso_ = std::nullopt;
            break;
        }
        case MotionGroup::Torso: {
            if (3 != move_request.value.size()) {
                LOG_CRIT << "MoveRequest value size mismatch for Torso! Expected: 3, Actual: "
                         << move_request.value.size();
                std::cout << "[Error] MoveRequest value size mismatch for Torso! Expected: 3, Actual: "
                          << move_request.value.size() << "\n";
                exit(EXIT_FAILURE);
            }
            Eigen::VectorXd q_start;
            utils::GetJointFromWholeBody(this->robot_model_, measurement.q_desired_pre, q_start, this->motion_group_);
            if (q_start.size() != ndof_torso_) {
                LOG_CRIT << "q_start size mismatch for Torso! Expected: " << ndof_torso_
                         << ", Actual: " << q_start.size();
                std::cout << "[Error] q_start size mismatch for Torso! Expected: " << ndof_torso_
                          << ", Actual: " << q_start.size() << "\n";
                exit(EXIT_FAILURE);
            }
            PoseQuaternion pose_start = kinematics_torso_ptr_->GetFK(q_start);
            PoseQuaternion pose_target = kinematics_torso_ptr_->GetTorsoPose(move_request.value);
            line_primary_ = std::nullopt;
            line_secondary_ = std::nullopt;
            line_torso_.emplace(CalcLineGeometry(pose_start, pose_target));
            LOG_INFO << "Torso motion plan: q_start: " << q_start << ", start pos: " << pose_start.ToString()
                     << ", target: " << pose_target.ToString() << ", direction: " << line_torso_->direction.transpose()
                     << ", length: " << line_torso_->length << ", slerp_angle: " << line_torso_->slerp_angle;
            break;
        }
        default:
            LOG_CRIT << "Unknown MotionGroup type in MoveLinePlanner::SetTarget";
            std::cout << "[Error] Unknown MotionGroup type in MoveLinePlanner::SetTarget\n";
            exit(EXIT_FAILURE);
            break;
    }
    ConfigRuckig();
    return true;
}

LineGeometry MoveLinePlanner::CalcLineGeometry(const PoseQuaternion& pose_start, const PoseQuaternion& pose_end) {
    LineGeometry line;
    line.pose_start = pose_start;
    line.pose_target = pose_end;
    Eigen::Vector3d dir = pose_end.position - pose_start.position;
    double distance = dir.norm();
    assert(!std::isnan(distance));
    if (distance < 1e-6) {
        dir = {1, 0, 0};
    } else {
        dir.normalize();
    }
    line.length = distance;
    line.direction = dir;
    assert(fabs(pose_start.orientation.norm() - 1.0) < 1e-6);
    assert(fabs(pose_end.orientation.norm() - 1.0) < 1e-6);
    Eigen::Quaterniond slerpQ = (pose_start.orientation.conjugate()) * (pose_end.orientation);
    slerpQ.normalize();
    line.slerp_angle = fabs(Eigen::AngleAxisd(slerpQ).angle());
    if (std::isnan(line.slerp_angle)) {
        line.slerp_angle = 0;
    }
    return line;
}

void MoveLinePlanner::ConfigRuckig() {
    for (int i = 0; i < ruckig_dof_; i++) {
        runtime_.input.current_position[i] = 0.0;
        runtime_.input.current_velocity[i] = 0.0;
        runtime_.input.current_acceleration[i] = 0.0;
        runtime_.input.target_position[i] = 0.0;
        runtime_.input.target_velocity[i] = 0.0;
        runtime_.input.target_acceleration[i] = 0.0;
    }
    if (line_primary_.has_value()) {
        runtime_.input.target_position[Trans] = line_primary_->length;
        runtime_.input.target_position[Rot] = line_primary_->slerp_angle;
    }
    if (line_secondary_.has_value()) {
        runtime_.input.target_position[TransSecondary] = line_secondary_->length;
        runtime_.input.target_position[RotSecondary] = line_secondary_->slerp_angle;
    }
    if (line_torso_.has_value()) {
        runtime_.input.target_position[TransTorso] = line_torso_->length;
        runtime_.input.target_position[RotTorso] = line_torso_->slerp_angle;
    }
}

bool MoveLinePlanner::ComputeOutputJoint(JointPositionDof& q_ndof) {
    if (line_primary_.has_value()) {
        PoseQuaternion pose_cmd =
            ComputeOutputPose(*line_primary_, runtime_.output.new_position[Trans], runtime_.output.new_position[Rot]);
        Eigen::VectorXd q_primary;
        utils::GetJointFromWholeBody(this->robot_model_, q_ndof, q_primary, MotionGroup::Primary);
        assert(q_primary.size() == ndof_primary_);
        if (!kinematics_primary_ptr_->GetIK(pose_cmd, q_primary, robot_property_.tcp_frame)) {
            LOG_CRIT << "Inverse Kinematics failed for target pose: " << pose_cmd.ToString();
            return false;
        }
        utils::MapJointToWholeBody(this->robot_model_, q_primary, q_ndof, MotionGroup::Primary);
    }
    if (line_secondary_.has_value()) {
        PoseQuaternion pose_cmd_secondary = ComputeOutputPose(
            *line_secondary_, runtime_.output.new_position[TransSecondary], runtime_.output.new_position[RotSecondary]);
        Eigen::VectorXd q_secondary;
        utils::GetJointFromWholeBody(this->robot_model_, q_ndof, q_secondary, MotionGroup::Secondary);
        assert(q_secondary.size() == ndof_secondary_);
        if (!kinematics_secondary_ptr_->GetIK(pose_cmd_secondary, q_secondary, robot_property_.tcp_frame_secondary)) {
            LOG_CRIT << "Inverse Kinematics failed for target pose: " << pose_cmd_secondary.ToString();
            return false;
        }
        utils::MapJointToWholeBody(this->robot_model_, q_secondary, q_ndof, MotionGroup::Secondary);
    }
    if (line_torso_.has_value()) {
        PoseQuaternion pose_cmd_torso = ComputeOutputPose(*line_torso_, runtime_.output.new_position[TransTorso],
                                                          runtime_.output.new_position[RotTorso]);
        Eigen::VectorXd q_torso;
        utils::GetJointFromWholeBody(this->robot_model_, q_ndof, q_torso, MotionGroup::Torso);
        assert(q_torso.size() == ndof_torso_);
        if (!kinematics_torso_ptr_->GetIK(pose_cmd_torso, q_torso)) {
            LOG_CRIT << "Inverse Kinematics failed for target pose: " << pose_cmd_torso.ToString();
            return false;
        }
        utils::MapJointToWholeBody(this->robot_model_, q_torso, q_ndof, MotionGroup::Torso);
    }
    return true;
}

PoseQuaternion MoveLinePlanner::ComputeOutputPose(LineGeometry line,
                                                  double ruckig_trans_position,
                                                  double ruckig_rot_position) {
    PoseQuaternion pose_cmd;
    pose_cmd.position = line.pose_start.position + line.direction * ruckig_trans_position;
    if (line.slerp_angle < 1e-5) {
        pose_cmd.orientation = line.pose_target.orientation;
    } else {
        double t = ruckig_rot_position / line.slerp_angle;
        t = std::clamp(t, 0.0, 1.0);
        pose_cmd.orientation = line.pose_start.orientation.slerp(t, line.pose_target.orientation);
        pose_cmd.orientation.normalize();
        assert(fabs(pose_cmd.orientation.norm() - 1.0) < 1e-6);
    }
    return pose_cmd;
}

}  // namespace robot_server_lib
