#include "idle_planner.h"

namespace robot_server_lib {

IdlePlanner::IdlePlanner(RobotModel robot_model) : MotionPlannerBase(robot_model) {}

IdlePlanner::~IdlePlanner() = default;

ComputeCommandStatus IdlePlanner::ComputeCommand(const JointMeasurement &measurement,
                                                 const TimeMetaInfo &time_meta,
                                                 JointCommand &joint_cmd) {
    joint_cmd.position = idle_position_ = measurement.q_desired_pre;
    joint_cmd.velocity.setZero();
    joint_cmd.acceleration.setZero();
    joint_cmd.torque_feedforward.setZero();
    return ComputeCommandStatus::TaskOngoing;
}

ComputeCommandStatus IdlePlanner::ComputeStopCommand(const JointMeasurement &measurement,
                                                     const TimeMetaInfo &time_meta,
                                                     JointCommand &joint_cmd) {
    joint_cmd.position = idle_position_ = measurement.q_desired_pre;
    joint_cmd.velocity.setZero();
    joint_cmd.acceleration.setZero();
    joint_cmd.torque_feedforward.setZero();
    return ComputeCommandStatus::TaskOngoing;
}

bool IdlePlanner::ResetForNewTrajectory(const PlannerRequest &request,
                                        const JointMeasurement &measurement,
                                        const TimeMetaInfo &time_meta) {
    idle_position_ = measurement.q_desired_pre;

    Eigen::VectorXd q_idle;
    utils::GetJointFromWholeBody(this->robot_model_, idle_position_, q_idle, MotionGroup::BothArms);
    // 获取起点
    PoseQuaternion pose_idle = kinematics_primary_ptr_->GetFK(q_idle.head(ndof_primary_), robot_property_.tcp_frame);
    PoseQuaternion pose_idle_secondary =
        kinematics_secondary_ptr_->GetFK(q_idle.tail(ndof_secondary_), robot_property_.tcp_frame_secondary);

    LOG_INFO << "Idle Joint Position: " << idle_position_.transpose();
    LOG_INFO << "Idle pose primary: " << pose_idle.ToString()
             << ", Idle pose secondary: " << pose_idle_secondary.ToString();
    LOG_INFO << "";
    return true;
}

}  // namespace robot_server_lib
