#include "idle_planner.h"

namespace robot_server_lib
{

IdlePlanner::IdlePlanner(RobotModel robot_model) : MotionPlannerBase(robot_model) {}

IdlePlanner::~IdlePlanner() = default;

ComputeCommandStatus IdlePlanner::ComputeCommand(const JointMeasurement &measurement, const TimeMetaInfo &time_meta, JointCommand &joint_cmd)
{
    joint_cmd.position = idle_position_ = measurement.q_desired_pre;
    joint_cmd.velocity.setZero();
    joint_cmd.acceleration.setZero();
    joint_cmd.torque_feedforward.setZero();
    return ComputeCommandStatus::TaskOngoing;
}

bool IdlePlanner::ResetForNewTrajectory(const PlannerRequest &request, const JointMeasurement &measurement, const TimeMetaInfo &time_meta)
{
    idle_position_ = measurement.q_desired_pre;

    Eigen::VectorXd q_idle;
    GetArmFromWholeBody(idle_position_, q_idle, MotionGroup::BothArms);
    // 获取起点
    PoseQuaternion pose_idle = kinematics_ptr_->GetFK(q_idle.head(ndof_primary_), robot_property_.tcp_frame);
    PoseQuaternion pose_idle_secondary = kinematics_secondary_ptr_->GetFK(q_idle.tail(ndof_secondary_), robot_property_.tcp_frame_secondary);

    std::stringstream ss;
    LOG_INFO << "";
    ss << "Reset idle joint to: " << idle_position_.transpose();
    LOG_INFO << ss.str();
    ss.str("");
    ss << "Primary_joint_idle: " << q_idle.head(ndof_primary_).transpose() << ", Secondary_joint_idle: " << q_idle.tail(ndof_secondary_).transpose();
    LOG_INFO << ss.str();
    LOG_INFO << "Idle pose primary: " << pose_idle.ToString();
    LOG_INFO << "Idle pose secondary: " << pose_idle_secondary.ToString();
    return true;
}

}  // namespace robot_server_lib
