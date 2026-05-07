#include "move_planner_base.h"

namespace robot_server_lib {

MotionPlannerBase::MotionPlannerBase(RobotModel robot_model) {
    // Initialize robot model
    robot_model_ = std::move(robot_model);
    robot_data_ = pinocchio::Data(*robot_model_.model_ptr);
    // Initialize kinematics for both arms
    if (robot_model_.params_ptr->left_arm_name == "W1LeftArmIndustrial") {
        bool is_left = true;
        bool is_anthropmorphic = false;
        kinematics_primary_ptr_ = std::make_shared<kinematics::SRSKinematics>(is_left, is_anthropmorphic);
    }
    if (robot_model_.params_ptr->left_arm_name == "W1LeftArmAnthropomorphic") {
        kinematics_primary_ptr_ = std::make_shared<kinematics::SRSKinematics>(true, true);
    }
    if (robot_model_.params_ptr->right_arm_name == "W1RightArmIndustrial") {
        kinematics_secondary_ptr_ = std::make_shared<kinematics::SRSKinematics>(false, false);
    }
    if (robot_model_.params_ptr->right_arm_name == "W1RightArmAnthropomorphic") {
        kinematics_secondary_ptr_ = std::make_shared<kinematics::SRSKinematics>(false, true);
    }
    kinematics_torso_ptr_ = std::make_shared<kinematics::TorsoKinematics>();
    assert(kinematics_primary_ptr_ && kinematics_secondary_ptr_ && kinematics_torso_ptr_);
}

ComputeCommandStatus MotionPlannerBase::ComputeStopCommand(const JointMeasurement& measurement,
                                                           const TimeMetaInfo& time_meta,
                                                           JointCommand& joint_cmd) {
    bool stop_finished = true;
    auto output_velocity = joint_cmd.velocity;
    bool robot_stopped = true;
    for (auto i = 0; i < ndof_robot; i++) {
        if (output_velocity[i] > 0.0 && output_velocity[i] > stop_bounds_.max_acceleration * controller_dt_) {
            output_velocity[i] -= stop_bounds_.max_acceleration * controller_dt_;
        } else if (output_velocity[i] < 0.0 && output_velocity[i] < -stop_bounds_.max_acceleration * controller_dt_) {
            output_velocity[i] += stop_bounds_.max_acceleration * controller_dt_;
        } else {
            output_velocity[i] = 0.0;
            stop_finished = stop_finished && true;
        }
        joint_cmd.velocity[i] = output_velocity[i];
        joint_cmd.position[i] += joint_cmd.velocity[i] * controller_dt_;
        robot_stopped = robot_stopped && measurement.velocity[i] < IdleVelocityThreshold;
    }
    robot_stopped = stop_finished && robot_stopped;
    if (robot_stopped) {
        LOG_INFO << "ComputeStopCommand: Robot stopped, velocity: " << measurement.velocity.transpose();
        return ComputeCommandStatus::TaskStopped;
    } else {
        return ComputeCommandStatus::TaskOngoing;
    }
}

void MotionPlannerBase::SetRobotProperty(const RobotProperty& property) { robot_property_ = property; }

}  // namespace robot_server_lib
