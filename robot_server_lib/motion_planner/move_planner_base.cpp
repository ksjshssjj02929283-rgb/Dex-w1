#include "move_planner_base.h"

namespace robot_server_lib
{

MotionPlannerBase::MotionPlannerBase(RobotModel robot_model)
{
    // Initialize robot model
    robot_model_ = std::move(robot_model);
    robot_data_ = pinocchio::Data(*robot_model_.model_ptr);
    // Initialize kinematics for both arms
    if (robot_model_.params_ptr->left_arm_name == "W1LeftArmIndustrial")
    {
        kinematics_ptr_ = std::make_shared<kinematics::SRSKinematics>(kinematics::W1LeftArmIndustrial());
    }
    if (robot_model_.params_ptr->left_arm_name == "W1LeftArmAnthropomorphic")
    {
        kinematics_ptr_ = std::make_shared<kinematics::SRSKinematics>(kinematics::W1LeftArmAnthropomorphic());
    }
    if (robot_model_.params_ptr->right_arm_name == "W1RightArmIndustrial")
    {
        kinematics_secondary_ptr_ = std::make_shared<kinematics::SRSKinematics>(kinematics::W1RightArmIndustrial());
    }
    if (robot_model_.params_ptr->right_arm_name == "W1RightArmAnthropomorphic")
    {
        kinematics_secondary_ptr_ = std::make_shared<kinematics::SRSKinematics>(kinematics::W1RightArmAnthropomorphic());
    }
    assert(kinematics_ptr_ && kinematics_secondary_ptr_);
}

void MotionPlannerBase::SetRobotProperty(const RobotProperty &property) { robot_property_ = property; }

}  // namespace robot_server_lib
