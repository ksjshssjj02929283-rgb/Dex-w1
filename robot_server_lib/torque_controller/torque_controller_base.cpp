#include "torque_controller_base.h"

namespace robot_server_lib
{
TorqueControllerBase::TorqueControllerBase(RobotModel robot_model) : robot_model_(std::move(robot_model)), robot_data_(*robot_model_.model_ptr) {}

TorqueControllerBase::~TorqueControllerBase() = default;

// 末端六维力传感器数据转换为关节外力矩
VectorDof TorqueControllerBase::GetJointExternalTorque(const Eigen::VectorXd &sensor_wrench, const VectorDof &q_desired)
{
    VectorDof torque_ndof = VectorDof::Zero();
    Eigen::VectorXd q_desired_arm;
    Eigen::VectorXd torque_arm;
    assert(sensor_wrench.size() == 12);
    Eigen::VectorXd q_desired_arm_secondary;
    utils::GetArmFromWholeBody(this->robot_model_, q_desired, q_desired_arm, MotionGroup::Primary);
    utils::GetArmFromWholeBody(this->robot_model_, q_desired, q_desired_arm_secondary, MotionGroup::Secondary);

    auto J_left = pinocchio::GetJacobiMatrix(*this->robot_model_.left_arm_model_ptr, q_desired_arm);
    auto J_right = pinocchio::GetJacobiMatrix(*this->robot_model_.right_arm_model_ptr, q_desired_arm_secondary);

    torque_arm.resize(q_desired_arm.size() + q_desired_arm_secondary.size());
    torque_arm << J_left.transpose() * sensor_wrench.head(6), J_right.transpose() * sensor_wrench.tail(6);

    utils::MapArmToWholeBody(this->robot_model_, torque_arm, torque_ndof, MotionGroup::BothArms);

    return torque_ndof;
}

}  // namespace robot_server_lib
