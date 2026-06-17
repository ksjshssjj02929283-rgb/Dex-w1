#include "inverse_dynamics.h"

namespace robot_server_lib
{
namespace dynamics
{

Eigen::VectorXd InverseDynamics(const RobotModel& robot_model, const VectorDof& q, const VectorDof& dq,
                                const VectorDof& ddq)
{
    const auto& model = *robot_model.model_ptr;
    // URDF 顺序与控制顺序一致，无需映射
    pinocchio::Data data(model);
    VectorDof tau = pinocchio::rnea(model, data, q, dq, ddq).template cast<double>();

    // 如果提供了动力学最小参数集矩阵，则进行替换
    const auto& dyn_base_params_primary = robot_model.params_ptr->dyn_base_params_primary;
    const auto& dyn_base_params_secondary = robot_model.params_ptr->dyn_base_params_secondary;
    if (dyn_base_params_primary.has_value())
    {
        // 计算重力方向在arm base坐标系下的表示
        auto first_joint_urdf_name = robot_model.params_ptr->ros2_to_urdf_map.at(robot_model.params_ptr->joints_primary_names[0]);
        if (robot_model.model_ptr->existFrame(first_joint_urdf_name))
        {
            auto pose = pinocchio::GetFkByFrameId(*robot_model.model_ptr, q, robot_model.model_ptr->getFrameId(first_joint_urdf_name) - 1);
            Vector3d gravity_acc_armbase = pose.rotation().transpose() * Vector3d{0.0, 0.0, -9.81};
            Eigen::VectorXd tau_arm = InvDynamikBySymbol(robot_model, q, dq, ddq, gravity_acc_armbase, MotionGroup::Primary);
            utils::MapJointToWholeBody(robot_model, tau_arm, tau, MotionGroup::Primary);
        }
        else
        {
            std::cerr << "[ERROR] Frame " << first_joint_urdf_name << " does not exist in the robot model." << std::endl;
            LOG_CRIT << "Frame " << first_joint_urdf_name << " does not exist in the robot model.";
            exit(EXIT_FAILURE);
        }
    }
    if (dyn_base_params_secondary.has_value())
    {
        // 计算重力方向在arm base坐标系下的表示
        auto first_joint_urdf_name = robot_model.params_ptr->ros2_to_urdf_map.at(robot_model.params_ptr->joints_secondary_names[0]);
        if (robot_model.model_ptr->existFrame(first_joint_urdf_name))
        {
            auto pose = pinocchio::GetFkByFrameId(*robot_model.model_ptr, q, robot_model.model_ptr->getFrameId(first_joint_urdf_name) - 1);
            Vector3d gravity_acc_armbase = pose.rotation().transpose() * Vector3d{0.0, 0.0, -9.81};
            Eigen::VectorXd arm_tau = InvDynamikBySymbol(robot_model, q, dq, ddq, gravity_acc_armbase, MotionGroup::Secondary);
            utils::MapJointToWholeBody(robot_model, arm_tau, tau, MotionGroup::Secondary);
        }
        else
        {
            std::cerr << "[ERROR] Frame " << first_joint_urdf_name << " does not exist in the robot model." << std::endl;
            LOG_CRIT << "Frame " << first_joint_urdf_name << " does not exist in the robot model.";
            exit(EXIT_FAILURE);
        }
    }
    return tau;
}
}  // namespace dynamics
}  // namespace robot_server_lib