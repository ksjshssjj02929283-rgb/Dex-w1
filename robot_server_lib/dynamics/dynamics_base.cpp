#include "dynamics_base.h"

namespace robot_server_lib
{
namespace dynamics
{

Eigen::VectorXd InverseDynamics(const RobotModel& robot_model, pinocchio::Data& data, const VectorDof& q, const VectorDof& dq,
                                const VectorDof& ddq)
{
    const auto& model = *robot_model.model_ptr;
    const auto& joint_command_sequence_by_name = robot_model.params_ptr->joint_command_sequence_by_name;
    const auto& dynamics_base_parameters_primary = robot_model.params_ptr->dynamics_base_parameters_primary;
    const auto& dynamics_base_parameters_secondary = robot_model.params_ptr->dynamics_base_parameters_secondary;
    // 将输入数据映射到URDF顺序
    Eigen::VectorXd q_urdf = MapToUrdfOrder(model, q, joint_command_sequence_by_name);
    Eigen::VectorXd dq_urdf = MapToUrdfOrder(model, dq, joint_command_sequence_by_name);
    Eigen::VectorXd ddq_urdf = MapToUrdfOrder(model, ddq, joint_command_sequence_by_name);
    // 计算逆动力学
    auto tau_urdf = pinocchio::rnea(model, data, q_urdf, dq_urdf, ddq_urdf);
    VectorDof tau = MapToParamOrder(model, tau_urdf, joint_command_sequence_by_name); // 将结果映射回joint_command_sequence_by_name顺序
    // 如果提供了动力学最小参数集矩阵，则进行替换
    if (dynamics_base_parameters_primary.has_value())
    {
        Eigen::VectorXd arm_tau = InvDynamikBySymbol(robot_model, data, q, dq, ddq, MotionGroup::Primary);
        utils::MapJointToWholeBody(robot_model, arm_tau, tau, MotionGroup::Primary);
    }
    if (dynamics_base_parameters_secondary.has_value())
    {
        Eigen::VectorXd arm_tau = InvDynamikBySymbol(robot_model, data, q, dq, ddq, MotionGroup::Secondary);
        utils::MapJointToWholeBody(robot_model, arm_tau, tau, MotionGroup::Secondary);
    }
    return tau;
}

Eigen::VectorXd ForwardDynamics(const pinocchio::Model& model, pinocchio::Data& data, const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
                                const Eigen::VectorXd& torque_cmd, const std::vector<std::string>& joint_command_sequence_by_name)
{
    // 将输入数据映射到URDF顺序
    Eigen::VectorXd q_urdf = MapToUrdfOrder(model, q, joint_command_sequence_by_name);
    Eigen::VectorXd dq_urdf = MapToUrdfOrder(model, dq, joint_command_sequence_by_name);
    Eigen::VectorXd torque_urdf = MapToUrdfOrder(model, torque_cmd, joint_command_sequence_by_name);

    // 计算正动力学
    auto ddq_urdf = pinocchio::aba(model, data, q_urdf, dq_urdf, torque_urdf);

    // 将结果映射回joint_command_sequence_by_name顺序
    auto ddq = MapToParamOrder(model, ddq_urdf, joint_command_sequence_by_name);
    return ddq;
}

Eigen::VectorXd MapToUrdfOrder(const pinocchio::Model& model, const Eigen::VectorXd& input,
                               const std::vector<std::string>& joint_command_sequence_by_name)
{
    Eigen::VectorXd output = Eigen::VectorXd::Zero(model.nv);

    for (size_t i = 0; i < joint_command_sequence_by_name.size(); ++i)
    {
        const std::string& joint_name = joint_command_sequence_by_name[i];
        if (model.existJointName(joint_name))
        {
            int joint_id = model.getJointId(joint_name);
            int urdf_index = joint_id - 1;

            if (urdf_index >= 0 && urdf_index < model.nv && i < size_t(input.size()))
            {
                output[urdf_index] = input[i];
            }
        }
    }

    return output;
}

Eigen::VectorXd MapToParamOrder(const pinocchio::Model& model, const Eigen::VectorXd& input,
                                const std::vector<std::string>& joint_command_sequence_by_name)
{
    Eigen::VectorXd output = Eigen::VectorXd::Zero(joint_command_sequence_by_name.size());

    for (size_t i = 0; i < joint_command_sequence_by_name.size(); ++i)
    {
        const std::string& joint_name = joint_command_sequence_by_name[i];
        if (model.existJointName(joint_name))
        {
            int joint_id = model.getJointId(joint_name);
            int urdf_index = joint_id - 1;

            if (urdf_index >= 0 && urdf_index < model.nv && urdf_index < input.size())
            {
                output[i] = input[urdf_index];
            }
        }
    }

    return output;
}
}  // namespace dynamics
}  // namespace robot_server_lib