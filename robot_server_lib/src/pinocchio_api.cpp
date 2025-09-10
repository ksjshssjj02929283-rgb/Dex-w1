#include "pinocchio_api.h"

namespace pinocchio
{

Eigen::VectorXd MapToUrdfOrder(const Model& model, const Eigen::VectorXd& input, const std::vector<std::string>& joint_command_sequence_by_name)
{
    Eigen::VectorXd output = Eigen::VectorXd::Zero(model.nv);

    for (size_t i = 0; i < joint_command_sequence_by_name.size(); ++i)
    {
        const std::string& joint_name = joint_command_sequence_by_name[i];
        if (model.existJointName(joint_name))
        {
            JointIndex joint_id = model.getJointId(joint_name);
            int urdf_index = joint_id - 1;

            if (urdf_index >= 0 && urdf_index < model.nv && i < size_t(input.size()))
            {
                output[urdf_index] = input[i];
            }
        }
    }

    return output;
}

Eigen::VectorXd MapToParamOrder(const Model& model, const Eigen::VectorXd& input, const std::vector<std::string>& joint_command_sequence_by_name)
{
    Eigen::VectorXd output = Eigen::VectorXd::Zero(joint_command_sequence_by_name.size());

    for (size_t i = 0; i < joint_command_sequence_by_name.size(); ++i)
    {
        const std::string& joint_name = joint_command_sequence_by_name[i];
        if (model.existJointName(joint_name))
        {
            JointIndex joint_id = model.getJointId(joint_name);
            int urdf_index = joint_id - 1;

            if (urdf_index >= 0 && urdf_index < model.nv && urdf_index < input.size())
            {
                output[i] = input[urdf_index];
            }
        }
    }

    return output;
}

Eigen::VectorXd InverseDynamics(const Model& model, Data& data, const Eigen::VectorXd& q, const Eigen::VectorXd& dq, const Eigen::VectorXd& ddq,
                                const std::vector<std::string>& joint_command_sequence_by_name)
{
    // 将输入数据映射到URDF顺序
    Eigen::VectorXd q_urdf = MapToUrdfOrder(model, q, joint_command_sequence_by_name);
    Eigen::VectorXd dq_urdf = MapToUrdfOrder(model, dq, joint_command_sequence_by_name);
    Eigen::VectorXd ddq_urdf = MapToUrdfOrder(model, ddq, joint_command_sequence_by_name);

    // 计算逆动力学
    auto tau_urdf = rnea(model, data, q_urdf, dq_urdf, ddq_urdf);

    // 将结果映射回joint_command_sequence_by_name顺序
    auto tau = MapToParamOrder(model, tau_urdf, joint_command_sequence_by_name);
    return tau;
}

Eigen::VectorXd ForwardDynamics(const Model& model, Data& data, const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
                                const Eigen::VectorXd& torque_cmd, const std::vector<std::string>& joint_command_sequence_by_name)
{
    // 将输入数据映射到URDF顺序
    Eigen::VectorXd q_urdf = MapToUrdfOrder(model, q, joint_command_sequence_by_name);
    Eigen::VectorXd dq_urdf = MapToUrdfOrder(model, dq, joint_command_sequence_by_name);
    Eigen::VectorXd torque_urdf = MapToUrdfOrder(model, torque_cmd, joint_command_sequence_by_name);

    // 计算正动力学
    auto ddq_urdf = aba(model, data, q_urdf, dq_urdf, torque_urdf);

    // 将结果映射回joint_command_sequence_by_name顺序
    auto ddq = MapToParamOrder(model, ddq_urdf, joint_command_sequence_by_name);
    return ddq;
}
}  // namespace pinocchio
