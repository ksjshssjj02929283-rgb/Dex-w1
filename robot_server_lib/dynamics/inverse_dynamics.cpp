#include "inverse_dynamics.h"

namespace robot_server_lib
{
namespace dynamics
{

Eigen::VectorXd InverseDynamics(const RobotModel& robot_model, const VectorDof& q, const VectorDof& dq, const VectorDof& ddq, bool use_simple_dynamics)
{
    const auto& model = *robot_model.model_ptr;
    const auto& joint_command_sequence_by_name = robot_model.params_ptr->joint_command_sequence_by_name;
    // 将输入数据映射到URDF顺序
    Eigen::VectorXd q_urdf = MapToUrdfOrder(model, q, joint_command_sequence_by_name);
    Eigen::VectorXd dq_urdf = MapToUrdfOrder(model, dq, joint_command_sequence_by_name);
    Eigen::VectorXd ddq_urdf = MapToUrdfOrder(model, ddq, joint_command_sequence_by_name);
    // 计算逆动力学
    pinocchio::Data data(model);
    auto tau_urdf = pinocchio::rnea(model, data, q_urdf, dq_urdf, ddq_urdf);
    VectorDof tau = MapToParamOrder(model, tau_urdf, joint_command_sequence_by_name); // 将结果映射回joint_command_sequence_by_name顺序

    // 如果 use_simple_dynamics 为 true，则跳过动力学参数计算，仅使用 pinocchio rnea 结果
    if (!use_simple_dynamics) {
        // 如果提供了动力学最小参数集矩阵，则进行替换
        const auto& dyn_base_params_primary = robot_model.params_ptr->dyn_base_params_primary;
        const auto& dyn_base_params_secondary = robot_model.params_ptr->dyn_base_params_secondary;
        if (dyn_base_params_primary.has_value())
        {
            // 计算重力方向在arm base坐标系下的表示
            int first_joint_id = robot_model.params_ptr->joints_primary[0];
            auto first_joint_name = robot_model.params_ptr->joint_command_sequence_by_name[first_joint_id];
            int arm_base_id = robot_model.model_ptr->getFrameId(first_joint_name) - 1;
            auto pose = pinocchio::GetFkByFrameId(*robot_model.model_ptr, q_urdf, arm_base_id);
            Vector3d gravity_acc_armbase = pose.rotation().transpose() * Vector3d{0.0, 0.0, -9.81};
            Eigen::VectorXd arm_tau = InvDynamikBySymbol(robot_model, q, dq, ddq, gravity_acc_armbase, MotionGroup::Primary);
            utils::MapJointToWholeBody(robot_model, arm_tau, tau, MotionGroup::Primary);
        }
        if (dyn_base_params_secondary.has_value())
        {
            // 计算重力方向在arm base坐标系下的表示
            int first_joint_id = robot_model.params_ptr->joints_secondary[0];
            auto first_joint_name = robot_model.params_ptr->joint_command_sequence_by_name[first_joint_id];
            int arm_base_id = robot_model.model_ptr->getFrameId(first_joint_name) - 1;
            auto pose = pinocchio::GetFkByFrameId(*robot_model.model_ptr, q_urdf, arm_base_id);
            Vector3d gravity_acc_armbase = pose.rotation().transpose() * Vector3d{0.0, 0.0, -9.81};
            Eigen::VectorXd arm_tau = InvDynamikBySymbol(robot_model, q, dq, ddq, gravity_acc_armbase, MotionGroup::Secondary);
            utils::MapJointToWholeBody(robot_model, arm_tau, tau, MotionGroup::Secondary);
        }
    }
    return tau;
}

Eigen::VectorXd ForwardDynamics(const pinocchio::Model& model, const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
                                const Eigen::VectorXd& torque_cmd, const std::vector<std::string>& joint_command_sequence_by_name)
{
    // 将输入数据映射到URDF顺序
    Eigen::VectorXd q_urdf = MapToUrdfOrder(model, q, joint_command_sequence_by_name);
    Eigen::VectorXd dq_urdf = MapToUrdfOrder(model, dq, joint_command_sequence_by_name);
    Eigen::VectorXd torque_urdf = MapToUrdfOrder(model, torque_cmd, joint_command_sequence_by_name);

    // 计算正动力学
    pinocchio::Data data(model);
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