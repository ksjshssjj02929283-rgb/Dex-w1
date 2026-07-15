#include "inverse_dynamics.h"

#include "tool_compensation.h"
#include "utils.h"

namespace robot_server_lib
{
namespace dynamics
{

Eigen::VectorXd InverseDynamics(const RobotModel& robot_model, const VectorDof& q, const VectorDof& dq,
                                const VectorDof& ddq)
{
    constexpr double kLogThresholdUs = 500.0;   // 总耗时超过此阈值才打印分解
    constexpr int kLogThrottleCount = 500;      // 每 500 次 (约 0.5s @1kHz) 最多打印一次
    static int64_t call_counter = 0;

    auto t_start = std::chrono::steady_clock::now();
    auto t_seg = t_start;

    const auto& model = *robot_model.model_ptr;
    // URDF 顺序与控制顺序一致，无需映射
    pinocchio::Data data(model);
    double us_data_construct = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - t_seg).count();
    t_seg = std::chrono::steady_clock::now();

    VectorDof tau = pinocchio::rnea(model, data, q, dq, ddq).template cast<double>();
    double us_rnea = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - t_seg).count();

    const auto& tool_cfg_primary = robot_model.params_ptr->tool_dynamic_primary;
    const auto& tool_cfg_secondary = robot_model.params_ptr->tool_dynamic_secondary;

    // 如果提供了动力学最小参数集矩阵，则进行替换
    const auto& dyn_base_params_primary = robot_model.params_ptr->dyn_base_params_primary;
    const auto& dyn_base_params_secondary = robot_model.params_ptr->dyn_base_params_secondary;
    double us_sym_pri = 0.0, us_sym_sec = 0.0;
    if (dyn_base_params_primary.has_value())
    {
        // 计算重力方向在arm base坐标系下的表示
        auto first_joint_urdf_name = robot_model.params_ptr->ros2_to_urdf_map.at(robot_model.params_ptr->joints_primary_names[0]);
        if (robot_model.model_ptr->existFrame(first_joint_urdf_name))
        {
            auto pose = pinocchio::GetFkByFrameId(*robot_model.model_ptr, q, robot_model.model_ptr->getFrameId(first_joint_urdf_name) - 1);
            Vector3d gravity_acc_armbase = pose.rotation().transpose() * Vector3d{0.0, 0.0, -9.8};

            // 调试: 打印重力向量信息
            static int64_t dbg_counter_pri = 0;
            if (++dbg_counter_pri % 1000 == 0) {
                int frame_id = robot_model.model_ptr->getFrameId(first_joint_urdf_name) - 1;
            }

            t_seg = std::chrono::steady_clock::now();
            Eigen::VectorXd arm_tau = InvDynamikBySymbol(robot_model, q, dq, ddq, gravity_acc_armbase, MotionGroup::Primary);
            us_sym_pri = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - t_seg).count();
            utils::MapJointToWholeBody(robot_model, arm_tau, tau, MotionGroup::Primary);

            // 末端工具重力补偿: 单独算 wrench, J^T 投影到 7 关节, 累加到前馈 (与手臂符号动力学解耦)
            if (tool_cfg_primary.gravity_enabled && robot_model.primary_arm_model_ptr)
            {
                // q_arm 在 InvDynamikBySymbol 内部已提取过一次, 这里为工具补偿的 FK/Jacobian 复用
                Eigen::VectorXd feedback_q_primary;
                utils::GetJointFromWholeBody(robot_model, q, feedback_q_primary, MotionGroup::Primary);
                Eigen::VectorXd tau_tool = tool_compensation::ComputeToolJointGravityTorque(
                    *robot_model.primary_arm_model_ptr, feedback_q_primary, tool_cfg_primary, gravity_acc_armbase);
                // MapJointToWholeBody 是覆盖语义 (输出 JointPositionDof), 用临时变量接收再累加
                VectorDof tau_tool_to_whole_body = VectorDof::Zero();
                utils::MapJointToWholeBody(robot_model, tau_tool, tau_tool_to_whole_body, MotionGroup::Primary);
                tau += tau_tool_to_whole_body;
            }
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
            Vector3d gravity_acc_armbase = pose.rotation().transpose() * Vector3d{0.0, 0.0, -9.8};

            // 调试: 打印重力向量信息
            static int64_t dbg_counter_sec = 0;
            if (++dbg_counter_sec % 1000 == 0) {
                int frame_id = robot_model.model_ptr->getFrameId(first_joint_urdf_name) - 1;
            }

            t_seg = std::chrono::steady_clock::now();
            Eigen::VectorXd arm_tau = InvDynamikBySymbol(robot_model, q, dq, ddq, gravity_acc_armbase, MotionGroup::Secondary);
            us_sym_sec = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - t_seg).count();
            utils::MapJointToWholeBody(robot_model, arm_tau, tau, MotionGroup::Secondary);

            // 末端工具重力补偿: 单独算 wrench, J^T 投影到 7 关节, 累加到前馈 (与手臂符号动力学解耦)
            if (tool_cfg_secondary.gravity_enabled && robot_model.secondary_arm_model_ptr)
            {
                // q_arm 在 InvDynamikBySymbol 内部已提取过一次, 这里为工具补偿的 FK/Jacobian 复用
                Eigen::VectorXd feedback_q_secondary;
                utils::GetJointFromWholeBody(robot_model, q, feedback_q_secondary, MotionGroup::Secondary);
                Eigen::VectorXd tau_tool = tool_compensation::ComputeToolJointGravityTorque(
                    *robot_model.secondary_arm_model_ptr, feedback_q_secondary, tool_cfg_secondary, gravity_acc_armbase);
                // MapJointToWholeBody 是覆盖语义 (输出 JointPositionDof), 用临时变量接收再累加
                VectorDof tau_tool_to_whole_body = VectorDof::Zero();
                utils::MapJointToWholeBody(robot_model, tau_tool, tau_tool_to_whole_body, MotionGroup::Secondary);
                tau += tau_tool_to_whole_body;
            }
        }
        else
        {
            std::cerr << "[ERROR] Frame " << first_joint_urdf_name << " does not exist in the robot model." << std::endl;
            LOG_CRIT << "Frame " << first_joint_urdf_name << " does not exist in the robot model.";
            exit(EXIT_FAILURE);
        }
    }

    // 超阈值时打印分段耗时 (节流: 每 kLogThrottleCount 次最多打印一次)
    double total_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - t_start).count();
    call_counter++;
    if (total_us > kLogThresholdUs && call_counter % kLogThrottleCount == 0) {
        LOG_CRIT << "[InverseDynamics] Total=" << total_us << "us"
                 << " | DataConstruct=" << us_data_construct
                 << " RNEA=" << us_rnea
                 << " Sym_pri=" << us_sym_pri
                 << " Sym_sec=" << us_sym_sec;
    }

    return tau;
}
}  // namespace dynamics
}  // namespace robot_server_lib