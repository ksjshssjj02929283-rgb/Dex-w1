#include "inverse_dynamics.h"

#include "tool_compensation.h"
#include "utils.h"

namespace robot_server_lib
{
namespace dynamics
{

namespace {
struct ToolCompensationCache {
    std::shared_ptr<const pinocchio::Model> arm_model;
    pinocchio::Inertia inertia = pinocchio::Inertia::Zero();
    std::optional<tool_compensation::ToolCompensation> compensation;
};

Eigen::VectorXd ComputeToolTorque(const RobotModel& robot_model,
                                  const std::shared_ptr<const pinocchio::Model>& arm_model,
                                  const tool_compensation::ToolDynamicsConfig& config,
                                  const VectorDof& q, const VectorDof& dq, const VectorDof& ddq,
                                  const Vector3d& gravity_armbase, MotionGroup group,
                                  ToolCompensationCache& cache);
}  // namespace

Eigen::VectorXd InverseDynamics(const RobotModel& robot_model, const VectorDof& q, const VectorDof& dq,
                                const VectorDof& ddq, VectorDof* friction_feedforward)
{
    if (friction_feedforward) friction_feedforward->setZero();
    auto t_start = std::chrono::steady_clock::now();
    auto t_seg = t_start;

    const auto& model = *robot_model.model_ptr;
    // thread_local 缓存 Data 避免每周期动态分配, 消除偶发 ~800us 分配器慢路径
    thread_local std::optional<pinocchio::Data> cached_data_whole;
    thread_local ToolCompensationCache tool_primary_cache, tool_secondary_cache;
    if (!cached_data_whole.has_value()) {
        cached_data_whole.emplace(model);
    }
    pinocchio::Data& data = *cached_data_whole;
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
    double us_fk_pri = 0.0, us_sym_pri = 0.0, us_tool_pri = 0.0;
    double us_fk_sec = 0.0, us_sym_sec = 0.0, us_tool_sec = 0.0;
    if (dyn_base_params_primary.has_value())
    {
        // 计算重力方向在arm base坐标系下的表示
        auto first_joint_urdf_name = robot_model.params_ptr->ros2_to_urdf_map.at(robot_model.params_ptr->joints_primary_names[0]);
        if (robot_model.model_ptr->existFrame(first_joint_urdf_name))
        {
            t_seg = std::chrono::steady_clock::now();
            auto pose = pinocchio::GetFkByFrameId(*robot_model.model_ptr, q, robot_model.model_ptr->getFrameId(first_joint_urdf_name) - 1, data);
            Vector3d gravity_acc_armbase = pose.rotation().transpose() * Vector3d{0.0, 0.0, -9.8};
            us_fk_pri = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - t_seg).count();

            t_seg = std::chrono::steady_clock::now();
            Eigen::VectorXd arm_friction;
            Eigen::VectorXd arm_tau = InvDynamikBySymbol(robot_model, q, dq, ddq, gravity_acc_armbase, MotionGroup::Primary,
                                                       friction_feedforward ? &arm_friction : nullptr);
            if (friction_feedforward) {
                utils::MapJointToWholeBody(robot_model, arm_friction, *friction_feedforward, MotionGroup::Primary);
            }
            us_sym_pri = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - t_seg).count();
            utils::MapJointToWholeBody(robot_model, arm_tau, tau, MotionGroup::Primary);

            if (tool_cfg_primary.enabled && robot_model.primary_arm_model_ptr)
            {
                t_seg = std::chrono::steady_clock::now();
                Eigen::VectorXd tau_tool = ComputeToolTorque(
                    robot_model, robot_model.primary_arm_model_ptr, tool_cfg_primary, q, dq, ddq,
                    gravity_acc_armbase, MotionGroup::Primary, tool_primary_cache);
                // MapJointToWholeBody 是覆盖语义 (输出 JointPositionDof), 用临时变量接收再累加
                VectorDof tau_tool_to_whole_body = VectorDof::Zero();
                utils::MapJointToWholeBody(robot_model, tau_tool, tau_tool_to_whole_body, MotionGroup::Primary);
                tau += tau_tool_to_whole_body;
                us_tool_pri = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now() - t_seg).count();
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
            t_seg = std::chrono::steady_clock::now();
            auto pose = pinocchio::GetFkByFrameId(*robot_model.model_ptr, q, robot_model.model_ptr->getFrameId(first_joint_urdf_name) - 1, data);
            Vector3d gravity_acc_armbase = pose.rotation().transpose() * Vector3d{0.0, 0.0, -9.8};
            us_fk_sec = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - t_seg).count();

            t_seg = std::chrono::steady_clock::now();
            Eigen::VectorXd arm_friction;
            Eigen::VectorXd arm_tau = InvDynamikBySymbol(robot_model, q, dq, ddq, gravity_acc_armbase, MotionGroup::Secondary,
                                                       friction_feedforward ? &arm_friction : nullptr);
            if (friction_feedforward) {
                utils::MapJointToWholeBody(robot_model, arm_friction, *friction_feedforward, MotionGroup::Secondary);
            }
            us_sym_sec = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - t_seg).count();
            utils::MapJointToWholeBody(robot_model, arm_tau, tau, MotionGroup::Secondary);

            if (tool_cfg_secondary.enabled && robot_model.secondary_arm_model_ptr)
            {
                t_seg = std::chrono::steady_clock::now();
                Eigen::VectorXd tau_tool = ComputeToolTorque(
                    robot_model, robot_model.secondary_arm_model_ptr, tool_cfg_secondary, q, dq, ddq,
                    gravity_acc_armbase, MotionGroup::Secondary, tool_secondary_cache);
                // MapJointToWholeBody 是覆盖语义 (输出 JointPositionDof), 用临时变量接收再累加
                VectorDof tau_tool_to_whole_body = VectorDof::Zero();
                utils::MapJointToWholeBody(robot_model, tau_tool, tau_tool_to_whole_body, MotionGroup::Secondary);
                tau += tau_tool_to_whole_body;
                us_tool_sec = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now() - t_seg).count();
            }
        }
        else
        {
            std::cerr << "[ERROR] Frame " << first_joint_urdf_name << " does not exist in the robot model." << std::endl;
            LOG_CRIT << "Frame " << first_joint_urdf_name << " does not exist in the robot model.";
            exit(EXIT_FAILURE);
        }
    }

    // 仅记录真正超过控制周期的调用, 避免实时线程正常运行时产生额外日志负担。
    double total_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - t_start).count();
    if (total_us > torque_ctrl_dt_micro) {
        LOG_CRIT_THROTTLE(1000) << "[InverseDynamics] Total=" << total_us << "us"
                                << " | DataConstruct=" << us_data_construct
                                << " RNEA=" << us_rnea
                                << " FK_pri=" << us_fk_pri
                                << " Sym_pri=" << us_sym_pri
                                << " Tool_pri=" << us_tool_pri
                                << " FK_sec=" << us_fk_sec
                                << " Sym_sec=" << us_sym_sec
                                << " Tool_sec=" << us_tool_sec;
    }

    return tau;
}

namespace {
Eigen::VectorXd ComputeToolTorque(const RobotModel& robot_model,
                                  const std::shared_ptr<const pinocchio::Model>& arm_model,
                                  const tool_compensation::ToolDynamicsConfig& config,
                                  const VectorDof& q, const VectorDof& dq, const VectorDof& ddq,
                                  const Vector3d& gravity_armbase, MotionGroup group,
                                  ToolCompensationCache& cache) {
    if (!cache.compensation || cache.arm_model != arm_model || !cache.inertia.isApprox(config.inertia)) {
        cache.compensation.emplace(*arm_model, config.inertia);
        cache.arm_model = arm_model;
        cache.inertia = config.inertia;
    }
    Eigen::VectorXd q_arm, dq_arm, ddq_arm;
    utils::GetJointFromWholeBody(robot_model, q, q_arm, group);
    utils::GetJointFromWholeBody(robot_model, dq, dq_arm, group);
    utils::GetJointFromWholeBody(robot_model, ddq, ddq_arm, group);
    return cache.compensation->ComputeTorque(q_arm, dq_arm, ddq_arm, gravity_armbase);
}
}  // namespace
}  // namespace dynamics
}  // namespace robot_server_lib
