#pragma once

#include "pinocchio_api.h"
#include "typedef.h"

namespace robot_server_lib {
namespace dynamics {
namespace tool_compensation {

// 工具重力补偿配置
// 坐标系约定: com_m 在 EE/法兰系下表达
struct ToolGravityCompensationConfig {
    bool gravity_enabled = false;
    double mass_kg = 0.0;
    Vector3d com_m = Vector3d::Zero();  // COM 相对 EE 原点偏移, 单位 m
};

// 计算工具自重经雅可比转置投影到关节的补偿力矩
Eigen::VectorXd ComputeToolJointGravityTorque(const pinocchio::Model& arm_model,
                                              const Eigen::VectorXd& feedback_q_arm,
                                              const ToolGravityCompensationConfig& config,
                                              const Vector3d& gravity_armbase);
Eigen::VectorXd ComputeToolJointGravityTorque(const pinocchio::Model& arm_model,
                                              const Eigen::VectorXd& feedback_q_arm,
                                              const ToolGravityCompensationConfig& config,
                                              const Vector3d& gravity_armbase,
                                              pinocchio::Data& arm_data);

}  // namespace tool_compensation
}  // namespace dynamics
}  // namespace robot_server_lib
