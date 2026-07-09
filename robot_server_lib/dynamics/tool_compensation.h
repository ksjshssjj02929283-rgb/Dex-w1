#pragma once

#include <string>

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
//   ᴬF_g = m * gravity_armbase              (arm base 系下的重力向量)
//   ᴬR_E  = FK(arm_model, q).rotation()      (arm base 系 <- EE 系)
//   ᴬτ_g_E = (ᴬR_E * com_m) × ᴬF_g            (力矩取在 EE 原点, 表达在 arm base 系)
//   ᴬwrench = [ᴬF_g ; ᴬτ_g_E]
//   tau_tool = J(q, LOCAL_WORLD_ALIGNED)^T * ᴬwrench
// gravity_armbase: arm base 系下的重力向量, 与 inverse_dynamics.cpp 的求法一致
//                  (全身 FK 取 first arm joint 旋转的转置 * (0,0,-g))
// 未启用或本周期未成功算到时返回零向量
Eigen::VectorXd ComputeToolJointGravityTorque(const pinocchio::Model& arm_model,
                                              const Eigen::VectorXd& feedback_q_arm,
                                              const ToolGravityCompensationConfig& config,
                                              const Vector3d& gravity_armbase,
                                              bool* applied = nullptr,
                                              std::string* failure_reason = nullptr);

}  // namespace tool_compensation
}  // namespace dynamics
}  // namespace robot_server_lib
