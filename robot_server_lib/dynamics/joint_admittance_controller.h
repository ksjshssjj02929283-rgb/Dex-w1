#pragma once

#include "data_structure.h"

namespace robot_server_lib {
namespace dynamics {

// 关节导纳控制器: 将外部力矩转化为位置/速度偏移
// 模型为二阶虚拟系统: ddq_offset + D·dq_offset + K·q_offset = torque_ext (M=1)
// 其中 K = (2π·fn)², D = 2·ζ·(2π·fn), 由 natural_frequency_hz 与 damping_ratio 推导
// 输出的 q_offset 叠加到轨迹指令上, 使机器人在受外力时产生柔顺的位置偏移
class JointAdmittanceController {
public:
    JointAdmittanceController(double dt = torque_ctrl_dt);
    explicit JointAdmittanceController(const JointAdmittanceConfig& config, double dt = torque_ctrl_dt);

    void Configure(const JointAdmittanceConfig& config);
    void Reset();

    // 单步更新: 输入外部力矩, 输出位置/速度偏移
    bool UpdateAdmittanceOffset(const VectorDof& torque_ext, VectorDof& q_offset_out, VectorDof& dq_offset_out);

    const VectorDof& q_offset() const { return q_offset_; }
    const VectorDof& dq_offset() const { return dq_offset_; }
    const VectorDof& ddq_offset() const { return ddq_offset_; }
    const VectorDof& stiffness() const { return stiffness_; }
    const VectorDof& damping() const { return damping_; }

private:
    void RecomputeDerivedParams();  // 由 fn/ζ 推导刚度 K 与阻尼 D
    void ClampState();              // 对 ddq/dq/q 偏移做限幅, 并在位置饱和时钳制外推方向

    JointAdmittanceConfig config_{};
    double dt_{torque_ctrl_dt};  // 控制周期, 构造时传入
    VectorDof natural_frequency_hz_{VectorDof::Constant(1.5)};
    VectorDof damping_ratio_{VectorDof::Constant(1.2)};
    VectorDof offset_limit_{VectorDof::Constant(0.05)};
    VectorDof velocity_limit_{VectorDof::Constant(0.4)};
    VectorDof acceleration_limit_{VectorDof::Constant(5.0)};
    VectorDof stiffness_{VectorDof::Zero()};   // 推导: K = ω_n²
    VectorDof damping_{VectorDof::Zero()};     // 推导: D = 2·ζ·ωn
    VectorDof q_offset_{VectorDof::Zero()};    // 位置偏移状态
    VectorDof dq_offset_{VectorDof::Zero()};   // 速度偏移状态
    VectorDof ddq_offset_{VectorDof::Zero()};  // 加速度偏移状态
};

}  // namespace dynamics
}  // namespace robot_server_lib
