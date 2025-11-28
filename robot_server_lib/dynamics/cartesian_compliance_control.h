#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include "NanoLog.hpp"
#include "dynamics_utils.h"

namespace robot_server_lib
{
namespace dynamics
{

// 6方向笛卡尔柔顺控制 [x, y, z]
// 无模型导纳：M * x_dd + B * x_d + K * x = F
class CartesianComplianceControl
{
   public:
    CartesianComplianceControl() = default;

    // 初始化参数（质量、阻尼、刚度，6轴可分别指定）
    CartesianComplianceControl(const Vector3d& M, const Vector3d& B, const Vector3d& K, double control_dt = 0.001)
        : M_(M), B_(B), K_(K), control_dt_(control_dt)
    {
    }

    // 每步更新：传入在当前时刻测得的力（N）和时间步长（s），返回位移偏置（m）
    Vector3d UpdatePosOffset(const Vector3d& force)
    {
        if (control_dt_ <= 0.0)
            return pos_offset_;

        // 半隐式欧拉：先更新加速度 -> 更新速度 -> 更新位置
        for (int i = 0; i < force.size(); ++i)
        {
            double m = std::max(1e-9, M_[i]);
            double acc = (force[i] - B_[i] * vel_[i] - K_[i] * pos_offset_[i]) / m;
            // 限制加速度（防止数值爆炸）
            if (std::isfinite(max_acc_[i]) && max_acc_[i] > 0.0)
            {
                acc = std::clamp(acc, -max_acc_[i], max_acc_[i]);
            }
            // 速度和位置积分
            vel_[i] += acc * control_dt_;
            // 限制速度
            if (std::isfinite(max_vel_[i]) && max_vel_[i] > 0.0)
            {
                vel_[i] = std::clamp(vel_[i], -max_vel_[i], max_vel_[i]);
            }
            pos_offset_[i] += vel_[i] * control_dt_;
            // 限制位置偏置
            if (std::isfinite(max_pos_[i]) && max_pos_[i] > 0.0)
            {
                pos_offset_[i] = std::clamp(pos_offset_[i], -max_pos_[i], max_pos_[i]);
            }
        }
        return pos_offset_;
    }

    void SetMass(const Vector3d& M) 
    {
        M_ = M;
        for (auto& b : B_)
        {
            double min_damping = 2.0 * std::sqrt(M_[&b - &B_[0]] * K_[&b - &K_[0]]);
            if (b < min_damping)
            {
                b = min_damping;
            }
        }
        LOG_INFO << "CartesianComplianceControl: Set mass to [" << M_.transpose() << "], updated damping to [" << B_.transpose() << "], stiffness: [" << K_.transpose() << "]";
    }
    void SetDamping(const Vector3d& B) 
    { 
        B_ = B; 
        for (auto& b : B_)
        {
            double min_damping = 2.0 * std::sqrt(M_[&b - &B_[0]] * K_[&b - &K_[0]]);
            if (b < min_damping)
            {
                b = min_damping;
                LOG_INFO << "CartesianComplianceControl: Updated damping to min critical damping: " << b;
            }
        }
        LOG_INFO << "CartesianComplianceControl: Set damping to [" << B_.transpose() << "], mass: [" << M_.transpose() << "], stiffness: [" << K_.transpose() << "]";
    }
    void SetStiffness(const Vector3d& K) 
    { 
        K_ = K; 
        for (auto& b : B_)
        {
            double min_damping = 2.0 * std::sqrt(M_[&b - &B_[0]] * K_[&b - &K_[0]]);
            if (b < min_damping)
                b = min_damping;
        }
        LOG_INFO << "CartesianComplianceControl: Set stiffness to [" << K_.transpose() << "], updated damping to [" << B_.transpose() << "], mass: [" << M_.transpose() << "]";
    }

    // 设置输出限制（非正数表示不限制）
    void setMaxPosition(const Vector3d& max_pos) { max_pos_ = max_pos; }
    void setMaxVelocity(const Vector3d& max_vel) { max_vel_ = max_vel; }
    void setMaxAcceleration(const Vector3d& max_acc) { max_acc_ = max_acc; }

    // 读取当前位移偏置和速度
    Vector3d getPositionOffset() const { return pos_offset_; }
    Vector3d getVelocity() const { return vel_; }

    // 重置状态（不改变模型参数）
    void reset(const Vector3d& pos = {0.0, 0.0, 0.0}, const Vector3d& vel = {0.0, 0.0, 0.0})
    {
        pos_offset_ = pos;
        vel_ = vel;
    }

   private:
    double control_dt_ = 0.001;              // 控制周期 (s)
    Vector3d M_ = {2, 2, 2};        // 质量 (kg)
    Vector3d B_ = {100.0, 100.0, 60.0};     // 阻尼 (N·s/m)
    Vector3d K_ = {500.0, 500.0, 200.0};  // 刚度 (N/m)

    // 限制（默认不限制：设置为非正数表示关闭）
    Vector3d max_pos_ = {0.1, 0.1, 0.1};
    Vector3d max_vel_ = {1.0, 1.0, 1.0};
    Vector3d max_acc_ = {5.0, 5.0, 5.0};

    Vector3d pos_offset_ = {0.0, 0.0, 0.0};  // 位移偏置 (m)
    Vector3d vel_ = {0.0, 0.0, 0.0};         // 速度 (m/s)
};

}  // namespace dynamics
}  // namespace robot_server_lib