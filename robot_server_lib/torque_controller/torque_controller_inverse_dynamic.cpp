#include "torque_controller_inverse_dynamic.h"

namespace robot_server_lib {

TorqueControllerInvDynamic::TorqueControllerInvDynamic(RobotModel robot_model) : TorqueControllerBase(robot_model) {
    joint_pd_controller_ = std::make_shared<dynamics::JointPdController>();
    joint_admittance_controller_ =
        std::make_shared<dynamics::JointAdmittanceController>(robot_model_.robot_property_default.joint_admittance);
}

bool TorqueControllerInvDynamic::ComputeCommand(const JointRawMeasurement& raw_measurement,
                                                const PlannerManagerCommand& input_cmd,
                                                TorqueCtrlCommand& output_cmd) {
    VectorDof feedback_q = raw_measurement.position;
    VectorDof feedback_dq = raw_measurement.velocity;
    VectorDof cmd_q = input_cmd.cmd.position;
    VectorDof cmd_dq = input_cmd.cmd.velocity;
    VectorDof torque_cart_impedance = input_cmd.cmd.torque_feedforward;
    VectorDof torque_joint_impedance = VectorDof::Zero();
    VectorDof K_p = input_cmd.cmd.joint_stiffness;
    VectorDof K_v = input_cmd.cmd.joint_damping;
    VectorDof K_i_ratio = input_cmd.cmd.joint_integral_ratio;
    VectorDof q_offset = VectorDof::Zero();
    VectorDof dq_offset = VectorDof::Zero();

    switch (state_) {
        case TorqueControllerState::Idle: {
            // 空闲态: 维持在 idle_q_, 导纳偏移自然衰减回零
            joint_admittance_controller_->UpdateAdmittanceOffset(VectorDof::Zero(), q_offset, dq_offset);
            // 将导纳偏移叠加到期望位置, 阻抗控制: PD + 积分
            const VectorDof q_error = (idle_q_ + q_offset) - feedback_q;
            const VectorDof dq_error = dq_offset - feedback_dq;
            joint_pd_controller_->UpdateTorqueCommand(q_error, dq_error, K_p, K_v, K_i_ratio, torque_joint_impedance);
            output_cmd.torque_feedforward =
                dynamics::InverseDynamics(this->robot_model_, feedback_q, feedback_dq, VectorDof::Zero(),
                                          &output_cmd.torque_friction);
            output_cmd.torque = output_cmd.torque_feedforward + torque_joint_impedance;
            output_cmd.q_desired = idle_q_;
            break;
        }
        case TorqueControllerState::Decelerating: {
            // 减速态: 仅阻尼项, 跟随当前位置, 导纳衰减
            joint_admittance_controller_->UpdateAdmittanceOffset(VectorDof::Zero(), q_offset, dq_offset);
            // 将导纳偏移叠加到期望位置(期望=当前位置), 阻抗控制: PD + 积分
            const VectorDof q_error = (feedback_q + q_offset) - feedback_q;
            const VectorDof dq_error = dq_offset - feedback_dq;
            joint_pd_controller_->UpdateTorqueCommand(q_error, dq_error, K_p, K_v, K_i_ratio, torque_joint_impedance);
            output_cmd.torque_feedforward =
                dynamics::InverseDynamics(this->robot_model_, feedback_q, feedback_dq, VectorDof::Zero(),
                                          &output_cmd.torque_friction);
            output_cmd.torque = output_cmd.torque_feedforward + torque_joint_impedance;
            output_cmd.q_desired = feedback_q;
            break;
        }
        case TorqueControllerState::Operating: {
            // === 级联导纳-阻抗控制核心流程 ===
            // 0. 逆动力学前馈: 补偿惯性/重力/科氏力/摩擦力(含工具重力补偿)
            output_cmd.torque_feedforward =
                dynamics::InverseDynamics(robot_model_, feedback_q, feedback_dq, input_cmd.cmd.acceleration,
                                          &output_cmd.torque_friction);
            // 1. 导纳控制: 反馈力矩减去前馈得到外力估计, 经死区处理抑制模型残差噪声
            VectorDof torque_ext = output_cmd.torque_feedforward - raw_measurement.torque;
            for (int i = 0; i < ndof_robot; ++i) {
                torque_ext[i] =
                    utils::DeadZoneFilter(torque_ext[i], robot_model_.params_ptr->joint_torque_ext_deadzone[i]);
            }
            joint_admittance_controller_->UpdateAdmittanceOffset(torque_ext, q_offset, dq_offset);
            // 2. 将导纳偏移叠加到轨迹指令, 得到实际参考位置/速度
            const VectorDof cmd_q_admittance = cmd_q + q_offset;
            const VectorDof cmd_dq_admittance = cmd_dq + dq_offset;

            // 3. 阻抗控制: PD + 积分
            const VectorDof q_error = cmd_q_admittance - feedback_q;
            const VectorDof dq_error = cmd_dq_admittance - feedback_dq;
            joint_pd_controller_->UpdateTorqueCommand(q_error, dq_error, K_p, K_v, K_i_ratio, torque_joint_impedance);

            // 4. 笛卡尔阻抗控制力矩输出
            output_cmd.torque_impedance = torque_cart_impedance + output_cmd.torque_feedforward;
            // 5. 关节阻抗导纳力矩 = 阻抗力矩 + 前馈力矩
            output_cmd.torque = torque_joint_impedance + output_cmd.torque_feedforward;
            output_cmd.q_desired = cmd_q;
            break;
        }
        default:
            break;
    }
    return true;
}

bool TorqueControllerInvDynamic::ResetForNewEpisode(const JointRawMeasurement& raw_measurement,
                                                    const TorqueControllerState& state) {
    state_ = state;
    joint_pd_controller_->Reset();
    joint_admittance_controller_->Reset();
    if (state_ == TorqueControllerState::Idle) {
        idle_q_ = raw_measurement.position;
        LOG_INFO << "TorqueControllerInvDynamic set idle_q_: " << idle_q_.transpose();
    }
    return true;
}

}  // namespace robot_server_lib
