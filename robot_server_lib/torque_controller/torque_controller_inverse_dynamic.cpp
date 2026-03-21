#include "torque_controller_inverse_dynamic.h"

namespace robot_server_lib {

TorqueControllerInvDynamic::TorqueControllerInvDynamic(RobotModel robot_model) : TorqueControllerBase(robot_model) {}

bool TorqueControllerInvDynamic::ComputeCommand(const JointRawMeasurement &raw_measurement,
                                                const PlannerManagerCommand &input_cmd,
                                                TorqueCtrlCommand &output_cmd) {
    VectorDof feedback_q = raw_measurement.position;
    VectorDof feedback_dq = raw_measurement.velocity;
    VectorDof cmd_q = input_cmd.cmd.position;
    VectorDof cmd_dq = input_cmd.cmd.velocity;
    VectorDof cmd_ddq = input_cmd.cmd.acceleration;
    VectorDof torque_cart_impedance = input_cmd.cmd.torque_feedforward;
    VectorDof torque_joint_impedance = VectorDof::Zero();
    VectorDof q_error = VectorDof::Zero();
    VectorDof dq_error = VectorDof::Zero();
    VectorDof K_p = input_cmd.cmd.joint_stiffness;
    VectorDof K_v = input_cmd.cmd.joint_damping;
    double K_i_ratio = input_cmd.cmd.joint_integral_ratio;
    bool use_simple_dynamics = input_cmd.use_simple_dynamics;
    switch (state_) {
        case TorqueControllerState::Idle:
            torque_joint_impedance = K_p.asDiagonal() * (idle_q_ - feedback_q) + K_v.asDiagonal() * (-feedback_dq);
            output_cmd.torque_feedforward = dynamics::InverseDynamics(this->robot_model_, feedback_q, feedback_dq,
                                                                      VectorDof::Zero(), use_simple_dynamics);
            output_cmd.torque = output_cmd.torque_feedforward + torque_joint_impedance;
            output_cmd.q_desired = idle_q_;
            break;
        case TorqueControllerState::Decelerating:
            torque_joint_impedance = K_v.asDiagonal() * (-feedback_dq);
            output_cmd.torque_feedforward = dynamics::InverseDynamics(this->robot_model_, feedback_q, feedback_dq,
                                                                      VectorDof::Zero(), use_simple_dynamics);
            output_cmd.torque = output_cmd.torque_feedforward + torque_joint_impedance;
            output_cmd.q_desired = feedback_q;
            break;
        case TorqueControllerState::Operating: {
            q_error = cmd_q - feedback_q;
            dq_error = cmd_dq - feedback_dq;
            // 积分误差累计并限幅
            for (int i = 0; i < ndof_robot; i++) {
                if (std::abs(q_error[i]) < 0.005 / 180 * M_PI) {
                    q_error[i] = 0.0;
                }
            }
            integral_error_ = (integral_error_ + q_error).cwiseMin(error_integral_max_).cwiseMax(-error_integral_max_);
            // 根据 K_p 百分比计算实际 K_i
            VectorDof torque_integral = (K_p * K_i_ratio).asDiagonal() * integral_error_;

            torque_joint_impedance = K_p.asDiagonal() * q_error + K_v.asDiagonal() * dq_error + torque_integral;
            output_cmd.torque_feedforward =
                dynamics::InverseDynamics(robot_model_, feedback_q, feedback_dq, VectorDof::Zero(),
                                          false);  // 补偿重力, 科氏力, 摩擦力
            output_cmd.torque_impedance = torque_cart_impedance + output_cmd.torque_feedforward;
            output_cmd.torque = torque_joint_impedance + output_cmd.torque_feedforward;
            output_cmd.q_desired = cmd_q;
        }
        default:
            break;
    }
    return true;
}

bool TorqueControllerInvDynamic::ResetForNewEpisode(const JointRawMeasurement &raw_measurement,
                                                    const TorqueControllerState &state) {
    state_ = state;
    integral_error_.setZero();  // 重置积分误差
    if (state_ == TorqueControllerState::Idle) {
        idle_q_ = raw_measurement.position;
        LOG_INFO << "TorqueControllerInvDynamic set idle_q_: " << idle_q_.transpose();
    }
    return true;
}

}  // namespace robot_server_lib
