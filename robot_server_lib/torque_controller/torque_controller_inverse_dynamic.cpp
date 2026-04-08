#include "torque_controller_inverse_dynamic.h"

namespace robot_server_lib {

TorqueControllerInvDynamic::TorqueControllerInvDynamic(RobotModel robot_model) : TorqueControllerBase(robot_model) {
    joint_impedance_controller_ = std::make_shared<dynamics::JointImpedanceController>(robot_model_);
}

bool TorqueControllerInvDynamic::ComputeCommand(const JointRawMeasurement &raw_measurement,
                                                const PlannerManagerCommand &input_cmd,
                                                TorqueCtrlCommand &output_cmd) {
    output_cmd = TorqueCtrlCommand{};
    VectorDof feedback_q = raw_measurement.position;
    VectorDof feedback_dq = raw_measurement.velocity;
    VectorDof cmd_q = input_cmd.cmd.position;
    VectorDof cmd_dq = input_cmd.cmd.velocity;
    VectorDof torque_cart_impedance = input_cmd.cmd.torque_feedforward;
    VectorDof torque_joint_impedance = VectorDof::Zero();
    VectorDof K_p = input_cmd.cmd.joint_stiffness;
    VectorDof K_v = input_cmd.cmd.joint_damping;
    double K_i_ratio = input_cmd.cmd.joint_integral_ratio;
    bool use_simple_dynamics = input_cmd.use_simple_dynamics;
    output_cmd.q_desired_base = cmd_q;
    output_cmd.q_desired_compliant = cmd_q;
    output_cmd.joint_compliance_offset.setZero();
    output_cmd.force_trigger_state = {0, 0};
    switch (state_) {
        case TorqueControllerState::Idle:
            torque_joint_impedance =
                joint_impedance_controller_->ComputeHoldTorque(feedback_q, feedback_dq, idle_q_, K_p, K_v);
            output_cmd.torque_feedforward = dynamics::InverseDynamics(this->robot_model_, feedback_q, feedback_dq,
                                                                      VectorDof::Zero(), use_simple_dynamics);
            output_cmd.torque = output_cmd.torque_feedforward + torque_joint_impedance;
            output_cmd.q_desired = idle_q_;
            output_cmd.q_desired_base = idle_q_;
            output_cmd.q_desired_compliant = idle_q_;
            break;
        case TorqueControllerState::Decelerating:
            torque_joint_impedance = joint_impedance_controller_->ComputeDampingTorque(feedback_dq, K_v);
            output_cmd.torque_feedforward = dynamics::InverseDynamics(this->robot_model_, feedback_q, feedback_dq,
                                                                      VectorDof::Zero(), use_simple_dynamics);
            output_cmd.torque = output_cmd.torque_feedforward + torque_joint_impedance;
            output_cmd.q_desired = feedback_q;
            output_cmd.q_desired_base = feedback_q;
            output_cmd.q_desired_compliant = feedback_q;
            break;
        case TorqueControllerState::Operating: {
            auto impedance_output =
                joint_impedance_controller_->UpdateTorqueCommand(feedback_q, feedback_dq, cmd_q, cmd_dq,
                                                                 raw_measurement.sensor_wrench, K_p, K_v, K_i_ratio);
            torque_joint_impedance = impedance_output.torque_joint;
            output_cmd.torque_feedforward =
                dynamics::InverseDynamics(robot_model_, feedback_q, feedback_dq, VectorDof::Zero(),
                                          false);  // 补偿重力, 科氏力, 摩擦力
            output_cmd.torque_impedance = torque_cart_impedance + output_cmd.torque_feedforward;
            output_cmd.torque = torque_joint_impedance + output_cmd.torque_feedforward;
            output_cmd.q_desired = impedance_output.q_desired_compliant;
            output_cmd.q_desired_base = impedance_output.q_desired_base;
            output_cmd.q_desired_compliant = impedance_output.q_desired_compliant;
            output_cmd.joint_compliance_offset = impedance_output.joint_compliance_offset;
            output_cmd.force_trigger_state = impedance_output.force_trigger_state;
            break;
        }
        default:
            break;
    }
    return true;
}

bool TorqueControllerInvDynamic::ResetForNewEpisode(const JointRawMeasurement &raw_measurement,
                                                    const TorqueControllerState &state) {
    state_ = state;
    joint_impedance_controller_->Reset(raw_measurement.position);
    if (state_ == TorqueControllerState::Idle) {
        idle_q_ = raw_measurement.position;
        LOG_INFO << "TorqueControllerInvDynamic set idle_q_: " << idle_q_.transpose();
    }
    return true;
}

}  // namespace robot_server_lib
