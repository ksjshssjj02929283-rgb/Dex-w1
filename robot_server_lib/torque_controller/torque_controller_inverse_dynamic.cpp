#include "torque_controller_inverse_dynamic.h"

#include <cmath>

namespace robot_server_lib {

namespace {
constexpr double kIntegralTorqueExtThresholdNm = 0.2;
constexpr double kIntegralQOffsetThresholdRad = 0.003;
constexpr double kIntegralDqOffsetThresholdRadPerSec = 0.03;
constexpr double kIntegralCommandVelocityThresholdRadPerSec = 0.03;
constexpr double kIntegralFeedbackVelocityThresholdRadPerSec = 0.05;
}  // namespace

TorqueControllerInvDynamic::TorqueControllerInvDynamic(RobotModel robot_model) : TorqueControllerBase(robot_model) {
    joint_impedance_controller_ = std::make_shared<dynamics::JointImpedanceController>(robot_model_);
    joint_admittance_controller_ =
        std::make_shared<dynamics::JointAdmittanceController>(robot_model_.robot_property_default.joint_admittance);
}

bool TorqueControllerInvDynamic::ComputeCommand(const JointRawMeasurement& raw_measurement,
                                                const PlannerManagerCommand& input_cmd,
                                                TorqueCtrlCommand& output_cmd) {
    output_cmd.torque_ext.setZero();
    output_cmd.joint_admittance_q_offset.setZero();
    output_cmd.joint_admittance_dq_offset.setZero();
    output_cmd.joint_position_ref_admittance.setZero();
    output_cmd.joint_impedance_error_after_admittance.setZero();
    VectorDof feedback_q = raw_measurement.position;
    VectorDof feedback_dq = raw_measurement.velocity;
    VectorDof cmd_q = input_cmd.cmd.position;
    VectorDof cmd_dq = input_cmd.cmd.velocity;
    VectorDof cmd_ddq = input_cmd.cmd.acceleration;
    VectorDof torque_cart_impedance = input_cmd.cmd.torque_feedforward;
    VectorDof torque_joint_impedance = VectorDof::Zero();
    VectorDof K_p = input_cmd.cmd.joint_stiffness;
    VectorDof K_v = input_cmd.cmd.joint_damping;
    VectorDof K_i_ratio = input_cmd.cmd.joint_integral_ratio;
    switch (state_) {
        case TorqueControllerState::Idle:
            torque_joint_impedance = K_p.asDiagonal() * (idle_q_ - feedback_q) + K_v.asDiagonal() * (-feedback_dq);
            output_cmd.torque_feedforward =
                dynamics::InverseDynamics(this->robot_model_, feedback_q, feedback_dq, VectorDof::Zero());
            output_cmd.torque = output_cmd.torque_feedforward + torque_joint_impedance;
            output_cmd.q_desired = idle_q_;
            if (robot_model_.robot_property_default.allow_force_sensor) {
                output_cmd.torque_ext =
                    joint_impedance_controller_->ComputeExternalTorque(feedback_q, raw_measurement.sensor_wrench);
            }
            if (joint_admittance_controller_) {
                joint_admittance_controller_->DecayToZero(torque_ctrl_dt);
                output_cmd.joint_admittance_q_offset = joint_admittance_controller_->q_offset();
                output_cmd.joint_admittance_dq_offset = joint_admittance_controller_->dq_offset();
            }
            output_cmd.joint_position_ref_admittance =
                output_cmd.q_desired + output_cmd.joint_admittance_q_offset;
            output_cmd.joint_impedance_error_after_admittance =
                output_cmd.joint_position_ref_admittance - feedback_q;
            break;
        case TorqueControllerState::Decelerating:
            torque_joint_impedance = K_v.asDiagonal() * (-feedback_dq);
            output_cmd.torque_feedforward =
                dynamics::InverseDynamics(this->robot_model_, feedback_q, feedback_dq, VectorDof::Zero());
            output_cmd.torque = output_cmd.torque_feedforward + torque_joint_impedance;
            output_cmd.q_desired = feedback_q;
            if (robot_model_.robot_property_default.allow_force_sensor) {
                output_cmd.torque_ext =
                    joint_impedance_controller_->ComputeExternalTorque(feedback_q, raw_measurement.sensor_wrench);
            }
            if (joint_admittance_controller_) {
                joint_admittance_controller_->DecayToZero(torque_ctrl_dt);
                output_cmd.joint_admittance_q_offset = joint_admittance_controller_->q_offset();
                output_cmd.joint_admittance_dq_offset = joint_admittance_controller_->dq_offset();
            }
            output_cmd.joint_position_ref_admittance =
                output_cmd.q_desired + output_cmd.joint_admittance_q_offset;
            output_cmd.joint_impedance_error_after_admittance =
                output_cmd.joint_position_ref_admittance - feedback_q;
            break;
        case TorqueControllerState::Operating: {
            const bool use_force_sensor = robot_model_.robot_property_default.allow_force_sensor;
            const VectorDof torque_ext_raw =
                use_force_sensor
                    ? joint_impedance_controller_->ComputeExternalTorque(feedback_q, raw_measurement.sensor_wrench)
                    : VectorDof::Zero();
            output_cmd.torque_ext = torque_ext_raw;

            VectorDof q_offset = VectorDof::Zero();
            VectorDof dq_offset = VectorDof::Zero();
            if (joint_admittance_controller_) {
                if (use_force_sensor) {
                    joint_admittance_controller_->Update(torque_ext_raw, torque_ctrl_dt, q_offset, dq_offset);
                } else {
                    joint_admittance_controller_->DecayToZero(torque_ctrl_dt);
                }
            }

            const VectorDof cmd_q_admittance = cmd_q + q_offset;
            const VectorDof cmd_dq_admittance = cmd_dq + dq_offset;
            VectorDof integral_enable = VectorDof::Zero();
            for (int i = 0; i < ndof_robot; ++i) {
                const bool external_force_settled = std::abs(torque_ext_raw[i]) < kIntegralTorqueExtThresholdNm;
                const bool admittance_offset_settled =
                    std::abs(q_offset[i]) < kIntegralQOffsetThresholdRad &&
                    std::abs(dq_offset[i]) < kIntegralDqOffsetThresholdRadPerSec;
                const bool command_settled =
                    std::abs(cmd_dq_admittance[i]) < kIntegralCommandVelocityThresholdRadPerSec;
                const bool feedback_settled =
                    std::abs(feedback_dq[i]) < kIntegralFeedbackVelocityThresholdRadPerSec;
                integral_enable[i] =
                    external_force_settled && admittance_offset_settled && command_settled && feedback_settled ? 1.0
                                                                                                                : 0.0;
            }

            joint_impedance_controller_->UpdateTorqueCommandWithoutExternalTorque(
                feedback_q, feedback_dq, cmd_q_admittance, cmd_dq_admittance, K_p, K_v, K_i_ratio,
                torque_joint_impedance, torque_ctrl_dt, &integral_enable);

            if (use_force_sensor) {
                if (robot_model_.robot_property_default.joint_admittance.enabled) {
                    torque_joint_impedance +=
                        robot_model_.robot_property_default.joint_admittance.torque_ext_direct_gain * torque_ext_raw;
                } else {
                    torque_joint_impedance += torque_ext_raw;
                }
            }

            output_cmd.torque_feedforward =
                dynamics::InverseDynamics(robot_model_, feedback_q, feedback_dq, VectorDof::Zero(),
                                          false);  // 补偿重力, 科氏力, 摩擦力
            output_cmd.torque_impedance = torque_cart_impedance + output_cmd.torque_feedforward;
            output_cmd.torque = torque_joint_impedance + output_cmd.torque_feedforward;
            output_cmd.q_desired = cmd_q;
            output_cmd.joint_admittance_q_offset = q_offset;
            output_cmd.joint_admittance_dq_offset = dq_offset;
            output_cmd.joint_position_ref_admittance = cmd_q_admittance;
            output_cmd.joint_impedance_error_after_admittance = cmd_q_admittance - feedback_q;
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
    joint_impedance_controller_->Reset();
    if (joint_admittance_controller_ &&
        (state == TorqueControllerState::Init || state == TorqueControllerState::Operating)) {
        joint_admittance_controller_->Reset();
    }
    if (state_ == TorqueControllerState::Idle) {
        idle_q_ = raw_measurement.position;
        LOG_INFO << "TorqueControllerInvDynamic set idle_q_: " << idle_q_.transpose();
    }
    return true;
}

}  // namespace robot_server_lib
