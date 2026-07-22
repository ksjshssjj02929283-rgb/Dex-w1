#pragma once
#include "inverse_dynamics.h"
#include "joint_admittance_controller.h"
#include "joint_pd_controller.h"
#include "torque_controller_base.h"

namespace robot_server_lib {

// 关节逆动力学力矩控制器: 级联导纳-阻抗控制
// 控制结构: 导纳控制器(位置偏移) -> 叠加到轨迹指令 -> 力矩控制器(PID) -> 力矩 + 逆动力学前馈(重力/科氏/摩擦)
class TorqueControllerInvDynamic : public TorqueControllerBase {
public:
    TorqueControllerInvDynamic(RobotModel robot_model);
    bool ComputeCommand(const JointRawMeasurement& raw_measurement,
                        const PlannerManagerCommand& input_cmd,
                        TorqueCtrlCommand& output_cmd) override;
    bool ResetForNewEpisode(const JointRawMeasurement& raw_measurement, const TorqueControllerState& state) override;

protected:
    VectorDof idle_q_;
    std::shared_ptr<dynamics::JointPdController> joint_pd_controller_{nullptr};  // 力矩控制(PID)
    std::shared_ptr<dynamics::JointAdmittanceController> joint_admittance_controller_{
        nullptr};  // 导纳控制(外力->位置偏移)
};

}  // namespace robot_server_lib
