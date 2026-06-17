#pragma once
#include "inverse_dynamics.h"
#include "joint_admittance_controller.h"
#include "joint_impedance_controller.h"
#include "torque_controller_base.h"

namespace robot_server_lib {

class TorqueControllerInvDynamic : public TorqueControllerBase {
public:
    TorqueControllerInvDynamic(RobotModel robot_model);
    bool ComputeCommand(const JointRawMeasurement& raw_measurement,
                        const PlannerManagerCommand& input_cmd,
                        TorqueCtrlCommand& output_cmd) override;
    bool ResetForNewEpisode(const JointRawMeasurement& raw_measurement, const TorqueControllerState& state) override;

protected:
    VectorDof idle_q_;
    std::shared_ptr<dynamics::JointImpedanceController> joint_impedance_controller_{nullptr};
    std::shared_ptr<dynamics::JointAdmittanceController> joint_admittance_controller_{nullptr};
};

}  // namespace robot_server_lib
