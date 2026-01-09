#pragma once
#include "torque_controller_base.h"
#include "inverse_dynamics.h"
#include "cartesian_impedance_controller.h"

namespace robot_server_lib
{

class TorqueControllerInvDynamic : public TorqueControllerBase
{
   public:
    TorqueControllerInvDynamic(RobotModel robot_model);
    bool ComputeCommand(const JointRawMeasurement &raw_measurement, const PlannerManagerCommand &input_cmd, TorqueCtrlCommand &output_cmd) override;
    bool ResetForNewEpisode(const JointRawMeasurement &raw_measurement, const TorqueControllerState &state) override;

   protected:
    VectorDof idle_q_;
    std::shared_ptr<dynamics::CartesianImpedanceController> impedance_ctrl_primary_{nullptr};
    std::shared_ptr<dynamics::CartesianImpedanceController> impedance_ctrl_secondary_{nullptr};
};

}  // namespace robot_server_lib