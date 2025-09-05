#pragma once
#include "torque_controller_base.h"

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
};

}  // namespace robot_server_lib