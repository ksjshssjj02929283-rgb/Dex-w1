#pragma once
#include <sstream>

#include "torque_controller_base.h"
namespace robot_server_lib {
// todo: param tune
class TorqueControllerTimeDelay : public TorqueControllerBase {
public:
    TorqueControllerTimeDelay(RobotModel robot_model);
    bool ComputeCommand(const JointRawMeasurement &raw_measurement,
                        const PlannerManagerCommand &input_cmd,
                        TorqueCtrlCommand &output_cmd) override;
    bool ResetForNewEpisode(const JointRawMeasurement &raw_measurement, const TorqueControllerState &state) override;

protected:
    VectorDof idle_q_;
    VectorDof last_feedback_dq_;
    TimePointSystem last_measurement_time_point_;
    VectorDof output_tau_;
    VectorDof M_bar_;
};

}  // namespace robot_server_lib