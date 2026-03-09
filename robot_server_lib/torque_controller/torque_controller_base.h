#pragma once
#include "utils.h"

namespace robot_server_lib {

enum class TorqueControllerState { Init, Idle, Decelerating, Operating };

class TorqueControllerBase {
public:
    TorqueControllerBase(RobotModel robot_model);
    virtual ~TorqueControllerBase();

    virtual bool ComputeCommand(const JointRawMeasurement &raw_measurement,
                                const PlannerManagerCommand &input_cmd,
                                TorqueCtrlCommand &output_cmd) = 0;

    virtual bool ResetForNewEpisode(const JointRawMeasurement &raw_measurement, const TorqueControllerState &state) = 0;

protected:
    RobotModel robot_model_;

    TorqueControllerState state_{TorqueControllerState::Init};
    // Gain parameter
    struct {
        double M_d = 2.0;
        VectorDof K_p = VectorDof::Constant(200.0);
        VectorDof K_v = VectorDof::Constant(20.0);
    } parameters_;
};
}  // namespace robot_server_lib