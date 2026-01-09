#pragma once
#include "utils.h"

namespace robot_server_lib
{

enum class TorqueControllerState
{
    Init,
    Idle,
    Decelerating,
    Operating
};

class TorqueControllerBase
{
   public:
    TorqueControllerBase(RobotModel robot_model);
    virtual ~TorqueControllerBase();

    virtual bool ComputeCommand(const JointRawMeasurement &raw_measurement, const PlannerManagerCommand &input_cmd,
                                TorqueCtrlCommand &output_cmd) = 0;

    virtual bool ResetForNewEpisode(const JointRawMeasurement &raw_measurement, const TorqueControllerState &state) = 0;


   protected:
    RobotModel robot_model_;

    TorqueControllerState state_{TorqueControllerState::Init};
    // Gain parameter
    struct
    {
        double M_d = 2.0;
        double K_p{300};
        double K_v = 2.0 * std::sqrt(K_p);
    } parameters_;
};
}  // namespace robot_server_lib