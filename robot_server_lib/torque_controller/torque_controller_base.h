#pragma once
#include "utils.h"

namespace robot_server_lib
{

struct TorqueCtrlCommand
{
    VectorDof torque;
    VectorDof q_desired;
};

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
    TorqueControllerBase(RobotModel robot_model) : robot_model_(std::move(robot_model)), robot_data_(*robot_model_.model_ptr) {}

    virtual ~TorqueControllerBase() = default;

    virtual bool ComputeCommand(const JointRawMeasurement &raw_measurement, const PlannerManagerCommand &input_cmd,
                                TorqueCtrlCommand &output_cmd) = 0;

    virtual bool ResetForNewEpisode(const JointRawMeasurement &raw_measurement, const TorqueControllerState &state) = 0;

   protected:
    RobotModel robot_model_;
    pinocchio::Data robot_data_;
    TorqueControllerState state_{TorqueControllerState::Init};
    // Gain parameter
    struct
    {
        double K_p{400};
        double K_v{40};
    } parameters_;
};
}  // namespace robot_server_lib