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
    TorqueControllerBase(RobotModel robot_model);
    virtual ~TorqueControllerBase();

    virtual bool ComputeCommand(const JointRawMeasurement &raw_measurement, const PlannerManagerCommand &input_cmd,
                                TorqueCtrlCommand &output_cmd) = 0;

    virtual bool ResetForNewEpisode(const JointRawMeasurement &raw_measurement, const TorqueControllerState &state) = 0;

    VectorDof GetJointExternalTorque(const Eigen::VectorXd &sensor_wrench, const VectorDof &q_desired);

   protected:
    RobotModel robot_model_;
    pinocchio::Data robot_data_;
    pinocchio::Data robot_data_primary_;
    pinocchio::Data robot_data_secondary_;

    TorqueControllerState state_{TorqueControllerState::Init};
    // Gain parameter
    struct
    {
        double M_d = 0.05;
        double K_p{400};
        double K_v{40};
    } parameters_;
};
}  // namespace robot_server_lib