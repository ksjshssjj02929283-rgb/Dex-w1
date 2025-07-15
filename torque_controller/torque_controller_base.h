#pragma once
#include "utilities.h"
#include "pinocchio_include.h"

namespace robot_server_lib {

struct TorqueCtrlCommand {
  VectorDof torque;
  VectorDof q_desired;
};

class TorqueControllerBase {
public:
    TorqueControllerBase(RobotModel robot_model)
        : robot_model_(std::move(robot_model)), robot_data_(*robot_model_.model_ptr) {}

    virtual ~TorqueControllerBase() = default;

    virtual bool ComputeCommand(const JointRawMeasurement& raw_measurement, const PlannerManagerCommand& input_cmd, TorqueCtrlCommand& output_cmd) = 0;

    virtual bool ResetForNewEpisode(const JointRawMeasurement& raw_measurement){ return true; };

protected:
    RobotModel robot_model_;
    pinocchio::Data robot_data_;
    // Gain parameter
    struct {
        double K_p{120};
        double K_v{45};
    } parameters_;
};
} // namespace robot_server_lib