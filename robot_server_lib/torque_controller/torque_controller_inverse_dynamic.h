#pragma once
#include "cartesian_impedance_controller.h"
#include "inverse_dynamics.h"
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
    VectorDof integral_error_ = VectorDof::Zero();  // 积分误差累计
    double error_integral_max_ = 0.1;               // 积分误差限幅
};

}  // namespace robot_server_lib
