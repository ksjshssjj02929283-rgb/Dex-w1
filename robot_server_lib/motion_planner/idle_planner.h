#pragma once

#include "move_planner_base.h"

namespace robot_server_lib
{

class IdlePlanner : public MotionPlannerBase
{
public:
    IdlePlanner(RobotModel robot_model);
    ~IdlePlanner() override;

    ComputeCommandStatus ComputeCommand(const JointMeasurement &measurement, const TimeMetaInfo &time_meta, JointCommand &joint_cmd) override;
    bool ResetForNewTrajectory(const PlannerRequest &request, const JointMeasurement &measurement, const TimeMetaInfo &time_meta) override;

protected:
    VectorDof idle_position_{};
};

}  // namespace robot_server_lib