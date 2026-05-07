#pragma once

#include "move_planner_base.h"

namespace robot_server_lib {

class MoveJointPlanner : public MotionPlannerBase {
public:
    MoveJointPlanner(RobotModel robot_model, double controller_dt, bool wait_trajectory_upon_finish);

    ~MoveJointPlanner() override;

    ComputeCommandStatus ComputeCommand(const JointMeasurement& measurement,
                                        const TimeMetaInfo& time_meta,
                                        JointCommand& joint_cmd) override;
    bool ResetForNewTrajectory(const PlannerRequest& request,
                               const JointMeasurement& measurement,
                               const TimeMetaInfo& time_meta) override;

protected:
    const bool wait_trajectory_upon_finish_;
    ruckig::Ruckig<ndof_robot> ruckig_otg_;
    struct {
        ruckig::InputParameter<ndof_robot> input;
        ruckig::OutputParameter<ndof_robot> output;
    } runtime_;
};
}  // namespace robot_server_lib