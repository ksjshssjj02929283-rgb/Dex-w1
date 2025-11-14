#pragma once

#include "move_planner_base.h"

namespace robot_server_lib
{

enum
{
    Trans = 0,
    Rot = 1,
    TransSecondary = 2,
    RotSecondary = 3
};

struct LineGeometry
{
    PoseQuaternion pose_start{};
    PoseQuaternion pose_target{};
    Eigen::Vector3d direction{0.0, 0.0, 0.0};
    double length{0.0};
    double slerp_angle{0.0};
};

class MoveLinePlanner : public MotionPlannerBase
{
   public:
    MoveLinePlanner(RobotModel robot_model, double controller_dt, bool wait_trajectory_upon_finish);

    ~MoveLinePlanner() override;

    ComputeCommandStatus ComputeCommand(const JointMeasurement &measurement, const TimeMetaInfo &time_meta, JointCommand &joint_cmd) override;

    bool ResetForNewTrajectory(const PlannerRequest &request, const JointMeasurement &measurement, const TimeMetaInfo &time_meta) override;

   protected:
    LineGeometry CalcLineGeometry(const PoseQuaternion &pose_start, const PoseQuaternion &pose_end);

    void InitializeRuckig();

    bool ComputeOutputJoint(JointPositionDof &q_ndof);

    PoseQuaternion ComputeOutputPose(LineGeometry line, double ruckig_trans_position, double ruckig_rot_position);

    const bool wait_trajectory_upon_finish_;

    // cache
    std::optional<LineGeometry> line_primary_;
    std::optional<LineGeometry> line_secondary_;
    ruckig::Ruckig<4> ruckig_otg_;
    struct
    {
        ruckig::InputParameter<4> input;
        ruckig::OutputParameter<4> output;
    } runtime_;
};

}  // namespace robot_server_lib
