#pragma once

#include "move_planner_base.h"

namespace robot_server_lib
{

enum
{
    Trans = 0,
    Rot = 1,
    TransSecondary = 2,
    RotSecondary = 3,
    TransTorso = 4,
    RotTorso = 5
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

    void ConfigRuckig();

    bool ComputeOutputJoint(JointPositionDof &q_ndof);

    PoseQuaternion ComputeOutputPose(LineGeometry line, double ruckig_trans_position, double ruckig_rot_position);

    const bool wait_trajectory_upon_finish_;
    static constexpr int ruckig_dof_ = 6;

    // cache
    std::optional<LineGeometry> line_primary_;
    std::optional<LineGeometry> line_secondary_;
    std::optional<LineGeometry> line_torso_;
    ruckig::Ruckig<ruckig_dof_> ruckig_otg_;
    struct
    {
        ruckig::InputParameter<ruckig_dof_> input;
        ruckig::OutputParameter<ruckig_dof_> output;
    } runtime_;
};

}  // namespace robot_server_lib
