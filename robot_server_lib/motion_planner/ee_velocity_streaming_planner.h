#pragma once

#include "move_planner_base.h"

namespace robot_server_lib {

class EEVelocityStreamingPlanner : public MotionPlannerBase {
public:
    EEVelocityStreamingPlanner(RobotModel robot_model,
                               std::shared_ptr<SingleWriterLatestBuffer<StreamingCommand>> streaming_buffer,
                               double controller_dt);
    ~EEVelocityStreamingPlanner() override;

    ComputeCommandStatus ComputeCommand(const JointMeasurement &measurement,
                                        const TimeMetaInfo &time_meta,
                                        JointCommand &output_joint_cmd) override;
    bool ResetForNewTrajectory(const PlannerRequest &request,
                               const JointMeasurement &measurement,
                               const TimeMetaInfo &time_meta) override;

    std::shared_ptr<SingleWriterLatestBuffer<StreamingCommand>> streaming_buffer_;

    struct {
        Int64 request_id{-1};
        bool streaming_finished{false};
        std::optional<StreamingCommand> latest_cmd;

        // Dual-arm specific fields
        Eigen::VectorXd qdot_solved_primary;    // For primary arm (7 DOF)
        Eigen::VectorXd qdot_solved_secondary;  // For secondary arm (7 DOF)
        Eigen::VectorXd qdot_solved;            // Backward compatibility
    } runtime_;

private:
    pinocchio::Model pinocchio_model_;
    TimePointSystem request_start_time_{TimePointSystem::Invalid()};
    ReferenceFrame reference_frame_{ReferenceFrame::WorldBase};
    int arm_base_id_{-1};
};

}  // namespace robot_server_lib