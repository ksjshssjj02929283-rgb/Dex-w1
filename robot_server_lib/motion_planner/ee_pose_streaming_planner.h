#pragma once

#include "move_planner_base.h"

namespace robot_server_lib {

class EEPoseStreamingPlanner : public MotionPlannerBase {
public:
    EEPoseStreamingPlanner(RobotModel robot_model,
                           std::shared_ptr<SingleWriterLatestBuffer<StreamingCommand>> streaming_buffer,
                           double controller_dt);
    ~EEPoseStreamingPlanner() override;

    ComputeCommandStatus ComputeCommand(const JointMeasurement& measurement,
                                        const TimeMetaInfo& time_meta,
                                        JointCommand& output_joint_cmd) override;
    bool ResetForNewTrajectory(const PlannerRequest& request,
                               const JointMeasurement& measurement,
                               const TimeMetaInfo& time_meta) override;

protected:
    bool ToJointStreamingCommand(StreamingCommand& cmd, const JointMeasurement& measurement);

    std::shared_ptr<SingleWriterLatestBuffer<StreamingCommand>> streaming_buffer_;
    ruckig::Ruckig<ndof_robot> ruckig_otg_;

    struct {
        Int64 request_id{-1};
        bool streaming_finished{false};
        std::optional<StreamingCommand> latest_cmd;
        std::optional<StreamingCommand> previous_cmd;

        ruckig::InputParameter<ndof_robot> input;
        ruckig::OutputParameter<ndof_robot> output;
    } runtime_;

private:
    TimePointSystem request_start_time_{TimePointSystem::Invalid()};
};

}  // namespace robot_server_lib