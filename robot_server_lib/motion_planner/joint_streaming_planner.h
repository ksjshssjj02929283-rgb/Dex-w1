#pragma once

#include "move_planner_base.h"

namespace robot_server_lib
{

class JointStreamingPlanner : public MotionPlannerBase
{
   public:
    JointStreamingPlanner(RobotModel robot_model, std::shared_ptr<SingleWriterLatestBuffer<StreamingCommand>> streaming_buffer, double controller_dt);
    ~JointStreamingPlanner() override;
    ComputeCommandStatus ComputeCommand(const JointMeasurement &measurement, const TimeMetaInfo &time_meta, JointCommand &output_joint_cmd) override;
    bool ResetForNewTrajectory(const PlannerRequest &request, const JointMeasurement &measurement, const TimeMetaInfo &time_meta) override;

   protected:
    std::shared_ptr<SingleWriterLatestBuffer<StreamingCommand>> streaming_buffer_;
    const double controller_dt_{0.01};

    struct
    {
        Int64 request_id{-1};
        bool streaming_finished{false};
        std::optional<StreamingCommand> latest_cmd;
    } runtime_;
};

}  // namespace robot_server_lib