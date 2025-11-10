#pragma once

#include <chrono>

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
    const double controller_dt_{0.004};  // 控制周期，单位s
    ruckig::Ruckig<ndof_robot> ruckig_otg_;

    struct
    {
        Int64 request_id{-1};
        bool streaming_finished{false};
        std::optional<StreamingCommand> latest_cmd;
        std::optional<StreamingCommand> previous_cmd;

        ruckig::InputParameter<ndof_robot> input;
        ruckig::OutputParameter<ndof_robot> output;
    } runtime_;

    TimePointSystem request_start_time_{TimePointSystem::Invalid()};
    bool no_ruckig_mode_ = false;
};

}  // namespace robot_server_lib