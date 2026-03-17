#pragma once

#include <chrono>

#include "butterworth_filter.h"
#include "move_planner_base.h"

namespace robot_server_lib {

class JointStreamingPlanner : public MotionPlannerBase {
public:
    JointStreamingPlanner(RobotModel robot_model,
                          std::shared_ptr<SingleWriterLatestBuffer<StreamingCommand>> streaming_buffer,
                          double controller_dt);
    ~JointStreamingPlanner() override;
    ComputeCommandStatus ComputeCommand(const JointMeasurement &measurement,
                                        const TimeMetaInfo &time_meta,
                                        JointCommand &output_joint_cmd) override;
    bool ResetForNewTrajectory(const PlannerRequest &request,
                               const JointMeasurement &measurement,
                               const TimeMetaInfo &time_meta) override;

protected:
    std::shared_ptr<SingleWriterLatestBuffer<StreamingCommand>> streaming_buffer_;
    ruckig::Ruckig<ndof_robot> ruckig_otg_;

    struct {
        Int64 request_id{-1};
        bool streaming_finished{false};
        bool stopping{false};  // 是否正在执行停止命令
        std::optional<StreamingCommand> latest_cmd;
        std::optional<StreamingCommand> previous_cmd;

        ruckig::InputParameter<ndof_robot> input;
        ruckig::OutputParameter<ndof_robot> output;
    } runtime_;

    TimePointSystem request_start_time_{TimePointSystem::Invalid()};
    TimePointSystem command_finished_time_{TimePointSystem::Invalid()};
    TimePointSystem last_log_time_{TimePointSystem::Invalid()};  // 上次打印日志的时间
    int frame_count_{0};                                         // 帧计数器
    bool use_ruckig_ = true;
    double streaming_velocity_timeout_{0.1};  // Ruckig 速度超时时间，默认 100ms

    // 巴特沃斯滤波器
    std::shared_ptr<ButterworthFilter> filter_[ndof_robot];
    bool use_filter_ = true;

    // 目标速度和加速度配置
    bool use_velocity_ = false;
    bool use_acceleration_ = false;
};

}  // namespace robot_server_lib
