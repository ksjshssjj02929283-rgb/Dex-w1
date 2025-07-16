#pragma once

#include "move_planner_base.h"

namespace robot_server_lib
{

class MoveJointPlanner : public MotionPlannerBase {
  public:
    MoveJointPlanner(RobotModel robot_model, double controller_dt, bool wait_trajectory_upon_finish)
    : MotionPlannerBase(robot_model),
      controller_dt_(controller_dt),
      ruckig_otg_(controller_dt),
      wait_trajectory_upon_finish_(wait_trajectory_upon_finish) {
        for (int i = 0; i < ndof_robot; ++i) {
            runtime_.input.max_velocity[i] = joint_bounds_.max_velocity;
            runtime_.input.max_acceleration[i] = joint_bounds_.max_acceleration;
            runtime_.input.max_jerk[i] = joint_bounds_.max_jerk;
        }
      }

    ~MoveJointPlanner() override = default;

    ComputeCommandStatus ComputeCommand(const JointMeasurement& measurement, const TimeMetaInfo& time_meta, JointCommand& joint_cmd) override
    {
        auto status = ruckig_otg_.update(runtime_.input, runtime_.output);
        switch (status)
        {
          case ruckig::Working: {
            // 输出当前轨迹点的期望位置、速度、加速度
            // 并将输出传递给下一步输入
            for (auto i = 0; i < ndof_robot; i++) {
              joint_cmd.position[i] = runtime_.output.new_position[i];
              joint_cmd.velocity[i] = runtime_.output.new_velocity[i];
              joint_cmd.acceleration[i] = runtime_.output.new_acceleration[i];
              joint_cmd.torque_feedforward[i] = 0;
            }

            // Done
            runtime_.output.pass_to_input(runtime_.input);
            return ComputeCommandStatus::TaskOngoing;
          };
          case ruckig::Finished: {
            // 输出当前轨迹点的期望位置、速度、加速度
            // 并将输出传递给下一步输入
            for (auto i = 0; i < ndof_robot; i++) {
              joint_cmd.position[i] = runtime_.output.new_position[i];
              joint_cmd.velocity[i] = 0;
              joint_cmd.acceleration[i] = 0;
              joint_cmd.torque_feedforward[i] = 0;
            }

            // Done
            if (wait_trajectory_upon_finish_) {
              const bool robot_stopped = (measurement.velocity.norm() < IdleVelocityThreshold);
              return robot_stopped ? ComputeCommandStatus::TaskFinished
                                  : ComputeCommandStatus::TaskOngoing;
            } else {
              return ComputeCommandStatus::TaskFinished;
            }
          };
          default:
            return ComputeCommandStatus::Error;
        }        
    }

    bool ResetForNewTrajectory(const PlannerRequest& request, const JointMeasurement& measurement, const TimeMetaInfo& time_meta) override
    {
        assert(request.data.has_value());
        assert(std::holds_alternative<MoveRequest>(request.data.value()));

        const auto& move_request = std::get<MoveRequest>(request.data.value());
        const auto& target_q = move_request.value;
        // set input parameters with global ratio except for jerk
        double speed_ratio = robot_property_.global_speed_ratio;
        if (move_request.speed.has_value()) {
            speed_ratio = speed_ratio * std::clamp(move_request.speed.value(), 0.01, 1.0);
        }
        for (int i = 0; i < ndof_robot; ++i) {
            runtime_.input.max_velocity[i] = joint_bounds_.max_velocity * speed_ratio;
            runtime_.input.max_acceleration[i] = joint_bounds_.max_acceleration * speed_ratio;
        }

        std::ostringstream oss;
        oss << target_q.transpose();
        LOG_INFO << "";
        LOG_INFO << "Move Joint Request: id: " << request.request_id << " motion_group: " 
                << ToString(move_request.motion_group) << " target_q: " << oss.str();
        // Reset data
        
        JointPositionDof target_ndof = measurement.q_desired_pre;
        SetMotionGroup(move_request.motion_group);
        MapArmToWholeBody(target_q, target_ndof);

        // Assign data
        for (auto i = 0; i < ndof_robot; i++) {
          runtime_.input.current_position[i] = measurement.q_desired_pre[i];
          runtime_.input.current_velocity[i] = measurement.velocity[i];
          runtime_.input.current_acceleration[i] = 0.0;

          runtime_.input.target_position[i] = target_ndof[i];
          runtime_.input.target_velocity[i] = 0.0;
          runtime_.input.target_acceleration[i] = 0.0;
        }

        // Done
        return true;
    }
  protected:
    const bool wait_trajectory_upon_finish_;
    const double controller_dt_{0.01};
    ruckig::Ruckig<ndof_robot> ruckig_otg_;
    // cache
    struct {
      ruckig::InputParameter<ndof_robot> input;
      ruckig::OutputParameter<ndof_robot> output;
    } runtime_;
};

} // namespace robot_server_lib
