#pragma once

#include <chrono>

#include "move_planner_base.h"

namespace robot_server_lib
{

class MoveTrajPlanner : public MotionPlannerBase
{
public:
    // 修改构造函数，增加 RobotModel
    explicit MoveTrajPlanner(RobotModel model, double controller_dt, bool wait_trajectory_upon_finish);
    ~MoveTrajPlanner() override;

    ComputeCommandStatus ComputeCommand(const JointMeasurement &measurement,
                                        const TimeMetaInfo &time_meta,
                                        JointCommand &output_joint_cmd) override;

    bool ResetForNewTrajectory(const PlannerRequest &request,
                               const JointMeasurement &measurement,
                               const TimeMetaInfo &time_meta) override;

    // 根据 timepoint 返回插值后的关节位置/速度/加速度，并判定阶段或完成状态
    ComputeCommandStatus GetInterpolatedPosition(JointCommand& joint_cmd, double timepoint);

private:
    const bool wait_trajectory_upon_finish_;
    struct
    {
        int traj_dof{0};
        int traj_npoints{0};
        double timepoint{0.0};                      // 当前已经执行的时间
        double total_duration{0.0};                 // 轨迹总时长
        double interval{0.01};                      // 轨迹原始点之间的时间间隔
        std::vector<int> endpoint_index;            // 阶段终止点（目标点序号）
        std::vector<Eigen::VectorXd> target_traj;   // 原始目标点序列

        void Reset()
        {
            traj_dof = 0;
            traj_npoints = 0;
            timepoint = 0.0;
            total_duration = 0.0;
            interval = 0.01;
            endpoint_index.clear();
            target_traj.clear();
        }
    } runtime_;
};

}  // namespace robot_server_lib