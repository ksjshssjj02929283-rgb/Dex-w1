#pragma once

#include <optional>
#include "kinematics_include.h"
#include "pinocchio_include.h"
#include "ruckig/ruckig.hpp"
#include "utilities.h"

namespace robot_server_lib
{

struct TrajectoryBounds
{
    double max_velocity{1.0};
    double max_acceleration{2.0};
    double max_jerk{100000.0};
};

enum class ComputeCommandStatus
{
    // The request is invalid and it should NOT be used
    // The state would be switched into error state
    Error = 0,

    // This controller::ComputeRequest would be invoked again
    // in the next control loop
    TaskOngoing = 1,

    // It is guaranteed that once TaskFinished has been returned
    // The same controller ComputeCommand would NOT be invoked
    // without the next ResetForNewEpisode
    TaskFinished = 2
};

class MotionPlannerBase
{
   public:
    explicit MotionPlannerBase(RobotModel robot_model);

    virtual ~MotionPlannerBase() = default;

    // Compute the next joint command in current episode
    virtual ComputeCommandStatus ComputeCommand(const JointMeasurement &measurement, const TimeMetaInfo &time_meta, JointCommand &joint_cmd) = 0;

    virtual bool ResetForNewTrajectory(const PlannerRequest &request, const JointMeasurement &measurement, const TimeMetaInfo &time_meta) = 0;

    void SetRobotProperty(const RobotProperty &property);

    bool MapArmToWholeBody(const Eigen::VectorXd &q, JointPositionDof &q_ndof) const;
    bool MapArmToWholeBody(const Eigen::VectorXd &q, JointPositionDof &q_ndof, const MotionGroup &motion_group) const;

    bool GetArmFromWholeBody(const JointPositionDof &q_ndof, Eigen::VectorXd &q) const;
    bool GetArmFromWholeBody(const JointPositionDof &q_ndof, Eigen::VectorXd &q, const MotionGroup &motion_group) const;
    std::shared_ptr<kinematics::KinematicsBase> Kinematics() const { return kinematics_ptr_; }
    std::shared_ptr<kinematics::KinematicsBase> KinematicsSecondary() const { return kinematics_secondary_ptr_; }

    Int64 episode_index{-1};  // 记录当前的episode索引

   protected:
    void SetMotionGroup(MotionGroup motion_group) { motion_group_ = motion_group; }
    RobotModel robot_model_;
    pinocchio::Data robot_data_;
    RobotProperty robot_property_;
    std::shared_ptr<kinematics::KinematicsBase> kinematics_ptr_;
    std::shared_ptr<kinematics::KinematicsBase> kinematics_secondary_ptr_;

    TrajectoryBounds streaming_bounds_{1.0 * M_PI, 4.0 * M_PI, 8.0 * M_PI};  // trajectory bounds, unit: rad
    TrajectoryBounds joint_bounds_{1.0 * M_PI, 1.0 * M_PI, 1.0 * M_PI};      // trajectory bounds, unit: rad

    struct
    {
        TrajectoryBounds translation{0.25, 0.25, 0.25};                 // cartesian space translation bounds, unit: m
        TrajectoryBounds rotation{0.5 * M_PI, 0.5 * M_PI, 0.5 * M_PI};  // cartesian space rotation bounds, unit: rad
    } cartesian_bounds_;

    MotionGroup motion_group_;

    int ndof_primary_{7};
    int ndof_secondary_{7};
};

}  // namespace robot_server_lib