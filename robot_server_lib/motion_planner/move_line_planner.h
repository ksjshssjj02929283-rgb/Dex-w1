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
    MoveLinePlanner(RobotModel robot_model, double controller_dt, bool wait_trajectory_upon_finish)
        : MotionPlannerBase(robot_model),
          controller_dt_(controller_dt),
          ruckig_otg_(controller_dt),
          wait_trajectory_upon_finish_(wait_trajectory_upon_finish)
    {
        // Initialize default input parameters
        for (int idx : {Trans, TransSecondary})
        {
            runtime_.input.max_velocity[idx] = cartesian_bounds_.translation.max_velocity;
            runtime_.input.max_acceleration[idx] = cartesian_bounds_.translation.max_acceleration;
            runtime_.input.max_jerk[idx] = cartesian_bounds_.translation.max_jerk;
        }
        for (int idx : {Rot, RotSecondary})
        {
            runtime_.input.max_velocity[idx] = cartesian_bounds_.rotation.max_velocity;
            runtime_.input.max_acceleration[idx] = cartesian_bounds_.rotation.max_acceleration;
            runtime_.input.max_jerk[idx] = cartesian_bounds_.rotation.max_jerk;
        }
    }

    ~MoveLinePlanner() override = default;

    ComputeCommandStatus ComputeCommand(const JointMeasurement &measurement, const TimeMetaInfo &time_meta, JointCommand &joint_cmd) override
    {
        auto status = ruckig_otg_.update(runtime_.input, runtime_.output);
        JointPositionDof q_ndof = measurement.q_desired_pre;

        // 计算, 逆解, 并映射到全身关节
        if (!ComputeOutputJoint(q_ndof))
        {
            LOG_CRIT << "MoveLine: Solve IK failed";
            return ComputeCommandStatus::Error;
        }
        // 输出关节cmd
        switch (status)
        {
        case ruckig::Working:
        {
            runtime_.output.pass_to_input(runtime_.input);

            // todo: 计算关节速度
            for (auto i = 0; i < ndof_robot; i++)
            {
                joint_cmd.position[i] = q_ndof[i];
                joint_cmd.velocity[i] = 0.0;
                joint_cmd.acceleration[i] = 0.0;
                joint_cmd.torque_feedforward[i] = 0.0;
            }
            // Done
            return ComputeCommandStatus::TaskOngoing;
        };
        case ruckig::Finished:
        {
            // todo: 计算关节速度
            for (auto i = 0; i < ndof_robot; i++)
            {
                joint_cmd.position[i] = q_ndof[i];
                joint_cmd.velocity[i] = 0.0;
                joint_cmd.acceleration[i] = 0.0;
                joint_cmd.torque_feedforward[i] = 0.0;
            }

            // Done
            if (wait_trajectory_upon_finish_)
            {
                const bool robot_stopped = (measurement.velocity.norm() < IdleVelocityThreshold);
                return robot_stopped ? ComputeCommandStatus::TaskFinished : ComputeCommandStatus::TaskOngoing;
            }
            else
            {
                return ComputeCommandStatus::TaskFinished;
            }
        };
        default:
            return ComputeCommandStatus::Error;
        }
    }

    bool ResetForNewTrajectory(const PlannerRequest &request, const JointMeasurement &measurement, const TimeMetaInfo &time_meta) override
    {
        assert(request.data.has_value());
        assert(std::holds_alternative<MoveRequest>(request.data.value()));

        const auto &move_request = std::get<MoveRequest>(request.data.value());
        SetMotionGroup(move_request.motion_group);
        double speed_ratio = robot_property_.global_speed_ratio;
        if (move_request.speed.has_value())
        {
            speed_ratio = speed_ratio * std::clamp(move_request.speed.value(), 0.01, 1.0);
        }

        // set input parameters with global ratio except for jerk
        for (int idx : {Trans, TransSecondary})
        {
            runtime_.input.max_velocity[idx] = cartesian_bounds_.translation.max_velocity * speed_ratio;
            runtime_.input.max_acceleration[idx] = cartesian_bounds_.translation.max_acceleration * speed_ratio;
        }
        for (int idx : {Rot, RotSecondary})
        {
            runtime_.input.max_velocity[idx] = cartesian_bounds_.rotation.max_velocity * speed_ratio;
            runtime_.input.max_acceleration[idx] = cartesian_bounds_.rotation.max_acceleration * speed_ratio;
        }

        switch (move_request.motion_group)
        {
        case MotionGroup::BothArms:
        {
            assert(14 == move_request.value.size());
            Eigen::VectorXd q_start;
            GetArmFromWholeBody(measurement.q_desired_pre, q_start);
            assert(q_start.size() == ndof_primary_ + ndof_secondary_);
            // 获取起点
            PoseQuaternion pose_start = kinematics_ptr_->GetFK(q_start.head(ndof_primary_), robot_property_.tcp_frame);
            PoseQuaternion pose_start_secondary =
                kinematics_secondary_ptr_->GetFK(q_start.tail(ndof_secondary_), robot_property_.tcp_frame_secondary);
            // 获取目标点
            PoseQuaternion pose_target = utilities::VectorXdToPoseQuaternion(move_request.value.head(7));
            PoseQuaternion pose_target_secondary = utilities::VectorXdToPoseQuaternion(move_request.value.tail(7));

            LOG_INFO << "Primary arm motion plan: start: " << pose_start.ToString() << ", target: " << pose_target.ToString();
            LOG_INFO << "Secondary arm motion plan: start: " << pose_start_secondary.ToString() << ", target: " << pose_target_secondary.ToString();

            line_primary_.emplace(CalcLineGeometry(pose_start, pose_target));
            line_secondary_.emplace(CalcLineGeometry(pose_start_secondary, pose_target_secondary));
            break;
        }
        case MotionGroup::Primary:
        {
            assert(7 == move_request.value.size());
            Eigen::VectorXd q_start;
            GetArmFromWholeBody(measurement.q_desired_pre, q_start);
            assert(q_start.size() == ndof_primary_);
            PoseQuaternion pose_start = kinematics_ptr_->GetFK(q_start, robot_property_.tcp_frame);
            PoseQuaternion pose_target = utilities::VectorXdToPoseQuaternion(move_request.value);

            LOG_INFO << "Primary arm motion plan: start: " << pose_start.ToString() << ", target: " << pose_target.ToString();

            line_primary_.emplace(CalcLineGeometry(pose_start, pose_target));
            line_secondary_ = std::nullopt;
            break;
        }
        case MotionGroup::Secondary:
        {
            assert(7 == move_request.value.size());
            Eigen::VectorXd q_start;
            GetArmFromWholeBody(measurement.q_desired_pre, q_start);
            assert(q_start.size() == ndof_secondary_);
            PoseQuaternion pose_start = kinematics_secondary_ptr_->GetFK(q_start, robot_property_.tcp_frame_secondary);
            PoseQuaternion pose_target = utilities::VectorXdToPoseQuaternion(move_request.value);

            LOG_INFO << "Secondary arm motion plan: start: " << pose_start.ToString() << ", target: " << pose_target.ToString();

            line_primary_ = std::nullopt;
            line_secondary_.emplace(CalcLineGeometry(pose_start, pose_target));
            break;
        }
        default:
            assert(false && "Unknown MotionGroup type");
            break;
        }

        InitializeRuckig();
        return true;
    }

   protected:
    LineGeometry CalcLineGeometry(const PoseQuaternion &pose_start, const PoseQuaternion &pose_end)
    {
        LineGeometry line;
        line.pose_start = pose_start;
        line.pose_target = pose_end;
        // translation
        Eigen::Vector3d dir = pose_end.position - pose_start.position;  // 计算r(AB)向量
        double distance = dir.norm();                                   // 计算两点之间的距离
        assert(!std::isnan(distance));
        if (distance < 1e-6)
        {
            dir = {1, 0, 0};  // 如果两点重合，设置默认方向
        }
        else
        {
            dir.normalize();  // 归一化方向向量
        }
        line.length = distance;
        line.direction = dir;

        // rotation
        assert(fabs(pose_start.orientation.norm() - 1.0) < 1e-6);
        assert(fabs(pose_end.orientation.norm() - 1.0) < 1e-6);
        Eigen::Quaterniond slerpQ = (pose_start.orientation.conjugate()) * (pose_end.orientation);
        slerpQ.normalize();
        line.slerp_angle = fabs(Eigen::AngleAxisd(slerpQ).angle());  //四元数对应的旋转角
        if (std::isnan(line.slerp_angle))
        {
            line.slerp_angle = 0;
        }
        return line;
    }

    void InitializeRuckig()
    {
        for (int i = 0; i < 4; i++)
        {
            // Initialize translation input parameters
            runtime_.input.current_position[i] = 0.0;
            runtime_.input.current_velocity[i] = 0.0;
            runtime_.input.current_acceleration[i] = 0.0;
            runtime_.input.target_position[i] = 0.0;
            runtime_.input.target_velocity[i] = 0.0;
            runtime_.input.target_acceleration[i] = 0.0;
        }
        if (line_primary_.has_value())
        {
            runtime_.input.target_position[Trans] = line_primary_->length;
            runtime_.input.target_position[Rot] = line_primary_->slerp_angle;
        }
        if (line_secondary_.has_value())
        {
            runtime_.input.target_position[TransSecondary] = line_secondary_->length;
            runtime_.input.target_position[RotSecondary] = line_secondary_->slerp_angle;
        }
    }

    bool ComputeOutputJoint(JointPositionDof &q_ndof)
    {
        if (line_primary_.has_value())
        {
            PoseQuaternion pose_cmd = ComputeOutputPose(*line_primary_, runtime_.output.new_position[Trans], runtime_.output.new_position[Rot]);
            // 逆解主臂
            Eigen::VectorXd q_primary;
            GetArmFromWholeBody(q_ndof, q_primary, MotionGroup::Primary);
            assert(q_primary.size() == ndof_primary_);
            if (!kinematics_ptr_->GetIK(pose_cmd, q_primary, robot_property_.tcp_frame))
            {
                // 逆运动学失败，返回错误 返回当前关节角
                LOG_CRIT << "Inverse Kinematics failed for target pose: " << pose_cmd.ToString();
                return false;
            }
            MapArmToWholeBody(q_primary, q_ndof, MotionGroup::Primary);
        }

        if (line_secondary_.has_value())
        {
            PoseQuaternion pose_cmd_secondary =
                ComputeOutputPose(*line_secondary_, runtime_.output.new_position[TransSecondary], runtime_.output.new_position[RotSecondary]);
            // 逆解副臂
            Eigen::VectorXd q_secondary;
            GetArmFromWholeBody(q_ndof, q_secondary, MotionGroup::Secondary);

            assert(q_secondary.size() == ndof_secondary_);
            if (!kinematics_secondary_ptr_->GetIK(pose_cmd_secondary, q_secondary, robot_property_.tcp_frame_secondary))
            {
                // 逆运动学失败，返回错误 todo: 需要处理错误
                LOG_CRIT << "Inverse Kinematics failed for target pose: " << pose_cmd_secondary.ToString();
                return false;
            }

            MapArmToWholeBody(q_secondary, q_ndof, MotionGroup::Secondary);
        }
        return true;
    }

    PoseQuaternion ComputeOutputPose(LineGeometry line, double ruckig_trans_position, double ruckig_rot_position)
    {
        PoseQuaternion pose_cmd;
        pose_cmd.position = line.pose_start.position + line.direction * ruckig_trans_position;
        if (line.slerp_angle < 1e-5)
        {
            pose_cmd.orientation = line.pose_target.orientation;
        }
        else
        {
            double t = ruckig_rot_position / line.slerp_angle;
            t = std::clamp(t, 0.0, 1.0);
            pose_cmd.orientation = line.pose_start.orientation.slerp(t, line.pose_target.orientation);
            pose_cmd.orientation.normalize();
            assert(fabs(pose_cmd.orientation.norm() - 1.0) < 1e-6);
        }
        return pose_cmd;
    }

    const bool wait_trajectory_upon_finish_;
    const double controller_dt_{0.01};

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
