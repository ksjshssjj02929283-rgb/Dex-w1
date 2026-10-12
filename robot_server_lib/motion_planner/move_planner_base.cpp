#include "move_planner_base.h"

namespace robot_server_lib {

std::shared_ptr<kinematics::KinematicsBase> CreateArmKinematics(const std::string& model_name) {
    // 使用 SDK 注册的型号列表判断，避免自行维护型号与算法的映射。
    for (const auto& [brand, model] : DexeRP::Kinematics::SRSKineParamsRegistry::SupportedRobots()) {
        if (brand == "DexForce" && model == model_name) {
            return std::make_shared<kinematics::SRSKinematics>(model_name);
        }
    }

    for (const auto& [brand, model] : DexeRP::Kinematics::FEPKineParamsRegistry::SupportedRobots()) {
        if (brand == "DexForce" && model == model_name) {
            return std::make_shared<kinematics::FEPKinematics>(model_name);
        }
    }

    throw std::invalid_argument("Unsupported robot kinematics model: " + model_name);
}

MotionPlannerBase::MotionPlannerBase(RobotModel robot_model) {
    // Initialize robot model
    robot_model_ = std::move(robot_model);
    robot_data_ = pinocchio::Data(*robot_model_.model_ptr);
    // Initialize kinematics for both arms
    // kinematics_primary_ptr_ = std::make_shared<kinematics::SRSKinematics>(true, true);
    // kinematics_secondary_ptr_ = std::make_shared<kinematics::SRSKinematics>(false, true);
    const auto& params = *robot_model_.params_ptr;
    std::string version = params.robot_version;
    std::transform(version.begin(), version.end(), version.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    if (params.robot_type != "w1" || version.empty() || version.front() != 'V') {
        throw std::invalid_argument("Unsupported robot type or version: " + params.robot_type + " " +
                                    params.robot_version);
    }
    const std::string model_name_prefix = "W1" + version;
    const std::string primary_model_name = model_name_prefix + "Left";
    const std::string secondary_model_name = model_name_prefix + "Right";
    kinematics_primary_ptr_ = CreateArmKinematics(primary_model_name);
    kinematics_secondary_ptr_ = CreateArmKinematics(secondary_model_name);

    kinematics_torso_ptr_ = std::make_shared<kinematics::TorsoKinematics>();
    assert(kinematics_primary_ptr_ && kinematics_secondary_ptr_ && kinematics_torso_ptr_);
}

ComputeCommandStatus MotionPlannerBase::ComputeStopCommand(const JointMeasurement& measurement,
                                                           const TimeMetaInfo& time_meta,
                                                           JointCommand& joint_cmd) {
    bool stop_finished = true;
    auto output_velocity = joint_cmd.velocity;
    bool robot_stopped = true;
    for (auto i = 0; i < ndof_robot; i++) {
        if (output_velocity[i] > 0.0 && output_velocity[i] > stop_bounds_.max_acceleration * controller_dt_) {
            output_velocity[i] -= stop_bounds_.max_acceleration * controller_dt_;
        } else if (output_velocity[i] < 0.0 && output_velocity[i] < -stop_bounds_.max_acceleration * controller_dt_) {
            output_velocity[i] += stop_bounds_.max_acceleration * controller_dt_;
        } else {
            output_velocity[i] = 0.0;
            stop_finished = stop_finished && true;
        }
        joint_cmd.velocity[i] = output_velocity[i];
        joint_cmd.position[i] += joint_cmd.velocity[i] * controller_dt_;
        robot_stopped = robot_stopped && measurement.velocity[i] < IdleVelocityThreshold;
    }
    robot_stopped = stop_finished && robot_stopped;
    if (robot_stopped) {
        LOG_INFO << "ComputeStopCommand: Robot stopped, velocity: " << measurement.velocity.transpose();
        return ComputeCommandStatus::TaskStopped;
    } else {
        return ComputeCommandStatus::TaskOngoing;
    }
}

void MotionPlannerBase::SetRobotProperty(const RobotProperty& property) { robot_property_ = property; }

}  // namespace robot_server_lib
