#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "NanoLog.hpp"
#include "move_trajectory_planner.h"
#include "time_utility.h"

using namespace robot_server_lib;

TEST(MovePlanner, MoveTrajectoryTest)
{
    std::filesystem::create_directories("log/");
    nanolog::initialize(nanolog::GuaranteedLogger(), "log/", "robot_server", 10);
    nanolog::set_log_level(nanolog::LogLevel::INFO);

    const auto package_share_path = ament_index_cpp::get_package_share_directory("sim_server");
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot.cfg";
    json config_json;
    if (!std::filesystem::exists(robot_cfg_path))
    {
        assert(false && "Robot configuration file not found");
    }
    std::ifstream ifs(robot_cfg_path);
    if (!ifs.is_open() || !(ifs >> config_json))
    {
        assert(false && "Can't open or parse robot.cfg");
    }
    // 初始化机器人模型
    RobotParams params;
    if (!utilities::GetRobotParamsFromConfig(config_json, params))
    {
        assert(false);
    }
    pinocchio::Model model, left_arm_model, right_arm_model;
    pinocchio::urdf::buildModel(params.urdf_path, model);
    pinocchio::urdf::buildModel(params.left_arm_urdf_path, left_arm_model);
    pinocchio::urdf::buildModel(params.right_arm_urdf_path, right_arm_model);

    RobotModel robot_model_;
    robot_model_.model_ptr = std::make_shared<pinocchio::Model>(model);
    robot_model_.left_arm_model_ptr = std::make_shared<pinocchio::Model>(left_arm_model);
    robot_model_.right_arm_model_ptr = std::make_shared<pinocchio::Model>(right_arm_model);
    robot_model_.params_ptr = std::make_shared<RobotParams>(params);

    Eigen::VectorXd pose_target(14);
    pose_target << -0.3, 0.363727, 0.262795, -0.229399, -0.440151, 0.599072, 0.628296,
                    0.3, 0.363727, 0.262795, 0.628296, -0.599072, 0.440151, -0.229399;

    PlannerRequest request;
    request.type = RequestType::MoveTrajectory;
    request.request_id = 1;
    request.data = MoveRequest{
        .motion_group = MotionGroup::BothArms,
        .value = pose_target,
        .reference_frame = ReferenceFrame::ArmBase,
        .speed = 1.0,
        .interp_degree = 1};

    JointMeasurement measurement;
    measurement.q_desired_pre << 0, 0, 0, 0, 0, 0, -0.45, -0.65, 0.35, -1.5, -1, 1.1, -0.4, 0.45, 0.65, -0.35, 1.5, 1, -1.1, 0.4;
    measurement.velocity = Eigen::VectorXd::Zero(20);

    auto move_trajectory_planner_ = std::make_shared<MoveTrajectoryPlanner>(robot_model_, manager_dt, false);
    auto trajectory_ok = move_trajectory_planner_->ResetForNewTrajectory(request, measurement, TimeMetaInfo());
    EXPECT_TRUE(trajectory_ok);
}
