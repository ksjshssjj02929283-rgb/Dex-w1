#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <chrono>

#include "inverse_dynamics.h"
#include "kinematics_include.h"
#include "safety_check.h"
#include "utils.h"

using namespace robot_server_lib;
TEST(SRSKinematicsTest, Kinematics) {
    kinematics::SRSKinematics srs_kine_left(true, true);

    Eigen::VectorXd q_in_left(7);
    q_in_left << 0.5, -1.2, 0.0, -1.0, 0.0, -0.5, -0.5;
    auto pose = srs_kine_left.GetFK(q_in_left);
    std::cout << "Left: Forward Kinematics Left q_in: " << q_in_left.transpose() << std::endl;
    std::cout << "Left: Forward Kinematics Left Pose_out: " << pose.ToString() << std::endl;
    bool success = false;
    auto pose_in = pose;
    auto q_in_and_out = q_in_left;
    auto start_time = std::chrono::high_resolution_clock::now();
    success = srs_kine_left.GetIK(pose_in, q_in_and_out);
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> duration = end_time - start_time;
    std::cout << "Left: Inverse Kinematics q_out: " << q_in_and_out.transpose() << std::endl;
    std::cout << "Left: Inverse Kinematics duration: " << duration.count() / 1000.0 << " ms" << std::endl;
    EXPECT_TRUE(success) << "Inverse Kinematics failed!";
    for (int i = 0; i < q_in_left.size(); ++i) {
        EXPECT_NEAR(q_in_left[i], q_in_and_out[i], 0.01) << "Joint " << i << "fk ik mismatch";
    }
}

TEST(SRSKinematicsTest, GetAllJointLimitsFromURDF) {
    const auto package_share_path = std::filesystem::path(std::getenv("HOME")) / "dexe_robot_server_data";
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot_config.yaml";
    RobotModel robot_model = utils::GetRobotModelFromConfig(robot_cfg_path);

    const auto& model = *robot_model.model_ptr;

    Eigen::VectorXd lb = model.lowerPositionLimit;
    Eigen::VectorXd ub = model.upperPositionLimit;

    std::cout << "All Joint Limits (URDF order, " << ndof_robot << " joints):" << std::endl;
    for (int i = 0; i < ndof_robot; ++i) {
        std::cout << "  Joint " << i << " (" << model.names[i + 1] << "): [" << lb[i] << ", " << ub[i] << "]"
                  << std::endl;
    }

    EXPECT_EQ(lb.size(), ndof_robot);
    EXPECT_EQ(ub.size(), ndof_robot);

    for (int i = 0; i < ndof_robot; ++i) {
        EXPECT_LE(lb[i], ub[i]) << "Joint " << i << " (" << model.names[i + 1] << ") lower > upper";
    }
}

TEST(SRSKinematicsTest, CheckSafety) {
    const auto package_share_path = std::filesystem::path(std::getenv("HOME")) / "dexe_robot_server_data";
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot_config.yaml";
    RobotModel robot_model = utils::GetRobotModelFromConfig(robot_cfg_path);

    JointMeasurement measurement;
    measurement.position = Eigen::VectorXd::Zero(ndof_robot);
    measurement.velocity = Eigen::VectorXd::Zero(ndof_robot);
    measurement.effort = Eigen::VectorXd::Zero(ndof_robot);

    ErrorCode error_code = ErrorCode::NoError;

    SafetyCheckConfig safety_config;
    safety_config.check_position_limit = true;
    safety_config.check_velocity_limit = true;
    safety_config.check_torque_limit = true;
    bool result = CheckSafety(measurement, robot_model, error_code, safety_config);
    EXPECT_TRUE(result);
    EXPECT_EQ(error_code, ErrorCode::NoError);
    std::cout << "CheckSafety with zero measurement: passed" << std::endl;
}
