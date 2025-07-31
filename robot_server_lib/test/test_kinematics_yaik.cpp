#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <chrono>

#include "kinematics_include.h"
#include "utilities.h"

using namespace robot_server_lib;

TEST(KinematicsTest, YaikKinematicsTest)
{
    kinematics::YaikKinematics kine_left(0);
    kinematics::YaikKinematics kine_right(1);

    Eigen::VectorXd pose0(7), pose1(7), q0(7), q1(7);
    q0 << -0.45, -0.65, 0.35, -1.5, -1, 1.1, -0.4;
    q1 << 0.45, 0.65, -0.35, 1.5, 1, -1.1, 0.4;
    auto pose = kine_left.GetFK(q0).ToVectorXd();
    Eigen::VectorXd pose_right_ref(7);
    pose0 << -0.163635, 0.363727, 0.262755, -0.229399, -0.440151, 0.599072, 0.628296;
    auto pose_right = kine_right.GetFK(q1).ToVectorXd();
    pose_right_ref << 0.163635, 0.363727, 0.262795, 0.628296, -0.599072, 0.440151, -0.229399;
    for (size_t i = 0; i < pose0.size(); ++i)
    {
        EXPECT_NEAR(pose[i], pose0[i], 0.01) << "FK mismatch at index " << i;
        EXPECT_NEAR(pose_right[i], pose_right_ref[i], 0.01) << "FK mismatch at index " << i;
    }
    pose1 << -0.3, 0.363727, 0.262795, -0.229399, -0.440151, 0.599072, 0.628296;

    auto pose_trajectory = PoseTrajectory::FromPoses({PoseQuaternion::FromVector7d(pose0), PoseQuaternion::FromVector7d(pose1)});

    std::vector<Eigen::Matrix<double, 7, 1>> joint_trajectory;
    auto trajectory_ok = kine_left.RunGlobalRedundancyResolutionForJointTrajectory(q0, pose_trajectory, joint_trajectory, pinocchio::SE3::Identity());
    EXPECT_TRUE(trajectory_ok) << "Trajectory planning failed!";
}
