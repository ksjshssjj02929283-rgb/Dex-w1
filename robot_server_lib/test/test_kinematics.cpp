#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <chrono>

#include "kinematics_include.h"
#include "utils.h"

using namespace robot_server_lib;
TEST(SRSKinematicsTest, Kinematics)
{
    kinematics::SRSKinematics srs_kine_left(true, false);

    Eigen::VectorXd q_in_left(7);
    q_in_left << -0.45, -0.65, 0.35, -1.5, -1, 1.1, -0.4;
    auto pose = srs_kine_left.GetFK(q_in_left);
    std::cout << "Left: Forward Kinematics Left Pose_out: " << pose.ToString() << std::endl;
    bool success = false;
    auto pose_in = pose;
    auto q_in_and_out = q_in_left;
    // 打印耗时
    auto start_time = std::chrono::high_resolution_clock::now();
    success = srs_kine_left.GetIK(pose_in, q_in_and_out);
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> duration = end_time - start_time;
    std::cout << "Left: Inverse Kinematics q_out: " << q_in_and_out.transpose() << std::endl;
    std::cout << "Left: Inverse Kinematics duration: " << duration.count() / 1000.0 << " ms" << std::endl;
    EXPECT_TRUE(success) << "Inverse Kinematics failed!";
    for (int i = 0; i < q_in_left.size(); ++i)
    {
        EXPECT_NEAR(q_in_left[i], q_in_and_out[i], 0.01) << "Joint " << i << "fk ik mismatch";
    }
}
