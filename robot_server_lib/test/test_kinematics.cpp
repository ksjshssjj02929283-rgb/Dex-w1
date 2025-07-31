#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <chrono>

#include "kinematics_include.h"
#include "utilities.h"

using namespace robot_server_lib;

// int main(int argc, char **argv)
TEST(KinematicsTest, Compare)
{
    std::filesystem::create_directories("log/");
    nanolog::initialize(nanolog::GuaranteedLogger(), "log/", "robot_server", 10);
    nanolog::set_log_level(nanolog::LogLevel::INFO);

    kinematics::SRSKinematics srs = kinematics::SRSKinematics(kinematics::W1LeftArmIndustrial());
    kinematics::YaikKinematics yaik(0);

    Eigen::VectorXd q(7), pose(7);
    q << -0.45, -0.65, 0.35, -1.5, -1, 1.1, -0.4;
    pose << -0.163635, 0.363727, 0.262755, -0.229399, -0.440151, 0.599072, 0.628296;
    auto poseQ = PoseQuaternion::FromVector7d(pose);
    poseQ.position(0) += 0.3;

    auto start_time0 = std::chrono::high_resolution_clock::now();
    auto pose0 = srs.GetFK(q);
    auto end_time0 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> time0 = end_time0 - start_time0;

    auto start_time1 = std::chrono::high_resolution_clock::now();
    auto pose1 = yaik.GetFK(q);
    auto end_time1 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> time1 = end_time1 - start_time1;

    Eigen::VectorXd q2(q);
    auto start_time2 = std::chrono::high_resolution_clock::now();
    auto ok2 = srs.GetIK(poseQ, q2);
    auto end_time2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> time2 = end_time2 - start_time2;

    Eigen::VectorXd q3(q);
    auto start_time3 = std::chrono::high_resolution_clock::now();
    auto ok3 = yaik.GetIK(poseQ, q3);
    auto end_time3 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> time3 = end_time3 - start_time3;

    std::cout << "FK results:\n"
              << "\tSRS :  " << pose0.ToString() << "  Time: " << time0.count() / 1000.0 << "ms\n"
              << "\tYAIK:  " << pose1.ToString() << "  Time: " << time1.count() / 1000.0 << "ms\n"
              << "IK results:\n"
              << "\tSRS :  Pose: " << q2.transpose() << "  Time: " << time2.count() / 1000.0 << "ms\n"
              << "\tYAIK:  Pose: " << q3.transpose() << "  Time: " << time3.count() / 1000.0 << "ms\n";
}
