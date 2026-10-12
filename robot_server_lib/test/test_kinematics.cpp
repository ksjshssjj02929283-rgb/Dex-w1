#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <chrono>
#include <filesystem>

#include "NanoLog.hpp"
#include "inverse_dynamics.h"
#include "kinematics_include.h"
#include "safety_check.h"
#include "utils.h"

using namespace robot_server_lib;
TEST(SRSKinematicsTest, Kinematics) {
    kinematics::SRSKinematics srs_kine_left("W1V024Left");

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

class FEPKinematicsTest : public ::testing::TestWithParam<const char*> {};

TEST_P(FEPKinematicsTest, FKIKRoundTrip) {
    kinematics::FEPKinematics fep_kine(GetParam());
    const bool is_left = std::string(GetParam()).find("Left") != std::string::npos;
    Eigen::VectorXd q_in(7);
    if (is_left) {
        q_in << 0.5, -1.2, 0.2, -1.0, 0.3, -0.5, -0.5;
    } else {
        q_in << -0.5, 1.2, -0.2, 1.0, -0.3, 0.5, 0.5;
    }
    const auto [qmin, qmax] = fep_kine.GetQposLimits();
    ASSERT_EQ(qmin.size(), 7);
    ASSERT_EQ(qmax.size(), 7);
    for (int i = 0; i < q_in.size(); ++i) {
        ASSERT_GE(q_in[i], qmin[i]);
        ASSERT_LE(q_in[i], qmax[i]);
    }

    // 同时验证默认 TCP 和含平移、旋转的 TCP。
    for (bool use_tcp : {false, true}) {
        SCOPED_TRACE(use_tcp ? "With TCP" : "Identity TCP");
        pinocchio::SE3 tcp = pinocchio::SE3::Identity();
        if (use_tcp) {
            tcp = pinocchio::SE3(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()).toRotationMatrix(),
                                 Eigen::Vector3d(0.03, -0.02, 0.08));
        }
        const auto pose = fep_kine.GetFK(q_in, tcp);
        ASSERT_TRUE(pose.position.allFinite());
        ASSERT_TRUE(pose.orientation.coeffs().allFinite());

        // 独立检查 TCP 是否正确应用，避免 FK/IK 同时忽略 TCP 而往返仍通过。
        const auto flange_pose = fep_kine.GetFK(q_in);
        const pinocchio::SE3 flange(flange_pose.orientation.toRotationMatrix(), flange_pose.position);
        const auto expected = flange * tcp;
        EXPECT_NEAR((pose.position - expected.translation()).norm(), 0.0, 1e-8);
        EXPECT_NEAR(pose.orientation.angularDistance(Eigen::Quaterniond(expected.rotation())), 0.0, 1e-8);

        auto q_out = q_in;
        // 使用稍有偏移的 seed，验证求解器确实恢复目标位姿。
        q_out[0] += 0.02;
        ASSERT_TRUE(fep_kine.GetIK(pose, q_out, tcp)) << GetParam();
        ASSERT_TRUE(q_out.allFinite());
        for (int i = 0; i < q_out.size(); ++i) {
            EXPECT_GE(q_out[i], qmin[i] - 1e-8);
            EXPECT_LE(q_out[i], qmax[i] + 1e-8);
        }
        const auto recovered = fep_kine.GetFK(q_out, tcp);
        EXPECT_NEAR((recovered.position - pose.position).norm(), 0.0, 1e-6);
        EXPECT_NEAR(recovered.orientation.angularDistance(pose.orientation), 0.0, 1e-6);
    }
}

INSTANTIATE_TEST_SUITE_P(W1Models,
                         FEPKinematicsTest,
                         ::testing::Values("W1V030Left", "W1V030Right", "W1V031Left", "W1V031Right"));

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

    std::array<ErrorCode, ndof_robot> error_codes{};

    SafetyCheckConfig safety_config;
    safety_config.check_position_limit = true;
    safety_config.check_velocity_limit = true;
    safety_config.check_torque_limit = true;
    SafetyChecker safety_checker;
    bool result = safety_checker.Check(measurement, robot_model, error_codes, safety_config);
    EXPECT_TRUE(result);
    for (auto code : error_codes) {
        EXPECT_EQ(code, ErrorCode::None);
    }
    std::cout << "CheckSafety with zero measurement: passed" << std::endl;
}

int main(int argc, char** argv) {
    const auto log_dir = std::filesystem::temp_directory_path() / "dexe_test_kinematics";
    std::filesystem::create_directories(log_dir);
    nanolog::initialize(nanolog::GuaranteedLogger(), log_dir.string() + "/", "test_kinematics", 1, 1);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
