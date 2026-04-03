#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <vector>

#include "parallel_ankle_calc.h"

using namespace robot_server_lib::utils;

TEST(PMAnkleTest, InverseKinematics) {
    PMAnkle ankle;
    ankle.PMAnkleInit();

    double q_pitch = 0.1;
    double q_roll = 0.1;

    auto solution = ankle.GetAnkleMotorAngles(q_pitch, q_roll);

    EXPECT_TRUE(solution.success_1) << "IK solution 1 failed";
    EXPECT_TRUE(solution.success_2) << "IK solution 2 failed";

    std::cout << "IK: q_pitch=" << q_pitch << ", q_roll=" << q_roll << std::endl;
    std::cout << "IK: theta_1=" << solution.theta_1_rad << " rad (" << solution.theta_1_rad * 180.0 / M_PI << " deg)"
              << std::endl;
    std::cout << "IK: theta_2=" << solution.theta_2_rad << " rad (" << solution.theta_2_rad * 180.0 / M_PI << " deg)"
              << std::endl;
}

TEST(PMAnkleTest, ForwardKinematics) {
    PMAnkle ankle;
    ankle.PMAnkleInit();

    double q_pitch = 0.1;
    double q_roll = 0.1;

    auto ik_solution = ankle.GetAnkleMotorAngles(q_pitch, q_roll);
    ASSERT_TRUE(ik_solution.success_1) << "IK solution 1 failed";
    ASSERT_TRUE(ik_solution.success_2) << "IK solution 2 failed";

    auto fk_result = ankle.GetAnklePoseFromMotorAngles(ik_solution.theta_1_rad, ik_solution.theta_2_rad);

    std::cout << "FK: theta_1=" << ik_solution.theta_1_rad << " rad, theta_2=" << ik_solution.theta_2_rad << " rad"
              << std::endl;
    std::cout << "FK: recovered q_pitch=" << fk_result(0) << " rad (" << fk_result(0) * 180.0 / M_PI << " deg)"
              << std::endl;
    std::cout << "FK: recovered q_roll=" << fk_result(1) << " rad (" << fk_result(1) * 180.0 / M_PI << " deg)"
              << std::endl;

    EXPECT_NEAR(fk_result(0), q_pitch, 1e-6) << "FK IK mismatch for pitch";
    EXPECT_NEAR(fk_result(1), q_roll, 1e-6) << "FK IK mismatch for roll";
}

TEST(PMAnkleTest, ForwardKinematicsVariousAngles) {
    PMAnkle ankle;
    ankle.PMAnkleInit();

    std::vector<std::pair<double, double>> test_angles = {
        {0.0, 0.0}, {0.1, 0.0}, {0.0, 0.1}, {-0.1, 0.0}, {0.0, -0.1}, {0.05, 0.08}, {-0.03, 0.06},
    };

    for (const auto& [q_pitch, q_roll] : test_angles) {
        auto ik_solution = ankle.GetAnkleMotorAngles(q_pitch, q_roll);

        if (!ik_solution.success_1 || !ik_solution.success_2) {
            std::cout << "IK failed for q_pitch=" << q_pitch << ", q_roll=" << q_roll << std::endl;
            continue;
        }

        auto fk_result = ankle.GetAnklePoseFromMotorAngles(ik_solution.theta_1_rad, ik_solution.theta_2_rad);

        double pitch_err = std::abs(fk_result(0) - q_pitch);
        double roll_err = std::abs(fk_result(1) - q_roll);

        EXPECT_NEAR(fk_result(0), q_pitch, 1e-5) << "FK IK mismatch for pitch=" << q_pitch;
        EXPECT_NEAR(fk_result(1), q_roll, 1e-5) << "FK IK mismatch for roll=" << q_roll;

        std::cout << "Test q_pitch=" << q_pitch << ", q_roll=" << q_roll << " -> pitch_err=" << pitch_err
                  << ", roll_err=" << roll_err << std::endl;
    }
}

TEST(PMAnkleTest, JacobianComputation) {
    PMAnkle ankle;
    ankle.PMAnkleInit();

    double q_pitch = 0.1;
    double q_roll = 0.1;

    auto J = ComputeAnkleJacobian(q_pitch, q_roll, ankle.params_1, ankle.params_2);

    std::cout << "Jacobian at q_pitch=" << q_pitch << ", q_roll=" << q_roll << std::endl;
    std::cout << "J = " << std::endl << J << std::endl;

    double det = J.determinant();
    std::cout << "Jacobian determinant = " << det << std::endl;

    if (std::abs(det) < 1e-6) {
        std::cout << "Warning: Near singular configuration" << std::endl;
    }
}

TEST(PMAnkleTest, IKFkSpeed) {
    PMAnkle ankle;
    ankle.PMAnkleInit();

    double q_pitch = 0.1;
    double q_roll = 0.1;

    auto start_ik = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 1000; ++i) {
        ankle.GetAnkleMotorAngles(q_pitch, q_roll);
    }
    auto end_ik = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> ik_duration = end_ik - start_ik;
    std::cout << "IK 1000 iterations: " << ik_duration.count() / 1000.0 << " us per call" << std::endl;

    auto ik_solution = ankle.GetAnkleMotorAngles(q_pitch, q_roll);

    auto start_fk = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 1000; ++i) {
        ankle.GetAnklePoseFromMotorAngles(ik_solution.theta_1_rad, ik_solution.theta_2_rad);
    }
    auto end_fk = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> fk_duration = end_fk - start_fk;
    std::cout << "FK 1000 iterations: " << fk_duration.count() / 1000.0 << " us per call" << std::endl;
}

TEST(PMAnkleTest, MotorToTaskTorqueConversion) {
    PMAnkle ankle;
    ankle.PMAnkleInit();

    double q_pitch = 0.1;
    double q_roll = 0.1;
    double tau_motor_1 = 10.0;
    double tau_motor_2 = 8.0;

    auto tau_task = ankle.GetTaskTorquesFromMotorTorques(q_pitch, q_roll, tau_motor_1, tau_motor_2);

    std::cout << "Motor to Task Torque Conversion:" << std::endl;
    std::cout << "Input: q_pitch=" << q_pitch << ", q_roll=" << q_roll << std::endl;
    std::cout << "Input: tau_motor_1=" << tau_motor_1 << ", tau_motor_2=" << tau_motor_2 << std::endl;
    std::cout << "Output: tau_pitch=" << tau_task(0) << ", tau_roll=" << tau_task(1) << std::endl;

    EXPECT_TRUE(std::isfinite(tau_task(0))) << "tau_pitch is not finite";
    EXPECT_TRUE(std::isfinite(tau_task(1))) << "tau_roll is not finite";
}

TEST(PMAnkleTest, MotorToTaskVelocityConversion) {
    PMAnkle ankle;
    ankle.PMAnkleInit();

    double q_pitch = 0.1;
    double q_roll = 0.1;
    double vel_motor_1 = 1.0;
    double vel_motor_2 = 0.8;

    auto vel_task = ankle.GetTaskVelocitiesFromMotorVelocities(q_pitch, q_roll, vel_motor_1, vel_motor_2);

    std::cout << "Motor to Task Velocity Conversion:" << std::endl;
    std::cout << "Input: q_pitch=" << q_pitch << ", q_roll=" << q_roll << std::endl;
    std::cout << "Input: vel_motor_1=" << vel_motor_1 << ", vel_motor_2=" << vel_motor_2 << std::endl;
    std::cout << "Output: vel_pitch=" << vel_task(0) << ", vel_roll=" << vel_task(1) << std::endl;

    EXPECT_TRUE(std::isfinite(vel_task(0))) << "vel_pitch is not finite";
    EXPECT_TRUE(std::isfinite(vel_task(1))) << "vel_roll is not finite";
}

TEST(PMAnkleTest, MotorStiffnessFromTaskStiffness) {
    PMAnkle ankle;
    ankle.PMAnkleInit();

    double q_pitch = 0.1;
    double q_roll = 0.1;
    double k_pitch = 100.0;
    double k_roll = 80.0;

    auto k_motor = ankle.GetMotorStiffnessFromTaskStiffness(q_pitch, q_roll, k_pitch, k_roll);

    std::cout << "Motor Stiffness from Task Stiffness:" << std::endl;
    std::cout << "Input: q_pitch=" << q_pitch << ", q_roll=" << q_roll << std::endl;
    std::cout << "Input: k_pitch=" << k_pitch << ", k_roll=" << k_roll << std::endl;
    std::cout << "Output: k_motor_1=" << k_motor(0) << ", k_motor_2=" << k_motor(1) << std::endl;

    EXPECT_TRUE(std::isfinite(k_motor(0))) << "k_motor_1 is not finite";
    EXPECT_TRUE(std::isfinite(k_motor(1))) << "k_motor_2 is not finite";
    EXPECT_GT(k_motor(0), 0.0) << "k_motor_1 should be positive";
    EXPECT_GT(k_motor(1), 0.0) << "k_motor_2 should be positive";
}

TEST(PMAnkleTest, MotorDampingFromTaskDamping) {
    PMAnkle ankle;
    ankle.PMAnkleInit();

    double q_pitch = 0.1;
    double q_roll = 0.1;
    double d_pitch = 10.0;
    double d_roll = 8.0;

    auto d_motor = ankle.GetMotorDampingFromTaskDamping(q_pitch, q_roll, d_pitch, d_roll);

    std::cout << "Motor Damping from Task Damping:" << std::endl;
    std::cout << "Input: q_pitch=" << q_pitch << ", q_roll=" << q_roll << std::endl;
    std::cout << "Input: d_pitch=" << d_pitch << ", d_roll=" << d_roll << std::endl;
    std::cout << "Output: d_motor_1=" << d_motor(0) << ", d_motor_2=" << d_motor(1) << std::endl;

    EXPECT_TRUE(std::isfinite(d_motor(0))) << "d_motor_1 is not finite";
    EXPECT_TRUE(std::isfinite(d_motor(1))) << "d_motor_2 is not finite";
    EXPECT_GT(d_motor(0), 0.0) << "d_motor_1 should be positive";
    EXPECT_GT(d_motor(1), 0.0) << "d_motor_2 should be positive";
}

TEST(PMAnkleTest, StiffnessDampingConversionVariousAngles) {
    PMAnkle ankle;
    ankle.PMAnkleInit();

    std::vector<std::pair<double, double>> test_angles = {
        {0.0, 0.0}, {0.1, 0.0}, {0.0, 0.1}, {-0.1, 0.0}, {0.0, -0.1}, {0.05, 0.08},
    };

    double k_pitch = 100.0;
    double k_roll = 80.0;
    double d_pitch = 10.0;
    double d_roll = 8.0;

    for (const auto& [q_pitch, q_roll] : test_angles) {
        auto k_motor = ankle.GetMotorStiffnessFromTaskStiffness(q_pitch, q_roll, k_pitch, k_roll);
        auto d_motor = ankle.GetMotorDampingFromTaskDamping(q_pitch, q_roll, d_pitch, d_roll);

        std::cout << "Angle q_pitch=" << q_pitch << ", q_roll=" << q_roll << " -> k_motor_1=" << k_motor(0)
                  << ", k_motor_2=" << k_motor(1) << ", d_motor_1=" << d_motor(0) << ", d_motor_2=" << d_motor(1)
                  << std::endl;

        EXPECT_TRUE(std::isfinite(k_motor(0))) << "k_motor_1 is not finite at q_pitch=" << q_pitch;
        EXPECT_TRUE(std::isfinite(k_motor(1))) << "k_motor_2 is not finite at q_roll=" << q_roll;
        EXPECT_TRUE(std::isfinite(d_motor(0))) << "d_motor_1 is not finite at q_pitch=" << q_pitch;
        EXPECT_TRUE(std::isfinite(d_motor(1))) << "d_motor_2 is not finite at q_roll=" << q_roll;
    }
}
