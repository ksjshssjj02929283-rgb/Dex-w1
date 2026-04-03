#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <vector>

#include "parallel_ankle_calc.h"

using namespace robot_server_lib::utils;

std::vector<std::pair<double, double>> test_angles = {
    {0.1, 0.0},
    {0.0, 0.1},
};

TEST(PMAnkleTest, InverseKinematics) {
    for (bool is_left : {true, false}) {
        for (const auto& [q_pitch, q_roll] : test_angles) {
            PMAnkle ankle(is_left);

            auto solution = ankle.GetAnkleMotorAngles(q_pitch, q_roll);

            EXPECT_TRUE(solution.success_1) << "IK solution 1 failed, is_left=" << is_left;
            EXPECT_TRUE(solution.success_2) << "IK solution 2 failed, is_left=" << is_left;

            std::cout << "IK: is_left=" << is_left << ", q_pitch=" << q_pitch << ", q_roll=" << q_roll << std::endl;
            std::cout << "IK: theta_1=" << solution.theta_1_rad << " rad (" << solution.theta_1_rad * 180.0 / M_PI
                      << " deg)" << std::endl;
            std::cout << "IK: theta_2=" << solution.theta_2_rad << " rad (" << solution.theta_2_rad * 180.0 / M_PI
                      << " deg) \n"
                      << std::endl;
        }
    }
}

TEST(PMAnkleTest, ForwardKinematics) {
    for (bool is_left : {true, false}) {
        for (const auto& [q_pitch, q_roll] : test_angles) {
            PMAnkle ankle(is_left);

            auto ik_solution = ankle.GetAnkleMotorAngles(q_pitch, q_roll);
            ASSERT_TRUE(ik_solution.success_1) << "IK solution 1 failed, is_left=" << is_left;
            ASSERT_TRUE(ik_solution.success_2) << "IK solution 2 failed, is_left=" << is_left;

            auto fk_result = ankle.GetAnklePoseFromMotorAngles(ik_solution.theta_1_rad, ik_solution.theta_2_rad);

            std::cout << "FK: is_left=" << is_left << ", q_pitch=" << q_pitch << ", q_roll=" << q_roll << std::endl;
            std::cout << "FK: theta_1=" << ik_solution.theta_1_rad << " rad, theta_2=" << ik_solution.theta_2_rad
                      << " rad" << std::endl;
            std::cout << "FK: recovered q_pitch=" << fk_result(0) << " rad (" << fk_result(0) * 180.0 / M_PI << " deg)"
                      << std::endl;
            std::cout << "FK: recovered q_roll=" << fk_result(1) << " rad (" << fk_result(1) * 180.0 / M_PI
                      << " deg) \n"
                      << std::endl;

            EXPECT_NEAR(fk_result(0), q_pitch, 1e-6) << "FK IK mismatch for pitch, is_left=" << is_left;
            EXPECT_NEAR(fk_result(1), q_roll, 1e-6) << "FK IK mismatch for roll, is_left=" << is_left;
        }
    }
}

TEST(PMAnkleTest, JacobianComputation) {
    for (bool is_left : {true, false}) {
        for (const auto& [q_pitch, q_roll] : test_angles) {
            PMAnkle ankle(is_left);

            auto J = ankle.ComputeJacobian(q_pitch, q_roll);

            std::cout << "Jacobian at is_left=" << is_left << ", q_pitch=" << q_pitch << ", q_roll=" << q_roll
                      << std::endl;
            std::cout << "J = " << std::endl << J << std::endl;

            double det = J.determinant();
            std::cout << "Jacobian determinant = " << det << "\n" << std::endl;

            if (std::abs(det) < 1e-6) {
                std::cout << "Warning: Near singular configuration" << std::endl;
            }
        }
    }
}

TEST(PMAnkleTest, IKFkSpeed) {
    for (bool is_left : {true, false}) {
        PMAnkle ankle(is_left);
        double q_pitch = 0.1;
        double q_roll = 0.0;

        auto start_ik = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < 1000; ++i) {
            ankle.GetAnkleMotorAngles(q_pitch, q_roll);
        }
        auto end_ik = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::micro> ik_duration = end_ik - start_ik;
        std::cout << "IK 1000 iterations (is_left=" << is_left << "): " << ik_duration.count() / 1000.0
                  << " us per call" << std::endl;

        auto ik_solution = ankle.GetAnkleMotorAngles(q_pitch, q_roll);

        auto start_fk = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < 1000; ++i) {
            ankle.GetAnklePoseFromMotorAngles(ik_solution.theta_1_rad, ik_solution.theta_2_rad);
        }
        auto end_fk = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::micro> fk_duration = end_fk - start_fk;
        std::cout << "FK 1000 iterations (is_left=" << is_left << "): " << fk_duration.count() / 1000.0
                  << " us per call" << std::endl;
    }
}

TEST(PMAnkleTest, MotorToTaskTorqueConversion) {
    for (bool is_left : {true, false}) {
        for (const auto& [q_pitch, q_roll] : test_angles) {
            PMAnkle ankle(is_left);
            double tau_motor_1 = 10.0;
            double tau_motor_2 = 8.0;

            auto tau_task = ankle.GetTaskTorquesFromMotorTorques(q_pitch, q_roll, tau_motor_1, tau_motor_2);

            std::cout << "Motor to Task Torque Conversion (is_left=" << is_left << "):" << std::endl;
            std::cout << "Input: q_pitch=" << q_pitch << ", q_roll=" << q_roll << std::endl;
            std::cout << "Input: tau_motor_1=" << tau_motor_1 << ", tau_motor_2=" << tau_motor_2 << std::endl;
            std::cout << "Output: tau_pitch=" << tau_task(0) << ", tau_roll=" << tau_task(1) << std::endl;

            EXPECT_TRUE(std::isfinite(tau_task(0))) << "tau_pitch is not finite, is_left=" << is_left;
            EXPECT_TRUE(std::isfinite(tau_task(1))) << "tau_roll is not finite, is_left=" << is_left;
        }
    }
}

TEST(PMAnkleTest, MotorToTaskVelocityConversion) {
    for (bool is_left : {true, false}) {
        for (const auto& [q_pitch, q_roll] : test_angles) {
            PMAnkle ankle(is_left);
            double vel_motor_1 = 1.0;
            double vel_motor_2 = 0.8;

            auto vel_task = ankle.GetTaskVelocitiesFromMotorVelocities(q_pitch, q_roll, vel_motor_1, vel_motor_2);

            std::cout << "Motor to Task Velocity Conversion (is_left=" << is_left << "):" << std::endl;
            std::cout << "Input: q_pitch=" << q_pitch << ", q_roll=" << q_roll << std::endl;
            std::cout << "Input: vel_motor_1=" << vel_motor_1 << ", vel_motor_2=" << vel_motor_2 << std::endl;
            std::cout << "Output: vel_pitch=" << vel_task(0) << ", vel_roll=" << vel_task(1) << std::endl;

            EXPECT_TRUE(std::isfinite(vel_task(0))) << "vel_pitch is not finite, is_left=" << is_left;
            EXPECT_TRUE(std::isfinite(vel_task(1))) << "vel_roll is not finite, is_left=" << is_left;
        }
    }
}

TEST(PMAnkleTest, MotorStiffnessFromTaskStiffness) {
    for (bool is_left : {true, false}) {
        for (const auto& [q_pitch, q_roll] : test_angles) {
            PMAnkle ankle(is_left);
            double k_pitch = 100.0;
            double k_roll = 80.0;

            auto k_motor = ankle.GetMotorStiffnessFromTaskStiffness(q_pitch, q_roll, k_pitch, k_roll);

            std::cout << "Motor Stiffness from Task Stiffness (is_left=" << is_left << "):" << std::endl;
            std::cout << "Input: q_pitch=" << q_pitch << ", q_roll=" << q_roll << std::endl;
            std::cout << "Input: k_pitch=" << k_pitch << ", k_roll=" << k_roll << std::endl;
            std::cout << "Output: k_motor_1=" << k_motor(0) << ", k_motor_2=" << k_motor(1) << std::endl;

            EXPECT_TRUE(std::isfinite(k_motor(0))) << "k_motor_1 is not finite, is_left=" << is_left;
            EXPECT_TRUE(std::isfinite(k_motor(1))) << "k_motor_2 is not finite, is_left=" << is_left;
            EXPECT_GT(k_motor(0), 0.0) << "k_motor_1 should be positive, is_left=" << is_left;
            EXPECT_GT(k_motor(1), 0.0) << "k_motor_2 should be positive, is_left=" << is_left;
        }
    }
}

TEST(PMAnkleTest, MotorDampingFromTaskDamping) {
    for (bool is_left : {true, false}) {
        for (const auto& [q_pitch, q_roll] : test_angles) {
            PMAnkle ankle(is_left);
            double d_pitch = 10.0;
            double d_roll = 8.0;

            auto d_motor = ankle.GetMotorDampingFromTaskDamping(q_pitch, q_roll, d_pitch, d_roll);

            std::cout << "Motor Damping from Task Damping (is_left=" << is_left << "):" << std::endl;
            std::cout << "Input: q_pitch=" << q_pitch << ", q_roll=" << q_roll << std::endl;
            std::cout << "Input: d_pitch=" << d_pitch << ", d_roll=" << d_roll << std::endl;
            std::cout << "Output: d_motor_1=" << d_motor(0) << ", d_motor_2=" << d_motor(1) << std::endl;

            EXPECT_TRUE(std::isfinite(d_motor(0))) << "d_motor_1 is not finite, is_left=" << is_left;
            EXPECT_TRUE(std::isfinite(d_motor(1))) << "d_motor_2 is not finite, is_left=" << is_left;
            EXPECT_GT(d_motor(0), 0.0) << "d_motor_1 should be positive, is_left=" << is_left;
            EXPECT_GT(d_motor(1), 0.0) << "d_motor_2 should be positive, is_left=" << is_left;
        }
    }
}
