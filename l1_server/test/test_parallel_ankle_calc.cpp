#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <vector>

#include "parallel_ankle_calc.h"
#include "utils.h"

using namespace robot_server_lib::utils;

// 构造测试用踝关节几何参数（与 l1_config.yaml 默认值一致）
robot_server_lib::AnkleParams MakeAnkleParams(bool is_left) {
    robot_server_lib::AnkleParams p;
    p.r_roll_center = {0.0, 0.0, 0.0};
    p.r_pitch_center = {0.0, 0.0, 10.0};
    if (is_left) {
        p.r_a1 = {-46.0, 0.0, 257.94};
        p.r_c1 = {-46.0, -59.0, 0.0};
        p.l_bar1 = 60.0;
        p.l_rod1 = 268.354;
        p.alpha_1_rad = 0.1745;
        p.r_a2 = {-46.0, 0.0, 177.94};
        p.r_c2 = {-46.0, 59.0, 0.0};
        p.l_bar2 = 60.0;
        p.l_rod2 = 188.355;
        p.alpha_2_rad = -0.1745;
    } else {
        p.r_a1 = {-46.0, 0.0, 257.94};
        p.r_c1 = {-46.0, 59.0, 0.0};
        p.l_bar1 = 60.0;
        p.l_rod1 = 268.354;
        p.alpha_1_rad = -0.1745;
        p.r_a2 = {-46.0, 0.0, 177.94};
        p.r_c2 = {-46.0, -59.0, 0.0};
        p.l_bar2 = 60.0;
        p.l_rod2 = 188.355;
        p.alpha_2_rad = 0.1745;
    }
    return p;
}

std::vector<std::pair<double, double>> test_angles = {
    {0.1, 0.0},
    {0.0, 0.1},
};

TEST(PMAnkleTest, InverseKinematics) {
    for (bool is_left : {true, false}) {
        for (const auto& [q_pitch, q_roll] : test_angles) {
            PMAnkle ankle(is_left, MakeAnkleParams(is_left));

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
            PMAnkle ankle(is_left, MakeAnkleParams(is_left));

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
            PMAnkle ankle(is_left, MakeAnkleParams(is_left));

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
        PMAnkle ankle(is_left, MakeAnkleParams(is_left));
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
            PMAnkle ankle(is_left, MakeAnkleParams(is_left));
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
            PMAnkle ankle(is_left, MakeAnkleParams(is_left));
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
            PMAnkle ankle(is_left, MakeAnkleParams(is_left));
            double k_pitch = 100.0;
            double k_roll = 80.0;

            auto k_motor = ankle.GetMotorStiffnessMatrixFromTaskStiffness(q_pitch, q_roll, k_pitch, k_roll);

            std::cout << "Motor Stiffness Matrix from Task Stiffness (is_left=" << is_left << "):" << std::endl;
            std::cout << "Input: q_pitch=" << q_pitch << ", q_roll=" << q_roll << std::endl;
            std::cout << "Input: k_pitch=" << k_pitch << ", k_roll=" << k_roll << std::endl;
            std::cout << "Output matrix:\n" << k_motor << std::endl;

            EXPECT_TRUE(std::isfinite(k_motor(0, 0))) << "k_motor(0,0) is not finite, is_left=" << is_left;
            EXPECT_TRUE(std::isfinite(k_motor(0, 1))) << "k_motor(0,1) is not finite, is_left=" << is_left;
            EXPECT_TRUE(std::isfinite(k_motor(1, 0))) << "k_motor(1,0) is not finite, is_left=" << is_left;
            EXPECT_TRUE(std::isfinite(k_motor(1, 1))) << "k_motor(1,1) is not finite, is_left=" << is_left;
            EXPECT_GT(k_motor(0, 0), 0.0) << "k_motor(0,0) should be positive, is_left=" << is_left;
            EXPECT_GT(k_motor(1, 1), 0.0) << "k_motor(1,1) should be positive, is_left=" << is_left;
        }
    }
}

TEST(PMAnkleTest, MotorDampingFromTaskDamping) {
    for (bool is_left : {true, false}) {
        for (const auto& [q_pitch, q_roll] : test_angles) {
            PMAnkle ankle(is_left, MakeAnkleParams(is_left));
            double d_pitch = 10.0;
            double d_roll = 8.0;

            auto d_motor = ankle.GetMotorDampingMatrixFromTaskDamping(q_pitch, q_roll, d_pitch, d_roll);

            std::cout << "Motor Damping Matrix from Task Damping (is_left=" << is_left << "):" << std::endl;
            std::cout << "Input: q_pitch=" << q_pitch << ", q_roll=" << q_roll << std::endl;
            std::cout << "Input: d_pitch=" << d_pitch << ", d_roll=" << d_roll << std::endl;
            std::cout << "Output matrix:\n" << d_motor << std::endl;

            EXPECT_TRUE(std::isfinite(d_motor(0, 0))) << "d_motor(0,0) is not finite, is_left=" << is_left;
            EXPECT_TRUE(std::isfinite(d_motor(0, 1))) << "d_motor(0,1) is not finite, is_left=" << is_left;
            EXPECT_TRUE(std::isfinite(d_motor(1, 0))) << "d_motor(1,0) is not finite, is_left=" << is_left;
            EXPECT_TRUE(std::isfinite(d_motor(1, 1))) << "d_motor(1,1) is not finite, is_left=" << is_left;
            EXPECT_GT(d_motor(0, 0), 0.0) << "d_motor(0,0) should be positive, is_left=" << is_left;
            EXPECT_GT(d_motor(1, 1), 0.0) << "d_motor(1,1) should be positive, is_left=" << is_left;
        }
    }
}

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>

#include "parallel_ankle_calc.h"

using namespace robot_server_lib::utils;

// 测试用例1：右脚 - 正向构型（正pitch，正roll）
// 电机角度由新模型（pitch中心比roll中心高10mm）IK 重新生成
// 电机1 pos=0.297397 rad, 电机2 pos=-0.20988 rad
// 期望: pitch pos=0.3102 rad, roll pos=0.0427 rad
TEST(PMAnkleTest, RightFootPositiveRollIKFK) {
    bool is_left = false;  // 右脚
    PMAnkle ankle(is_left, MakeAnkleParams(is_left));

    // 期望的任务空间位姿
    double expected_pitch = 0.3102;
    double expected_roll = 0.0427;

    // 已知的电机角度
    double theta_1_known = 0.297397;  // 电机1
    double theta_2_known = -0.20988;  // 电机2

    std::cout << "\n========== Right Foot Positive Roll Test ==========" << std::endl;
    std::cout << "Expected: pitch=" << expected_pitch << " rad (" << expected_pitch * 180.0 / M_PI
              << " deg), roll=" << expected_roll << " rad (" << expected_roll * 180.0 / M_PI << " deg)" << std::endl;
    std::cout << "Given motor angles: theta1=" << theta_1_known << " rad, theta2=" << theta_2_known << " rad"
              << std::endl;

    // 测试1: 从电机角度正向运动学计算位姿
    auto fk_result = ankle.GetAnklePoseFromMotorAngles(theta_1_known, theta_2_known);

    std::cout << "FK result: pitch=" << fk_result(0) << " rad (" << fk_result(0) * 180.0 / M_PI
              << " deg), roll=" << fk_result(1) << " rad (" << fk_result(1) * 180.0 / M_PI << " deg)" << std::endl;

    EXPECT_NEAR(fk_result(0), expected_pitch, 1e-4) << "FK pitch mismatch for right foot positive roll";
    EXPECT_NEAR(fk_result(1), expected_roll, 1e-4) << "FK roll mismatch for right foot positive roll";

    // 测试2: 从期望位姿逆向运动学计算电机角度
    auto ik_solution = ankle.GetAnkleMotorAngles(expected_pitch, expected_roll);

    std::cout << "IK result: theta1=" << ik_solution.theta_1_rad << " rad, theta2=" << ik_solution.theta_2_rad << " rad"
              << std::endl;
    std::cout << "IK success: " << ik_solution.success_1 << ", " << ik_solution.success_2 << std::endl;

    EXPECT_TRUE(ik_solution.success_1) << "IK solution 1 failed for right foot positive roll";
    EXPECT_TRUE(ik_solution.success_2) << "IK solution 2 failed for right foot positive roll";

    EXPECT_NEAR(ik_solution.theta_1_rad, theta_1_known, 1e-4) << "IK theta1 mismatch for right foot positive roll";
    EXPECT_NEAR(ik_solution.theta_2_rad, theta_2_known, 1e-4) << "IK theta2 mismatch for right foot positive roll";

    // 测试3: 验证 FK(IK(q)) = q
    auto fk_from_ik = ankle.GetAnklePoseFromMotorAngles(ik_solution.theta_1_rad, ik_solution.theta_2_rad);

    EXPECT_NEAR(fk_from_ik(0), expected_pitch, 1e-6) << "FK(IK(q)) consistency failed for pitch";
    EXPECT_NEAR(fk_from_ik(1), expected_roll, 1e-6) << "FK(IK(q)) consistency failed for roll";

    std::cout << "==================================================\n" << std::endl;
}

// 测试用例2：右脚 - 负向构型（正pitch，负roll）
// 电机角度由新模型（pitch中心比roll中心高10mm）IK 重新生成
// 电机1 pos=0.210056 rad, 电机2 pos=-0.296973 rad
// 期望: pitch pos=0.3102 rad, roll pos=-0.0427 rad
TEST(PMAnkleTest, RightFootNegativeRollIKFK) {
    bool is_left = false;  // 右脚
    PMAnkle ankle(is_left, MakeAnkleParams(is_left));

    // 期望的任务空间位姿
    double expected_pitch = 0.3102;
    double expected_roll = -0.0427;

    // 已知的电机角度
    double theta_1_known = 0.210056;   // 电机1
    double theta_2_known = -0.296973;  // 电机2

    std::cout << "\n========== Right Foot Negative Roll Test ==========" << std::endl;
    std::cout << "Expected: pitch=" << expected_pitch << " rad (" << expected_pitch * 180.0 / M_PI
              << " deg), roll=" << expected_roll << " rad (" << expected_roll * 180.0 / M_PI << " deg)" << std::endl;
    std::cout << "Given motor angles: theta1=" << theta_1_known << " rad, theta2=" << theta_2_known << " rad"
              << std::endl;

    // 测试1: 从电机角度正向运动学计算位姿
    auto fk_result = ankle.GetAnklePoseFromMotorAngles(theta_1_known, theta_2_known);

    std::cout << "FK result: pitch=" << fk_result(0) << " rad (" << fk_result(0) * 180.0 / M_PI
              << " deg), roll=" << fk_result(1) << " rad (" << fk_result(1) * 180.0 / M_PI << " deg)" << std::endl;

    EXPECT_NEAR(fk_result(0), expected_pitch, 1e-4) << "FK pitch mismatch for right foot negative roll";
    EXPECT_NEAR(fk_result(1), expected_roll, 1e-4) << "FK roll mismatch for right foot negative roll";

    // 测试2: 从期望位姿逆向运动学计算电机角度
    auto ik_solution = ankle.GetAnkleMotorAngles(expected_pitch, expected_roll);

    std::cout << "IK result: theta1=" << ik_solution.theta_1_rad << " rad, theta2=" << ik_solution.theta_2_rad << " rad"
              << std::endl;
    std::cout << "IK success: " << ik_solution.success_1 << ", " << ik_solution.success_2 << std::endl;

    EXPECT_TRUE(ik_solution.success_1) << "IK solution 1 failed for right foot negative roll";
    EXPECT_TRUE(ik_solution.success_2) << "IK solution 2 failed for right foot negative roll";

    EXPECT_NEAR(ik_solution.theta_1_rad, theta_1_known, 1e-4) << "IK theta1 mismatch for right foot negative roll";
    EXPECT_NEAR(ik_solution.theta_2_rad, theta_2_known, 1e-4) << "IK theta2 mismatch for right foot negative roll";

    // 测试3: 验证 FK(IK(q)) = q
    auto fk_from_ik = ankle.GetAnklePoseFromMotorAngles(ik_solution.theta_1_rad, ik_solution.theta_2_rad);

    EXPECT_NEAR(fk_from_ik(0), expected_pitch, 1e-6) << "FK(IK(q)) consistency failed for pitch";
    EXPECT_NEAR(fk_from_ik(1), expected_roll, 1e-6) << "FK(IK(q)) consistency failed for roll";

    std::cout << "==================================================\n" << std::endl;
}

// 测试用例3：左脚 - 正向构型（根据右脚对称性推导）
// 左脚在相同pitch角度下，roll方向会相反
// 电机角度由新模型（pitch中心比roll中心高10mm）IK 重新生成
// 电机1 pos=-0.297397 rad, 电机2 pos=0.20988 rad
// 期望: pitch pos=0.3102 rad, roll pos=-0.0427 rad
TEST(PMAnkleTest, LeftFootPositiveRollIKFK) {
    bool is_left = true;  // 左脚
    PMAnkle ankle(is_left, MakeAnkleParams(is_left));

    // 期望的任务空间位姿（左脚的正roll对应电机符号相反）
    double expected_pitch = 0.3102;
    double expected_roll = -0.0427;  // 左脚的正roll方向与右脚相反

    // 已知的电机角度（根据对称性推导）
    double theta_1_known = -0.297397;  // 电机1
    double theta_2_known = 0.20988;    // 电机2

    std::cout << "\n========== Left Foot Positive Roll Test ==========" << std::endl;
    std::cout << "Expected: pitch=" << expected_pitch << " rad (" << expected_pitch * 180.0 / M_PI
              << " deg), roll=" << expected_roll << " rad (" << expected_roll * 180.0 / M_PI << " deg)" << std::endl;
    std::cout << "Given motor angles: theta1=" << theta_1_known << " rad, theta2=" << theta_2_known << " rad"
              << std::endl;

    // 测试1: 从电机角度正向运动学计算位姿
    auto fk_result = ankle.GetAnklePoseFromMotorAngles(theta_1_known, theta_2_known);

    std::cout << "FK result: pitch=" << fk_result(0) << " rad (" << fk_result(0) * 180.0 / M_PI
              << " deg), roll=" << fk_result(1) << " rad (" << fk_result(1) * 180.0 / M_PI << " deg)" << std::endl;

    EXPECT_NEAR(fk_result(0), expected_pitch, 1e-4) << "FK pitch mismatch for left foot positive roll";
    EXPECT_NEAR(fk_result(1), expected_roll, 1e-4) << "FK roll mismatch for left foot positive roll";

    // 测试2: 从期望位姿逆向运动学计算电机角度
    auto ik_solution = ankle.GetAnkleMotorAngles(expected_pitch, expected_roll);

    std::cout << "IK result: theta1=" << ik_solution.theta_1_rad << " rad, theta2=" << ik_solution.theta_2_rad << " rad"
              << std::endl;
    std::cout << "IK success: " << ik_solution.success_1 << ", " << ik_solution.success_2 << std::endl;

    EXPECT_TRUE(ik_solution.success_1) << "IK solution 1 failed for left foot positive roll";
    EXPECT_TRUE(ik_solution.success_2) << "IK solution 2 failed for left foot positive roll";

    EXPECT_NEAR(ik_solution.theta_1_rad, theta_1_known, 1e-4) << "IK theta1 mismatch for left foot positive roll";
    EXPECT_NEAR(ik_solution.theta_2_rad, theta_2_known, 1e-4) << "IK theta2 mismatch for left foot positive roll";

    // 测试3: 验证 FK(IK(q)) = q
    auto fk_from_ik = ankle.GetAnklePoseFromMotorAngles(ik_solution.theta_1_rad, ik_solution.theta_2_rad);

    EXPECT_NEAR(fk_from_ik(0), expected_pitch, 1e-6) << "FK(IK(q)) consistency failed for pitch";
    EXPECT_NEAR(fk_from_ik(1), expected_roll, 1e-6) << "FK(IK(q)) consistency failed for roll";

    std::cout << "================================================\n" << std::endl;
}

// 测试用例4：左脚 - 负向构型（根据右脚对称性推导）
// 电机角度由新模型（pitch中心比roll中心高10mm）IK 重新生成
// 电机1 pos=-0.210056 rad, 电机2 pos=0.296973 rad
// 期望: pitch pos=0.3102 rad, roll pos=0.0427 rad
TEST(PMAnkleTest, LeftFootNegativeRollIKFK) {
    bool is_left = true;  // 左脚
    PMAnkle ankle(is_left, MakeAnkleParams(is_left));

    // 期望的任务空间位姿
    double expected_pitch = 0.3102;
    double expected_roll = 0.0427;  // 左脚的负roll对应正电机角度

    // 已知的电机角度（根据对称性推导）
    double theta_1_known = -0.210056;  // 电机1
    double theta_2_known = 0.296973;   // 电机2

    std::cout << "\n========== Left Foot Negative Roll Test ==========" << std::endl;
    std::cout << "Expected: pitch=" << expected_pitch << " rad (" << expected_pitch * 180.0 / M_PI
              << " deg), roll=" << expected_roll << " rad (" << expected_roll * 180.0 / M_PI << " deg)" << std::endl;
    std::cout << "Given motor angles: theta1=" << theta_1_known << " rad, theta2=" << theta_2_known << " rad"
              << std::endl;

    // 测试1: 从电机角度正向运动学计算位姿
    auto fk_result = ankle.GetAnklePoseFromMotorAngles(theta_1_known, theta_2_known);

    std::cout << "FK result: pitch=" << fk_result(0) << " rad (" << fk_result(0) * 180.0 / M_PI
              << " deg), roll=" << fk_result(1) << " rad (" << fk_result(1) * 180.0 / M_PI << " deg)" << std::endl;

    EXPECT_NEAR(fk_result(0), expected_pitch, 1e-4) << "FK pitch mismatch for left foot negative roll";
    EXPECT_NEAR(fk_result(1), expected_roll, 1e-4) << "FK roll mismatch for left foot negative roll";

    // 测试2: 从期望位姿逆向运动学计算电机角度
    auto ik_solution = ankle.GetAnkleMotorAngles(expected_pitch, expected_roll);

    std::cout << "IK result: theta1=" << ik_solution.theta_1_rad << " rad, theta2=" << ik_solution.theta_2_rad << " rad"
              << std::endl;
    std::cout << "IK success: " << ik_solution.success_1 << ", " << ik_solution.success_2 << std::endl;

    EXPECT_TRUE(ik_solution.success_1) << "IK solution 1 failed for left foot negative roll";
    EXPECT_TRUE(ik_solution.success_2) << "IK solution 2 failed for left foot negative roll";

    EXPECT_NEAR(ik_solution.theta_1_rad, theta_1_known, 1e-4) << "IK theta1 mismatch for left foot negative roll";
    EXPECT_NEAR(ik_solution.theta_2_rad, theta_2_known, 1e-4) << "IK theta2 mismatch for left foot negative roll";

    // 测试3: 验证 FK(IK(q)) = q
    auto fk_from_ik = ankle.GetAnklePoseFromMotorAngles(ik_solution.theta_1_rad, ik_solution.theta_2_rad);

    EXPECT_NEAR(fk_from_ik(0), expected_pitch, 1e-6) << "FK(IK(q)) consistency failed for pitch";
    EXPECT_NEAR(fk_from_ik(1), expected_roll, 1e-6) << "FK(IK(q)) consistency failed for roll";

    std::cout << "===============================================\n" << std::endl;
}