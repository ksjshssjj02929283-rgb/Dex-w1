#include "parallel_ankle_calc.h"

#include <cmath>
#include <iostream>

namespace robot_server_lib {
namespace utils {

Eigen::Matrix3d Rx(double angle_rad) {
    return Eigen::AngleAxisd(angle_rad, Eigen::Vector3d::UnitX()).toRotationMatrix();
}

Eigen::Matrix3d Ry(double angle_rad) {
    return Eigen::AngleAxisd(angle_rad, Eigen::Vector3d::UnitY()).toRotationMatrix();
}

PMAnkle::PMAnkle(bool is_left_) : is_left_(is_left_) {
    Eigen::Vector3d r_a_1_vec, r_c_1_vec, r_a_2_vec, r_c_2_vec;
    double l_bar_1, l_rod_1, l_bar_2, l_rod_2, alpha_1_rad, alpha_2_rad;

    if (is_left_) {
        r_a_1_vec = Eigen::Vector3d(-46, 0, 257.94);
        r_c_1_vec = Eigen::Vector3d(-46, -59, 0.0);
        l_bar_1 = 60;
        l_rod_1 = 268.354;
        alpha_1_rad = 0.1745;

        r_a_2_vec = Eigen::Vector3d(-46, 0, 177.94);
        r_c_2_vec = Eigen::Vector3d(-46, 59, 0.0);
        l_bar_2 = 60;
        l_rod_2 = 188.355;
        alpha_2_rad = -0.1745;

    } else {
        r_a_1_vec = Eigen::Vector3d(-46, 0, 257.94);
        r_c_1_vec = Eigen::Vector3d(-46, 59, 0.0);
        l_bar_1 = 60;
        l_rod_1 = 268.354;
        alpha_1_rad = -0.1745;

        r_a_2_vec = Eigen::Vector3d(-46, 0, 177.94);
        r_c_2_vec = Eigen::Vector3d(-46, -59, 0.0);
        l_bar_2 = 60;
        l_rod_2 = 188.355;
        alpha_2_rad = 0.1745;
    }

    params_1_ = AnkleActuatorParams(true, r_a_1_vec, r_c_1_vec, l_bar_1, l_rod_1, alpha_1_rad);
    params_2_ = AnkleActuatorParams(false, r_a_2_vec, r_c_2_vec, l_bar_2, l_rod_2, alpha_2_rad);
}

std::pair<double, bool> PMAnkle::CalculateSingleThetaIInternal(double q_pitch_rad,
                                                               double q_roll_rad,
                                                               const AnkleActuatorParams& params) {
    const Eigen::Vector3d& r_ai = params.r_ai_0;
    Eigen::Matrix3d x_rot = Ry(q_pitch_rad) * Rx(q_roll_rad);
    Eigen::Vector3d r_ci = x_rot * params.r_ci_0_initial;
    Eigen::Vector3d r_ci_minus_r_ai = r_ci - r_ai;

    double a_val = r_ci_minus_r_ai.y();
    double b_val = (r_ai - r_ci).z();
    double norm_sq_r_ci_minus_r_ai = r_ci_minus_r_ai.squaredNorm();

    if (params.l_bar < 1e-9) {
        std::cerr << "错误: l_bar接近零" << std::endl;
        return {0.0, false};
    }
    double c_val = (params.l_rod_sq - params.l_bar_sq - norm_sq_r_ci_minus_r_ai) / (2.0 * params.l_bar);

    double den_asin = a_val * a_val + b_val * b_val;
    double inner_sqrt_term = a_val * a_val + b_val * b_val - c_val * c_val;

    if (inner_sqrt_term < -1e-9) {
        std::cerr << "错误: 配置不可达，平方根项为负 (" << inner_sqrt_term << ")" << std::endl;
        return {0.0, false};
    }
    if (inner_sqrt_term < 0) inner_sqrt_term = 0;

    double num_asin = b_val * c_val + std::abs(a_val) * std::sqrt(inner_sqrt_term);

    if (std::abs(den_asin) < 1e-12) {
        std::cerr << "错误: arcsin分母接近零" << std::endl;
        if (std::abs(num_asin) < 1e-9) {
            std::cout << "警告: 奇异构型，theta未定义" << std::endl;
        }
        return {0.0, false};
    }

    double arg_asin = num_asin / den_asin;

    if (std::abs(arg_asin) > 1.0 + 1e-9) {
        std::cerr << "错误: arcsin参数超出定义域 [-1, 1]: " << arg_asin << std::endl;
        return {0.0, false};
    }
    arg_asin = std::max(-1.0, std::min(1.0, arg_asin));

    double theta_i_rad = std::asin(arg_asin);
    // 根据实际电机转向异或取反theta_i_rad
    if (is_left_ == params.is_first_actuator) {
        theta_i_rad = -theta_i_rad;
    }
    theta_i_rad += params.alpha_offset_rad;

    return {theta_i_rad, true};
}

AnkleIKSolution PMAnkle::CalculateAnkleIKBothActuators(double q_pitch_rad, double q_roll_rad) {
    AnkleIKSolution solution;

    auto result_1 = CalculateSingleThetaIInternal(q_pitch_rad, q_roll_rad, params_1_);
    solution.theta_1_rad = result_1.first;
    solution.success_1 = result_1.second;

    auto result_2 = CalculateSingleThetaIInternal(q_pitch_rad, q_roll_rad, params_2_);
    solution.theta_2_rad = result_2.first;
    solution.success_2 = result_2.second;

    return solution;
}

AnkleIKSolution PMAnkle::GetAnkleMotorAngles(double q_pitch_rad, double q_roll_rad) {
    return CalculateAnkleIKBothActuators(q_pitch_rad, q_roll_rad);
}

Eigen::Vector2d PMAnkle::ComputeNumericalJacobianSingle(double q_pitch,
                                                        double q_roll,
                                                        const AnkleActuatorParams& params,
                                                        double eps) {
    auto compute_theta = [&](double qp, double qr) -> double {
        auto result = CalculateSingleThetaIInternal(qp, qr, params);
        return result.first;
    };

    Eigen::Vector2d J;
    double theta_0 = compute_theta(q_pitch, q_roll);

    J(0) = (compute_theta(q_pitch + eps, q_roll) - theta_0) / eps;
    J(1) = (compute_theta(q_pitch, q_roll + eps) - theta_0) / eps;

    return J;
}

Eigen::Matrix2d PMAnkle::ComputeAnkleJacobian(double q_pitch, double q_roll, double eps) {
    Eigen::Matrix2d J;
    J.row(0) = ComputeNumericalJacobianSingle(q_pitch, q_roll, params_1_, eps);
    J.row(1) = ComputeNumericalJacobianSingle(q_pitch, q_roll, params_2_, eps);
    return J;
}

Eigen::Matrix2d PMAnkle::ComputeJacobian(double q_pitch_rad, double q_roll_rad) {
    return ComputeAnkleJacobian(q_pitch_rad, q_roll_rad);
}

Eigen::Vector2d PMAnkle::GetAnklePoseFromMotorAngles(double theta_1_rad, double theta_2_rad) {
    auto residual = [&](const Eigen::Vector2d& q) -> Eigen::Vector2d {
        auto ik = CalculateAnkleIKBothActuators(q(0), q(1));
        Eigen::Vector2d f;
        f(0) = ik.theta_1_rad - theta_1_rad;
        f(1) = ik.theta_2_rad - theta_2_rad;
        return f;
    };

    auto jacobian = [&](const Eigen::Vector2d& q) -> Eigen::Matrix2d { return ComputeAnkleJacobian(q(0), q(1)); };

    Eigen::Vector2d q = Eigen::Vector2d::Zero();
    double eps = 1e-6;
    int max_iter = 20;

    for (int i = 0; i < max_iter; ++i) {
        Eigen::Vector2d f = residual(q);
        if (f.norm() < eps) {
            break;
        }

        Eigen::Matrix2d J = jacobian(q);
        if (std::abs(J.determinant()) < 1e-12) {
            break;
        }

        q = q - J.inverse() * f;
    }

    return q;
}

Eigen::Vector2d PMAnkle::GetTaskTorquesFromMotorTorques(double q_pitch_rad,
                                                        double q_roll_rad,
                                                        double tau_motor_1,
                                                        double tau_motor_2) {
    Eigen::Matrix2d J = ComputeAnkleJacobian(q_pitch_rad, q_roll_rad);
    Eigen::Vector2d tau_motor(tau_motor_1, tau_motor_2);
    
    return J.transpose() * tau_motor;
}

Eigen::Vector2d PMAnkle::GetMotorTorquesFromTaskTorques(double q_pitch_rad,
                                                        double q_roll_rad,
                                                        double tau_task_pitch,
                                                        double tau_task_roll) {
    Eigen::Matrix2d J = ComputeAnkleJacobian(q_pitch_rad, q_roll_rad);
    Eigen::Vector2d tau_task(tau_task_pitch, tau_task_roll);
    Eigen::Vector2d tau_motor = J.transpose().inverse() * tau_task;
    return tau_motor;
}

Eigen::Vector2d PMAnkle::GetTaskVelocitiesFromMotorVelocities(double q_pitch_rad,
                                                              double q_roll_rad,
                                                              double vel_motor_1,
                                                              double vel_motor_2) {
    Eigen::Matrix2d J = ComputeAnkleJacobian(q_pitch_rad, q_roll_rad);
    Eigen::Vector2d vel_motor(vel_motor_1, vel_motor_2);
    return J.inverse() * vel_motor;
}

Eigen::Matrix2d PMAnkle::GetMotorStiffnessMatrixFromTaskStiffness(double q_pitch_rad,
                                                            double q_roll_rad,
                                                            double k_pitch,
                                                            double k_roll) {
    Eigen::Matrix2d J = ComputeAnkleJacobian(q_pitch_rad, q_roll_rad);
    Eigen::Matrix2d J_inv = J.inverse();
    Eigen::Matrix2d K_task;
    K_task << k_pitch, 0,
              0, k_roll;
    Eigen::Matrix2d K_motor = J_inv.transpose() * K_task * J_inv;
    return K_motor;
}

Eigen::Matrix2d PMAnkle::GetMotorDampingMatrixFromTaskDamping(double q_pitch_rad,
                                                        double q_roll_rad,
                                                        double d_pitch,
                                                        double d_roll) {
    Eigen::Matrix2d J = ComputeAnkleJacobian(q_pitch_rad, q_roll_rad);
    Eigen::Matrix2d J_inv = J.inverse();
    Eigen::Matrix2d D_task;
    D_task << d_pitch, 0,
              0, d_roll;
    Eigen::Matrix2d D_motor = J_inv.transpose() * D_task * J_inv;
    return D_motor;
}

}  // namespace utils
}  // namespace robot_server_lib
