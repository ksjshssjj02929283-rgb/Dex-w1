#include "parallel_ankle_calc.h"

#include <cmath>
#include <iostream>

namespace robot_server_lib {
namespace utils {

AnkleActuatorParams::AnkleActuatorParams() : l_bar(0), l_rod(0), alpha_offset_rad(0), l_bar_sq(0), l_rod_sq(0) {}

AnkleActuatorParams::AnkleActuatorParams(const Eigen::Vector3d& r_ai_0,
                                         const Eigen::Vector3d& r_ci_0_initial,
                                         double l_bar,
                                         double l_rod,
                                         double alpha_offset_rad)
    : r_ai_0(r_ai_0), r_ci_0_initial(r_ci_0_initial), l_bar(l_bar), l_rod(l_rod), alpha_offset_rad(alpha_offset_rad) {
    if (l_bar <= 1e-9) {
        throw std::runtime_error("l_bar必须大于零");
    }
    l_bar_sq = l_bar * l_bar;
    l_rod_sq = l_rod * l_rod;
}

AnkleIKSolution::AnkleIKSolution() : theta_1_rad(0), success_1(false), theta_2_rad(0), success_2(false) {}

PMAnkle::PMAnkle() { PMAnkleInit(); }

Eigen::Matrix3d Rx(double angle_rad) {
    return Eigen::AngleAxisd(angle_rad, Eigen::Vector3d::UnitX()).toRotationMatrix();
}

Eigen::Matrix3d Ry(double angle_rad) {
    return Eigen::AngleAxisd(angle_rad, Eigen::Vector3d::UnitY()).toRotationMatrix();
}

std::pair<double, bool> CalculateSingleThetaIInternal(double q_pitch_rad,
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
    theta_i_rad += params.alpha_offset_rad;

    return {theta_i_rad, true};
}

AnkleIKSolution CalculateAnkleIKBothActuators(double q_pitch_rad,
                                              double q_roll_rad,
                                              const AnkleActuatorParams& params_1,
                                              const AnkleActuatorParams& params_2) {
    AnkleIKSolution solution;

    auto result_1 = CalculateSingleThetaIInternal(q_pitch_rad, q_roll_rad, params_1);
    solution.theta_1_rad = result_1.first;
    solution.success_1 = result_1.second;

    auto result_2 = CalculateSingleThetaIInternal(q_pitch_rad, q_roll_rad, params_2);
    solution.theta_2_rad = result_2.first;
    solution.success_2 = result_2.second;

    return solution;
}

void PMAnkle::PMAnkleInit() {
    Eigen::Vector3d r_a_1_vec(ext_params.pm_ankle_a_1[0], ext_params.pm_ankle_a_1[1], ext_params.pm_ankle_a_1[2]);
    Eigen::Vector3d r_c_1_vec(ext_params.pm_ankle_c_1[0], ext_params.pm_ankle_c_1[1], ext_params.pm_ankle_c_1[2]);
    double l_bar_1 = ext_params.pm_ankle_bar_rod_1[0];
    double l_rod_1 = ext_params.pm_ankle_bar_rod_1[1];
    double alpha_1_rad = -0.1745;
    params_1 = AnkleActuatorParams(r_a_1_vec, r_c_1_vec, l_bar_1, l_rod_1, alpha_1_rad);

    Eigen::Vector3d r_a_2_vec(ext_params.pm_ankle_a_2[0], ext_params.pm_ankle_a_2[1], ext_params.pm_ankle_a_2[2]);
    Eigen::Vector3d r_c_2_vec(ext_params.pm_ankle_c_2[0], ext_params.pm_ankle_c_2[1], ext_params.pm_ankle_c_2[2]);
    double l_bar_2 = ext_params.pm_ankle_bar_rod_2[0];
    double l_rod_2 = ext_params.pm_ankle_bar_rod_2[1];
    double alpha_2_rad = -0.1745;
    params_2 = AnkleActuatorParams(r_a_2_vec, r_c_2_vec, l_bar_2, l_rod_2, alpha_2_rad);
}

AnkleIKSolution PMAnkle::GetAnkleMotorAngles(double q_pitch_rad, double q_roll_rad) {
    return CalculateAnkleIKBothActuators(q_pitch_rad, q_roll_rad, params_1, params_2);
}

Eigen::Vector2d ComputeNumericalJacobianSingle(double q_pitch,
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

Eigen::Matrix2d ComputeAnkleJacobian(double q_pitch,
                                     double q_roll,
                                     const AnkleActuatorParams& params_1,
                                     const AnkleActuatorParams& params_2,
                                     double eps) {
    Eigen::Matrix2d J;
    J.row(0) = ComputeNumericalJacobianSingle(q_pitch, q_roll, params_1, eps);
    J.row(1) = ComputeNumericalJacobianSingle(q_pitch, q_roll, params_2, eps);
    return J;
}

Eigen::Vector2d PMAnkle::GetAnklePoseFromMotorAngles(double theta_1_rad, double theta_2_rad) {
    auto residual = [&](const Eigen::Vector2d& q) -> Eigen::Vector2d {
        auto ik = CalculateAnkleIKBothActuators(q(0), q(1), params_1, params_2);
        Eigen::Vector2d f;
        f(0) = ik.theta_1_rad - theta_1_rad;
        f(1) = ik.theta_2_rad - theta_2_rad;
        return f;
    };

    auto jacobian = [&](const Eigen::Vector2d& q) -> Eigen::Matrix2d {
        return ComputeAnkleJacobian(q(0), q(1), params_1, params_2);
    };

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
    Eigen::Matrix2d J = ComputeAnkleJacobian(q_pitch_rad, q_roll_rad, params_1, params_2);
    Eigen::Vector2d tau_motor(tau_motor_1, tau_motor_2);
    return J.inverse().transpose() * tau_motor;
}

Eigen::Vector2d PMAnkle::GetTaskVelocitiesFromMotorVelocities(double q_pitch_rad,
                                                              double q_roll_rad,
                                                              double vel_motor_1,
                                                              double vel_motor_2) {
    Eigen::Matrix2d J = ComputeAnkleJacobian(q_pitch_rad, q_roll_rad, params_1, params_2);
    Eigen::Vector2d vel_motor(vel_motor_1, vel_motor_2);
    return J * vel_motor;
}

Eigen::Vector2d PMAnkle::GetMotorStiffnessFromTaskStiffness(double q_pitch_rad,
                                                            double q_roll_rad,
                                                            double k_pitch,
                                                            double k_roll) {
    Eigen::Matrix2d J = ComputeAnkleJacobian(q_pitch_rad, q_roll_rad, params_1, params_2);
    Eigen::Matrix2d J_inv = J.inverse();
    Eigen::Vector2d k_motor;
    k_motor(0) = k_pitch * J_inv(0, 0) * J_inv(0, 0) + k_roll * J_inv(0, 1) * J_inv(0, 1);
    k_motor(1) = k_pitch * J_inv(1, 0) * J_inv(1, 0) + k_roll * J_inv(1, 1) * J_inv(1, 1);
    return k_motor;
}

Eigen::Vector2d PMAnkle::GetMotorDampingFromTaskDamping(double q_pitch_rad,
                                                        double q_roll_rad,
                                                        double d_pitch,
                                                        double d_roll) {
    Eigen::Matrix2d J = ComputeAnkleJacobian(q_pitch_rad, q_roll_rad, params_1, params_2);
    Eigen::Matrix2d J_inv = J.inverse();
    Eigen::Vector2d d_motor;
    d_motor(0) = d_pitch * J_inv(0, 0) * J_inv(0, 0) + d_roll * J_inv(0, 1) * J_inv(0, 1);
    d_motor(1) = d_pitch * J_inv(1, 0) * J_inv(1, 0) + d_roll * J_inv(1, 1) * J_inv(1, 1);
    return d_motor;
}

}  // namespace utils
}  // namespace robot_server_lib
