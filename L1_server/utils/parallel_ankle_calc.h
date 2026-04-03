#pragma once

#include <Eigen/Dense>
#include <stdexcept>

namespace robot_server_lib {
namespace utils {

struct ExternalParameters {
    double pm_ankle_a_1[3] = {-46, 0, 257.94};
    double pm_ankle_c_1[3] = {-46, 59, 0.0};
    double pm_ankle_bar_rod_1[2] = {60, 268.354};

    double pm_ankle_a_2[3] = {-46, 0, 177.94};
    double pm_ankle_c_2[3] = {-46, -59, 0.0};
    double pm_ankle_bar_rod_2[2] = {60, 188.355};
};

struct AnkleActuatorParams {
    Eigen::Vector3d r_ai_0;
    Eigen::Vector3d r_ci_0_initial;

    double l_bar;
    double l_rod;
    double alpha_offset_rad;

    double l_bar_sq;
    double l_rod_sq;

    AnkleActuatorParams();

    AnkleActuatorParams(const Eigen::Vector3d& r_ai_0,
                        const Eigen::Vector3d& r_ci_0_initial,
                        double l_bar,
                        double l_rod,
                        double alpha_offset_rad = 0.0);
};

struct AnkleIKSolution {
    double theta_1_rad;
    bool success_1;
    double theta_2_rad;
    bool success_2;

    AnkleIKSolution();
};

class PMAnkle {
public:
    ExternalParameters ext_params;
    AnkleActuatorParams params_1;
    AnkleActuatorParams params_2;

    PMAnkle();

    void PMAnkleInit();

    AnkleIKSolution GetAnkleMotorAngles(double q_pitch_rad, double q_roll_rad);

    Eigen::Vector2d GetAnklePoseFromMotorAngles(double theta_1_rad, double theta_2_rad);

    Eigen::Vector2d GetTaskTorquesFromMotorTorques(double q_pitch_rad,
                                                   double q_roll_rad,
                                                   double tau_motor_1,
                                                   double tau_motor_2);

    Eigen::Vector2d GetTaskVelocitiesFromMotorVelocities(double q_pitch_rad,
                                                         double q_roll_rad,
                                                         double vel_motor_1,
                                                         double vel_motor_2);

    Eigen::Vector2d GetMotorStiffnessFromTaskStiffness(double q_pitch_rad,
                                                       double q_roll_rad,
                                                       double k_pitch,
                                                       double k_roll);

    Eigen::Vector2d GetMotorDampingFromTaskDamping(double q_pitch_rad,
                                                   double q_roll_rad,
                                                   double d_pitch,
                                                   double d_roll);
};

std::pair<double, bool> CalculateSingleThetaIInternal(double q_pitch_rad,
                                                      double q_roll_rad,
                                                      const AnkleActuatorParams& params);

AnkleIKSolution CalculateAnkleIKBothActuators(double q_pitch_rad,
                                              double q_roll_rad,
                                              const AnkleActuatorParams& params_1,
                                              const AnkleActuatorParams& params_2);

Eigen::Vector2d ComputeNumericalJacobianSingle(double q_pitch,
                                               double q_roll,
                                               const AnkleActuatorParams& params,
                                               double eps = 1e-6);

Eigen::Matrix2d ComputeAnkleJacobian(double q_pitch,
                                     double q_roll,
                                     const AnkleActuatorParams& params_1,
                                     const AnkleActuatorParams& params_2,
                                     double eps = 1e-6);

}  // namespace utils
}  // namespace robot_server_lib
