#pragma once

#include <Eigen/Dense>
#include <stdexcept>

namespace robot_server_lib {
namespace utils {

struct AnkleActuatorParams {
    bool is_first_actuator;

    Eigen::Vector3d r_ai_0;
    Eigen::Vector3d r_ci_0_initial;

    double l_bar;
    double l_rod;
    double alpha_offset_rad;

    double l_bar_sq;
    double l_rod_sq;

    AnkleActuatorParams()
        : is_first_actuator(true), l_bar(0), l_rod(0), alpha_offset_rad(0), l_bar_sq(0), l_rod_sq(0) {}

    AnkleActuatorParams(bool is_first_actuator,
                        const Eigen::Vector3d& r_ai_0,
                        const Eigen::Vector3d& r_ci_0_initial,
                        double l_bar,
                        double l_rod,
                        double alpha_offset_rad)
        : is_first_actuator(is_first_actuator),
          r_ai_0(r_ai_0),
          r_ci_0_initial(r_ci_0_initial),
          l_bar(l_bar),
          l_rod(l_rod),
          alpha_offset_rad(alpha_offset_rad) {
        if (l_bar <= 1e-9) {
            throw std::runtime_error("l_bar必须大于零");
        }
        l_bar_sq = l_bar * l_bar;
        l_rod_sq = l_rod * l_rod;
    }
};

struct AnkleIKSolution {
    double theta_1_rad;
    bool success_1;
    double theta_2_rad;
    bool success_2;

    AnkleIKSolution() : theta_1_rad(0), success_1(false), theta_2_rad(0), success_2(false) {}
};

class PMAnkle {
public:
    AnkleActuatorParams params_1_;
    AnkleActuatorParams params_2_;
    bool is_left_;

    PMAnkle(bool is_left_);

    AnkleIKSolution GetAnkleMotorAngles(double q_pitch_rad, double q_roll_rad);

    Eigen::Vector2d GetAnklePoseFromMotorAngles(double theta_1_rad, double theta_2_rad);

    Eigen::Vector2d GetTaskTorquesFromMotorTorques(double q_pitch_rad,
                                                   double q_roll_rad,
                                                   double tau_motor_1,
                                                   double tau_motor_2);

    Eigen::Vector2d GetMotorTorquesFromTaskTorques(double q_pitch_rad,
                                                    double q_roll_rad,
                                                    double tau_task_pitch,
                                                    double tau_task_roll);

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

    Eigen::Matrix2d ComputeJacobian(double q_pitch_rad, double q_roll_rad);

private:
    std::pair<double, bool> CalculateSingleThetaIInternal(double q_pitch_rad,
                                                          double q_roll_rad,
                                                          const AnkleActuatorParams& params);

    AnkleIKSolution CalculateAnkleIKBothActuators(double q_pitch_rad, double q_roll_rad);

    Eigen::Vector2d ComputeNumericalJacobianSingle(double q_pitch,
                                                   double q_roll,
                                                   const AnkleActuatorParams& params,
                                                   double eps = 1e-6);

    Eigen::Matrix2d ComputeAnkleJacobian(double q_pitch, double q_roll, double eps = 1e-6);
};

}  // namespace utils
}  // namespace robot_server_lib
