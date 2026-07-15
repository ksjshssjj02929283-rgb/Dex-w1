#pragma once

#include <Eigen/Dense>
#include <stdexcept>

namespace robot_server_lib {

// 前向声明：踝关节几何参数结构（完整定义见 utils.h）
struct AnkleParams;

namespace utils {

struct AnkleActuatorParams {
    bool is_first_actuator;

    Eigen::Vector3d r_ai_0;
    Eigen::Vector3d r_ci_0_initial;

    double l_bar;
    double l_rod;
    double alpha_offset_rad;
    bool is_left_motor;

    double l_bar_sq;
    double l_rod_sq;

    AnkleActuatorParams()
        : is_first_actuator(true), l_bar(0), l_rod(0), alpha_offset_rad(0), is_left_motor(true), l_bar_sq(0), l_rod_sq(0) {}

    AnkleActuatorParams(bool is_first_actuator,
                        const Eigen::Vector3d& r_ai_0,
                        const Eigen::Vector3d& r_ci_0_initial,
                        double l_bar,
                        double l_rod,
                        double alpha_offset_rad,
                        bool is_left_motor)
        : is_first_actuator(is_first_actuator),
          r_ai_0(r_ai_0),
          r_ci_0_initial(r_ci_0_initial),
          l_bar(l_bar),
          l_rod(l_rod),
          alpha_offset_rad(alpha_offset_rad),
          is_left_motor(is_left_motor) {
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
    bool is_left_ = false;

    // 旋转中心（基坐标系下）。实际机器人中 pitch 与 roll 的旋转中心不重合，
    // pitch 旋转中心比 roll 旋转中心沿 +Z 方向高出 10mm。
    Eigen::Vector3d roll_center_;
    Eigen::Vector3d pitch_center_;

    PMAnkle() = default;  // 默认构造，构造后需重新赋值才可用于计算
    // 由外部传入的踝关节几何参数构造（参数来自 l1_config.yaml）
    explicit PMAnkle(bool is_left, const AnkleParams& params);

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

    Eigen::Matrix2d GetMotorStiffnessMatrixFromTaskStiffness(double q_pitch_rad,
                                                             double q_roll_rad,
                                                             double k_pitch,
                                                             double k_roll);

    Eigen::Matrix2d GetMotorDampingMatrixFromTaskDamping(double q_pitch_rad,
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
