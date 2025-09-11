#pragma once

#include "kinematics_base.h"

namespace robot_server_lib
{
namespace kinematics
{

struct SRSParameters
{
    double d1, d2, d3, d4, d5, d6, d7;
    Eigen::Vector4d link_lengths;

    Eigen::MatrixXd dh_params;

    Eigen::VectorXd q_min;
    Eigen::VectorXd q_max;

    Eigen::Matrix4d T_b_ob;  ///< from "base" frame to ur_base frame
    Eigen::Matrix4d T_oe_e;  ///< from ur_end_effector frame to "end_effector" frame

    std::array<signed char, 7> rotation_directions;
};

struct DexforceW1V02IndustrialParameters : SRSParameters
{
   protected:
    void InitializeCommonParameters()
    {
        d1 = 0;
        d2 = 0;
        d3 = 0.260;
        d4 = 0.0;
        d5 = 0.166;
        d6 = 0.098;
        d7 = 0;

        link_lengths = Eigen::Vector4d(d1 + d2, d3 + d4, d5 + d6, d7);
        dh_params.resize(7, 4);

        /* clang-format off */
        dh_params << 
                link_lengths(0), -M_PI_2, 0, 0, 
                0, M_PI_2, 0, 0,
                link_lengths(1), M_PI_2, 0, M_PI_2, 
                0, -M_PI_2, 0, 0,
                link_lengths(2), -M_PI_2, 0, 0, 
                0, M_PI_2, 0, 0,
                link_lengths(3), 0, 0, 0;
        /* clang-format on */

        q_min = Eigen::VectorXd(7);
        q_max = Eigen::VectorXd(7);

        T_b_ob << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.10254, 0.0, 0.0, 0.0, 1.0;
        T_oe_e << -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.066, 0.0, 0.0, 0.0, 1.0;
    }
};

struct DexforceW1V02AnthropomorphicParameters : SRSParameters
{
   protected:
    void initializeCommonParameters()
    {
        d1 = 0;
        d2 = 0;
        d3 = 0.260;
        d4 = 0.0;
        d5 = 0.166;
        d6 = 0.098;
        d7 = 0;

        link_lengths = Eigen::Vector4d(d1 + d2, d3 + d4, d5 + d6, d7);
        dh_params.resize(7, 4);

        /* clang-format off */
        dh_params << 
                link_lengths(0), -M_PI_2, 0, 0, 
                0, M_PI_2, 0, 0,
                link_lengths(1), M_PI_2, 0, M_PI_2, 
                0, -M_PI_2, 0, 0,
                link_lengths(2), -M_PI_2, 0, 0, 
                0, M_PI_2, 0, M_PI_2,
                link_lengths(3), 0, 0, 0;
        /* clang-format on */

        q_min = Eigen::VectorXd(7);
        q_max = Eigen::VectorXd(7);

        T_b_ob << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.10254, 0.0, 0.0, 0.0, 1.0;
        T_oe_e << 0.0, 0.0, -1.0, -0.066, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    }
};

struct W1LeftArmIndustrial : DexforceW1V02IndustrialParameters
{
    W1LeftArmIndustrial()
    {
        InitializeCommonParameters();
        rotation_directions = {1, 1, 1, 1, 1, -1, 1};
        q_min << -2.9670597283903604, -2.0943951023931953, -2.9670597283903604, -2.356194490192345, -2.9670597283903604, -1.5707963267948966,
            -2.9670597283903604;
        q_max << 2.9670597283903604, 1.5707963267948966, 2.9670597283903604, 1.5707963267948966, 2.96705972839036041, 1.5707963267948966,
            2.9670597283903604;
    }
};

struct W1RightArmIndustrial : DexforceW1V02IndustrialParameters
{
    W1RightArmIndustrial()
    {
        InitializeCommonParameters();
        rotation_directions = {1, 1, 1, -1, 1, 1, 1};
        q_min << -2.9670597283903604, -1.5707963267948966, -2.9670597283903604, -1.5707963267948966, -2.9670597283903604, -1.5707963267948966,
            -2.9670597283903604;
        q_max << 2.9670597283903604, 2.0943951023931953, 2.9670597283903604, 2.356194490192345, 2.9670597283903604, 1.5707963267948966,
            2.9670597283903604;
    }
};

struct W1LeftArmAnthropomorphic : DexforceW1V02AnthropomorphicParameters
{
    W1LeftArmAnthropomorphic()
    {
        initializeCommonParameters();
        rotation_directions = {1, 1, 1, 1, 1, -1, 1};
        q_min << -2.9670597283903604, -2.0943951023931953, -2.9670597283903604, -2.356194490192345, -2.9670597283903604, -0.7853981633974483,
            -1.5707963267948966;
        q_max << 2.9670597283903604, 1.5707963267948966, 2.9670597283903604, 1.5707963267948966, 2.96705972839036041, 0.7853981633974483,
            1.0471975511965976;
    }
};

struct W1RightArmAnthropomorphic : DexforceW1V02AnthropomorphicParameters
{
    W1RightArmAnthropomorphic()
    {
        initializeCommonParameters();
        rotation_directions = {1, 1, 1, -1, 1, 1, 1};
        q_min << -2.9670597283903604, -1.5707963267948966, -2.9670597283903604, -1.5707963267948966, -2.9670597283903604, -0.7853981633974483,
            -1.0471975511965976;
        q_max << 2.9670597283903604, 2.0943951023931953, 2.9670597283903604, 2.356194490192345, 2.9670597283903604, 0.7853981633974483,
            1.5707963267948966;
    }
};

class SRSKinematics : public KinematicsBase
{
   public:
    SRSKinematics(const SRSParameters &srs_params);
    ~SRSKinematics() override;
    PoseQuaternion GetFK(const Eigen::VectorXd &q_in, const pinocchio::SE3 &tcp_frame = pinocchio::SE3::Identity()) override;
    bool GetIK(const PoseQuaternion &pose_in, Eigen::VectorXd &q_in_and_out, const pinocchio::SE3 &tcp_frame = pinocchio::SE3::Identity()) override;

   private:
    bool _ComputeInverseKinematics(const pinocchio::SE3 &target_pose_se3, const Eigen::VectorXd &joint_seed, double nsparam,
                                   Eigen::Matrix<double, 4, 7> &joints_list) const;
    bool _ChooseIKSolution(std::vector<Eigen::VectorXd> &ik_solutions, const Eigen::VectorXd &joints_seed) const;
    bool _CalculateJointAngles(const Eigen::Vector3d &P_s_to_w, int elbow_GC4, Eigen::VectorXd &joints) const;
    bool _ReferencePlane(const pinocchio::SE3 &pose, int elbow_GC4, Eigen::Vector3d &V_v_to_sew, Eigen::Matrix3d &R03_v,
                         Eigen::VectorXd &joint_v) const;
    double _CalculateAnglePhi(const Eigen::VectorXd &joint_angles, const pinocchio::SE3 &current_pose);
    pinocchio::SE3 _DHCalc(double d, double alpha, double a, double theta) const;
    Eigen::Matrix3d _SkewSymmetric(const Eigen::Vector3d &vector) const;
    double _SafeAcos(double x) const;
    int _Sign(double value) const;

    int ndof_;
    SRSParameters params_;
    int num_samples_ = 50;
    double joint_threshold_ = M_PI / 180.0 * 30;
    double small_range_ = M_PI / 18000.0 * num_samples_;
    Eigen::VectorXd ik_weight_;
};

}  // namespace kinematics
}  // namespace robot_server_lib
