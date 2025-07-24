// ----------------------------------------------------------------------------
// Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
//
// All rights reserved.
// ----------------------------------------------------------------------------
#pragma once
#include "pinocchio_include.h"
#include "utilities.h"

namespace robot_server_lib
{
namespace kinematics
{

class KinematicsBase
{
   public:
    virtual ~KinematicsBase() = default;

    virtual PoseQuaternion GetFK(const Eigen::VectorXd &q_in, const pinocchio::SE3 &tcp_frame = pinocchio::SE3::Identity()) = 0;

    virtual bool GetIK(const PoseQuaternion &pose, Eigen::VectorXd &q_in_and_out, const pinocchio::SE3 &tcp_frame = pinocchio::SE3::Identity()) = 0;
};

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
}  // namespace kinematics
}  // namespace robot_server_lib