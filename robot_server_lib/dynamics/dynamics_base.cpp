#include "dynamics_base.h"

namespace robot_server_lib
{
namespace dynamics
{

Eigen::VectorXd InverseDynamics(const RobotModel& robot_model, pinocchio::Data& data, const VectorDof& q, const VectorDof& dq,
                                const VectorDof& ddq)
{
    const auto& model = *robot_model.model_ptr;
    const auto& joint_command_sequence_by_name = robot_model.params_ptr->joint_command_sequence_by_name;
    const auto& dynamics_base_parameters_primary = robot_model.params_ptr->dynamics_base_parameters_primary;
    const auto& dynamics_base_parameters_secondary = robot_model.params_ptr->dynamics_base_parameters_secondary;

    // 将输入数据映射到URDF顺序
    Eigen::VectorXd q_urdf = MapToUrdfOrder(model, q, joint_command_sequence_by_name);
    Eigen::VectorXd dq_urdf = MapToUrdfOrder(model, dq, joint_command_sequence_by_name);
    Eigen::VectorXd ddq_urdf = MapToUrdfOrder(model, ddq, joint_command_sequence_by_name);

    // 计算逆动力学
    auto tau_urdf = pinocchio::rnea(model, data, q_urdf, dq_urdf, ddq_urdf);
    VectorDof tau = MapToParamOrder(model, tau_urdf, joint_command_sequence_by_name); // 将结果映射回joint_command_sequence_by_name顺序

    bool is_anthropmorphic = true;
    if (robot_model.params_ptr->left_arm_name == "W1LeftArmIndustrial")
    {
        is_anthropmorphic = false;
    }
    // 如果提供了动力学最小参数集矩阵，则进行替换
    if (dynamics_base_parameters_primary.has_value())
    {
        Eigen::VectorXd inertia_term_tau = InvDynamicsInertiaTerm(robot_model, data, q, dq, ddq, MotionGroup::Primary, is_anthropmorphic);
        Eigen::VectorXd coriolis_term_tau = InvDynamicsCoriolisTerm(robot_model, data, q, dq, MotionGroup::Primary, is_anthropmorphic);
        Eigen::VectorXd gravity_term_tau = InvDynamicsGravityTerm(robot_model, data, q, MotionGroup::Primary, is_anthropmorphic);
        utils::MapJointToWholeBody(robot_model, inertia_term_tau + coriolis_term_tau + gravity_term_tau, tau, MotionGroup::Primary);
    }
    if (dynamics_base_parameters_secondary.has_value())
    {
        Eigen::VectorXd inertia_term_tau = InvDynamicsInertiaTerm(robot_model, data, q, dq, ddq, MotionGroup::Secondary, is_anthropmorphic);
        Eigen::VectorXd gravity_term_tau = InvDynamicsGravityTerm(robot_model, data, q, MotionGroup::Secondary, is_anthropmorphic);
        Eigen::VectorXd coriolis_term_tau = InvDynamicsCoriolisTerm(robot_model, data, q, dq, MotionGroup::Secondary, is_anthropmorphic);
        utils::MapJointToWholeBody(robot_model, inertia_term_tau + coriolis_term_tau + gravity_term_tau, tau, MotionGroup::Secondary);
    }
    return tau;
}

Eigen::VectorXd InvDynamicsInertiaTerm(const RobotModel& robot_model, pinocchio::Data& data, const VectorDof& q, const VectorDof& dq, const VectorDof& ddq, MotionGroup motion_group, bool is_anthropmorphic)
{
    Eigen::VectorXd q_arm, dq_arm, ddq_arm, inertia_term_tau, dynamics_base_parameters;
    Eigen::MatrixXd invdyn_regressor_matrix, invdyn_regressor_matrix_without_inertia;
    std::string first_joint_name;
    utils::GetJointFromWholeBody(robot_model, q, q_arm, motion_group);
    utils::GetJointFromWholeBody(robot_model, dq, dq_arm, motion_group);
    utils::GetJointFromWholeBody(robot_model, ddq, ddq_arm, motion_group);
    Vector3d gravity_acc_world = Vector3d::Zero();
    gravity_acc_world[2] = -9.81;
    if (motion_group == MotionGroup::Primary)
    {
        int first_joint_id = robot_model.params_ptr->joints_primary[0];
        first_joint_name = robot_model.params_ptr->joint_command_sequence_by_name[first_joint_id];
        // 计算arm base坐标系下的重力方向
        auto pose = pinocchio::GetFkByFrameName(*robot_model.model_ptr, data, q, first_joint_name);
        Vector3d gravity_acc_armbase = pose.rotation().transpose() * gravity_acc_world;
        dynamics_base_parameters = robot_model.params_ptr->dynamics_base_parameters_primary.value();
        invdyn_regressor_matrix = InvDynamicRegressorPrimary(is_anthropmorphic, gravity_acc_armbase, q_arm, dq_arm, ddq_arm);
        invdyn_regressor_matrix_without_inertia = InvDynamicRegressorPrimary(is_anthropmorphic, gravity_acc_armbase, q_arm, dq_arm, Eigen::VectorXd::Zero(q_arm.size()));
    }
    else
    {
        int first_joint_id = robot_model.params_ptr->joints_secondary[0];
        first_joint_name = robot_model.params_ptr->joint_command_sequence_by_name[first_joint_id];
        // 计算arm base坐标系下的重力方向
        auto pose = pinocchio::GetFkByFrameName(*robot_model.model_ptr, data, q, first_joint_name);
        Vector3d gravity_acc_armbase = pose.rotation().transpose() * gravity_acc_world;
        dynamics_base_parameters = robot_model.params_ptr->dynamics_base_parameters_secondary.value();
        invdyn_regressor_matrix = InvDynamicRegressorSecondary(is_anthropmorphic, gravity_acc_armbase, q_arm, dq_arm, ddq_arm);
        invdyn_regressor_matrix_without_inertia = InvDynamicRegressorSecondary(is_anthropmorphic, gravity_acc_armbase, q_arm, dq_arm, Eigen::VectorXd::Zero(q_arm.size()));
    }
    // 计算回归矩阵
    inertia_term_tau = invdyn_regressor_matrix * dynamics_base_parameters - invdyn_regressor_matrix_without_inertia * dynamics_base_parameters;
    return inertia_term_tau;
}

Eigen::VectorXd InvDynamicsCoriolisTerm(const RobotModel& robot_model, pinocchio::Data& data, const VectorDof& q, const VectorDof& dq, MotionGroup motion_group, bool is_anthropmorphic)
{
    Eigen::VectorXd q_arm, dq_arm, coriolis_term_tau, dynamics_base_parameters;
    Eigen::MatrixXd invdyn_regressor_matrix, invdyn_regressor_matrix_without_gravity;
    std::string first_joint_name;
    utils::GetJointFromWholeBody(robot_model, q, q_arm, motion_group);
    utils::GetJointFromWholeBody(robot_model, dq, dq_arm, motion_group);
    Vector3d gravity_acc_world = Vector3d::Zero();
    gravity_acc_world[2] = -9.81;
    if (motion_group == MotionGroup::Primary)
    {
        int first_joint_id = robot_model.params_ptr->joints_primary[0];
        first_joint_name = robot_model.params_ptr->joint_command_sequence_by_name[first_joint_id];
        // 计算arm base坐标系下的重力方向
        auto pose = pinocchio::GetFkByFrameName(*robot_model.model_ptr, data, q, first_joint_name);
        Vector3d gravity_acc_armbase = pose.rotation().transpose() * gravity_acc_world;
        dynamics_base_parameters = robot_model.params_ptr->dynamics_base_parameters_primary.value();
        invdyn_regressor_matrix = InvDynamicRegressorPrimary(is_anthropmorphic, gravity_acc_armbase, q_arm, dq_arm, Eigen::VectorXd::Zero(q_arm.size()));
    }
    else
    {
        int first_joint_id = robot_model.params_ptr->joints_secondary[0];
        first_joint_name = robot_model.params_ptr->joint_command_sequence_by_name[first_joint_id];
        // 计算arm base坐标系下的重力方向
        auto pose = pinocchio::GetFkByFrameName(*robot_model.model_ptr, data, q, first_joint_name);
        Vector3d gravity_acc_armbase = pose.rotation().transpose() * gravity_acc_world;
        dynamics_base_parameters = robot_model.params_ptr->dynamics_base_parameters_secondary.value();
        invdyn_regressor_matrix = InvDynamicRegressorSecondary(is_anthropmorphic, gravity_acc_armbase, q_arm, dq_arm, Eigen::VectorXd::Zero(q_arm.size()));
    }
    // 计算回归矩阵
    coriolis_term_tau = invdyn_regressor_matrix * dynamics_base_parameters - InvDynamicsGravityTerm(robot_model, data, q, motion_group, is_anthropmorphic);
    return coriolis_term_tau;
}

Eigen::VectorXd InvDynamicsGravityTerm(const RobotModel& robot_model, pinocchio::Data& data, const VectorDof& q, MotionGroup motion_group, bool is_anthropmorphic)
{
    Eigen::VectorXd q_arm, gravity_term_tau, dynamics_base_parameters;
    Eigen::MatrixXd invdyn_regressor_matrix;
    std::string first_joint_name;
    utils::GetJointFromWholeBody(robot_model, q, q_arm, motion_group);
    Vector3d gravity_acc_world = Vector3d::Zero();
    gravity_acc_world[2] = -9.81;
    if (motion_group == MotionGroup::Primary)
    {
        int first_joint_id = robot_model.params_ptr->joints_primary[0];
        first_joint_name = robot_model.params_ptr->joint_command_sequence_by_name[first_joint_id];
        // 计算arm base坐标系下的重力方向
        auto pose = pinocchio::GetFkByFrameName(*robot_model.model_ptr, data, q, first_joint_name);
        Vector3d gravity_acc_armbase = pose.rotation().transpose() * gravity_acc_world;
        dynamics_base_parameters = robot_model.params_ptr->dynamics_base_parameters_primary.value();
        invdyn_regressor_matrix = InvDynamicRegressorPrimary(is_anthropmorphic, gravity_acc_armbase, q_arm, Eigen::VectorXd::Zero(q_arm.size()), Eigen::VectorXd::Zero(q_arm.size()));
    }
    else
    {
        int first_joint_id = robot_model.params_ptr->joints_secondary[0];
        first_joint_name = robot_model.params_ptr->joint_command_sequence_by_name[first_joint_id];
        // 计算arm base坐标系下的重力方向
        auto pose = pinocchio::GetFkByFrameName(*robot_model.model_ptr, data, q, first_joint_name);
        Vector3d gravity_acc_armbase = pose.rotation().transpose() * gravity_acc_world;
        dynamics_base_parameters = robot_model.params_ptr->dynamics_base_parameters_secondary.value();
        invdyn_regressor_matrix = InvDynamicRegressorSecondary(is_anthropmorphic, gravity_acc_armbase, q_arm, Eigen::VectorXd::Zero(q_arm.size()), Eigen::VectorXd::Zero(q_arm.size()));
    }
    gravity_term_tau = invdyn_regressor_matrix * dynamics_base_parameters;
    return gravity_term_tau;
}

Eigen::VectorXd ForwardDynamics(const pinocchio::Model& model, pinocchio::Data& data, const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
                                const Eigen::VectorXd& torque_cmd, const std::vector<std::string>& joint_command_sequence_by_name)
{
    // 将输入数据映射到URDF顺序
    Eigen::VectorXd q_urdf = MapToUrdfOrder(model, q, joint_command_sequence_by_name);
    Eigen::VectorXd dq_urdf = MapToUrdfOrder(model, dq, joint_command_sequence_by_name);
    Eigen::VectorXd torque_urdf = MapToUrdfOrder(model, torque_cmd, joint_command_sequence_by_name);

    // 计算正动力学
    auto ddq_urdf = pinocchio::aba(model, data, q_urdf, dq_urdf, torque_urdf);

    // 将结果映射回joint_command_sequence_by_name顺序
    auto ddq = MapToParamOrder(model, ddq_urdf, joint_command_sequence_by_name);
    return ddq;
}

Eigen::VectorXd MapToUrdfOrder(const pinocchio::Model& model, const Eigen::VectorXd& input,
                               const std::vector<std::string>& joint_command_sequence_by_name)
{
    Eigen::VectorXd output = Eigen::VectorXd::Zero(model.nv);

    for (size_t i = 0; i < joint_command_sequence_by_name.size(); ++i)
    {
        const std::string& joint_name = joint_command_sequence_by_name[i];
        if (model.existJointName(joint_name))
        {
            int joint_id = model.getJointId(joint_name);
            int urdf_index = joint_id - 1;

            if (urdf_index >= 0 && urdf_index < model.nv && i < size_t(input.size()))
            {
                output[urdf_index] = input[i];
            }
        }
    }

    return output;
}

Eigen::VectorXd MapToParamOrder(const pinocchio::Model& model, const Eigen::VectorXd& input,
                                const std::vector<std::string>& joint_command_sequence_by_name)
{
    Eigen::VectorXd output = Eigen::VectorXd::Zero(joint_command_sequence_by_name.size());

    for (size_t i = 0; i < joint_command_sequence_by_name.size(); ++i)
    {
        const std::string& joint_name = joint_command_sequence_by_name[i];
        if (model.existJointName(joint_name))
        {
            int joint_id = model.getJointId(joint_name);
            int urdf_index = joint_id - 1;

            if (urdf_index >= 0 && urdf_index < model.nv && urdf_index < input.size())
            {
                output[i] = input[urdf_index];
            }
        }
    }

    return output;
}

Eigen::MatrixXd InvDynamicRegressorPrimary(bool is_anthropmorphic, const Vector3d& gravity_acc, const Eigen::VectorXd& q, const Eigen::VectorXd& dq, const Eigen::VectorXd& ddq)
{
    Eigen::VectorXd Y_out = Eigen::VectorXd::Zero(497);
    // 由sympybotics生成的回归矩阵代码
    double x0 = -ddq[0];
    double x1 = sin(q[0]);
    double x2 = cos(q[0]);
    double x3 = gravity_acc[0]*x1 - gravity_acc[1]*x2;
    double x4 = -x3;
    double x5 = -gravity_acc[1]*x1 - gravity_acc[0]*x2;
    double x6 = sin(q[1]);
    double x7 = -dq[0];
    double x8 = cos(q[1]);
    double x9 = -x8;
    double x10 = x7*x9;
    double x11 = -x10;
    double x12 = dq[1]*x11;
    double x13 = x0*x6 + x12;
    double x14 = x6*x7;
    double x15 = dq[1]*x14;
    double x16 = -x15;
    double x17 = ((x14)*(x14));
    double x18 = ((dq[1])*(dq[1]));
    double x19 = x0*x9 + x15;
    double x20 = ((x10)*(x10));
    double x21 = x10*x14;
    double x22 = cos(q[2]);
    double x23 = sin(q[2]);
    double x24 = ddq[1]*x23;
    double x25 = x13*x22;
    double x26 = dq[1]*x22;
    double x27 = x14*x23;
    double x28 = x26 - x27;
    double x29 = -dq[2];
    double x30 = -x29;
    double x31 = x24 + x25 + x28*x30;
    double x32 = dq[1]*x23;
    double x33 = x14*x22;
    double x34 = x32 + x33;
    double x35 = x11 + x29;
    double x36 = x34*x35;
    double x37 = -x36;
    double x38 = -x23;
    double x39 = x28*x34;
    double x40 = -x39;
    double x41 = -ddq[2] - x19;
    double x42 = x40 + x41;
    double x43 = ((x34)*(x34));
    double x44 = ((x35)*(x35));
    double x45 = -x44;
    double x46 = x43 + x45;
    double x47 = x28*x35;
    double x48 = -x31 - x47;
    double x49 = ddq[1]*x22;
    double x50 = x13*x23;
    double x51 = x29*x34 + x49 - x50;
    double x52 = x36 + x51;
    double x53 = -x47;
    double x54 = x31 + x53;
    double x55 = ((x28)*(x28));
    double x56 = -x43;
    double x57 = -x55 - x56;
    double x58 = -x41;
    double x59 = -x55;
    double x60 = x44 + x59;
    double x61 = x39 + x41;
    double x62 = -x37 - x51;
    double x63 = 0.26*x26 - 0.26*x27;
    double x64 = x28*x63;
    double x65 = -0.26*x32 - 0.26*x33;
    double x66 = -x65;
    double x67 = -x5*x6 + gravity_acc[2]*x8;
    double x68 = x34*x66 + x64 + x67;
    double x69 = x45 + x59;
    double x70 = -0.26*x23;
    double x71 = x39 + x58;
    double x72 = x5*x8 + gravity_acc[2]*x6;
    double x73 = x22*x3 - x23*x72 - 0.26*x24 - 0.26*x25 + x29*x63;
    double x74 = -x73;
    double x75 = -x35*x63 - x74;
    double x76 = x34*x65 - x64 - x67;
    double x77 = x45 + x56;
    double x78 = x22*x72 + x23*x3 + x30*x65 + 0.26*x49 - 0.26*x50;
    double x79 = -x35*x65 - x78;
    double x80 = cos(q[3]);
    double x81 = sin(q[3]);
    double x82 = -dq[3];
    double x83 = -x80;
    double x84 = x34*x81 + x35*x83;
    double x85 = x31*x80 + x41*x81 + x82*x84;
    double x86 = x34*x80 + x35*x81;
    double x87 = dq[3] + x28;
    double x88 = x86*x87;
    double x89 = -x88;
    double x90 = x80*x85 + x81*x89;
    double x91 = x84*x86;
    double x92 = -x81*x85 - x83*x89;
    double x93 = ddq[3] + x51;
    double x94 = -x84;
    double x95 = x86*x94 + x93;
    double x96 = ((x86)*(x86));
    double x97 = ((x87)*(x87));
    double x98 = -x97;
    double x99 = x96 + x98;
    double x100 = x80*x95 + x81*x99;
    double x101 = x84*x87;
    double x102 = x101 + x85;
    double x103 = -x81*x95 - x83*x99;
    double x104 = dq[3]*x86 + x31*x81 + x41*x83;
    double x105 = x104 + x88;
    double x106 = x87*x94;
    double x107 = x106 + x85;
    double x108 = x105*x80 + x107*x81;
    double x109 = ((x84)*(x84));
    double x110 = -x96;
    double x111 = x109 + x110;
    double x112 = -x105*x81 - x107*x83;
    double x113 = x106*x80 + x81*x88;
    double x114 = -x106*x81 - x83*x88;
    double x115 = -x109;
    double x116 = x115 + x97;
    double x117 = x91 + x93;
    double x118 = x116*x80 + x117*x81;
    double x119 = x104 + x89;
    double x120 = -x116*x81 - x117*x83;
    double x121 = x63*x80;
    double x122 = x63*x81;
    double x123 = -x122;
    double x124 = x121*x84 + x123*x86 + x73;
    double x125 = x124*x81;
    double x126 = -dq[3]*x121 - x67*x83 - x78*x81;
    double x127 = x121*x87 + x126 + x66*x86;
    double x128 = x80*(x115 + x98) + x81*(x91 - x93);
    double x129 = -x124*x83;
    double x130 = x121*x94 + x122*x86 + x74;
    double x131 = x130*x80;
    double x132 = x122*x82 + x67*x81 + x78*x80;
    double x133 = x122*x87 + x132 + x65*x94;
    double x134 = x101 - x85;
    double x135 = x117*x80 + x81*(x110 + x98);
    double x136 = -x130*x81;
    double x137 = cos(q[4]);
    double x138 = x137*x86;
    double x139 = sin(q[4]);
    double x140 = x139*x87;
    double x141 = x138 + x140;
    double x142 = x137*x87;
    double x143 = x139*x86;
    double x144 = x142 - x143;
    double x145 = x141*x144;
    double x146 = -x145;
    double x147 = x137*x85;
    double x148 = x139*x93;
    double x149 = -dq[4];
    double x150 = -x149;
    double x151 = x144*x150 + x147 + x148;
    double x152 = -x139;
    double x153 = x149 + x94;
    double x154 = x141*x153;
    double x155 = -x154;
    double x156 = x137*x151 + x152*x155;
    double x157 = x146*x81 + x156*x80;
    double x158 = x137*x155 + x139*x151;
    double x159 = -x146*x83 - x156*x81;
    double x160 = x144*x153;
    double x161 = -x151 - x160;
    double x162 = -ddq[4] - x104;
    double x163 = x146 + x162;
    double x164 = ((x141)*(x141));
    double x165 = ((x153)*(x153));
    double x166 = -x165;
    double x167 = x164 + x166;
    double x168 = x137*x163 + x152*x167;
    double x169 = x161*x81 + x168*x80;
    double x170 = x137*x167 + x139*x163;
    double x171 = -x161*x83 - x168*x81;
    double x172 = ((x144)*(x144));
    double x173 = -x164;
    double x174 = -x172 - x173;
    double x175 = x137*x93;
    double x176 = x141*x149 + x152*x85 + x175;
    double x177 = x154 + x176;
    double x178 = -x160;
    double x179 = x151 + x178;
    double x180 = x137*x177 + x152*x179;
    double x181 = x174*x81 + x180*x80;
    double x182 = x137*x179 + x139*x177;
    double x183 = -x174*x83 - x180*x81;
    double x184 = x137*x178 + x152*x154;
    double x185 = -x162;
    double x186 = x184*x80 + x185*x81;
    double x187 = x137*x154 + x139*x178;
    double x188 = -x184*x81 - x185*x83;
    double x189 = -x155 - x176;
    double x190 = -x172;
    double x191 = x165 + x190;
    double x192 = x145 + x162;
    double x193 = x137*x191 + x152*x192;
    double x194 = x189*x81 + x193*x80;
    double x195 = x137*x192;
    double x196 = x139*x191 + x195;
    double x197 = -x189*x83 - x193*x81;
    double x198 = x121*x137 + x139*x65 + 0.264*x142 - 0.264*x143;
    double x199 = x132*x152 + x137*x73 - 0.264*x147 - 0.264*x148 + x149*x198;
    double x200 = -x199;
    double x201 = -x141;
    double x202 = -x123*x201 - x153*x198 - x200;
    double x203 = x144*x198;
    double x204 = x121*x152 + x137*x65 - 0.264*x138 - 0.264*x140;
    double x205 = x126 + x201*x204 + x203;
    double x206 = x166 + x190;
    double x207 = -0.264*x139;
    double x208 = x145 + x185;
    double x209 = x137*x208;
    double x210 = x152*x205 + x206*x207 - 0.264*x209;
    double x211 = x202*x81 + x210*x80;
    double x212 = x137*x206;
    double x213 = x137*x205 + x207*x208 + 0.264*x212;
    double x214 = x139*x206 + x209;
    double x215 = -x177*x81 + x80*(x152*x208 + x212);
    double x216 = -x202*x83 - x210*x81;
    double x217 = x132*x137 + x139*x73 + x150*x204 + 0.264*x175 + x207*x85;
    double x218 = x123*x144 - x153*x204 - x217;
    double x219 = -x126 + x141*x204 - x203;
    double x220 = x166 + x173;
    double x221 = x137*x220;
    double x222 = x137*x219 + x192*x207 - 0.264*x221;
    double x223 = x218*x81 + x222*x80;
    double x224 = x139*x219 + 0.264*x195 + x207*x220;
    double x225 = x139*x192 + x221;
    double x226 = x80*(x152*x220 + x195) + x81*(x151 - x160);
    double x227 = -x218*x83 - x222*x81;
    double x228 = sin(q[5]);
    double x229 = cos(q[5]);
    double x230 = -x229;
    double x231 = x141*x228 + x153*x230;
    double x232 = -dq[5];
    double x233 = x151*x229 + x162*x228 + x231*x232;
    double x234 = x141*x229 + x153*x228;
    double x235 = dq[5] + x144;
    double x236 = x234*x235;
    double x237 = -x236;
    double x238 = -x228*x233 - x230*x237;
    double x239 = x228*x237 + x229*x233;
    double x240 = x231*x234;
    double x241 = x137*x239 + x152*x240;
    double x242 = x238*x81 + x241*x80;
    double x243 = x137*x240 + x139*x239;
    double x244 = -x238*x83 - x241*x81;
    double x245 = ddq[5] + x176;
    double x246 = -x240 + x245;
    double x247 = ((x234)*(x234));
    double x248 = ((x235)*(x235));
    double x249 = -x248;
    double x250 = x247 + x249;
    double x251 = -x228*x246 - x230*x250;
    double x252 = x228*x250 + x229*x246;
    double x253 = x231*x235;
    double x254 = x233 + x253;
    double x255 = x137*x252 + x152*x254;
    double x256 = x251*x81 + x255*x80;
    double x257 = x137*x254 + x139*x252;
    double x258 = -x251*x83 - x255*x81;
    double x259 = dq[5]*x234 + x151*x228 + x162*x230;
    double x260 = x236 + x259;
    double x261 = -x253;
    double x262 = x233 + x261;
    double x263 = -x228*x260 - x230*x262;
    double x264 = x228*x262 + x229*x260;
    double x265 = ((x231)*(x231));
    double x266 = -x247;
    double x267 = x265 + x266;
    double x268 = x137*x264 + x152*x267;
    double x269 = x263*x81 + x268*x80;
    double x270 = x137*x267 + x139*x264;
    double x271 = -x263*x83 - x268*x81;
    double x272 = -x228*x261 - x230*x236;
    double x273 = x228*x236 + x229*x261;
    double x274 = x137*x273 + x152*x245;
    double x275 = x272*x81 + x274*x80;
    double x276 = x137*x245 + x139*x273;
    double x277 = -x272*x83 - x274*x81;
    double x278 = -x265;
    double x279 = x248 + x278;
    double x280 = x240 + x245;
    double x281 = -x228*x279 - x230*x280;
    double x282 = x228*x280;
    double x283 = x229*x279 + x282;
    double x284 = x237 + x259;
    double x285 = x137*x283 + x152*x284;
    double x286 = x281*x81 + x285*x80;
    double x287 = x137*x284 + x139*x283;
    double x288 = -x281*x83 - x285*x81;
    double x289 = x123*x228 + x198*x229;
    double x290 = x231*x289;
    double x291 = -x234;
    double x292 = x123*x230 + x198*x228;
    double x293 = x199 + x290 + x291*x292;
    double x294 = -x230*x293;
    double x295 = x228*x293;
    double x296 = dq[5]*x289 + x126*x230 + x217*x228;
    double x297 = -x296;
    double x298 = x204*x291 + x235*x289 + x297;
    double x299 = x137*x260;
    double x300 = x249 + x278;
    double x301 = x240 - x245;
    double x302 = x228*x301 + x229*x300;
    double x303 = x137*x295 + x152*x298 + x207*x302 - 0.264*x299;
    double x304 = x294*x81 + x303*x80;
    double x305 = x137*x302;
    double x306 = x137*x298 + x139*x295 + x207*x260 + 0.264*x305;
    double x307 = x139*x302 + x299;
    double x308 = x80*(x152*x260 + x305) + x81*(-x228*x300 - x230*x301);
    double x309 = -x294*x83 - x303*x81;
    double x310 = x200 + x234*x292 - x290;
    double x311 = -x228*x310;
    double x312 = x229*x310;
    double x313 = x126*x228 + x217*x229 + x232*x292;
    double x314 = -x204*x231 + x235*x292 + x313;
    double x315 = -x233 + x253;
    double x316 = x137*x315;
    double x317 = x249 + x266;
    double x318 = x228*x317 + x229*x280;
    double x319 = x137*x312 + x152*x314 + x207*x318 - 0.264*x316;
    double x320 = x311*x81 + x319*x80;
    double x321 = x137*x318;
    double x322 = x137*x314 + x139*x312 + x207*x315 + 0.264*x321;
    double x323 = x139*x318 + x316;
    double x324 = x80*(x152*x315 + x321) + x81*(-x230*x317 - x282);
    double x325 = -x311*x83 - x319*x81;
    double x326 = cos(q[6]);
    double x327 = sin(q[6]);
    double x328 = -x327;
    double x329 = x234*x328 + x235*x326;
    double x330 = dq[6]*x329 + x233*x326 + x245*x327;
    double x331 = x234*x326 + x235*x327;
    double x332 = dq[6] + x231;
    double x333 = x331*x332;
    double x334 = x326*x330 + x328*x333;
    double x335 = x329*x331;
    double x336 = -x335;
    double x337 = -x228*x334 - x230*x336;
    double x338 = x228*x336 + x229*x334;
    double x339 = x326*x333 + x327*x330;
    double x340 = x137*x338 + x152*x339;
    double x341 = x337*x81 + x340*x80;
    double x342 = x137*x339 + x139*x338;
    double x343 = -x337*x83 - x340*x81;
    double x344 = -x333;
    double x345 = -dq[6];
    double x346 = x233*x328 + x245*x326 + x331*x345;
    double x347 = x344 + x346;
    double x348 = x329*x332;
    double x349 = x330 + x348;
    double x350 = x326*x347 + x328*x349;
    double x351 = ((x331)*(x331));
    double x352 = ((x329)*(x329));
    double x353 = -x352;
    double x354 = x351 + x353;
    double x355 = -x228*x350 - x230*x354;
    double x356 = x228*x354 + x229*x350;
    double x357 = x326*x349 + x327*x347;
    double x358 = x137*x356 + x152*x357;
    double x359 = x355*x81 + x358*x80;
    double x360 = x137*x357 + x139*x356;
    double x361 = -x355*x83 - x358*x81;
    double x362 = ddq[6] + x259;
    double x363 = x335 + x362;
    double x364 = x326*x363;
    double x365 = ((x332)*(x332));
    double x366 = -x351;
    double x367 = x365 + x366;
    double x368 = x328*x367 + x364;
    double x369 = x330 - x348;
    double x370 = -x228*x368 - x230*x369;
    double x371 = x228*x369 + x229*x368;
    double x372 = x326*x367 + x327*x363;
    double x373 = x137*x371 + x152*x372;
    double x374 = x370*x81 + x373*x80;
    double x375 = x137*x372 + x139*x371;
    double x376 = -x370*x83 - x373*x81;
    double x377 = -x365;
    double x378 = x352 + x377;
    double x379 = x336 + x362;
    double x380 = x326*x378 + x328*x379;
    double x381 = x333 + x346;
    double x382 = -x228*x380 - x230*x381;
    double x383 = x228*x381 + x229*x380;
    double x384 = x326*x379 + x327*x378;
    double x385 = x137*x383 + x152*x384;
    double x386 = x382*x81 + x385*x80;
    double x387 = x137*x384 + x139*x383;
    double x388 = -x382*x83 - x385*x81;
    double x389 = x326*x348 + x328*x344;
    double x390 = -x228*x389 - x230*x362;
    double x391 = x228*x362 + x229*x389;
    double x392 = x326*x344 + x327*x348;
    double x393 = x137*x391 + x152*x392;
    double x394 = x390*x81 + x393*x80;
    double x395 = x137*x392 + x139*x391;
    double x396 = -x390*x83 - x393*x81;
    double x397 = x204*x327 + x289*x326;
    double x398 = x329*x397;
    double x399 = -x331;
    double x400 = x204*x326 + x289*x328;
    double x401 = x297 + x398 + x399*x400;
    double x402 = x328*x401;
    double x403 = x199*x326 + x292*x399 + x313*x328 + x332*x397 + x345*x397;
    double x404 = -x228*x402 - x230*x403;
    double x405 = x228*x403 + x229*x402;
    double x406 = x326*x401;
    double x407 = x353 + x377;
    double x408 = x327*x407 + x364;
    double x409 = x137*x408;
    double x410 = x333 - x346;
    double x411 = x326*x407 + x328*x363;
    double x412 = x228*x410 + x229*x411;
    double x413 = x137*x405 + x152*x406 + x207*x412 - 0.264*x409;
    double x414 = x404*x81 + x413*x80;
    double x415 = x137*x412;
    double x416 = x137*x406 + x139*x405 + x207*x408 + 0.264*x415;
    double x417 = x139*x412 + x409;
    double x418 = x80*(x152*x408 + x415) + x81*(-x228*x411 - x230*x410);
    double x419 = -x404*x83 - x413*x81;
    double x420 = x296 + x331*x400 - x398;
    double x421 = x326*x420;
    double x422 = -dq[6]*x400 - x199*x327 - x292*x329 - x313*x326 + x332*x400;
    double x423 = -x228*x421 - x230*x422;
    double x424 = x228*x422 + x229*x421;
    double x425 = x327*x420;
    double x426 = x335 - x362;
    double x427 = x366 + x377;
    double x428 = x326*x427 + x327*x426;
    double x429 = x137*x428;
    double x430 = x326*x426 + x328*x427;
    double x431 = x228*x349 + x229*x430;
    double x432 = x137*x424 + x152*x425 + x207*x431 - 0.264*x429;
    double x433 = x423*x81 + x432*x80;
    double x434 = x137*x431;
    double x435 = x137*x425 + x139*x424 + x207*x428 + 0.264*x434;
    double x436 = x139*x431 + x429;
    double x437 = x80*(x152*x428 + x434) + x81*(-x228*x430 - x230*x349);
    double x438 = -x423*x83 - x432*x81;
    double x439 = x22*x61;
    //
    Y_out[0] = -x0;
    Y_out[1] = -x4;
    Y_out[2] = -x5;
    Y_out[3] = dq[0];
    Y_out[4] = (((dq[0]) > 0) - ((dq[0]) < 0));
    Y_out[5] = 1;
    Y_out[6] = -x13*x6 - x16*x9;
    Y_out[7] = -x6*(ddq[1] + x11*x14) - x9*(x17 - x18);
    Y_out[8] = -x6*(x15 + x19) - x9*(x12 + x13);
    Y_out[9] = -x12*x6 - x15*x9;
    Y_out[10] = -x6*(x18 - x20) - x9*(ddq[1] + x21);
    Y_out[11] = -x3*x9;
    Y_out[12] = -x4*x6;
    Y_out[13] = 0;
    Y_out[14] = 0;
    Y_out[15] = 0;
    Y_out[16] = -x40*x9 - x6*(x22*x31 + x37*x38);
    Y_out[17] = -x48*x9 - x6*(x22*x42 + x38*x46);
    Y_out[18] = -x57*x9 - x6*(x22*x52 + x38*x54);
    Y_out[19] = -x58*x9 - x6*(x22*x53 + x36*x38);
    Y_out[20] = -x6*(x22*x60 + x38*x61) - x62*x9;
    Y_out[21] = -x6*(-0.26*x22*x71 + x38*x68 + x69*x70) - x75*x9;
    Y_out[22] = -x6*(x22*x76 - 0.26*x22*x77 + x61*x70) - x79*x9;
    Y_out[23] = 0;
    Y_out[24] = 0;
    Y_out[25] = 0;
    Y_out[26] = 0;
    Y_out[27] = -x6*(x22*x90 + x38*x91) - x9*x92;
    Y_out[28] = -x103*x9 - x6*(x100*x22 + x102*x38);
    Y_out[29] = -x112*x9 - x6*(x108*x22 + x111*x38);
    Y_out[30] = -x114*x9 - x6*(x113*x22 + x38*x93);
    Y_out[31] = -x120*x9 - x6*(x118*x22 + x119*x38);
    Y_out[32] = -x129*x9 - x6*(-0.26*x105*x22 + x125*x22 + x127*x38 + x128*x70);
    Y_out[33] = -x136*x9 - x6*(x131*x22 + x133*x38 - 0.26*x134*x22 + x135*x70);
    Y_out[34] = 0;
    Y_out[35] = 0;
    Y_out[36] = 0;
    Y_out[37] = 0;
    Y_out[38] = -x159*x9 - x6*(x157*x22 + x158*x38);
    Y_out[39] = -x171*x9 - x6*(x169*x22 + x170*x38);
    Y_out[40] = -x183*x9 - x6*(x181*x22 + x182*x38);
    Y_out[41] = -x188*x9 - x6*(x186*x22 + x187*x38);
    Y_out[42] = -x197*x9 - x6*(x194*x22 + x196*x38);
    Y_out[43] = -x216*x9 - x6*(x211*x22 + x213*x38 - 0.26*x214*x22 + x215*x70);
    Y_out[44] = -x227*x9 - x6*(x22*x223 - 0.26*x22*x225 + x224*x38 + x226*x70);
    Y_out[45] = 0;
    Y_out[46] = 0;
    Y_out[47] = 0;
    Y_out[48] = 0;
    Y_out[49] = -x244*x9 - x6*(x22*x242 + x243*x38);
    Y_out[50] = -x258*x9 - x6*(x22*x256 + x257*x38);
    Y_out[51] = -x271*x9 - x6*(x22*x269 + x270*x38);
    Y_out[52] = -x277*x9 - x6*(x22*x275 + x276*x38);
    Y_out[53] = -x288*x9 - x6*(x22*x286 + x287*x38);
    Y_out[54] = -x309*x9 - x6*(x22*x304 - 0.26*x22*x307 + x306*x38 + x308*x70);
    Y_out[55] = -x325*x9 - x6*(x22*x320 - 0.26*x22*x323 + x322*x38 + x324*x70);
    Y_out[56] = 0;
    Y_out[57] = 0;
    Y_out[58] = 0;
    Y_out[59] = 0;
    Y_out[60] = -x343*x9 - x6*(x22*x341 + x342*x38);
    Y_out[61] = -x361*x9 - x6*(x22*x359 + x360*x38);
    Y_out[62] = -x376*x9 - x6*(x22*x374 + x375*x38);
    Y_out[63] = -x388*x9 - x6*(x22*x386 + x38*x387);
    Y_out[64] = -x396*x9 - x6*(x22*x394 + x38*x395);
    Y_out[65] = -x419*x9 - x6*(x22*x414 - 0.26*x22*x417 + x38*x416 + x418*x70);
    Y_out[66] = -x438*x9 - x6*(x22*x433 - 0.26*x22*x436 + x38*x435 + x437*x70);
    Y_out[67] = 0;
    Y_out[68] = 0;
    Y_out[69] = 0;
    Y_out[70] = 0;
    Y_out[71] = 0;
    Y_out[72] = 0;
    Y_out[73] = 0;
    Y_out[74] = 0;
    Y_out[75] = 0;
    Y_out[76] = 0;
    Y_out[77] = x21;
    Y_out[78] = dq[1]*x10 + x13;
    Y_out[79] = -x17 + x20;
    Y_out[80] = ddq[1];
    Y_out[81] = x16 + x19;
    Y_out[82] = x67;
    Y_out[83] = x72;
    Y_out[84] = dq[1];
    Y_out[85] = (((dq[1]) > 0) - ((dq[1]) < 0));
    Y_out[86] = 1;
    Y_out[87] = x22*x37 + x23*x31;
    Y_out[88] = x22*x46 + x23*x42;
    Y_out[89] = x22*x54 + x23*x52;
    Y_out[90] = x22*x36 + x23*x53;
    Y_out[91] = x23*x60 + x439;
    Y_out[92] = x22*x68 + 0.26*x22*x69 + x70*x71;
    Y_out[93] = x23*x76 + 0.26*x439 + x70*x77;
    Y_out[94] = 0;
    Y_out[95] = 0;
    Y_out[96] = 0;
    Y_out[97] = 0;
    Y_out[98] = x22*x91 + x23*x90;
    Y_out[99] = x100*x23 + x102*x22;
    Y_out[100] = x108*x23 + x111*x22;
    Y_out[101] = x113*x23 + x22*x93;
    Y_out[102] = x118*x23 + x119*x22;
    Y_out[103] = x105*x70 + x125*x23 + x127*x22 + 0.26*x128*x22;
    Y_out[104] = x131*x23 + x133*x22 + x134*x70 + 0.26*x135*x22;
    Y_out[105] = 0;
    Y_out[106] = 0;
    Y_out[107] = 0;
    Y_out[108] = 0;
    Y_out[109] = x157*x23 + x158*x22;
    Y_out[110] = x169*x23 + x170*x22;
    Y_out[111] = x181*x23 + x182*x22;
    Y_out[112] = x186*x23 + x187*x22;
    Y_out[113] = x194*x23 + x196*x22;
    Y_out[114] = x211*x23 + x213*x22 + x214*x70 + 0.26*x215*x22;
    Y_out[115] = x22*x224 + 0.26*x22*x226 + x223*x23 + x225*x70;
    Y_out[116] = 0;
    Y_out[117] = 0;
    Y_out[118] = 0;
    Y_out[119] = 0;
    Y_out[120] = x22*x243 + x23*x242;
    Y_out[121] = x22*x257 + x23*x256;
    Y_out[122] = x22*x270 + x23*x269;
    Y_out[123] = x22*x276 + x23*x275;
    Y_out[124] = x22*x287 + x23*x286;
    Y_out[125] = x22*x306 + 0.26*x22*x308 + x23*x304 + x307*x70;
    Y_out[126] = x22*x322 + 0.26*x22*x324 + x23*x320 + x323*x70;
    Y_out[127] = 0;
    Y_out[128] = 0;
    Y_out[129] = 0;
    Y_out[130] = 0;
    Y_out[131] = x22*x342 + x23*x341;
    Y_out[132] = x22*x360 + x23*x359;
    Y_out[133] = x22*x375 + x23*x374;
    Y_out[134] = x22*x387 + x23*x386;
    Y_out[135] = x22*x395 + x23*x394;
    Y_out[136] = x22*x416 + 0.26*x22*x418 + x23*x414 + x417*x70;
    Y_out[137] = x22*x435 + 0.26*x22*x437 + x23*x433 + x436*x70;
    Y_out[138] = 0;
    Y_out[139] = 0;
    Y_out[140] = 0;
    Y_out[141] = 0;
    Y_out[142] = 0;
    Y_out[143] = 0;
    Y_out[144] = 0;
    Y_out[145] = 0;
    Y_out[146] = 0;
    Y_out[147] = 0;
    Y_out[148] = 0;
    Y_out[149] = 0;
    Y_out[150] = 0;
    Y_out[151] = 0;
    Y_out[152] = 0;
    Y_out[153] = 0;
    Y_out[154] = 0;
    Y_out[155] = 0;
    Y_out[156] = 0;
    Y_out[157] = 0;
    Y_out[158] = x40;
    Y_out[159] = x48;
    Y_out[160] = x57;
    Y_out[161] = x58;
    Y_out[162] = x62;
    Y_out[163] = x75;
    Y_out[164] = x79;
    Y_out[165] = ddq[2];
    Y_out[166] = dq[2];
    Y_out[167] = (((dq[2]) > 0) - ((dq[2]) < 0));
    Y_out[168] = 1;
    Y_out[169] = x92;
    Y_out[170] = x103;
    Y_out[171] = x112;
    Y_out[172] = x114;
    Y_out[173] = x120;
    Y_out[174] = x129;
    Y_out[175] = x136;
    Y_out[176] = 0;
    Y_out[177] = 0;
    Y_out[178] = 0;
    Y_out[179] = 0;
    Y_out[180] = x159;
    Y_out[181] = x171;
    Y_out[182] = x183;
    Y_out[183] = x188;
    Y_out[184] = x197;
    Y_out[185] = x216;
    Y_out[186] = x227;
    Y_out[187] = 0;
    Y_out[188] = 0;
    Y_out[189] = 0;
    Y_out[190] = 0;
    Y_out[191] = x244;
    Y_out[192] = x258;
    Y_out[193] = x271;
    Y_out[194] = x277;
    Y_out[195] = x288;
    Y_out[196] = x309;
    Y_out[197] = x325;
    Y_out[198] = 0;
    Y_out[199] = 0;
    Y_out[200] = 0;
    Y_out[201] = 0;
    Y_out[202] = x343;
    Y_out[203] = x361;
    Y_out[204] = x376;
    Y_out[205] = x388;
    Y_out[206] = x396;
    Y_out[207] = x419;
    Y_out[208] = x438;
    Y_out[209] = 0;
    Y_out[210] = 0;
    Y_out[211] = 0;
    Y_out[212] = 0;
    Y_out[213] = 0;
    Y_out[214] = 0;
    Y_out[215] = 0;
    Y_out[216] = 0;
    Y_out[217] = 0;
    Y_out[218] = 0;
    Y_out[219] = 0;
    Y_out[220] = 0;
    Y_out[221] = 0;
    Y_out[222] = 0;
    Y_out[223] = 0;
    Y_out[224] = 0;
    Y_out[225] = 0;
    Y_out[226] = 0;
    Y_out[227] = 0;
    Y_out[228] = 0;
    Y_out[229] = 0;
    Y_out[230] = 0;
    Y_out[231] = 0;
    Y_out[232] = 0;
    Y_out[233] = 0;
    Y_out[234] = 0;
    Y_out[235] = 0;
    Y_out[236] = 0;
    Y_out[237] = 0;
    Y_out[238] = 0;
    Y_out[239] = 0;
    Y_out[240] = x91;
    Y_out[241] = x102;
    Y_out[242] = x111;
    Y_out[243] = x93;
    Y_out[244] = x119;
    Y_out[245] = x127;
    Y_out[246] = x133;
    Y_out[247] = ddq[3];
    Y_out[248] = dq[3];
    Y_out[249] = (((dq[3]) > 0) - ((dq[3]) < 0));
    Y_out[250] = 1;
    Y_out[251] = x158;
    Y_out[252] = x170;
    Y_out[253] = x182;
    Y_out[254] = x187;
    Y_out[255] = x196;
    Y_out[256] = x213;
    Y_out[257] = x224;
    Y_out[258] = 0;
    Y_out[259] = 0;
    Y_out[260] = 0;
    Y_out[261] = 0;
    Y_out[262] = x243;
    Y_out[263] = x257;
    Y_out[264] = x270;
    Y_out[265] = x276;
    Y_out[266] = x287;
    Y_out[267] = x306;
    Y_out[268] = x322;
    Y_out[269] = 0;
    Y_out[270] = 0;
    Y_out[271] = 0;
    Y_out[272] = 0;
    Y_out[273] = x342;
    Y_out[274] = x360;
    Y_out[275] = x375;
    Y_out[276] = x387;
    Y_out[277] = x395;
    Y_out[278] = x416;
    Y_out[279] = x435;
    Y_out[280] = 0;
    Y_out[281] = 0;
    Y_out[282] = 0;
    Y_out[283] = 0;
    Y_out[284] = 0;
    Y_out[285] = 0;
    Y_out[286] = 0;
    Y_out[287] = 0;
    Y_out[288] = 0;
    Y_out[289] = 0;
    Y_out[290] = 0;
    Y_out[291] = 0;
    Y_out[292] = 0;
    Y_out[293] = 0;
    Y_out[294] = 0;
    Y_out[295] = 0;
    Y_out[296] = 0;
    Y_out[297] = 0;
    Y_out[298] = 0;
    Y_out[299] = 0;
    Y_out[300] = 0;
    Y_out[301] = 0;
    Y_out[302] = 0;
    Y_out[303] = 0;
    Y_out[304] = 0;
    Y_out[305] = 0;
    Y_out[306] = 0;
    Y_out[307] = 0;
    Y_out[308] = 0;
    Y_out[309] = 0;
    Y_out[310] = 0;
    Y_out[311] = 0;
    Y_out[312] = 0;
    Y_out[313] = 0;
    Y_out[314] = 0;
    Y_out[315] = 0;
    Y_out[316] = 0;
    Y_out[317] = 0;
    Y_out[318] = 0;
    Y_out[319] = 0;
    Y_out[320] = 0;
    Y_out[321] = 0;
    Y_out[322] = x146;
    Y_out[323] = x161;
    Y_out[324] = x174;
    Y_out[325] = x185;
    Y_out[326] = x189;
    Y_out[327] = x202;
    Y_out[328] = x218;
    Y_out[329] = ddq[4];
    Y_out[330] = dq[4];
    Y_out[331] = (((dq[4]) > 0) - ((dq[4]) < 0));
    Y_out[332] = 1;
    Y_out[333] = x238;
    Y_out[334] = x251;
    Y_out[335] = x263;
    Y_out[336] = x272;
    Y_out[337] = x281;
    Y_out[338] = x294;
    Y_out[339] = x311;
    Y_out[340] = 0;
    Y_out[341] = 0;
    Y_out[342] = 0;
    Y_out[343] = 0;
    Y_out[344] = x337;
    Y_out[345] = x355;
    Y_out[346] = x370;
    Y_out[347] = x382;
    Y_out[348] = x390;
    Y_out[349] = x404;
    Y_out[350] = x423;
    Y_out[351] = 0;
    Y_out[352] = 0;
    Y_out[353] = 0;
    Y_out[354] = 0;
    Y_out[355] = 0;
    Y_out[356] = 0;
    Y_out[357] = 0;
    Y_out[358] = 0;
    Y_out[359] = 0;
    Y_out[360] = 0;
    Y_out[361] = 0;
    Y_out[362] = 0;
    Y_out[363] = 0;
    Y_out[364] = 0;
    Y_out[365] = 0;
    Y_out[366] = 0;
    Y_out[367] = 0;
    Y_out[368] = 0;
    Y_out[369] = 0;
    Y_out[370] = 0;
    Y_out[371] = 0;
    Y_out[372] = 0;
    Y_out[373] = 0;
    Y_out[374] = 0;
    Y_out[375] = 0;
    Y_out[376] = 0;
    Y_out[377] = 0;
    Y_out[378] = 0;
    Y_out[379] = 0;
    Y_out[380] = 0;
    Y_out[381] = 0;
    Y_out[382] = 0;
    Y_out[383] = 0;
    Y_out[384] = 0;
    Y_out[385] = 0;
    Y_out[386] = 0;
    Y_out[387] = 0;
    Y_out[388] = 0;
    Y_out[389] = 0;
    Y_out[390] = 0;
    Y_out[391] = 0;
    Y_out[392] = 0;
    Y_out[393] = 0;
    Y_out[394] = 0;
    Y_out[395] = 0;
    Y_out[396] = 0;
    Y_out[397] = 0;
    Y_out[398] = 0;
    Y_out[399] = 0;
    Y_out[400] = 0;
    Y_out[401] = 0;
    Y_out[402] = 0;
    Y_out[403] = 0;
    Y_out[404] = x240;
    Y_out[405] = x254;
    Y_out[406] = x267;
    Y_out[407] = x245;
    Y_out[408] = x284;
    Y_out[409] = x298;
    Y_out[410] = x314;
    Y_out[411] = ddq[5];
    Y_out[412] = dq[5];
    Y_out[413] = (((dq[5]) > 0) - ((dq[5]) < 0));
    Y_out[414] = 1;
    Y_out[415] = x339;
    Y_out[416] = x357;
    Y_out[417] = x372;
    Y_out[418] = x384;
    Y_out[419] = x392;
    Y_out[420] = x406;
    Y_out[421] = x425;
    Y_out[422] = 0;
    Y_out[423] = 0;
    Y_out[424] = 0;
    Y_out[425] = 0;
    Y_out[426] = 0;
    Y_out[427] = 0;
    Y_out[428] = 0;
    Y_out[429] = 0;
    Y_out[430] = 0;
    Y_out[431] = 0;
    Y_out[432] = 0;
    Y_out[433] = 0;
    Y_out[434] = 0;
    Y_out[435] = 0;
    Y_out[436] = 0;
    Y_out[437] = 0;
    Y_out[438] = 0;
    Y_out[439] = 0;
    Y_out[440] = 0;
    Y_out[441] = 0;
    Y_out[442] = 0;
    Y_out[443] = 0;
    Y_out[444] = 0;
    Y_out[445] = 0;
    Y_out[446] = 0;
    Y_out[447] = 0;
    Y_out[448] = 0;
    Y_out[449] = 0;
    Y_out[450] = 0;
    Y_out[451] = 0;
    Y_out[452] = 0;
    Y_out[453] = 0;
    Y_out[454] = 0;
    Y_out[455] = 0;
    Y_out[456] = 0;
    Y_out[457] = 0;
    Y_out[458] = 0;
    Y_out[459] = 0;
    Y_out[460] = 0;
    Y_out[461] = 0;
    Y_out[462] = 0;
    Y_out[463] = 0;
    Y_out[464] = 0;
    Y_out[465] = 0;
    Y_out[466] = 0;
    Y_out[467] = 0;
    Y_out[468] = 0;
    Y_out[469] = 0;
    Y_out[470] = 0;
    Y_out[471] = 0;
    Y_out[472] = 0;
    Y_out[473] = 0;
    Y_out[474] = 0;
    Y_out[475] = 0;
    Y_out[476] = 0;
    Y_out[477] = 0;
    Y_out[478] = 0;
    Y_out[479] = 0;
    Y_out[480] = 0;
    Y_out[481] = 0;
    Y_out[482] = 0;
    Y_out[483] = 0;
    Y_out[484] = 0;
    Y_out[485] = 0;
    Y_out[486] = x336;
    Y_out[487] = x354;
    Y_out[488] = x369;
    Y_out[489] = x381;
    Y_out[490] = x362;
    Y_out[491] = x403;
    Y_out[492] = x422;
    Y_out[493] = ddq[6];
    Y_out[494] = dq[6];
    Y_out[495] = (((dq[6]) > 0) - ((dq[6]) < 0));
    Y_out[496] = 1;

    Eigen::VectorXd q_offset(7);
    if (is_anthropmorphic)
    {
        q_offset << 0.0, 0.0, -M_PI/2.0, 0.0, 0.0, -M_PI/2.0, 0.0;
    }
    else
    {
        q_offset << 0.0, 0.0, -M_PI/2.0, 0.0, 0.0, 0.0, 0.0;
    }
    Eigen::MatrixXd Y_out_mat;
    Y_out_mat = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(Y_out.data(), q.size(), 497 / q.size());
    return Y_out_mat;
}

Eigen::MatrixXd InvDynamicRegressorSecondary(bool is_anthropmorphic, const Vector3d& gravity_acc, const Eigen::VectorXd& q, const Eigen::VectorXd& dq, const Eigen::VectorXd& ddq)
{
    return Eigen::MatrixXd{};
}
}  // namespace dynamics
}  // namespace robot_server_lib