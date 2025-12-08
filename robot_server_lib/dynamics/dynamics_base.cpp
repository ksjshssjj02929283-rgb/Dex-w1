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

    // 如果提供了动力学最小参数集矩阵，则进行调整
    if (dynamics_base_parameters_primary.has_value())
    {
        // 计算手臂基坐标系在world base下的转换)
        // int first_joint_id_primary = robot_model.params_ptr->joints_primary[0];
        // std::string first_joint_name_primary = robot_model.params_ptr->joint_command_sequence_by_name[first_joint_id_primary];
        auto pose = GetFkByFrameName(model, data, q, "arml1j");
        // auto ori = pose.rotation();
        Eigen::VectorXd gravity_term_tau = InvDynamicsGravityTerm(robot_model, q, MotionGroup::Primary);
        Eigen::VectorXd coriolis_term_tau = InvDynamicsCoriolisTerm(robot_model, q, dq, MotionGroup::Primary);
        LOG_CRIT << "Gravity term tau (primary): " << gravity_term_tau.transpose();
        LOG_CRIT << "Coriolis term tau (primary): " << coriolis_term_tau.transpose();
        utils::MapJointToWholeBody(robot_model, gravity_term_tau + coriolis_term_tau, tau, MotionGroup::Primary);
    }
    if (dynamics_base_parameters_secondary.has_value())
    {
        Eigen::VectorXd gravity_term_tau = InvDynamicsGravityTerm(robot_model, q, MotionGroup::Secondary);
        Eigen::VectorXd coriolis_term_tau = InvDynamicsCoriolisTerm(robot_model, q, dq, MotionGroup::Secondary);
        utils::MapJointToWholeBody(robot_model, gravity_term_tau + coriolis_term_tau, tau, MotionGroup::Secondary);
    }
    return tau;
}

Eigen::VectorXd InvDynamicsGravityTerm(const RobotModel& robot_model, const VectorDof& q, MotionGroup motion_group)
{
    Eigen::Vector3d gravity_acc;
    Eigen::VectorXd q_arm, gravity_term_tau, dynamics_base_parameters;
    Eigen::MatrixXd invdyn_regressor_matrix;
    utils::GetJointFromWholeBody(robot_model, q, q_arm, motion_group);
    if (motion_group == MotionGroup::Primary)
    {
        invdyn_regressor_matrix = InvDynamicRegressorPrimary(q_arm, Eigen::VectorXd::Zero(q_arm.size()), Eigen::VectorXd::Zero(q_arm.size()));
        dynamics_base_parameters = robot_model.params_ptr->dynamics_base_parameters_primary.value();
    }
    else if (motion_group == MotionGroup::Secondary)
    {
        invdyn_regressor_matrix = InvDynamicRegressorSecondary(q_arm, Eigen::VectorXd::Zero(q_arm.size()), Eigen::VectorXd::Zero(q_arm.size()));
        dynamics_base_parameters = robot_model.params_ptr->dynamics_base_parameters_secondary.value();
    }
    else
    {
        LOG_CRIT << "Invalid motion group for InvDynamicsGravityTerm";
        assert(false && "Invalid motion group for InvDynamicsGravityTerm");
        return Eigen::VectorXd::Zero(0);
    }
    gravity_term_tau = invdyn_regressor_matrix * dynamics_base_parameters;
    return gravity_term_tau;
}

Eigen::VectorXd InvDynamicsCoriolisTerm(const RobotModel& robot_model, const VectorDof& q, const VectorDof& dq, MotionGroup motion_group)
{
    Eigen::VectorXd q_arm, dq_arm, coriolis_term_tau, dynamics_base_parameters;
    Eigen::MatrixXd invdyn_regressor_matrix;
    utils::GetJointFromWholeBody(robot_model, q, q_arm, motion_group);
    utils::GetJointFromWholeBody(robot_model, dq, dq_arm, motion_group);

    if (motion_group == MotionGroup::Primary)
    {
        invdyn_regressor_matrix = InvDynamicRegressorPrimary(q_arm, dq_arm, Eigen::VectorXd::Zero(q_arm.size()));
        dynamics_base_parameters = robot_model.params_ptr->dynamics_base_parameters_primary.value();
    }
    else if (motion_group == MotionGroup::Secondary)
    {
        invdyn_regressor_matrix = InvDynamicRegressorSecondary(q_arm, dq_arm, Eigen::VectorXd::Zero(q_arm.size()));
        dynamics_base_parameters = robot_model.params_ptr->dynamics_base_parameters_secondary.value();
    }
    else
    {
        LOG_CRIT << "Invalid motion group for InvDynamicsGravityTerm";
        assert(false && "Invalid motion group for InvDynamicsGravityTerm");
        return Eigen::VectorXd::Zero(0);
    }
    coriolis_term_tau = invdyn_regressor_matrix * dynamics_base_parameters - InvDynamicsGravityTerm(robot_model, q, motion_group);
    return coriolis_term_tau;
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

Eigen::MatrixXd InvDynamicRegressorPrimary(const Eigen::VectorXd& q, const Eigen::VectorXd& dq, const Eigen::VectorXd& ddq)
{
    Eigen::VectorXd q_offset(7);
    q_offset << 0.0, 0.0, -M_PI/2.0, 0.0, 0.0, -M_PI/2.0, 0.0;
    Eigen::VectorXd q_with_offset = q + q_offset;
    double x0 = -ddq[0];
    double x1 = -9.81*sin(q_with_offset[0]);
    double x2 = -x1;
    double x3 = 9.81*cos(q_with_offset[0]);
    double x4 = sin(q_with_offset[1]);
    double x5 = -dq[0];
    double x6 = cos(q_with_offset[1]);
    double x7 = -x6;
    double x8 = x5*x7;
    double x9 = -x8;
    double x10 = dq[1]*x9;
    double x11 = x0*x4 + x10;
    double x12 = x4*x5;
    double x13 = dq[1]*x12;
    double x14 = -x13;
    double x15 = ((x12)*(x12));
    double x16 = ((dq[1])*(dq[1]));
    double x17 = x0*x7 + x13;
    double x18 = ((x8)*(x8));
    double x19 = x12*x8;
    double x20 = cos(q_with_offset[2]);
    double x21 = sin(q_with_offset[2]);
    double x22 = ddq[1]*x21;
    double x23 = x11*x20;
    double x24 = dq[1]*x20;
    double x25 = x12*x21;
    double x26 = x24 - x25;
    double x27 = -dq[2];
    double x28 = -x27;
    double x29 = x22 + x23 + x26*x28;
    double x30 = dq[1]*x21;
    double x31 = x12*x20;
    double x32 = x30 + x31;
    double x33 = x27 + x9;
    double x34 = x32*x33;
    double x35 = -x34;
    double x36 = -x21;
    double x37 = x26*x32;
    double x38 = -x37;
    double x39 = -ddq[2] - x17;
    double x40 = x38 + x39;
    double x41 = ((x32)*(x32));
    double x42 = ((x33)*(x33));
    double x43 = -x42;
    double x44 = x41 + x43;
    double x45 = x26*x33;
    double x46 = -x29 - x45;
    double x47 = ddq[1]*x20;
    double x48 = x11*x21;
    double x49 = x27*x32 + x47 - x48;
    double x50 = x34 + x49;
    double x51 = -x45;
    double x52 = x29 + x51;
    double x53 = ((x26)*(x26));
    double x54 = -x41;
    double x55 = -x53 - x54;
    double x56 = -x39;
    double x57 = -x53;
    double x58 = x42 + x57;
    double x59 = x37 + x39;
    double x60 = -x35 - x49;
    double x61 = 0.26*x24 - 0.26*x25;
    double x62 = x26*x61;
    double x63 = -0.26*x30 - 0.26*x31;
    double x64 = -x63;
    double x65 = -x3*x4;
    double x66 = x32*x64 + x62 + x65;
    double x67 = x43 + x57;
    double x68 = -0.26*x21;
    double x69 = x37 + x56;
    double x70 = x3*x6;
    double x71 = x1*x20 - x21*x70 - 0.26*x22 - 0.26*x23 + x27*x61;
    double x72 = -x71;
    double x73 = -x33*x61 - x72;
    double x74 = x32*x63 - x62 - x65;
    double x75 = x43 + x54;
    double x76 = x1*x21 + x20*x70 + x28*x63 + 0.26*x47 - 0.26*x48;
    double x77 = -x33*x63 - x76;
    double x78 = cos(q_with_offset[3]);
    double x79 = sin(q_with_offset[3]);
    double x80 = x32*x78 + x33*x79;
    double x81 = dq[3] + x26;
    double x82 = x80*x81;
    double x83 = -x82;
    double x84 = -x78;
    double x85 = x32*x79 + x33*x84;
    double x86 = -dq[3];
    double x87 = x29*x78 + x39*x79 + x85*x86;
    double x88 = x78*x87 + x79*x83;
    double x89 = x80*x85;
    double x90 = -x79*x87 - x83*x84;
    double x91 = -x85;
    double x92 = ddq[3] + x49;
    double x93 = x80*x91 + x92;
    double x94 = ((x80)*(x80));
    double x95 = ((x81)*(x81));
    double x96 = -x95;
    double x97 = x94 + x96;
    double x98 = x78*x93 + x79*x97;
    double x99 = x81*x85;
    double x100 = x87 + x99;
    double x101 = -x79*x93 - x84*x97;
    double x102 = dq[3]*x80 + x29*x79 + x39*x84;
    double x103 = x102 + x82;
    double x104 = x81*x91;
    double x105 = x104 + x87;
    double x106 = x103*x78 + x105*x79;
    double x107 = ((x85)*(x85));
    double x108 = -x94;
    double x109 = x107 + x108;
    double x110 = -x103*x79 - x105*x84;
    double x111 = x104*x78 + x79*x82;
    double x112 = -x104*x79 - x82*x84;
    double x113 = -x107;
    double x114 = x113 + x95;
    double x115 = x89 + x92;
    double x116 = x114*x78 + x115*x79;
    double x117 = x102 + x83;
    double x118 = -x114*x79 - x115*x84;
    double x119 = x61*x78;
    double x120 = x61*x79;
    double x121 = -x120;
    double x122 = x119*x85 + x121*x80 + x71;
    double x123 = x122*x79;
    double x124 = -dq[3]*x119 - x65*x84 - x76*x79;
    double x125 = x119*x81 + x124 + x64*x80;
    double x126 = x78*(x113 + x96) + x79*(x89 - x92);
    double x127 = -x122*x84;
    double x128 = x119*x91 + x120*x80 + x72;
    double x129 = x128*x78;
    double x130 = x120*x86 + x65*x79 + x76*x78;
    double x131 = x120*x81 + x130 + x63*x91;
    double x132 = -x87 + x99;
    double x133 = x115*x78 + x79*(x108 + x96);
    double x134 = -x128*x79;
    double x135 = cos(q_with_offset[4]);
    double x136 = x135*x80;
    double x137 = sin(q_with_offset[4]);
    double x138 = x137*x81;
    double x139 = x136 + x138;
    double x140 = x135*x81;
    double x141 = x137*x80;
    double x142 = x140 - x141;
    double x143 = x139*x142;
    double x144 = -x143;
    double x145 = x135*x87;
    double x146 = x137*x92;
    double x147 = -dq[4];
    double x148 = -x147;
    double x149 = x142*x148 + x145 + x146;
    double x150 = -x137;
    double x151 = x147 + x91;
    double x152 = x139*x151;
    double x153 = -x152;
    double x154 = x135*x149 + x150*x153;
    double x155 = x144*x79 + x154*x78;
    double x156 = x135*x153 + x137*x149;
    double x157 = -x144*x84 - x154*x79;
    double x158 = x142*x151;
    double x159 = -x149 - x158;
    double x160 = -ddq[4] - x102;
    double x161 = x144 + x160;
    double x162 = ((x139)*(x139));
    double x163 = ((x151)*(x151));
    double x164 = -x163;
    double x165 = x162 + x164;
    double x166 = x135*x161 + x150*x165;
    double x167 = x159*x79 + x166*x78;
    double x168 = x135*x165 + x137*x161;
    double x169 = -x159*x84 - x166*x79;
    double x170 = ((x142)*(x142));
    double x171 = -x162;
    double x172 = -x170 - x171;
    double x173 = x135*x92;
    double x174 = x139*x147 + x150*x87 + x173;
    double x175 = x152 + x174;
    double x176 = -x158;
    double x177 = x149 + x176;
    double x178 = x135*x175 + x150*x177;
    double x179 = x172*x79 + x178*x78;
    double x180 = x135*x177 + x137*x175;
    double x181 = -x172*x84 - x178*x79;
    double x182 = -x160;
    double x183 = x135*x176 + x150*x152;
    double x184 = x182*x79 + x183*x78;
    double x185 = x135*x152 + x137*x176;
    double x186 = -x182*x84 - x183*x79;
    double x187 = -x153 - x174;
    double x188 = -x170;
    double x189 = x163 + x188;
    double x190 = x143 + x160;
    double x191 = x135*x189 + x150*x190;
    double x192 = x187*x79 + x191*x78;
    double x193 = x135*x190;
    double x194 = x137*x189 + x193;
    double x195 = -x187*x84 - x191*x79;
    double x196 = x119*x135 + x137*x63 + 0.26400000000000001*x140 - 0.26400000000000001*x141;
    double x197 = x130*x150 + x135*x71 - 0.26400000000000001*x145 - 0.26400000000000001*x146 + x147*x196;
    double x198 = -x197;
    double x199 = -x139;
    double x200 = -x121*x199 - x151*x196 - x198;
    double x201 = x142*x196;
    double x202 = x119*x150 + x135*x63 - 0.26400000000000001*x136 - 0.26400000000000001*x138;
    double x203 = x124 + x199*x202 + x201;
    double x204 = x164 + x188;
    double x205 = -0.26400000000000001*x137;
    double x206 = x143 + x182;
    double x207 = x135*x206;
    double x208 = x150*x203 + x204*x205 - 0.26400000000000001*x207;
    double x209 = x200*x79 + x208*x78;
    double x210 = x135*x204;
    double x211 = x135*x203 + x205*x206 + 0.26400000000000001*x210;
    double x212 = x137*x204 + x207;
    double x213 = -x175*x79 + x78*(x150*x206 + x210);
    double x214 = -x200*x84 - x208*x79;
    double x215 = x130*x135 + x137*x71 + x148*x202 + 0.26400000000000001*x173 + x205*x87;
    double x216 = x121*x142 - x151*x202 - x215;
    double x217 = -x124 + x139*x202 - x201;
    double x218 = x164 + x171;
    double x219 = x135*x218;
    double x220 = x135*x217 + x190*x205 - 0.26400000000000001*x219;
    double x221 = x216*x79 + x220*x78;
    double x222 = x137*x217 + 0.26400000000000001*x193 + x205*x218;
    double x223 = x137*x190 + x219;
    double x224 = x78*(x150*x218 + x193) + x79*(x149 - x158);
    double x225 = -x216*x84 - x220*x79;
    double x226 = sin(q_with_offset[5]);
    double x227 = cos(q_with_offset[5]);
    double x228 = -x227;
    double x229 = x139*x226 + x151*x228;
    double x230 = -dq[5];
    double x231 = x149*x227 + x160*x226 + x229*x230;
    double x232 = x139*x227 + x151*x226;
    double x233 = dq[5] + x142;
    double x234 = x232*x233;
    double x235 = -x234;
    double x236 = -x226*x231 - x228*x235;
    double x237 = x226*x235 + x227*x231;
    double x238 = x229*x232;
    double x239 = x135*x237 + x150*x238;
    double x240 = x236*x79 + x239*x78;
    double x241 = x135*x238 + x137*x237;
    double x242 = -x236*x84 - x239*x79;
    double x243 = ddq[5] + x174;
    double x244 = -x238 + x243;
    double x245 = ((x232)*(x232));
    double x246 = ((x233)*(x233));
    double x247 = -x246;
    double x248 = x245 + x247;
    double x249 = -x226*x244 - x228*x248;
    double x250 = x226*x248 + x227*x244;
    double x251 = x229*x233;
    double x252 = x231 + x251;
    double x253 = x135*x250 + x150*x252;
    double x254 = x249*x79 + x253*x78;
    double x255 = x135*x252 + x137*x250;
    double x256 = -x249*x84 - x253*x79;
    double x257 = dq[5]*x232 + x149*x226 + x160*x228;
    double x258 = x234 + x257;
    double x259 = -x251;
    double x260 = x231 + x259;
    double x261 = -x226*x258 - x228*x260;
    double x262 = x226*x260 + x227*x258;
    double x263 = ((x229)*(x229));
    double x264 = -x245;
    double x265 = x263 + x264;
    double x266 = x135*x262 + x150*x265;
    double x267 = x261*x79 + x266*x78;
    double x268 = x135*x265 + x137*x262;
    double x269 = -x261*x84 - x266*x79;
    double x270 = -x226*x259 - x228*x234;
    double x271 = x226*x234 + x227*x259;
    double x272 = x135*x271 + x150*x243;
    double x273 = x270*x79 + x272*x78;
    double x274 = x135*x243 + x137*x271;
    double x275 = -x270*x84 - x272*x79;
    double x276 = -x263;
    double x277 = x246 + x276;
    double x278 = x238 + x243;
    double x279 = -x226*x277 - x228*x278;
    double x280 = x226*x278;
    double x281 = x227*x277 + x280;
    double x282 = x235 + x257;
    double x283 = x135*x281 + x150*x282;
    double x284 = x279*x79 + x283*x78;
    double x285 = x135*x282 + x137*x281;
    double x286 = -x279*x84 - x283*x79;
    double x287 = x121*x226 + x196*x227;
    double x288 = x229*x287;
    double x289 = -x232;
    double x290 = x121*x228 + x196*x226;
    double x291 = x197 + x288 + x289*x290;
    double x292 = -x228*x291;
    double x293 = x226*x291;
    double x294 = dq[5]*x287 + x124*x228 + x215*x226;
    double x295 = -x294;
    double x296 = x202*x289 + x233*x287 + x295;
    double x297 = x135*x258;
    double x298 = x247 + x276;
    double x299 = x238 - x243;
    double x300 = x226*x299 + x227*x298;
    double x301 = x135*x293 + x150*x296 + x205*x300 - 0.26400000000000001*x297;
    double x302 = x292*x79 + x301*x78;
    double x303 = x135*x300;
    double x304 = x135*x296 + x137*x293 + x205*x258 + 0.26400000000000001*x303;
    double x305 = x137*x300 + x297;
    double x306 = x78*(x150*x258 + x303) + x79*(-x226*x298 - x228*x299);
    double x307 = -x292*x84 - x301*x79;
    double x308 = x198 + x232*x290 - x288;
    double x309 = -x226*x308;
    double x310 = x227*x308;
    double x311 = x124*x226 + x215*x227 + x230*x290;
    double x312 = -x202*x229 + x233*x290 + x311;
    double x313 = -x231 + x251;
    double x314 = x135*x313;
    double x315 = x247 + x264;
    double x316 = x226*x315 + x227*x278;
    double x317 = x135*x310 + x150*x312 + x205*x316 - 0.26400000000000001*x314;
    double x318 = x309*x79 + x317*x78;
    double x319 = x135*x316;
    double x320 = x135*x312 + x137*x310 + x205*x313 + 0.26400000000000001*x319;
    double x321 = x137*x316 + x314;
    double x322 = x78*(x150*x313 + x319) + x79*(-x228*x315 - x280);
    double x323 = -x309*x84 - x317*x79;
    double x324 = cos(q_with_offset[6]);
    double x325 = sin(q_with_offset[6]);
    double x326 = -x325;
    double x327 = x232*x326 + x233*x324;
    double x328 = dq[6]*x327 + x231*x324 + x243*x325;
    double x329 = x232*x324 + x233*x325;
    double x330 = dq[6] + x229;
    double x331 = x329*x330;
    double x332 = x324*x328 + x326*x331;
    double x333 = x327*x329;
    double x334 = -x333;
    double x335 = -x226*x332 - x228*x334;
    double x336 = x226*x334 + x227*x332;
    double x337 = x324*x331 + x325*x328;
    double x338 = x135*x336 + x150*x337;
    double x339 = x335*x79 + x338*x78;
    double x340 = x135*x337 + x137*x336;
    double x341 = -x335*x84 - x338*x79;
    double x342 = -x331;
    double x343 = -dq[6];
    double x344 = x231*x326 + x243*x324 + x329*x343;
    double x345 = x342 + x344;
    double x346 = x327*x330;
    double x347 = x328 + x346;
    double x348 = x324*x345 + x326*x347;
    double x349 = ((x329)*(x329));
    double x350 = ((x327)*(x327));
    double x351 = -x350;
    double x352 = x349 + x351;
    double x353 = -x226*x348 - x228*x352;
    double x354 = x226*x352 + x227*x348;
    double x355 = x324*x347 + x325*x345;
    double x356 = x135*x354 + x150*x355;
    double x357 = x353*x79 + x356*x78;
    double x358 = x135*x355 + x137*x354;
    double x359 = -x353*x84 - x356*x79;
    double x360 = ddq[6] + x257;
    double x361 = x333 + x360;
    double x362 = x324*x361;
    double x363 = ((x330)*(x330));
    double x364 = -x349;
    double x365 = x363 + x364;
    double x366 = x326*x365 + x362;
    double x367 = x328 - x346;
    double x368 = -x226*x366 - x228*x367;
    double x369 = x226*x367 + x227*x366;
    double x370 = x324*x365 + x325*x361;
    double x371 = x135*x369 + x150*x370;
    double x372 = x368*x79 + x371*x78;
    double x373 = x135*x370 + x137*x369;
    double x374 = -x368*x84 - x371*x79;
    double x375 = -x363;
    double x376 = x350 + x375;
    double x377 = x334 + x360;
    double x378 = x324*x376 + x326*x377;
    double x379 = x331 + x344;
    double x380 = -x226*x378 - x228*x379;
    double x381 = x226*x379 + x227*x378;
    double x382 = x324*x377 + x325*x376;
    double x383 = x135*x381 + x150*x382;
    double x384 = x380*x79 + x383*x78;
    double x385 = x135*x382 + x137*x381;
    double x386 = -x380*x84 - x383*x79;
    double x387 = x324*x346 + x326*x342;
    double x388 = -x226*x387 - x228*x360;
    double x389 = x226*x360 + x227*x387;
    double x390 = x324*x342 + x325*x346;
    double x391 = x135*x389 + x150*x390;
    double x392 = x388*x79 + x391*x78;
    double x393 = x135*x390 + x137*x389;
    double x394 = -x388*x84 - x391*x79;
    double x395 = x202*x325 + x287*x324;
    double x396 = x327*x395;
    double x397 = x202*x324 + x287*x326;
    double x398 = -x329;
    double x399 = x295 + x396 + x397*x398;
    double x400 = x326*x399;
    double x401 = x197*x324 + x290*x398 + x311*x326 + x330*x395 + x343*x395;
    double x402 = -x226*x400 - x228*x401;
    double x403 = x226*x401 + x227*x400;
    double x404 = x324*x399;
    double x405 = x351 + x375;
    double x406 = x325*x405 + x362;
    double x407 = x135*x406;
    double x408 = x331 - x344;
    double x409 = x324*x405 + x326*x361;
    double x410 = x226*x408 + x227*x409;
    double x411 = x135*x403 + x150*x404 + x205*x410 - 0.26400000000000001*x407;
    double x412 = x402*x79 + x411*x78;
    double x413 = x135*x410;
    double x414 = x135*x404 + x137*x403 + x205*x406 + 0.26400000000000001*x413;
    double x415 = x137*x410 + x407;
    double x416 = x78*(x150*x406 + x413) + x79*(-x226*x409 - x228*x408);
    double x417 = -x402*x84 - x411*x79;
    double x418 = x294 + x329*x397 - x396;
    double x419 = x324*x418;
    double x420 = -dq[6]*x397 - x197*x325 - x290*x327 - x311*x324 + x330*x397;
    double x421 = -x226*x419 - x228*x420;
    double x422 = x226*x420 + x227*x419;
    double x423 = x325*x418;
    double x424 = x333 - x360;
    double x425 = x364 + x375;
    double x426 = x324*x425 + x325*x424;
    double x427 = x135*x426;
    double x428 = x324*x424 + x326*x425;
    double x429 = x226*x347 + x227*x428;
    double x430 = x135*x422 + x150*x423 + x205*x429 - 0.26400000000000001*x427;
    double x431 = x421*x79 + x430*x78;
    double x432 = x135*x429;
    double x433 = x135*x423 + x137*x422 + x205*x426 + 0.26400000000000001*x432;
    double x434 = x137*x429 + x427;
    double x435 = x78*(x150*x426 + x432) + x79*(-x226*x428 - x228*x347);
    double x436 = -x421*x84 - x430*x79;
    double x437 = x20*x59;

    Eigen::VectorXd Y_out_vec = Eigen::VectorXd::Zero(497);
    Eigen::MatrixXd Y_out_mat;
    if (497 % q.size() != 0)
    {
        LOG_CRIT << "InvDynamicRegressorPrimary: The output size is not a multiple of the input size.";
        return Y_out_mat;
    }
    else
    {
        Y_out_vec[0] = -x0;
        Y_out_vec[1] = -x2;
        Y_out_vec[2] = -x3;
        Y_out_vec[3] = dq[0];
        Y_out_vec[4] = (((dq[0]) > 0) - ((dq[0]) < 0));
        Y_out_vec[5] = 1;
        Y_out_vec[6] = -x11*x4 - x14*x7;
        Y_out_vec[7] = -x4*(ddq[1] + x12*x9) - x7*(x15 - x16);
        Y_out_vec[8] = -x4*(x13 + x17) - x7*(x10 + x11);
        Y_out_vec[9] = -x10*x4 - x13*x7;
        Y_out_vec[10] = -x4*(x16 - x18) - x7*(ddq[1] + x19);
        Y_out_vec[11] = -x1*x7;
        Y_out_vec[12] = -x2*x4;
        Y_out_vec[13] = 0;
        Y_out_vec[14] = 0;
        Y_out_vec[15] = 0;
        Y_out_vec[16] = -x38*x7 - x4*(x20*x29 + x35*x36);
        Y_out_vec[17] = -x4*(x20*x40 + x36*x44) - x46*x7;
        Y_out_vec[18] = -x4*(x20*x50 + x36*x52) - x55*x7;
        Y_out_vec[19] = -x4*(x20*x51 + x34*x36) - x56*x7;
        Y_out_vec[20] = -x4*(x20*x58 + x36*x59) - x60*x7;
        Y_out_vec[21] = -x4*(-0.26*x20*x69 + x36*x66 + x67*x68) - x7*x73;
        Y_out_vec[22] = -x4*(x20*x74 - 0.26*x20*x75 + x59*x68) - x7*x77;
        Y_out_vec[23] = 0;
        Y_out_vec[24] = 0;
        Y_out_vec[25] = 0;
        Y_out_vec[26] = 0;
        Y_out_vec[27] = -x4*(x20*x88 + x36*x89) - x7*x90;
        Y_out_vec[28] = -x101*x7 - x4*(x100*x36 + x20*x98);
        Y_out_vec[29] = -x110*x7 - x4*(x106*x20 + x109*x36);
        Y_out_vec[30] = -x112*x7 - x4*(x111*x20 + x36*x92);
        Y_out_vec[31] = -x118*x7 - x4*(x116*x20 + x117*x36);
        Y_out_vec[32] = -x127*x7 - x4*(-0.26*x103*x20 + x123*x20 + x125*x36 + x126*x68);
        Y_out_vec[33] = -x134*x7 - x4*(x129*x20 + x131*x36 - 0.26*x132*x20 + x133*x68);
        Y_out_vec[34] = 0;
        Y_out_vec[35] = 0;
        Y_out_vec[36] = 0;
        Y_out_vec[37] = 0;
        Y_out_vec[38] = -x157*x7 - x4*(x155*x20 + x156*x36);
        Y_out_vec[39] = -x169*x7 - x4*(x167*x20 + x168*x36);
        Y_out_vec[40] = -x181*x7 - x4*(x179*x20 + x180*x36);
        Y_out_vec[41] = -x186*x7 - x4*(x184*x20 + x185*x36);
        Y_out_vec[42] = -x195*x7 - x4*(x192*x20 + x194*x36);
        Y_out_vec[43] = -x214*x7 - x4*(x20*x209 - 0.26*x20*x212 + x211*x36 + x213*x68);
        Y_out_vec[44] = -x225*x7 - x4*(x20*x221 - 0.26*x20*x223 + x222*x36 + x224*x68);
        Y_out_vec[45] = 0;
        Y_out_vec[46] = 0;
        Y_out_vec[47] = 0;
        Y_out_vec[48] = 0;
        Y_out_vec[49] = -x242*x7 - x4*(x20*x240 + x241*x36);
        Y_out_vec[50] = -x256*x7 - x4*(x20*x254 + x255*x36);
        Y_out_vec[51] = -x269*x7 - x4*(x20*x267 + x268*x36);
        Y_out_vec[52] = -x275*x7 - x4*(x20*x273 + x274*x36);
        Y_out_vec[53] = -x286*x7 - x4*(x20*x284 + x285*x36);
        Y_out_vec[54] = -x307*x7 - x4*(x20*x302 - 0.26*x20*x305 + x304*x36 + x306*x68);
        Y_out_vec[55] = -x323*x7 - x4*(x20*x318 - 0.26*x20*x321 + x320*x36 + x322*x68);
        Y_out_vec[56] = 0;
        Y_out_vec[57] = 0;
        Y_out_vec[58] = 0;
        Y_out_vec[59] = 0;
        Y_out_vec[60] = -x341*x7 - x4*(x20*x339 + x340*x36);
        Y_out_vec[61] = -x359*x7 - x4*(x20*x357 + x358*x36);
        Y_out_vec[62] = -x374*x7 - x4*(x20*x372 + x36*x373);
        Y_out_vec[63] = -x386*x7 - x4*(x20*x384 + x36*x385);
        Y_out_vec[64] = -x394*x7 - x4*(x20*x392 + x36*x393);
        Y_out_vec[65] = -x4*(x20*x412 - 0.26*x20*x415 + x36*x414 + x416*x68) - x417*x7;
        Y_out_vec[66] = -x4*(x20*x431 - 0.26*x20*x434 + x36*x433 + x435*x68) - x436*x7;
        Y_out_vec[67] = 0;
        Y_out_vec[68] = 0;
        Y_out_vec[69] = 0;
        Y_out_vec[70] = 0;
        Y_out_vec[71] = 0;
        Y_out_vec[72] = 0;
        Y_out_vec[73] = 0;
        Y_out_vec[74] = 0;
        Y_out_vec[75] = 0;
        Y_out_vec[76] = 0;
        Y_out_vec[77] = x19;
        Y_out_vec[78] = dq[1]*x8 + x11;
        Y_out_vec[79] = -x15 + x18;
        Y_out_vec[80] = ddq[1];
        Y_out_vec[81] = x14 + x17;
        Y_out_vec[82] = x65;
        Y_out_vec[83] = x70;
        Y_out_vec[84] = dq[1];
        Y_out_vec[85] = (((dq[1]) > 0) - ((dq[1]) < 0));
        Y_out_vec[86] = 1;
        Y_out_vec[87] = x20*x35 + x21*x29;
        Y_out_vec[88] = x20*x44 + x21*x40;
        Y_out_vec[89] = x20*x52 + x21*x50;
        Y_out_vec[90] = x20*x34 + x21*x51;
        Y_out_vec[91] = x21*x58 + x437;
        Y_out_vec[92] = x20*x66 + 0.26*x20*x67 + x68*x69;
        Y_out_vec[93] = x21*x74 + 0.26*x437 + x68*x75;
        Y_out_vec[94] = 0;
        Y_out_vec[95] = 0;
        Y_out_vec[96] = 0;
        Y_out_vec[97] = 0;
        Y_out_vec[98] = x20*x89 + x21*x88;
        Y_out_vec[99] = x100*x20 + x21*x98;
        Y_out_vec[100] = x106*x21 + x109*x20;
        Y_out_vec[101] = x111*x21 + x20*x92;
        Y_out_vec[102] = x116*x21 + x117*x20;
        Y_out_vec[103] = x103*x68 + x123*x21 + x125*x20 + 0.26*x126*x20;
        Y_out_vec[104] = x129*x21 + x131*x20 + x132*x68 + 0.26*x133*x20;
        Y_out_vec[105] = 0;
        Y_out_vec[106] = 0;
        Y_out_vec[107] = 0;
        Y_out_vec[108] = 0;
        Y_out_vec[109] = x155*x21 + x156*x20;
        Y_out_vec[110] = x167*x21 + x168*x20;
        Y_out_vec[111] = x179*x21 + x180*x20;
        Y_out_vec[112] = x184*x21 + x185*x20;
        Y_out_vec[113] = x192*x21 + x194*x20;
        Y_out_vec[114] = x20*x211 + 0.26*x20*x213 + x209*x21 + x212*x68;
        Y_out_vec[115] = x20*x222 + 0.26*x20*x224 + x21*x221 + x223*x68;
        Y_out_vec[116] = 0;
        Y_out_vec[117] = 0;
        Y_out_vec[118] = 0;
        Y_out_vec[119] = 0;
        Y_out_vec[120] = x20*x241 + x21*x240;
        Y_out_vec[121] = x20*x255 + x21*x254;
        Y_out_vec[122] = x20*x268 + x21*x267;
        Y_out_vec[123] = x20*x274 + x21*x273;
        Y_out_vec[124] = x20*x285 + x21*x284;
        Y_out_vec[125] = x20*x304 + 0.26*x20*x306 + x21*x302 + x305*x68;
        Y_out_vec[126] = x20*x320 + 0.26*x20*x322 + x21*x318 + x321*x68;
        Y_out_vec[127] = 0;
        Y_out_vec[128] = 0;
        Y_out_vec[129] = 0;
        Y_out_vec[130] = 0;
        Y_out_vec[131] = x20*x340 + x21*x339;
        Y_out_vec[132] = x20*x358 + x21*x357;
        Y_out_vec[133] = x20*x373 + x21*x372;
        Y_out_vec[134] = x20*x385 + x21*x384;
        Y_out_vec[135] = x20*x393 + x21*x392;
        Y_out_vec[136] = x20*x414 + 0.26*x20*x416 + x21*x412 + x415*x68;
        Y_out_vec[137] = x20*x433 + 0.26*x20*x435 + x21*x431 + x434*x68;
        Y_out_vec[138] = 0;
        Y_out_vec[139] = 0;
        Y_out_vec[140] = 0;
        Y_out_vec[141] = 0;
        Y_out_vec[142] = 0;
        Y_out_vec[143] = 0;
        Y_out_vec[144] = 0;
        Y_out_vec[145] = 0;
        Y_out_vec[146] = 0;
        Y_out_vec[147] = 0;
        Y_out_vec[148] = 0;
        Y_out_vec[149] = 0;
        Y_out_vec[150] = 0;
        Y_out_vec[151] = 0;
        Y_out_vec[152] = 0;
        Y_out_vec[153] = 0;
        Y_out_vec[154] = 0;
        Y_out_vec[155] = 0;
        Y_out_vec[156] = 0;
        Y_out_vec[157] = 0;
        Y_out_vec[158] = x38;
        Y_out_vec[159] = x46;
        Y_out_vec[160] = x55;
        Y_out_vec[161] = x56;
        Y_out_vec[162] = x60;
        Y_out_vec[163] = x73;
        Y_out_vec[164] = x77;
        Y_out_vec[165] = ddq[2];
        Y_out_vec[166] = dq[2];
        Y_out_vec[167] = (((dq[2]) > 0) - ((dq[2]) < 0));
        Y_out_vec[168] = 1;
        Y_out_vec[169] = x90;
        Y_out_vec[170] = x101;
        Y_out_vec[171] = x110;
        Y_out_vec[172] = x112;
        Y_out_vec[173] = x118;
        Y_out_vec[174] = x127;
        Y_out_vec[175] = x134;
        Y_out_vec[176] = 0;
        Y_out_vec[177] = 0;
        Y_out_vec[178] = 0;
        Y_out_vec[179] = 0;
        Y_out_vec[180] = x157;
        Y_out_vec[181] = x169;
        Y_out_vec[182] = x181;
        Y_out_vec[183] = x186;
        Y_out_vec[184] = x195;
        Y_out_vec[185] = x214;
        Y_out_vec[186] = x225;
        Y_out_vec[187] = 0;
        Y_out_vec[188] = 0;
        Y_out_vec[189] = 0;
        Y_out_vec[190] = 0;
        Y_out_vec[191] = x242;
        Y_out_vec[192] = x256;
        Y_out_vec[193] = x269;
        Y_out_vec[194] = x275;
        Y_out_vec[195] = x286;
        Y_out_vec[196] = x307;
        Y_out_vec[197] = x323;
        Y_out_vec[198] = 0;
        Y_out_vec[199] = 0;
        Y_out_vec[200] = 0;
        Y_out_vec[201] = 0;
        Y_out_vec[202] = x341;
        Y_out_vec[203] = x359;
        Y_out_vec[204] = x374;
        Y_out_vec[205] = x386;
        Y_out_vec[206] = x394;
        Y_out_vec[207] = x417;
        Y_out_vec[208] = x436;
        Y_out_vec[209] = 0;
        Y_out_vec[210] = 0;
        Y_out_vec[211] = 0;
        Y_out_vec[212] = 0;
        Y_out_vec[213] = 0;
        Y_out_vec[214] = 0;
        Y_out_vec[215] = 0;
        Y_out_vec[216] = 0;
        Y_out_vec[217] = 0;
        Y_out_vec[218] = 0;
        Y_out_vec[219] = 0;
        Y_out_vec[220] = 0;
        Y_out_vec[221] = 0;
        Y_out_vec[222] = 0;
        Y_out_vec[223] = 0;
        Y_out_vec[224] = 0;
        Y_out_vec[225] = 0;
        Y_out_vec[226] = 0;
        Y_out_vec[227] = 0;
        Y_out_vec[228] = 0;
        Y_out_vec[229] = 0;
        Y_out_vec[230] = 0;
        Y_out_vec[231] = 0;
        Y_out_vec[232] = 0;
        Y_out_vec[233] = 0;
        Y_out_vec[234] = 0;
        Y_out_vec[235] = 0;
        Y_out_vec[236] = 0;
        Y_out_vec[237] = 0;
        Y_out_vec[238] = 0;
        Y_out_vec[239] = 0;
        Y_out_vec[240] = x89;
        Y_out_vec[241] = x100;
        Y_out_vec[242] = x109;
        Y_out_vec[243] = x92;
        Y_out_vec[244] = x117;
        Y_out_vec[245] = x125;
        Y_out_vec[246] = x131;
        Y_out_vec[247] = ddq[3];
        Y_out_vec[248] = dq[3];
        Y_out_vec[249] = (((dq[3]) > 0) - ((dq[3]) < 0));
        Y_out_vec[250] = 1;
        Y_out_vec[251] = x156;
        Y_out_vec[252] = x168;
        Y_out_vec[253] = x180;
        Y_out_vec[254] = x185;
        Y_out_vec[255] = x194;
        Y_out_vec[256] = x211;
        Y_out_vec[257] = x222;
        Y_out_vec[258] = 0;
        Y_out_vec[259] = 0;
        Y_out_vec[260] = 0;
        Y_out_vec[261] = 0;
        Y_out_vec[262] = x241;
        Y_out_vec[263] = x255;
        Y_out_vec[264] = x268;
        Y_out_vec[265] = x274;
        Y_out_vec[266] = x285;
        Y_out_vec[267] = x304;
        Y_out_vec[268] = x320;
        Y_out_vec[269] = 0;
        Y_out_vec[270] = 0;
        Y_out_vec[271] = 0;
        Y_out_vec[272] = 0;
        Y_out_vec[273] = x340;
        Y_out_vec[274] = x358;
        Y_out_vec[275] = x373;
        Y_out_vec[276] = x385;
        Y_out_vec[277] = x393;
        Y_out_vec[278] = x414;
        Y_out_vec[279] = x433;
        Y_out_vec[280] = 0;
        Y_out_vec[281] = 0;
        Y_out_vec[282] = 0;
        Y_out_vec[283] = 0;
        Y_out_vec[284] = 0;
        Y_out_vec[285] = 0;
        Y_out_vec[286] = 0;
        Y_out_vec[287] = 0;
        Y_out_vec[288] = 0;
        Y_out_vec[289] = 0;
        Y_out_vec[290] = 0;
        Y_out_vec[291] = 0;
        Y_out_vec[292] = 0;
        Y_out_vec[293] = 0;
        Y_out_vec[294] = 0;
        Y_out_vec[295] = 0;
        Y_out_vec[296] = 0;
        Y_out_vec[297] = 0;
        Y_out_vec[298] = 0;
        Y_out_vec[299] = 0;
        Y_out_vec[300] = 0;
        Y_out_vec[301] = 0;
        Y_out_vec[302] = 0;
        Y_out_vec[303] = 0;
        Y_out_vec[304] = 0;
        Y_out_vec[305] = 0;
        Y_out_vec[306] = 0;
        Y_out_vec[307] = 0;
        Y_out_vec[308] = 0;
        Y_out_vec[309] = 0;
        Y_out_vec[310] = 0;
        Y_out_vec[311] = 0;
        Y_out_vec[312] = 0;
        Y_out_vec[313] = 0;
        Y_out_vec[314] = 0;
        Y_out_vec[315] = 0;
        Y_out_vec[316] = 0;
        Y_out_vec[317] = 0;
        Y_out_vec[318] = 0;
        Y_out_vec[319] = 0;
        Y_out_vec[320] = 0;
        Y_out_vec[321] = 0;
        Y_out_vec[322] = x144;
        Y_out_vec[323] = x159;
        Y_out_vec[324] = x172;
        Y_out_vec[325] = x182;
        Y_out_vec[326] = x187;
        Y_out_vec[327] = x200;
        Y_out_vec[328] = x216;
        Y_out_vec[329] = ddq[4];
        Y_out_vec[330] = dq[4];
        Y_out_vec[331] = (((dq[4]) > 0) - ((dq[4]) < 0));
        Y_out_vec[332] = 1;
        Y_out_vec[333] = x236;
        Y_out_vec[334] = x249;
        Y_out_vec[335] = x261;
        Y_out_vec[336] = x270;
        Y_out_vec[337] = x279;
        Y_out_vec[338] = x292;
        Y_out_vec[339] = x309;
        Y_out_vec[340] = 0;
        Y_out_vec[341] = 0;
        Y_out_vec[342] = 0;
        Y_out_vec[343] = 0;
        Y_out_vec[344] = x335;
        Y_out_vec[345] = x353;
        Y_out_vec[346] = x368;
        Y_out_vec[347] = x380;
        Y_out_vec[348] = x388;
        Y_out_vec[349] = x402;
        Y_out_vec[350] = x421;
        Y_out_vec[351] = 0;
        Y_out_vec[352] = 0;
        Y_out_vec[353] = 0;
        Y_out_vec[354] = 0;
        Y_out_vec[355] = 0;
        Y_out_vec[356] = 0;
        Y_out_vec[357] = 0;
        Y_out_vec[358] = 0;
        Y_out_vec[359] = 0;
        Y_out_vec[360] = 0;
        Y_out_vec[361] = 0;
        Y_out_vec[362] = 0;
        Y_out_vec[363] = 0;
        Y_out_vec[364] = 0;
        Y_out_vec[365] = 0;
        Y_out_vec[366] = 0;
        Y_out_vec[367] = 0;
        Y_out_vec[368] = 0;
        Y_out_vec[369] = 0;
        Y_out_vec[370] = 0;
        Y_out_vec[371] = 0;
        Y_out_vec[372] = 0;
        Y_out_vec[373] = 0;
        Y_out_vec[374] = 0;
        Y_out_vec[375] = 0;
        Y_out_vec[376] = 0;
        Y_out_vec[377] = 0;
        Y_out_vec[378] = 0;
        Y_out_vec[379] = 0;
        Y_out_vec[380] = 0;
        Y_out_vec[381] = 0;
        Y_out_vec[382] = 0;
        Y_out_vec[383] = 0;
        Y_out_vec[384] = 0;
        Y_out_vec[385] = 0;
        Y_out_vec[386] = 0;
        Y_out_vec[387] = 0;
        Y_out_vec[388] = 0;
        Y_out_vec[389] = 0;
        Y_out_vec[390] = 0;
        Y_out_vec[391] = 0;
        Y_out_vec[392] = 0;
        Y_out_vec[393] = 0;
        Y_out_vec[394] = 0;
        Y_out_vec[395] = 0;
        Y_out_vec[396] = 0;
        Y_out_vec[397] = 0;
        Y_out_vec[398] = 0;
        Y_out_vec[399] = 0;
        Y_out_vec[400] = 0;
        Y_out_vec[401] = 0;
        Y_out_vec[402] = 0;
        Y_out_vec[403] = 0;
        Y_out_vec[404] = x238;
        Y_out_vec[405] = x252;
        Y_out_vec[406] = x265;
        Y_out_vec[407] = x243;
        Y_out_vec[408] = x282;
        Y_out_vec[409] = x296;
        Y_out_vec[410] = x312;
        Y_out_vec[411] = ddq[5];
        Y_out_vec[412] = dq[5];
        Y_out_vec[413] = (((dq[5]) > 0) - ((dq[5]) < 0));
        Y_out_vec[414] = 1;
        Y_out_vec[415] = x337;
        Y_out_vec[416] = x355;
        Y_out_vec[417] = x370;
        Y_out_vec[418] = x382;
        Y_out_vec[419] = x390;
        Y_out_vec[420] = x404;
        Y_out_vec[421] = x423;
        Y_out_vec[422] = 0;
        Y_out_vec[423] = 0;
        Y_out_vec[424] = 0;
        Y_out_vec[425] = 0;
        Y_out_vec[426] = 0;
        Y_out_vec[427] = 0;
        Y_out_vec[428] = 0;
        Y_out_vec[429] = 0;
        Y_out_vec[430] = 0;
        Y_out_vec[431] = 0;
        Y_out_vec[432] = 0;
        Y_out_vec[433] = 0;
        Y_out_vec[434] = 0;
        Y_out_vec[435] = 0;
        Y_out_vec[436] = 0;
        Y_out_vec[437] = 0;
        Y_out_vec[438] = 0;
        Y_out_vec[439] = 0;
        Y_out_vec[440] = 0;
        Y_out_vec[441] = 0;
        Y_out_vec[442] = 0;
        Y_out_vec[443] = 0;
        Y_out_vec[444] = 0;
        Y_out_vec[445] = 0;
        Y_out_vec[446] = 0;
        Y_out_vec[447] = 0;
        Y_out_vec[448] = 0;
        Y_out_vec[449] = 0;
        Y_out_vec[450] = 0;
        Y_out_vec[451] = 0;
        Y_out_vec[452] = 0;
        Y_out_vec[453] = 0;
        Y_out_vec[454] = 0;
        Y_out_vec[455] = 0;
        Y_out_vec[456] = 0;
        Y_out_vec[457] = 0;
        Y_out_vec[458] = 0;
        Y_out_vec[459] = 0;
        Y_out_vec[460] = 0;
        Y_out_vec[461] = 0;
        Y_out_vec[462] = 0;
        Y_out_vec[463] = 0;
        Y_out_vec[464] = 0;
        Y_out_vec[465] = 0;
        Y_out_vec[466] = 0;
        Y_out_vec[467] = 0;
        Y_out_vec[468] = 0;
        Y_out_vec[469] = 0;
        Y_out_vec[470] = 0;
        Y_out_vec[471] = 0;
        Y_out_vec[472] = 0;
        Y_out_vec[473] = 0;
        Y_out_vec[474] = 0;
        Y_out_vec[475] = 0;
        Y_out_vec[476] = 0;
        Y_out_vec[477] = 0;
        Y_out_vec[478] = 0;
        Y_out_vec[479] = 0;
        Y_out_vec[480] = 0;
        Y_out_vec[481] = 0;
        Y_out_vec[482] = 0;
        Y_out_vec[483] = 0;
        Y_out_vec[484] = 0;
        Y_out_vec[485] = 0;
        Y_out_vec[486] = x334;
        Y_out_vec[487] = x352;
        Y_out_vec[488] = x367;
        Y_out_vec[489] = x379;
        Y_out_vec[490] = x360;
        Y_out_vec[491] = x401;
        Y_out_vec[492] = x420;
        Y_out_vec[493] = ddq[6];
        Y_out_vec[494] = dq[6];
        Y_out_vec[495] = (((dq[6]) > 0) - ((dq[6]) < 0));
        Y_out_vec[496] = 1;
    }
    Y_out_mat = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(Y_out_vec.data(), q.size(), 497 / q.size());
    LOG_INFO << "InvDynamicRegressorPrimary: Output size: [" << Y_out_mat.rows() << ", " << Y_out_mat.cols() << "]";
    return Y_out_mat;
}

Eigen::MatrixXd InvDynamicRegressorSecondary(const Eigen::VectorXd& q, const Eigen::VectorXd& dq, const Eigen::VectorXd& ddq)
{
    return Eigen::MatrixXd{};
}
}  // namespace dynamics
}  // namespace robot_server_lib