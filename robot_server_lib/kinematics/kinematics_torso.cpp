#include "kinematics_torso.h"

namespace robot_server_lib
{
namespace kinematics
{

PoseQuaternion TorsoKinematics::GetFK(const Eigen::VectorXd& q, const pinocchio::SE3& tcp_frame)
{
    if (q.size() != 3)
    {
        LOG_CRIT << "GetFK: input q size must be 3";
        assert(q.size() != 3 && "TorsoKinematics: input q size must be 3");
    }
    PoseQuaternion out{};

    const double q1 = q[0];
    const double q2 = q[1];
    const double q3 = q[2];

    const double y_theta = q1 + q2 + q3;
    const double x = l1_ * std::sin(q1) + l2_ * std::sin(q1 + q2) + l3_ * std::sin(y_theta);
    const double z = l0_ + l1_ * std::cos(q1) + l2_ * std::cos(q1 + q2) + l3_ * std::cos(y_theta);

    // 先构造 buttock 末端位姿（关于机体/chassis），再右乘 tcp_frame
    Eigen::Vector3d p_buttock(x, 0.0, z);
    Eigen::AngleAxisd Ry(y_theta, Eigen::Vector3d::UnitY());
    Eigen::Quaterniond q_buttock(Ry);

    pinocchio::SE3 T_buttock(q_buttock, p_buttock);
    pinocchio::SE3 T_out = T_buttock * tcp_frame;

    out.position = T_out.translation();
    out.orientation = Eigen::Quaterniond(T_out.rotation());
    return out;
}

PoseQuaternion TorsoKinematics::GetTorsoPose(const Eigen::VectorXd& XZRY)
{
    LOG_CRIT << "GetTorsoPose: output q size must be 3";
    assert(XZRY.size() == 3 && "TorsoKinematics: XZRY size must be 3");
    PoseQuaternion out{};

    const double x = XZRY[0];
    const double z = XZRY[1];
    const double ry = XZRY[2];

    Eigen::Vector3d p_buttock(x, 0.0, z);
    Eigen::AngleAxisd Ry(ry, Eigen::Vector3d::UnitY());
    Eigen::Quaterniond q_buttock(Ry);

    pinocchio::SE3 T_buttock(q_buttock, p_buttock);
    out.position = T_buttock.translation();
    out.orientation = Eigen::Quaterniond(T_buttock.rotation());
    return out;
}

bool TorsoKinematics::GetIK(const PoseQuaternion& pose, Eigen::VectorXd& q_out, const pinocchio::SE3& tcp_frame)
{
    if (q_out.size() != 3)
    {
        LOG_CRIT << "TorsoKinematics GetIK: output q size must be 3";
        return false;
    }
    pinocchio::SE3 T_tcp(Eigen::Quaterniond(pose.orientation).normalized(), pose.position);
    pinocchio::SE3 T_buttock = T_tcp * tcp_frame.inverse();

    const Eigen::Vector3d p = T_buttock.translation();
    const Eigen::Matrix3d R = T_buttock.rotation();

    // 仅考虑平面 x-z，旋转绕 y
    const double x = p.x();
    const double z = p.z();
    const double y_theta = std::atan2(R(0, 2), R(2, 2));  // Ry: sin=R02, cos=R22

    // 工作空间检查
    const double dist = std::sqrt(x * x + std::pow(z - l0_, 2));
    if (dist > (l1_ + l2_ + l3_))
    {
        return false;
    }

    // 去掉末端段 l3 带来的位移
    const double x_ = x - l3_ * std::sin(y_theta);
    const double z_ = (z - l0_) - l3_ * std::cos(y_theta);

    // 二连杆 IK
    double c2 = (l1_ * l1_ + l2_ * l2_ - (x_ * x_ + z_ * z_)) / (2.0 * l1_ * l2_);
    c2 = std::clamp(c2, -1.0, 1.0);
    const double acos_c2 = std::acos(c2);

    double q2 = knee_front_ ? (acos_c2 - M_PI) : (M_PI - acos_c2);
    const double s2 = std::sin(q2);
    const double q1 = std::atan2(x_, z_) - std::atan2(l2_ * s2, l1_ + l2_ * std::cos(q2));
    const double q3 = y_theta - q1 - q2;

    Eigen::Vector3d qsol(q1, q2, q3);
    // 限幅检查
    if ((qsol.array() < qmin_.array()).any())
        return false;
    if ((qsol.array() > qmax_.array()).any())
        return false;

    q_out.segment<3>(0) = qsol;
    return true;
}

}  // namespace kinematics
}  // namespace robot_server_lib