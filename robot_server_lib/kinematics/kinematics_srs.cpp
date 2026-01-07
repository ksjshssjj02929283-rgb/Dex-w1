#include "kinematics_srs.h"

namespace robot_server_lib
{
namespace kinematics
{

using namespace DexeRP::Kinematics;
// SRSKinematics 构造函数
SRSKinematics::SRSKinematics(bool is_left, bool is_anthropmorphic) : KinematicsBase()
{
    std::shared_ptr<W1V02KineParams> w1_kine_params = std::make_shared<W1V02KineParams>(is_left, is_anthropmorphic);
    this->w1_kine_ = std::make_shared<DexeRP::Kinematics::SRSKinematics>(w1_kine_params);
}

SRSKinematics::~SRSKinematics() = default;

PoseQuaternion SRSKinematics::GetFK(const Eigen::VectorXd &q_in, const pinocchio::SE3 &tcp_frame)
{
    // 1) 把 pinocchio::SE3 转为库中使用的 Mat4d (row-major)
    DexeRP::Mat4d tcp;
    Eigen::Matrix4d tcp_eig = tcp_frame.toHomogeneousMatrix();  // use toHomogeneousMatrix()
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            tcp(i, j) = tcp_eig(i, j);
        }
    }
    w1_kine_->SetTCP(tcp);

    // 2) 把 q_in (column vector) 转为 DexeRP::VecXd (row vector alias)
    DexeRP::CVecXd q_row(q_in.size());
    for (int i = 0; i < q_in.size(); ++i)
    {
        q_row(i) = q_in(i);
    }

    // 3) 调用底层 FK 接口
    DexeRP::Mat4d xpos;
    bool res_fk = w1_kine_->GetFK(q_row, xpos);

    // 4) 将返回的 4x4 矩阵转换为 平移 + 四元数，并返回 PoseQuaternion
    if (!res_fk)
    {
        // FK 失败，返回默认 PoseQuaternion
        return PoseQuaternion{};
    }

    Eigen::Matrix4d xpos_eig;
    xpos_eig.setZero();
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            xpos_eig(i, j) = xpos(i, j);
        }
    }

    Eigen::Vector3d trans = xpos_eig.block<3, 1>(0, 3);
    Eigen::Matrix3d rot = xpos_eig.block<3, 3>(0, 0);
    Eigen::Quaterniond quat(rot);

    // 使用成员赋值确保不调用不同参数数量的构造函数
    PoseQuaternion out;
    out.position = trans;
    out.orientation = quat;
    return out;
}

bool SRSKinematics::GetIK(const PoseQuaternion &pose_in, Eigen::VectorXd &q_in_and_out, const pinocchio::SE3 &tcp_frame)
{
    // 1) 把 pinocchio::SE3 转为库中使用的 Mat4d (row-major)
    DexeRP::Mat4d tcp;
    Eigen::Matrix4d tcp_eig = tcp_frame.toHomogeneousMatrix();  // use toHomogeneousMatrix()
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            tcp(i, j) = tcp_eig(i, j);
        }
    }
    w1_kine_->SetTCP(tcp);

    // 2) 把 PoseQuaternion 转为 Mat4d
    DexeRP::Mat4d xpos;
    Eigen::Matrix4d pose_eig = Eigen::Matrix4d::Identity();
    pose_eig.block<3, 1>(0, 3) = pose_in.position;
    pose_eig.block<3, 3>(0, 0) = pose_in.orientation.toRotationMatrix();
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            xpos(i, j) = pose_eig(i, j);
        }
    }

    // 3) 把 q_in_and_out (column vector) 转为 CVecXd (column vector alias)
    DexeRP::CVecXd qpos_seed(q_in_and_out.size());
    for (int i = 0; i < q_in_and_out.size(); ++i)
    {
        qpos_seed(i) = q_in_and_out(i);
    }

    // 4) 调用底层 IK 接口
    DexeRP::CVecXd qpos;
    auto [res_psi, psi] = w1_kine_->CalcArmAngle(qpos_seed);
    if (!res_psi)
        return false;
    bool res_ik = w1_kine_->GetNearestIK(xpos, qpos_seed, qpos);
    if (!res_ik)
        return false;

    // 5) 把结果 CVecXd 转回 q_in_and_out (column vector)
    for (int i = 0; i < q_in_and_out.size(); ++i)
    {
        q_in_and_out(i) = qpos(i);
    }

    return true;
}

}  // namespace kinematics
}  // namespace robot_server_lib
