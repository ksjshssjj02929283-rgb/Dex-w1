#pragma once

#include "kinematics_base.h"

namespace robot_server_lib
{
namespace kinematics
{

class NumericalKinematics : public KinematicsBase
{
   public:
    NumericalKinematics(const pinocchio::Model &model) : KinematicsBase()
    {
        // Initialize the kinematics with the provided parameters
        this->model_ = model;
        this->data_ = pinocchio::Data(model);
        this->ndof_ = model.nv;  // Number of degrees of freedom
    }

    ~NumericalKinematics() override = default;

    PoseQuaternion GetFK(const Eigen::VectorXd &q_in, const pinocchio::SE3 &tcp_frame = pinocchio::SE3::Identity()) override
    {
        pinocchio::SE3 pose;
        pinocchio::forwardKinematics(model_, data_, q_in);
        pose = data_.oMi[model_.nv] * tcp_frame;
        PoseQuaternion pose_out;
        pose_out.position = pose.translation();
        pose_out.orientation = Eigen::Quaterniond(pose.rotation());
        pose_out.orientation.normalize();
        return pose_out;
    }

    bool GetIK(const PoseQuaternion &pose_in, Eigen::VectorXd &q_in_and_out, const pinocchio::SE3 &tcp_frame = pinocchio::SE3::Identity()) override
    {
        assert(q_in_and_out.size() == this->ndof_);

        // remove the TCP frame from the pose
        pinocchio::SE3 pose_in_se3;
        pose_in_se3.translation() = pose_in.position;
        pose_in_se3.rotation() = pose_in.orientation.toRotationMatrix();
        pose_in_se3 = pose_in_se3 * tcp_frame.inverse();
        int joint_id = model_.njoints - 1;

        using Vector6d = Eigen::Matrix<double, 6, 1>;
        Vector6d err;
        Eigen::VectorXd v(model_.nv);
        pinocchio::Data::Matrix6x J(6, model_.nv);
        J.setZero();
        const Eigen::VectorXd q_in = q_in_and_out;

        for (int i = 0;; ++i)
        {
            pinocchio::forwardKinematics(model_, data_, q_in_and_out);
            const pinocchio::SE3 iMd = data_.oMi[joint_id].actInv(pose_in_se3);
            err = pinocchio::log6(iMd).toVector();
            if (err.norm() < eps_)
                return true;
            if (i >= it_max_)
                return false;

            pinocchio::computeJointJacobian(model_, data_, q_in_and_out, joint_id, J);
            pinocchio::Data::Matrix6 Jlog;
            pinocchio::Jlog6(iMd.inverse(), Jlog);
            J = -Jlog * J;
            pinocchio::Data::Matrix6 JJt;
            JJt.noalias() = J * J.transpose();
            JJt.diagonal().array() += damp_;
            v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
            q_in_and_out = integrate(model_, q_in_and_out, v * dt_);
        }

        for (int i = 0; i < q_in_and_out.size(); ++i)
        {
            //遍历每个关节， 如果变化量超过阈值则返回false
            if (std::abs(q_in[i] - q_in_and_out[i]) > threshold_)
                return false;
        }
        return true;
    }

   private:
    pinocchio::Model model_;
    pinocchio::Data data_;
    int ndof_;  // Number of degrees of freedom
    double eps_ = 1e-5;
    int it_max_ = 1000;
    double dt_ = 1e-1;
    double damp_ = 1e-6;
    double threshold_ = 0.0174532;  // 1 deg
};
}  // namespace kinematics
}  // namespace robot_server_lib