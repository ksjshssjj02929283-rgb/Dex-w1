#pragma once

#include "kinematics_base.h"

namespace robot_server_lib
{
namespace kinematics
{

class SRSKinematics : public KinematicsBase
{
public:
    SRSKinematics(const SRSParameters &srs_params);
    ~SRSKinematics() override;
    PoseQuaternion GetFK(const Eigen::VectorXd &q_in, const pinocchio::SE3 &tcp_frame = pinocchio::SE3::Identity()) override;
    bool GetIK(const PoseQuaternion &pose_in, Eigen::VectorXd &q_in_and_out, const pinocchio::SE3 &tcp_frame = pinocchio::SE3::Identity()) override;
private:
    bool _ComputeInverseKinematics(const pinocchio::SE3 &target_pose_se3, const Eigen::VectorXd &joint_seed, double nsparam, Eigen::Matrix<double, 4, 7> &joints_list) const;
    bool _ChooseIKSolution(std::vector<Eigen::VectorXd> &ik_solutions, const Eigen::VectorXd &joints_seed) const;
    bool _CalculateJointAngles(const Eigen::Vector3d &P_s_to_w, int elbow_GC4, Eigen::VectorXd &joints) const;
    bool _ReferencePlane(const pinocchio::SE3 &pose, int elbow_GC4, Eigen::Vector3d &V_v_to_sew, Eigen::Matrix3d &R03_v, Eigen::VectorXd &joint_v) const;
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
